/*
 *  omap_ov7675_isp.c -- Archos Glue : Omap3 <--> OV7675 driver
 *
 * Copyright 2010 Archos
 * Author: Jean-Christophe RONA
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/clock.h>

#include <media/v4l2-int-device.h>

/* include V4L2 camera driver related header file */
#include <media/ov7675.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define OMAP_OV7675_ISP_DEV_NAME	"omap-ov7675-isp"

#define BOARD_I2C_BUSNUM		2

static struct i2c_client *ov7675_client;

static int reset_gpio;
static int pwdn_gpio;
static struct regulator * vdda;

static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
	.capture_mem = PAGE_ALIGN(640 * 480 * 2) * 6,
	.ival_default	= { 1, OV7675_FRAME_RATE },
};

static int ov7675_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
//	hwc->interface_type = ISP_PARLL;
	return 0;
}

static struct isp_interface_config ov7675_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
//	.vdint0_timing = 0x0,
//	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_hs_vs = 0,
//	.dcsub = 42,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_RG_GB,
	.pixelclk = 0,
//	.wbal.coef0 = 0x23,
//	.wbal.coef1 = 0x20,
//	.wbal.coef2 = 0x20,
//	.wbal.coef3 = 0x30,
	.u.par.par_bridge = 0x2,
	.u.par.par_clk_pol = 0x0,
	.cam_mclk = CM_CAM_MCLK_HZ,
};

static int ov7675_sensor_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	switch (power) {
	case V4L2_POWER_OFF:
		printk("ov7675_sensor_power_set : Power OFF...\n");
		/* Power Down Sequence */
		/* PWDN is active HIGH */
		gpio_set_value(pwdn_gpio, 1);
		if (previous_power != V4L2_POWER_OFF)
			isp_disable_mclk(isp);
		break;
		
	case V4L2_POWER_STANDBY:
	case V4L2_POWER_ON:
		printk("ov7675_sensor_power_set : Power ON...\n");
		if (previous_power == V4L2_POWER_OFF) {

			/* Power Up Sequence */
			isp_configure_interface(vdev->cam->isp, &ov7675_if_config);

			// fyi, regulator_enable takes +/-8ms

			/* Give sensor sometime to get out of the reset.
			 * Datasheet says >=5 ms.
			 */
			msleep(10);

			/* PWDN is active HIGH. Set LOW to release
			 * PWDN at the end of the power up sequence.
			 */
			gpio_set_value(pwdn_gpio, 0);

			/* We need to wait a little before any I2C access.
			 * Datasheet says >= 20 ms.
			 */
			msleep(40);
		}
		break;
	}

	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

static u32 ov7675_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	int ret;

	ret = isp_set_xclk(vdev->cam->isp, xclkfreq, OV7675_USE_XCLKA);
	printk("ov7675_sensor_set_xclk : Setting XCLK to %d (needed = %d)...\n", ret, xclkfreq);
	if ((xclkfreq != 0 && ret == 0) || (xclkfreq == 0 && ret != 0))
		return -1;

	return 0;
}

static struct ov7675_platform_data ov7675_pdata = {
	.power_set      = ov7675_sensor_power_set,
	.priv_data_set  = ov7675_sensor_set_prv_data,
	.set_xclk	= ov7675_sensor_set_xclk,
	.ifparm		= NULL,
};

static struct i2c_board_info __initdata ov7675_i2c_board_info = {
	I2C_BOARD_INFO("ov7675", OV7675_I2C_ADDR),
	.platform_data = &ov7675_pdata,
};


static int __init omap_ov7675_isp_probe(struct platform_device *pdev)
{
	struct i2c_adapter *adap;
	int *pdata;

	pdata = (int*) pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "omap_ov7675_isp_probe: No platform data !\n");
		return -ENOMEM;
	}

	reset_gpio = pdata[0];
	pwdn_gpio = pdata[1];

	vdda = regulator_get(&pdev->dev, "vdd_cam");

	msleep(10);

	adap = i2c_get_adapter(BOARD_I2C_BUSNUM);
	if ( !(ov7675_client = i2c_new_device(adap, &ov7675_i2c_board_info)) ) {
		printk(KERN_DEBUG "omap_ov7675_isp_probe: OV7675 I2C Registration failed \n");
		return -ENODEV;
	}

	return 0;
}

static int omap_ov7675_isp_remove(struct platform_device *pdev)
{
	i2c_unregister_device(ov7675_client);

#if 0
	/* Put the sensor to reset */
	gpio_set_value(reset_gpio, 1);
	gpio_set_value(pwdn_gpio, 1);
#endif

	return 0;
}

static int omap_ov7675_isp_suspend(struct platform_device *pdev, 
		pm_message_t msg)
{
	gpio_direction_input(pwdn_gpio);
	return 0;
}

static int omap_ov7675_isp_resume(struct platform_device *pdev)
{
	gpio_direction_output(pwdn_gpio, 0);
	return 0;
}

static struct platform_driver omap_ov7675_isp_driver = {
	.probe		= omap_ov7675_isp_probe,
	.remove		= omap_ov7675_isp_remove,
	.suspend_late	= omap_ov7675_isp_suspend,
	.resume_early	= omap_ov7675_isp_resume,
	.driver		= {
		.name	= OMAP_OV7675_ISP_DEV_NAME,
	},
};

static int __devinit omap_ov7675_isp_init_module(void)
{
	return platform_driver_register(&omap_ov7675_isp_driver);
}

static void __exit omap_ov7675_isp_exit_module(void)
{
	platform_driver_unregister(&omap_ov7675_isp_driver);
}


module_init( omap_ov7675_isp_init_module );
module_exit( omap_ov7675_isp_exit_module );

/* Module information */
MODULE_AUTHOR("Jean-Christophe RONA, rona@archos.com");
MODULE_DESCRIPTION("Archos Glue : Omap3 <--> OV7675 driver");
MODULE_LICENSE("GPL");
