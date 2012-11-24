/*
 * omap_mt9d113_isp.c : 02/03/2010
 * g.revaillot, revaillot@archos.com
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
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
#include <media/mt9d113.h>

#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define OMAP_MT9D113_ISP_DEV_NAME	"omap-mt9d113-isp"

#define BOARD_I2C_BUSNUM		2

static int reset_gpio;
static int pwdn_gpio;
struct regulator * mt9d113_regulator;

static int dev_addr = MT9D113_I2C_ADDR;

#define OMAP2_CONTROL_PBIAS 		0x48002520
#define OMAP2_CONTROL_PBIAS_VMODE1	(1 << 8)
#define OMAP2_CONTROL_PBIAS_PWRDNZ1	(1 << 9)

#define OMAP2_CONTROL_WKUP_CTRL 		0x48002A5C
#define OMAP2_CONTROL_WKUP_GPIO_IO_PWRDNZ	(1 << 6)

static void init_buffer_pbias(void) 
{
	u32 ctrl;

	ctrl = omap_readl(OMAP2_CONTROL_WKUP_CTRL);
	ctrl |= OMAP2_CONTROL_WKUP_GPIO_IO_PWRDNZ;
	omap_writel(ctrl, OMAP2_CONTROL_WKUP_CTRL);

	ctrl = omap_readl(OMAP2_CONTROL_PBIAS);
	ctrl &= ~OMAP2_CONTROL_PBIAS_PWRDNZ1;
	omap_writel(ctrl, OMAP2_CONTROL_PBIAS);
	
	ctrl = omap_readl(OMAP2_CONTROL_PBIAS);
	ctrl &= ~OMAP2_CONTROL_PBIAS_VMODE1;
	omap_writel(ctrl, OMAP2_CONTROL_PBIAS);

	ctrl = omap_readl(OMAP2_CONTROL_PBIAS);
	ctrl |= OMAP2_CONTROL_PBIAS_PWRDNZ1;
	omap_writel(ctrl, OMAP2_CONTROL_PBIAS);
}

static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
	.capture_mem = PAGE_ALIGN(1600 * 1200 * 2) * 4,
	.ival_default	= { 1, MT9D113_FRAME_RATE },
};

static int mt9d113_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
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

static struct isp_interface_config mt9d113_if_config = {
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

static u32 mt9d113_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	int ret = 0;

	printk("%s to %d\n", __FUNCTION__, xclkfreq);
	ret = isp_set_xclk(vdev->cam->isp, xclkfreq, MT9D113_USE_XCLKA);

	return ret;
}

static int mt9d113_reset_pulse(struct v4l2_int_device *s)
{
	// pulse reset low for t4 (min 10xtclk) 
	gpio_set_value(reset_gpio, 0);
	msleep(1);
	gpio_set_value(reset_gpio, 1);

	return 0;
}

static int mt9d113_sensor_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	switch (power) {
		case V4L2_POWER_OFF:
			printk("mt9d113_sensor_power_set : disable power.\n");
			// XXX: release isp interface ?

			// enable reset
			gpio_set_value(reset_gpio, 0);

			// set device to hard standby 
			gpio_set_value(pwdn_gpio, 1);

			if (previous_power != V4L2_POWER_OFF)
				isp_disable_mclk(isp);

			break;

		case V4L2_POWER_STANDBY:
		case V4L2_POWER_ON:
			if (previous_power == V4L2_POWER_OFF) {
				printk("mt9d113_sensor_power_set : enable power.\n");
				// timings are faaaaar too big for testing.
		
				// XXX: aren't we supposed to request that interface before ?
				isp_configure_interface(vdev->cam->isp, &mt9d113_if_config);

				// release standby
				gpio_set_value(pwdn_gpio, 0);

				// release reset (was forced to low)
				gpio_set_value(reset_gpio, 1);

				// XXX: wait for t?, since we're not doing what's expected...
				msleep(5);
			}

			break;
	}

	previous_power = power;

	return 0;
}

static struct mt9d113_platform_data mt9d113_pdata = {
	.reset_pulse	= mt9d113_reset_pulse,
	.power_set      = mt9d113_sensor_power_set,
	.priv_data_set  = mt9d113_sensor_set_prv_data,
	.set_xclk	= mt9d113_sensor_set_xclk,
	.ifparm		= NULL,
};

static struct i2c_board_info __initdata mt9d113_i2c_board_info = {
	I2C_BOARD_INFO("mt9d113", MT9D113_I2C_ADDR),
	.platform_data = &mt9d113_pdata,
};

static int __init omap_mt9d113_isp_probe(struct platform_device *pdev)
{
	struct i2c_adapter *adap;
	int *pdata;

	pdata = (int*) pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "omap_mt9d113_isp_probe: No platform data !\n");
		return -ENOMEM;
	}

	reset_gpio = pdata[0];
	pwdn_gpio = pdata[1];

	mt9d113_regulator = regulator_get(&pdev->dev, "2v8d_cam");
	if (mt9d113_regulator == NULL) {
		printk(KERN_DEBUG "omap_mt9d113_isp_probe: I2C Probe failed : could not get regulator\n");
		return -ENODEV;
	}

	adap = i2c_get_adapter(BOARD_I2C_BUSNUM);

	mt9d113_i2c_board_info.addr = dev_addr;

	if ( !(mt9d113_client = i2c_new_device(adap, &mt9d113_i2c_board_info)) ) {
		printk(KERN_DEBUG "omap_mt9d113_isp_probe: MT9D113 I2C Registration failed \n");
		return -ENODEV;
	}

	return 0;
}

static int omap_mt9d113_isp_remove(struct platform_device *pdev)
{
	printk("%s\n", __FUNCTION__);

	if (mt9d113_regulator != NULL) {
		// regulator_disable(mt9d113_regulator);
		regulator_put(mt9d113_regulator);
	}

	i2c_unregister_device(mt9d113_client);
	return 0;
}

static struct platform_driver omap_mt9d113_isp_driver = {
	.probe		= omap_mt9d113_isp_probe,
	.remove		= omap_mt9d113_isp_remove,
	.driver		= {
		.name	= OMAP_MT9D113_ISP_DEV_NAME,
	},
};

static int __devinit omap_mt9d113_isp_init_module(void)
{
	// XXX: move and clean that for real boards
	init_buffer_pbias();
	//omap_cfg_reg(R25_3430_GPIO129);
	//gpio_direction_output(129, 0);

	return platform_driver_register(&omap_mt9d113_isp_driver);
}

static void __exit omap_mt9d113_isp_exit_module(void)
{
	platform_driver_unregister(&omap_mt9d113_isp_driver);
}


module_init( omap_mt9d113_isp_init_module );
module_exit( omap_mt9d113_isp_exit_module );

/* Module information */
MODULE_AUTHOR("Guillaume Revaillot, revaillot@archos.com");
MODULE_DESCRIPTION("Archos Glue : OMAP3 <--> MT9D113 driver");
MODULE_LICENSE("GPL");
