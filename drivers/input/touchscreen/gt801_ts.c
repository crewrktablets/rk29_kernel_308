/*
 * drivers/input/touchscreen/gt801_ts.c
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <mach/iomux.h>
#include <linux/platform_device.h>

#include "gt801_ts.h"

#define GT801_DEBUG	0
#if GT801_DEBUG
	#define gt801printk(msg...)	printk(msg);
#else
	#define gt801printk(msg...)
#endif

/** Number of INIT registers */
#define GT801_REGS_NUM 53

/** Number of touch points */
#define NUM_FINGERS  5
/** number of registers per touch point */
#define TOUCH_REG_NUM 5

/**
 * Register adress for each touch point measurement
 */
static const unsigned char touchRegMap[NUM_FINGERS][TOUCH_REG_NUM] =
{ //  xh, xl, yh, yl, press
#if NUM_FINGERS > 0
    {  0,  1,  2,  3,  4  },
#endif
#if NUM_FINGERS > 1
    {  5,  6,  7,  8,  9  },
#endif
#if NUM_FINGERS > 2
    { 10, 11, 12, 13, 14  },
#endif
#if NUM_FINGERS > 3
    { 15, 22, 23, 24, 25  },
#endif
#if NUM_FINGERS > 4
    { 26, 27, 28, 29, 30  },
#endif
};

/** Note: This assumes that address of pressure is always the highest address */
#define TOUCH_RX_BUF_SIZE (touchRegMap[NUM_FINGERS-1][ptpressure]+1)

const unsigned char GT801_RegData[GT801_REGS_NUM]=
{
#if 0
    0x19,0x05,0x06,0x28,0x02,0x14,0x14,0x10,
    0x40,0xB0,0x01,0xE0,0x03,0x4C,0x78,0x9A,
    0xBC,0xDE,0x65,0x43,0x20,0x11,0x00,0x00,
    0x00,0x00,0x05,0xCF,0x20,0x0B,0x0D,0x8D,
    0x32,0x3C,0x1E,0x28,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x01
#else
    0x19,0x02,0x07,0x28,0x02,0x14,0x14,0x10,
    0x28,0xB0,0x02,0x58,0x03,0x20,0x01,0x23,
    0x45,0x67,0x89,0xab,0xcd,0xe1,0x00,0x00,
    0x35,0x2e,0x4d,0xc1,0x20,0x00,0xe3,0x80,
    0x50,0x3c,0x1e,0xb4,0x00,0x33,0x2c,0x01,
    0xec,0x00,0x32,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x01

    /* values extracted out of LOOX PLUS kernel
	0x19,0x02,0x07,0x28,0x02,0x14,0x14,0x10,
	0x28,0xfa,0x02,0x58,0x03,0x20,0x01,0x23,
	0x45,0x67,0x89,0xab,0xcd,0xe1,0x00,0x00,
	0x35,0x2e,0x4d,0xc1,0x20,0x00,0xe3,0x80,
	0x50,0x3c,0x1e,0xb4,0x00,0x33,0x2c,0x01,
	0xec,0x00,0x32,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x01
	*/
#endif
};

struct gt801_ts_data {
	u16		model;			/* 801. */	
    u16     options;        /* swap x and y axes, etc */
	u16		x_min, x_max;	
	u16		y_min, y_max;
    uint16_t addr;
    int 	use_irq;
	int 	gpio_pendown;
	int 	gpio_reset;
	int 	gpio_reset_active_low;
	int		pendown_iomux_mode;	
	int		resetpin_iomux_mode;
	char	pendown_iomux_name[IOMUX_NAME_SIZE];	
	char	resetpin_iomux_name[IOMUX_NAME_SIZE];	
	char	phys[32];
	char	name[32];
	struct 	i2c_client *client;
    struct 	input_dev *input_dev;
    struct 	hrtimer timer;
    struct 	work_struct  work;
    struct 	early_suspend early_suspend;
};
/*tochscreen private data*/
static struct workqueue_struct *gt801_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h);
static void gt801_ts_late_resume(struct early_suspend *h);
#endif

/*read the gt801 register ,used i2c bus*/
static int gt801_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	ret =i2c_master_reg8_recv(client, reg, buf, len, 200*1000);
	if(ret < 0)
		printk("gt801_ts_work_func:i2c_transfer fail =%d\n",ret);
	return ret;
}
/* set the gt801 registe,used i2c bus*/
static int gt801_write_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	ret = i2c_master_reg8_send(client,reg, buf, len, 200*1000);
 	if (ret < 0) {
	  printk("gt801_ts_work_func:i2c_transfer fail =%d\n",ret);
    }
	return ret;
}
static int gt801_init_panel(struct gt801_ts_data *ts)
{
    return 0;
}

static void gt801_ts_work_func(struct work_struct *work)
{
	int  touchIdx = 0;

	unsigned char start_reg = 0x02;
    unsigned char buf[TOUCH_RX_BUF_SIZE];
	unsigned short x;
	unsigned short y;
    int i, j, ret;

    struct gt801_ts_data *ts = container_of(work, struct gt801_ts_data, work);
	
	gt801printk("%s\n",__FUNCTION__);
    
	ret=gt801_read_regs(ts->client, start_reg, buf, TOUCH_RX_BUF_SIZE);
	if (ret < 0) {
	  	printk("%s:i2c_transfer fail =%d\n",__FUNCTION__,ret);
		if (ts->use_irq) 
   	  		enable_irq(ts->client->irq);
		
		return;
    }
	/*
	 * dump received data
	 */
	// loop through touch points
	for(i=0; i<NUM_FINGERS; i++)
	{
      gt801printk("RAW[%d]: ",i);
      // loop through touch point registers
      for(j=0; j<TOUCH_REG_NUM; j++)
      {
        gt801printk("%02x ",buf[touchRegMap[i][j]]);
      }
      gt801printk("\n");
	}

	// calculate and report all reported touch events
    for(touchIdx=0; touchIdx<NUM_FINGERS;touchIdx++)
	{
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, touchIdx);
		if(buf[touchRegMap[touchIdx][ptpressure]] == 0)
		{
			gt801printk("%s:-%d-:buf=%d touch up\n",__FUNCTION__,touchIdx,buf[touchRegMap[touchIdx][ptpressure]]);
		}
		else
		{
            // get coordinates from buffer
            x = (((((unsigned short)buf[touchRegMap[touchIdx][ptxh]] )<< 8) )
                | buf[touchRegMap[touchIdx][ptxl]]);
		    y = (((((unsigned short)buf[touchRegMap[touchIdx][ptyh]] )<< 8) )
		        | buf[touchRegMap[touchIdx][ptyl]]);

		    // correct coordinates based on selected options
	        if (ts->options & GT801_OPT_SWAP_XY) swap(x, y);
            if (ts->options & GT801_OPT_INV_X) x = ts->x_max - x;
            if (ts->options & GT801_OPT_INV_Y) y = ts->y_max - y;

            // report coordinates
	        gt801printk("input_report_abs-%d-(%d/%d)\n",touchIdx, x, y);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
            input_report_abs(ts->input_dev, ABS_MT_PRESSURE,   255);
		}
        input_mt_sync(ts->input_dev);
    }

    input_sync(ts->input_dev);

   	if (ts->use_irq) {
   		enable_irq(ts->client->irq);
   	}
	return;
}

static enum hrtimer_restart gt801_ts_timer_func(struct hrtimer *timer)
{
    struct gt801_ts_data *ts = container_of(timer, struct gt801_ts_data, timer);
    gt801printk("%s\n",__FUNCTION__); 

    queue_work(gt801_wq, &ts->work);

    hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static irqreturn_t gt801_ts_irq_handler(int irq, void *dev_id)
{
    struct gt801_ts_data *ts = dev_id;
    gt801printk("%s=%d,%d\n",__FUNCTION__,ts->client->irq,ts->use_irq);
	
	if(ts->use_irq){
    	disable_irq_nosync(ts->client->irq);
	}
	queue_work(gt801_wq, &ts->work);
    return IRQ_HANDLED;
}
static int __devinit setup_resetPin(struct i2c_client *client, struct gt801_ts_data *ts)
{
	struct gt801_platform_data	*pdata = client->dev.platform_data;
	int err;
	
	ts->gpio_reset = pdata->gpio_reset;
    ts->gpio_reset_active_low = pdata->gpio_reset_active_low;
    ts->resetpin_iomux_mode = pdata->resetpin_iomux_mode;

    if(pdata->resetpin_iomux_name != NULL)
	    strcpy(ts->resetpin_iomux_name,pdata->resetpin_iomux_name);
		 
	gt801printk("%s=%d,%s,%d,%d\n",__FUNCTION__,ts->gpio_reset,ts->resetpin_iomux_name,ts->resetpin_iomux_mode,ts->gpio_reset_active_low);
	if (!gpio_is_valid(ts->gpio_reset)) {
		dev_err(&client->dev, "no gpio_reset?\n");
		return -EINVAL;
	}

    rk29_mux_api_set(ts->resetpin_iomux_name,ts->resetpin_iomux_mode); 

	err = gpio_request(ts->gpio_reset, "gt801_resetPin");
	if (err) {
		dev_err(&client->dev, "failed to request resetPin GPIO%d\n",
				ts->gpio_reset);
		return err;
	}
	
	err = gpio_direction_output(ts->gpio_reset, ts->gpio_reset_active_low? GPIO_LOW:GPIO_HIGH);
	if (err) {
		dev_err(&client->dev, "failed to pulldown resetPin GPIO%d,err%d\n",
				ts->gpio_reset,err);
		gpio_free(ts->gpio_reset);
		return err;
	}
	mdelay(100);
	gpio_set_value(ts->gpio_reset, ts->gpio_reset_active_low? GPIO_HIGH:GPIO_LOW);
	mdelay(100);

	return 0;
}

static int __devinit setup_pendown(struct i2c_client *client, struct gt801_ts_data *ts)
{
	int err;
	struct gt801_platform_data	*pdata = client->dev.platform_data;
	
	if (!client->irq) {
		dev_dbg(&client->dev, "no IRQ?\n");
		return -ENODEV;
	}
	
	if (!gpio_is_valid(pdata->gpio_pendown)) {
		dev_err(&client->dev, "no gpio_pendown?\n");
		return -EINVAL;
	}
	
	ts->gpio_pendown = pdata->gpio_pendown;
	strcpy(ts->pendown_iomux_name,pdata->pendown_iomux_name);
	ts->pendown_iomux_mode = pdata->pendown_iomux_mode;
	
	gt801printk("%s=%d,%s,%d\n",__FUNCTION__,ts->gpio_pendown,ts->pendown_iomux_name,ts->pendown_iomux_mode);
	
	if (!gpio_is_valid(ts->gpio_pendown)) {
		dev_err(&client->dev, "no gpio_pendown?\n");
		return -EINVAL;
	}
	
    rk29_mux_api_set(ts->pendown_iomux_name,ts->pendown_iomux_mode); 
	err = gpio_request(ts->gpio_pendown, "gt801_pendown");
	if (err) {
		dev_err(&client->dev, "failed to request pendown GPIO%d\n",
				ts->gpio_pendown);
		return err;
	}
	
	err = gpio_pull_updown(ts->gpio_pendown, GPIOPullUp);
	if (err) {
		dev_err(&client->dev, "failed to pullup pendown GPIO%d\n",
				ts->gpio_pendown);
		gpio_free(ts->gpio_pendown);
		return err;
	}
	return 0;
}

static int gt801_chip_Init(struct i2c_client *client)
{
	u8 i,j;
	int ret=0;
	u8 start_reg=0x30;
	u8 buf[GT801_REGS_NUM];
	
	gt801printk("enter gt801_chip_Init!!!!\n");

	for(j=0;j<2;j++)
	{
		ret=gt801_write_regs(client,start_reg, GT801_RegData,GT801_REGS_NUM);	
		if(ret<0)
		{
			printk("\n--%s--Set Register values error !!!\n",__FUNCTION__);
		}
		
		ret=gt801_read_regs(client, start_reg, buf,GT801_REGS_NUM);
		if(ret<0)
		{
			printk("\n--%s--Read Register values error !!!\n",__FUNCTION__);
		}
			
	 	for(i=0;i<GT801_REGS_NUM-1;i++)
		{
			if(buf[i]!=GT801_RegData[i])
			{
				printk("!!!!!!!!gt801_chip_Init err may be i2c errorat adress=%x var=%x i=%x\n",0x30+i, buf[i],i);
				break;
			}
		}
		if(i==GT801_REGS_NUM-1)
			break;
		else if(j==1)
			return -1;
		
		mdelay(500);
	}
	mdelay(100);
	
	return ret;
}

static int gt801_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct gt801_ts_data *ts;
	struct gt801_platform_data	*pdata = client->dev.platform_data;
    int ret = 0;

    gt801printk("%s \n",__FUNCTION__);
	
    if (!pdata) {
		dev_err(&client->dev, "empty platform_data\n");
		goto err_check_functionality_failed;
    }

    // set up platform hardware
    if(pdata->init_platform_hw)
    {
      ret = pdata->init_platform_hw();
      if(ret)
        goto err_check_functionality_failed;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk(KERN_ERR "gt801_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
	
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL) {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
    INIT_WORK(&ts->work, gt801_ts_work_func);
    ts->client = client;
    i2c_set_clientdata(client, ts);

	ret = setup_resetPin(client,ts);
	if(ret)
	{
		 printk("%s:setup_resetPin fail\n",__FUNCTION__);
		 goto err_input_dev_alloc_failed;
	}
	
	ret=gt801_chip_Init(ts->client);
	if(ret<0)
	{
		printk("%s:chips init failed\n",__FUNCTION__);
		goto err_resetpin_failed;
	}
	
    /* allocate input device */
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        ret = -ENOMEM;
        printk(KERN_ERR "%s: Failed to allocate input device\n",__FUNCTION__);
        goto err_input_dev_alloc_failed;
    }
	
	ts->model = pdata->model ? : 801;
	ts->options = pdata->options;
	ts->x_min = pdata->x_min;
	ts->x_max = pdata->x_max;
	ts->y_min = pdata->y_min;
	ts->y_max = pdata->y_max;
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));
	snprintf(ts->name, sizeof(ts->name), "gt%d-touchscreen", ts->model);
	ts->input_dev->phys = ts->phys;
	ts->input_dev->name = ts->name;
	ts->input_dev->dev.parent = &client->dev;

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS);
  //  ts->input_dev->absbit[0] = 
	//	BIT(ABS_MT_POSITION_X) | BIT(ABS_MT_POSITION_Y) | 
	//	BIT(ABS_MT_TOUCH_MAJOR) | BIT(ABS_MT_WIDTH_MAJOR);  // for android
    input_set_abs_params(ts->input_dev,ABS_X,
            ts->x_min ? : 0,
            ts->x_max ? : 800,
            0, 0);
    input_set_abs_params(ts->input_dev,ABS_Y,
            ts->y_min ? : 0,
            ts->y_max ? : 600,
            0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 
		    ts->x_min ? : 0,
			ts->x_max ? : 800,
			0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			ts->y_min ? : 0,
			ts->y_max ? : 600,
			0, 0);
    // input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0); //Finger Size
    // input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 10, 0, 0); //Touch Size
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,   0, 255,   0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, NUM_FINGERS, 0, 0);

    ret = input_register_device(ts->input_dev);
    if (ret) {
        printk(KERN_ERR "%s: Unable to register %s input device\n", __FUNCTION__,ts->input_dev->name);
        goto err_input_register_device_failed;
    }
	
	client->irq = gpio_to_irq(client->irq);
    if (client->irq) {
		ret = setup_pendown(client,ts);
		if(ret)
		{
			 printk("%s:setup_pendown fail\n",__FUNCTION__);
			 goto err_input_register_device_failed;
		}
		
        ret = request_irq(client->irq, gt801_ts_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_LOW, client->name, ts);
        if (ret == 0) {
            gt801printk("%s:register ISR (irq=%d)\n", __FUNCTION__,client->irq);
            ts->use_irq = 1;
        }
        else 
			dev_err(&client->dev, "request_irq failed\n");
    }

    if (!ts->use_irq) {
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = gt801_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = gt801_ts_early_suspend;
    ts->early_suspend.resume = gt801_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    printk(KERN_INFO "%s: Start touchscreen %s in %s mode\n", __FUNCTION__,ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

    return 0;

err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_resetpin_failed:
	gpio_free(ts->gpio_reset);
err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	
    return ret;
}

static int gt801_ts_remove(struct i2c_client *client)
{
    struct gt801_ts_data *ts = i2c_get_clientdata(client);
    unregister_early_suspend(&ts->early_suspend);
    if (ts->use_irq)
        free_irq(client->irq, ts);
    else
        hrtimer_cancel(&ts->timer);
    input_unregister_device(ts->input_dev);
	gpio_free(ts->gpio_pendown);
	gpio_free(ts->gpio_reset);
    kfree(ts);
    return 0;
}

static int gt801_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct gt801_ts_data *ts = i2c_get_clientdata(client);

    printk("gt801 TS Suspend\n");
    
    if (ts->use_irq)
        disable_irq(client->irq);
    else
        hrtimer_cancel(&ts->timer);

    ret = cancel_work_sync(&ts->work);
    if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
        enable_irq(client->irq);

	gpio_set_value(ts->gpio_reset, ts->gpio_reset_active_low? GPIO_LOW:GPIO_HIGH);
	
    return 0;
}


static void gt801_ts_resume_work_func(struct work_struct *work)
{
	struct gt801_ts_data *ts = container_of(work, struct gt801_ts_data, work);
	msleep(50);    //touch panel will generate an interrupt when it sleeps out,so as to avoid tihs by delaying 50ms
	enable_irq(ts->client->irq);
	PREPARE_WORK(&ts->work, gt801_ts_work_func);
	printk("enabling gt801_ts IRQ %d\n", ts->client->irq);
}


static int gt801_ts_resume(struct i2c_client *client)
{
    struct gt801_ts_data *ts = i2c_get_clientdata(client);

    gt801_init_panel(ts);
    
    printk("gt801 TS Resume\n");
	
    gpio_set_value(ts->gpio_reset, ts->gpio_reset_active_low? GPIO_HIGH:GPIO_LOW);
	
    if (ts->use_irq) {
        if(!work_pending(&ts->work)){
        	PREPARE_WORK(&ts->work, gt801_ts_resume_work_func);
        	queue_work(gt801_wq, &ts->work);
        }
    }
    else {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h)
{
    struct gt801_ts_data *ts;
    ts = container_of(h, struct gt801_ts_data, early_suspend);
    gt801_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void gt801_ts_late_resume(struct early_suspend *h)
{
    struct gt801_ts_data *ts;
    ts = container_of(h, struct gt801_ts_data, early_suspend);
    gt801_ts_resume(ts->client);
}
#endif

#define gt801_TS_NAME "gt801_ts"

static const struct i2c_device_id gt801_ts_id[] = {
    { gt801_TS_NAME, 0 },
    { }
};

static struct i2c_driver gt801_ts_driver = {
    .probe      = gt801_ts_probe,
    .remove     = gt801_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = gt801_ts_suspend,
    .resume     = gt801_ts_resume,
#endif
    .id_table   = gt801_ts_id,
    .driver = {
        .name   = gt801_TS_NAME,
    },
};

static int __devinit gt801_ts_init(void)
{
    printk("%s\n",__FUNCTION__);
    gt801_wq = create_singlethread_workqueue("gt801_wq");
    if (!gt801_wq)
        return -ENOMEM;
    return i2c_add_driver(&gt801_ts_driver);
}

static void __exit gt801_ts_exit(void)
{
    printk("%s\n",__FUNCTION__);
    i2c_del_driver(&gt801_ts_driver);
    if (gt801_wq)
        destroy_workqueue(gt801_wq);
}

module_init(gt801_ts_init);
module_exit(gt801_ts_exit);

MODULE_DESCRIPTION("gt801 Touchscreen Driver");
MODULE_LICENSE("GPL");
