/****************************************************************************************
 * driver/input/touchscreen/hannstar_sis809.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: 	sfm
 *Date		:  2010.2.5
 *This driver use for rk28 chip extern touchscreen. Use i2c IF ,the chip is Hannstar
 *description：
 ********************************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/board.h>

#define MAX_SUPPORT_POINT	2// //  4
#define PACKGE_BUFLEN		10

#define SIS809_RST_PIN          RK29_PIN6_PC3
//#define Singltouch_Mode
#define SAKURA_DBG                  0
#if SAKURA_DBG
#define sakura_dbg_msg(fmt,...)       do {                                      \
                                   printk("sakura dbg msg------>"                       \
                                          " (func-->%s ; line-->%d) " fmt, __func__, __LINE__ , ##__VA_ARGS__); \
                                  } while(0)
#define sakura_dbg_report_key_msg(fmt,...)      do{                                                     \
                                                    printk("sakura report " fmt,##__VA_ARGS__);          \
                                                }while(0)
#else
#define sakura_dbg_msg(fmt,...)       do {} while(0)
#define sakura_dbg_report_key_msg(fmt,...)      do{}while(0)
#endif

#if 1
#define DBG(x...)	printk( x)
#else
#define DBG(x...)
#endif

#if 1
#define TOUCH_MAX_X  800//1024
#define TOUCH_MAX_Y  600//768
#else
#define TOUCH_MAX_X  1024
#define TOUCH_MAX_Y  768
#endif

s16 X_LIMITS = 0;
s16 Y_LIMITS = 0;

#define SINGLETOUCH_MODE	0
#define MULTITOUCH_MODE		1

#define TP_STATE_IDLE       0
#define TP_STATE_PRE_DOWN   1
#define TP_STATE_DOWN       2


struct point_data {
	short status;
	short x;
	short y;
    short z;
};

struct multitouch_event{
	struct point_data point_data[MAX_SUPPORT_POINT];
	int contactid;
    int validtouch;

};

struct ts_sis809 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;
	struct workqueue_struct *wq;

	struct i2c_client	*client;
    struct multitouch_event mt_event;
	u16			model;

    unsigned char tp_state ;  //add by yxj

	bool		pendown;
	bool 	 	status;
	int			irq;
	int 		has_relative_report;
	int			(*get_pendown_state)(void);
	void		(*clear_penirq)(void);
};

static int cal_status = 0;
struct ts_sis809 *ts_sis809_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend sis809_early_suspend;
#endif

int sis809_get_pendown_state(void)
{
	return 0;
}

static void sis809_report_event(struct ts_sis809 *ts,struct multitouch_event *tc)
{
	struct input_dev *input = ts->input;
    int i,pandown = 0;
	dev_dbg(&ts->client->dev, "UP\n");

    for(i=0; i<MAX_SUPPORT_POINT;i++){
        if(tc->point_data[i].status >= 0){
            pandown |= tc->point_data[i].status;
            input_report_abs(input, ABS_MT_TRACKING_ID, i);
            input_report_abs(input, ABS_MT_TOUCH_MAJOR, tc->point_data[i].status);
            input_report_abs(input, ABS_MT_WIDTH_MAJOR, 0);
            input_report_abs(input, ABS_MT_POSITION_X, tc->point_data[i].x);
            input_report_abs(input, ABS_MT_POSITION_Y, tc->point_data[i].y);
            input_mt_sync(input);

            sakura_dbg_report_key_msg("ABS_MT_TRACKING_ID = %x, ABS_MT_TOUCH_MAJOR = %x\n ABS_MT_POSITION_X = %x, ABS_MT_POSITION_Y = %x\n",i,tc->point_data[i].status,tc->point_data[i].x,tc->point_data[i].y);
#if defined(CONFIG_HANNSTAR_DEBUG)
			printk("hannstar sis809 Px = [%d],Py = [%d] \n",tc->point_data[i].x,tc->point_data[i].y);
#endif

            if(tc->point_data[i].status == 0)
            	tc->point_data[i].status--;
        }

    }

    ts->pendown = pandown;
    input_sync(input);
}

#if defined (Singltouch_Mode)
static void sis809_report_single_event(struct ts_sis809 *ts,struct multitouch_event *tc)
{
	struct input_dev *input = ts->input;
    int cid;

    cid = tc->contactid;
    if (ts->status) {
        input_report_abs(input, ABS_X, tc->point_data[cid].x);
        input_report_abs(input, ABS_Y, tc->point_data[cid].y);
        input_sync(input);
    }
    if(ts->pendown != ts->status){
        ts->pendown = ts->status;
        input_report_key(input, BTN_TOUCH, ts->status);
        input_sync(input);
        sakura_dbg_report_key_msg("%s x =0x%x,y = 0x%x \n",ts->status?"down":"up",tc->point_data[cid].x,tc->point_data[cid].y);
    }
}
#endif

static inline int sis809_check_firmwork(struct ts_sis809 *ts)
{
    int data;
    int len = 10;
    char buf[10] = {0x03 , 0x03 , 0x0a , 0x01 , 'D' , 0x00 , 0x00 , 0x00 , 0x00 , 0x00};
	int i;
    short contactid=0;

    data = i2c_master_normal_send(ts->client, buf,len, 200*1000);

	if(data < 0){
		dev_err(&ts->client->dev, "i2c io error %d \n", data);
		return data;
	}

	data = i2c_master_normal_recv(ts->client, buf,len, 200*1000);

	if(data < 0){
		dev_err(&ts->client->dev, "i2c io error %d \n", data);
		return data;
	}

	printk("sis809 reg[5] = %c ,reg[6] = %c, reg[7] = %c, reg[8] = %c\n" , buf[5],buf[6],buf[7],buf[8]);
	printk("sis809 reg[5] = %x ,reg[6] = %x, reg[7] = %x, reg[8] = %x\n" , buf[5],buf[6],buf[7],buf[8]);
    return data;
}

static bool judgtouchstatus(short a,short b)
{
   bool staus=0;
   if((a!=0) && (b!=0))
   	staus =1;
  return staus;

}

static inline int sis809_read_values(struct ts_sis809 *ts_dev, struct multitouch_event *tc)
{
	u8	buf[10];
	int ret;
	u8 start_reg=0x00;
	int i;
	uint checksum = 0;
	uint len = 8;
	int xraw, yraw;
	char pressure_point,z,w;
	short X=0, Y=0,  X2=0,Y2=0,ContactID=0,Status=0;
	int i2c_ret;
	memset(buf,0,sizeof(u8)*len);
	i2c_ret=i2c_master_reg8_recv(ts_dev->client, start_reg, buf,len,200*1000);
       //add by yyz
       if(i2c_ret<0){
           printk("yyz_________i2c touch transfer error");
	    enable_irq(ts_dev->irq);
	    return 0;
       }
#if SINGLETOUCH_MODE
		X = ((buf[1]<<8) + buf[0]);
		Y = ((buf[3]<<8) + buf[2]);
		printk("Status = %d,X = %d, Y= %d\n",Status,X,Y);
		if(judgtouchstatus(X,Y)){
			if(ts_dev->status == 0)
			{
				ts_dev->status = 1;
				input_report_abs(ts_dev->input, ABS_X, (800-X) );
				input_report_abs(ts_dev->input, ABS_Y, (600-Y));
				input_report_key(ts_dev->input, BTN_TOUCH, 1);
			}else{
				input_report_abs(ts_dev->input, ABS_X, (800-X));
				input_report_abs(ts_dev->input, ABS_Y, (600-Y));
			}
				xpos =X;
				ypos = Y;
				input_sync(ts_dev->input);
		}
		else
		{
			ts_dev->status = 0;
			input_report_abs(ts_dev->input, ABS_X, (800-X));
			input_report_abs(ts_dev->input, ABS_Y, (600-Y));
			input_report_key(ts_dev->input, BTN_TOUCH, 0);
			input_sync(ts_dev->input);
		}
#else

	//	bool Validtouch;
	//	Validtouch = buf[1]&0x80;
		Status = buf[1]&0x01;
		checksum = (buf[1]&0x7C)>>2;

		X =  ((buf[1]<<8) + buf[0]);
		Y =  ((buf[3]<<8) + buf[2]);
		X2 = ((buf[5]<<8) + buf[4]);
		Y2 = ((buf[7]<<8) + buf[6]);
		ContactID =0;


		if((tc->point_data[ContactID].status >=0) ||judgtouchstatus(X,Y))
		{
			tc->point_data[ContactID].status = judgtouchstatus(X,Y);
			if(judgtouchstatus(X,Y))
			{
				tc->point_data[ContactID].x = TOUCH_MAX_X-X;//*TOUCH_MAX_X/800;
				tc->point_data[ContactID].y = TOUCH_MAX_Y-Y;//*TOUCH_MAX_Y/600;
			}
			ContactID =1;
			Status=1;

		}

		if((tc->point_data[ContactID].status >=0)||judgtouchstatus(X2,Y2))
		{
			tc->point_data[ContactID].status = judgtouchstatus(X2,Y2);
			if(judgtouchstatus(X2,Y2))
				{
			tc->point_data[ContactID].x = TOUCH_MAX_X-X2;//*TOUCH_MAX_X/800;
			tc->point_data[ContactID].y = TOUCH_MAX_Y-Y2;//*TOUCH_MAX_Y/600;
				}
		}

		 //printk("----%s----Status = %d,X = %d,Y= %d,X2=%d,Y2=%d,checksum=%d\n",__func__,Status,X,Y,X2,Y2,ContactID);

//	printk("Validtouch =%d,Status= %d,ContactID =%d\n",Validtouch,Status,ContactID);




		for(i=0; i<MAX_SUPPORT_POINT;i++)
		{
			if(tc->point_data[i].status >= 0)
			{
				input_report_abs(ts_dev->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(ts_dev->input, ABS_MT_TOUCH_MAJOR, tc->point_data[i].status);
				input_report_abs(ts_dev->input, ABS_MT_WIDTH_MAJOR, 0);
				input_report_abs(ts_dev->input, ABS_MT_POSITION_X, (tc->point_data[i].x));
				input_report_abs(ts_dev->input, ABS_MT_POSITION_Y, (tc->point_data[i].y));
				input_mt_sync(ts_dev->input);
				//printk("report point data i= %d,PointBuf[i].Status =%d,PointBuf[%d].X = %d,PointBuf[%d].Y =%d\n",i,tc->point_data[i].status,i,tc->point_data[i].x,i,tc->point_data[i].y);
				//printk("\n");
				if(tc->point_data[i].status == 0)
					tc->point_data[i].status--;
			}
		}
		if(Status==0)
		{
			enable_irq(ts_dev->irq);
		}
		else
		{
			queue_delayed_work(ts_dev->wq, &ts_dev->work, msecs_to_jiffies(15));
		}
		Status=0;
		input_sync(ts_dev->input);

#endif

	}



static void sis809_work(struct work_struct *work)
{
	struct ts_sis809 *ts =container_of(to_delayed_work(work), struct ts_sis809, work);
	struct multitouch_event *tc = &ts->mt_event;
	int rt;
	rt = sis809_read_values(ts,tc);

}

static irqreturn_t sis809_irq(int irq, void *handle)
{
	struct ts_sis809 *ts = handle;
	//printk("yyz_____________%s>>>>>>>\n",__func__);
#if 1
	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
	       disable_irq_nosync(ts->irq);
		queue_delayed_work(ts->wq, &ts->work, msecs_to_jiffies(10));
	}

#endif
	return IRQ_HANDLED;
}

static void sis809_free_irq(struct ts_sis809 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}
static void sis_chip_reset_gain(struct ts_sis809 *ts_dev)
{
       u8 i,j;
	u8 ucData;
	int ret=0;
	u8 start_reg=0xcc;
	u8 buf[2];
	buf[0]=0x9b;
       u8 cnt=20;
	printk("-----%s----- coming\n",__func__);
	while(cnt--)
	{
             ret=i2c_master_reg8_send(ts_dev->client, start_reg, buf,1,100*1000);
		if(ret<0)
		{
			printk("-----reset store data%s----- error\n",__func__);
		}
		else
		{
			printk("-----reset store data%s----- sucessfull\n",__func__);
			msleep(20);
			break;
		}
	}

}

static void sis_chip_Init(struct ts_sis809 *ts_dev)
{
	u8 i,j;
	u8 ucData;
	int ret=0;
	u8 start_reg;
	u8 buf[2];
	u8 cnt=40;
	printk("-----%s----- coming\n",__func__);
	start_reg=0x78;
	buf[0]=0x3;
       cnt=20;
	while(cnt--)
	{
		ret=i2c_master_reg8_send(ts_dev->client, start_reg, buf,1,100*1000);
		if(ret<0)
		{
			printk("-----%s----- error1\n",__func__);
			cal_status=0;
		}
		else
		{
			printk("-----%s----- sucessfull1\n",__func__);
			msleep(2000);
		       cal_status=1;
			break;
		}
	}
	return;
}

//自动校准
ssize_t tp_cal_show(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{

	if(cal_status)
		return sprintf(buf,"successful");
	else
		return sprintf(buf,"fail");
}

ssize_t tp_cal_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{

    if( !strncmp(buf,"tp_cal" , strlen("tp_cal")) )
    {
         sis_chip_Init(ts_sis809_dev);
    }
    return count;
}

struct kobj_attribute tp_cal_attrs =
{
        .attr = {
                .name = "tp_calibration",
                .mode = 0777},
        .show = tp_cal_show,
        .store = tp_cal_store,
};

struct attribute *tp_attrs[] =
{
        &tp_cal_attrs.attr,
        NULL
};

static struct kobj_type tp_kset_ktype = {
	.sysfs_ops	= &kobj_sysfs_ops,
	.default_attrs = &tp_attrs[0],
};

static int tp_cal_add_attr(struct ts_sis809 *ts_dev)
{
	int result;
	struct input_dev *input;
	struct kobject *parentkobject;
	struct kobject * me = kmalloc(sizeof(struct kobject) , GFP_KERNEL );
	if( !me )
		return -ENOMEM;
	memset(me ,0,sizeof(struct kobject));
	kobject_init(me, &tp_kset_ktype);
	parentkobject = &ts_dev->input->dev.kobj ;
	result = kobject_add( me , parentkobject->parent->parent->parent, "%s", "tp_calibration" );
	return result;
}


/*sleep*/
static void sis809_chip_sleep(void)
{
       int  ret=0;
	u8  start_reg=0x73;
	u8  buf[2];
	buf[0]=0x51;
	int cnt=3;
	while(cnt--){
            ret=i2c_master_reg8_send(ts_sis809_dev->client, start_reg, buf,1,100*1000);
            if(ret<0)
            {
                printk("-----sis809 sleep failed%s----- error\n",__func__);
            }
	     break;
     }
}

/*wake up*/
static void sis809_chip_wakeup(void)
{
	int  ret=0;
	u8  start_reg=0x73;
	u8  buf[2];
	buf[0]=0x50;
	int cnt=3;
	while(cnt--){
            ret=i2c_master_reg8_send(ts_sis809_dev->client, start_reg, buf,1,100*1000);
            if(ret<0)
            {
                printk("-----sis809 wakeup failed%s----- error\n",__func__);
            }
	     break;
     }
}

/***/

static void sis809_suspend(struct early_suspend *h)
{
     if(!ts_sis809_dev)
	 return;
    // printk("-----yyz_______enter sis809 suspend %s\n",__func__);
     disable_irq(ts_sis809_dev->irq);
     //sis809_chip_sleep();
     gpio_set_value(SIS809_RST_PIN, GPIO_LOW);
}

static void sis809_resume(struct early_suspend *h)
{
     if(!ts_sis809_dev)
	 return;
     //printk("-----yyz_______enter sis809 resume %s\n",__func__);
     gpio_set_value(SIS809_RST_PIN, GPIO_HIGH);
     msleep(200);
     //sis809_chip_wakeup();
     enable_irq(ts_sis809_dev->irq);
}


static int __devinit sis809_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct ts_sis809 *ts;
	struct sis809_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	printk("%s:>>>>>>>>>\n",__func__);
	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = kzalloc(sizeof(struct ts_sis809), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	ts->status =0 ;// fjp add by 2010-9-30
	ts->pendown = 0; // fjp add by 2010-10-06
	ts->tp_state = TP_STATE_IDLE ;
       ts->get_pendown_state=NULL;
	ts->wq = create_rt_workqueue("sis809_wq");
	INIT_DELAYED_WORK(&ts->work, sis809_work);

	ts->model = pdata->model;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "sis809 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

#if defined (Singltouch_Mode)
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_2, input_dev->keybit);
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev,ABS_X,0,TOUCH_MAX_X,0,0);
	input_set_abs_params(input_dev,ABS_Y,0,TOUCH_MAX_Y,0,0);
#else
	ts->has_relative_report = 0;
	input_dev->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY)|BIT_MASK(EV_SYN);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_dev->keybit[BIT_WORD(BTN_2)] = BIT_MASK(BTN_2); //jaocbchen for dual
	input_set_abs_params(input_dev, ABS_X, 0, TOUCH_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0X, 0, TOUCH_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0Y, 0, TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,0, TOUCH_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
#endif

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	if (!ts->irq) {
		dev_dbg(&ts->client->dev, "no IRQ?\n");
		return -ENODEV;
	}else{
		ts->irq = gpio_to_irq(ts->irq);
	}

	err = request_irq(ts->irq, sis809_irq, 0,
			client->dev.driver->name, ts);

	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	if (err < 0)
		goto err_free_irq;
#if 0
	err = set_irq_type(ts->irq,IRQ_TYPE_LEVEL_LOW);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}
	if (err < 0)
		goto err_free_irq;
#endif
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);
       ts_sis809_dev=ts;
	//sis_chip_Init(ts);
       sis_chip_reset_gain(ts);
	//sis809_check_firmwork(ts);
	tp_cal_add_attr(ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
    sis809_early_suspend.suspend = sis809_suspend;
    sis809_early_suspend.resume =  sis809_resume;
    register_early_suspend(&sis809_early_suspend);
#endif
	return 0;

 err_free_irq:
	sis809_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit sis809_remove(struct i2c_client *client)
{
	struct ts_sis809 *ts = i2c_get_clientdata(client);
	struct sis809_platform_data *pdata = client->dev.platform_data;

	sis809_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);
       ts_sis809_dev=NULL;
	return 0;
}

static struct i2c_device_id sis809_idtable[] = {
	{ "sis809_touch", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sis809_idtable);

static struct i2c_driver sis809_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sis809_touch"
	},
	.id_table	= sis809_idtable,
	.probe		= sis809_probe,
	.remove		= __devexit_p(sis809_remove),
};

/*static void __init sis809_init_async(void *unused, async_cookie_t cookie)
{
	printk("--------> %s <-------------\n",__func__);
	i2c_add_driver(&sis809_driver);
}*/

static int __init sis809_init(void)
{
	//async_schedule(sis809_init_async, NULL);
	printk("--------> %s <-------------\n",__func__);
	i2c_add_driver(&sis809_driver);
	return 0;
}

static void __exit sis809_exit(void)
{
	return i2c_del_driver(&sis809_driver);
}
module_init(sis809_init);
module_exit(sis809_exit);
MODULE_LICENSE("GPL");

