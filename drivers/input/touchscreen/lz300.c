/*
 * drivers/input/touchscreen/lz300msf.c
 *
 * Copyright (c) 2011 elec. Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include "calibration_ts.h"
#include <linux/earlysuspend.h>

#define DEBUG 0

#if DEBUG==1
#define DBG(format, args...) printk(format, ##args)
#else
#define DBG(format, ...)
#endif

#define MAX_SUPPORT_POINT	2// //  4

struct lz300msf
{
  struct input_dev *input;
  char phys[32];
  struct timer_list timer;

  struct delayed_work work;
  struct workqueue_struct *wq;

  struct i2c_client *client;
  //struct multitouch_event mt_event;
  u16 model;
  int irq;
  s16 wid_threshold;
  s16 hgt_threshold;
  u16 single_cnt;
  u16 dual_cnt;
  u8 touchmode;
  s32 x, y, rx, ry;
  s32 tmp_x, tmp_y, tmp_rx, tmp_ry;
  s32 m_dx, m_dy;
//u8                  m_dx, m_dy;
};

struct lz300msf *gb_lz300msf;
struct i2c_client *pclient;

volatile struct adc_point gADPoint;

#define ZERO_TOUCH		0
#define ONE_TOUCH		1
#define TWO_TOUCH		2	//for tow touch zoom in/out
#define TWO_TOUCH_SF	3	//for two touch moving
#define lz300msf_I2C_ADDR     0x48//0x49            //device address
#define LZ300MSF_IRQ          RK29_PIN0_PA2 //18        //interrupt pin
#define DEVICE_NAME			  "lz300msf"

/* Register Map*/
#define REG_SystemReset		0x00
#define REG_SetupCmd		0x01
#define REG_SequenceCmd		0x02
#define REG_Status			0x10

#define REG_SeqData_Start	0x11
#define REG_SeqData_X1		0x11
#define REG_SeqData_Y1		0x13
#define REG_SeqData_Z1		0x15
#define REG_SeqData_Z2		0x17
#define REG_SeqData_X2		0x19
#define REG_SeqData_Y2		0x1B

/*Setup Command Configuration*/
#define NormalMode		0x00
#define SleepMode1		0x10
#define SleepMode2		0x20
#define PD0				0x01

/*Sequence Command Configuration*/
#define ADConver_6_Times 			0x00
#define ADConver_10_Times 		0x08

#define SampTimes_0u 			0x00
#define SampTimes_5u 			0x01
#define SampTimes_10u 		0x02
#define SampTimes_20u 		0x03
#define SampTimes_50u 		0x04
#define SampTimes_100u 		0x05
#define SampTimes_200u 		0x06
#define SampTimes_500u 		0x07
#define SampTimes_Value     SampTimes_50u

#define SeqMode_XYZ_Scan 		0x00
#define SeqMode_XY_Scan 		0x10
#define SeqMode_X_Scan 			0x20
#define SeqMode_Y_Scan 			0x30
#define SeqMode_Z_Scan 			0x40
#define SeqMode_X2_Scan 		0x50
#define SeqMode_All_Scan 		0x60
#define SeqMode_Y2_Scan 		0x70
#define R_MAX 	0x7FFF

#define Rx_Plate		    1024    /* original value 700 */
#define R_Threshold_High 	2200    /* Rtouch (TP Pressure) */
#define R_Threshold_Low 	 700    /* 1300//700	Rtouch * 0.5(0.6) */
#define R_Threshold_invalide    1   /* 170	        Rtouch * 0.5(0.6) */

#define Diff_th               55	/* Move_diff_Threshold for single Touch */
/*For Dual Touch*/
#define Tth 	              10	/* for comparing Two or One touch by Limits values, Two_Touch_Threshold===ok */
#define Sth 	               3 	/* Shift value Threshold which shift between two limits for Dual Touch(for ZoomIn and ZoomOut) */
/*Gesture Judgement Threshold*/
#define LineTh_length	     700	/* for line gesture for single touch */
#define LineTh_width	     300	/* for line gesture for single touch */
#define PanTh_length	     700	/* for Pan gesture for dual touch */
#define PanTh_width		     300	/* for Pan gesture for dual touch */
#define CWTh	            1500	/* for circular direction Judgement */
#define TapTh	               2	/* for Tap Judgement(times) */
#define JudgeDataMax	       3	/* for tow touch number of data to Judgement */

s16 X_LIMITS = 0;
s16 Y_LIMITS = 0;

#if 0 // 2.6.38
extern int uncali_limit_x;
extern int uncali_limit_y;

#define X_LIMITS_NUM uncali_limit_x     /* 2835//2798//2828//2391 */
#define Y_LIMITS_NUM uncali_limit_y     /* 2374//2507//2375//2762 */
#else
int uncali_limit_x = 2391;
int uncali_limit_y = 2762;

#define X_LIMITS_NUM uncali_limit_x     /* 2835//2798//2828//2391 */
#define Y_LIMITS_NUM uncali_limit_y     /* 2374//2507//2375//2762 */
#endif

#define NumberFilter	6

s16 X1_BACK, Y1_BACK, X2_BACK, Y2_BACK;

#define MAX_12BIT		       ((1 << 12) - 1)
#define TP_SIZE_WID         800         /* 3948//4095// 800 */
#define TP_SIZE_WID_MIN       0         /* 100//4095//600 */
#define TP_SIZE_HGT         600         /* 3988//4095//600 */
#define TP_SIZE_HGT_MIN       0         /* 197 */
#define TP_SIZE_HGT_DEC  (TP_SIZE_HGT_MIN+TP_SIZE_HGT)

#define TP_DEF_DX           (8)         /* //1//5 */
#define TP_DEF_DY           (12)        /* //1//16 */

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend lz300_early_suspend;
#endif

/**
 * Noise Filter
 * get 6 or 8 elements
 * Drop the max and min value,average the other 4 values.
 *
 * @param datatemp
 * @return
 */
static s16 Noise_Filter_Fast(s16* datatemp)
{
  s8 i;
  s16 swaptemp;
  s16 min, max;
  // init total, min, max
  swaptemp = 0;
  min = 32767;
  max = -32768;
  // sum up the values
  for (i = 0; i < NumberFilter; i++)
  {
    // sum up values
    swaptemp = swaptemp + datatemp[i];
    // get max value
    if (datatemp[i] > max)
      max = datatemp[i];
    // get min value
    if (datatemp[i] < min)
      min = datatemp[i];
  }
  // subtract extremes
  swaptemp -= (min + max);
  // calc average
  swaptemp = swaptemp >> 2;

  return swaptemp;
}

#define CONFIG_SENSOR_I2C_SPEED     100000       /* 100000 Hz */

/**
 * LZ300MSF Master i2c Write
 * @param client
 * @param SubAddr
 * @param Command
 * @param data
 * @return
 */
static s8 LZ300MSF_IICWrite1(struct i2c_client *client, u8 SubAddr, u8 Command, u8 data)
{
  int err, cnt;
  struct i2c_msg msg[1];

  u8 buf[3];
  buf[0] = SubAddr;
  buf[1] = Command;
  buf[2] = data;

  msg->addr = 0x48;
  msg->flags = 0;
  msg->buf = buf;
  msg->len = sizeof(buf);
  msg->scl_rate = CONFIG_SENSOR_I2C_SPEED; /* ddl@rock-chips.com : 100kHz */
  msg->read_type = 0; /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */

  cnt = 3;
  err = -EAGAIN;

  while ((cnt--) && (err < 0))
  {
    /* ddl@rock-chips.com :  Transfer again if transent is failed   */
    err = i2c_transfer(client->adapter, msg, 1);

    if (err >= 0)
    {
      // printk("\n  write reg ok, try to write again!\n");
      return 0;
    }
    else
    {
      printk("\n  write reg failed, try to write again!\n");
      udelay(10);
    }
  }

  return err;
}
#define LZ300MSF_IICWrite(a,b) LZ300MSF_IICWrite1(client,a,b,0x04)

/**
 * LZ300 master read from i2c
 *   LZ300 uses non standard i2c-protocol which sends addr, subaddr
 *   and then directly responds
 *   (damn chinese dilletants)
 * @param client
 * @param SubAddr
 * @param readdata
 * @param nread
 * @return
 */
static s8 LZ300MSF_IICRead1(struct i2c_client *client, u8 SubAddr, u8 *readdata, s8 nread)
{
  int err, cnt;

  struct i2c_msg msg[1];
  readdata[0] = SubAddr;
  msg[0].addr = 0x48;
  msg[0].flags = I2C_M_RD | I2C_M_REV_DIR_ADDR; // I2C_M_REV_DIR_ADDR = flag for special i2c-handling
  msg[0].buf = readdata;
  msg[0].len = nread;
  msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED; /* ddl@rock-chips.com : 100kHz */
  msg[0].read_type = 2; /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

  cnt = 3;
  err = -EAGAIN;
  while ((cnt--) && (err < 0))
  {
    /* ddl@rock-chips.com :  Transfer again if transent is failed   */
    err = i2c_transfer(client->adapter, msg, 1);

    if (err >= 0)
    {
      //printk("\n read reg ok, try to read again! reg:\n");
      return 0;
    }
    else
    {
      printk("\n read reg failed, try to read again! reg:\n");
      udelay(10);
    }
  }

  return err;
}
#define LZ300MSF_IICRead(a,b,c) LZ300MSF_IICRead1(client,a,b,c)

/**
 * Get touch ADC limits
 * TODO: check if this is working, LIMIT_X/_Y report 0!!
 * @param client
 * @param xlimits
 * @param ylimits
 * @return
 */
static s8 lz300msf_GetLimits(struct i2c_client *client, s16 *xlimits, s16 *ylimits)
{
  u8 i, j, icdata[2];
  s16 x, y, TempDxy[NumberFilter];
  u8 status;

  for (j = 0; j < NumberFilter; j++)
  {
    LZ300MSF_IICWrite(REG_SequenceCmd,
                      SeqMode_X2_Scan | SampTimes_Value | ADConver_6_Times);
    // wait for ADC conversion to be finished
    for (i = 10, status = 1; i && status != 0; i--)
    {
      udelay(0x20 << SampTimes_Value);
      LZ300MSF_IICRead(REG_Status, &status, 1);
    }
    LZ300MSF_IICRead(REG_SeqData_Start, icdata, 2);
    // icdata[0]: high byte, icdata[1]: low byte.
    TempDxy[j] = (((s16) icdata[0]) << 4 | ((s16) icdata[1]) >> 4);
  }
  x = Noise_Filter_Fast(TempDxy);
  DBG("LZ300-LIMITX=%04X %04X %04X %04X %04X %04X %04X\n",
      TempDxy[0], TempDxy[1], TempDxy[2],
      TempDxy[3], TempDxy[4], TempDxy[5], x);

  for (j = 0; j < NumberFilter; j++)
  {
    LZ300MSF_IICWrite(REG_SequenceCmd,
                      SeqMode_Y2_Scan | SampTimes_Value | ADConver_6_Times);
    // wait for ADC conversion to be finished
    for (i = 10, status = 1; i && status != 0; i--)
    {
      udelay(0x20 << SampTimes_Value);
      LZ300MSF_IICRead(REG_Status, &status, 1);
    }
    LZ300MSF_IICRead(REG_SeqData_Start, icdata, 2);
    // icdata[0]: high byte, icdata[1]: low byte.
    TempDxy[j] = (((s16) icdata[0]) << 4 | ((s16) icdata[1]) >> 4);
  }
  y = Noise_Filter_Fast(TempDxy);
  DBG("LZ300-LIMITY=%04X %04X %04X %04X %04X %04X %04X\n",
      TempDxy[0], TempDxy[1], TempDxy[2],
      TempDxy[3], TempDxy[4], TempDxy[5], y);

  *xlimits = x;
  *ylimits = y;

  gADPoint.x = x;
  gADPoint.y = y;

  return 0;
}

/**
 * Initialize touch device
 * @param client
 * @param xlimits
 * @param ylimits
 * @return
 */
static s8 Init_LZ300MSF(struct i2c_client *client, s16 *xlimits, s16 *ylimits)
{
  s16 x = X_LIMITS_NUM, y = Y_LIMITS_NUM;
  int ret = (__gpio_get_value(LZ300MSF_IRQ) == 0);
  disable_irq_nosync(gb_lz300msf->irq);
  LZ300MSF_IICWrite(0xFF, 0xFF);
  // Initiate TS System reset
  LZ300MSF_IICWrite(REG_SystemReset, 0x01);
  // wait for System Reset
  mdelay(500);
  // set up normal mode
  LZ300MSF_IICWrite(REG_SetupCmd, NormalMode);

  lz300msf_GetLimits(client, xlimits, ylimits);
  ret = ret || (__gpio_get_value(LZ300MSF_IRQ) == 0);

  printk("threshold value:(%d, %d)\n", *xlimits, *ylimits);

  if ((*xlimits != 0) && (*ylimits != 0) && (ret == 0))
  {
    printk("------------LZ300MSF INIT ok------------\n");
    x = *xlimits;
    y = *ylimits;
    *xlimits = x;
    *ylimits = y;
    X_LIMITS = x;
    Y_LIMITS = y;
    ret = 1;
  }
  else
  {
    printk("------------LZ300MSF INIT ERROR ------------\n");
    ret = 0;
  }
  enable_irq((gb_lz300msf->irq));
  return ret;
}

/**
 * Read ADC values
 * @param client
 * @param pAdcArr
 * @return
 */
static s16 lz300msf_read_adc(struct i2c_client *client, s16 *pAdcArr)
{
  u32 x_temp = 0;
  u16 z1     = 0;
  u16 z2     = 0;
  u16 dx     = 0;
  u16 dy     = 0;
  u16 pRatio = 0;
  u8  icdata[12];
  u8  status[1] = { 1 };
  u8 touch_mode = ZERO_TOUCH;
  u8 i;
  u8 j;

  // send ADC request (All channels)
  LZ300MSF_IICWrite(REG_SequenceCmd,
                    SeqMode_All_Scan | SampTimes_Value | ADConver_6_Times);
  // wait for ADC conversion to be finished
  for (i = 10, status[0] = 1; i && status[0] != 0; i--)
  {
    udelay(0x20 << SampTimes_Value);
    LZ300MSF_IICRead(REG_Status, status, 1);
  }
  // now read ADC values
  LZ300MSF_IICRead(REG_SeqData_X1, icdata, 12);

  // dump raw I2C values
  DBG("LZ300-I2C=%02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x (%02x)\n",
      icdata[0], icdata[1], icdata[2], icdata[3], icdata[4], icdata[5],
      icdata[6], icdata[7], icdata[8], icdata[9], icdata[10], icdata[11], i);

  // convert i2c values to uint16
  for (i = 0; i < 6; i++)
  {
    pAdcArr[i] = icdata[i * 2] << 4 | icdata[i * 2 + 1] >> 4;
  }

  // store to "sensible var names"
  x_temp = pAdcArr[0];
  z1 = pAdcArr[2];
  z2 = pAdcArr[3];
  dx = pAdcArr[4];
  dy = pAdcArr[5];

  // find max limits for x2,y2 (since they reference to high value)
  if (dx > X_LIMITS) X_LIMITS = dx;
  if (dy > Y_LIMITS) Y_LIMITS = dy;

  /*
   * figure out the current touch mode
   */

  // is touch pressed ...
  if (__gpio_get_value(LZ300MSF_IRQ) == 0 && z1 >= Tth)
  {
    // calculate pressure ration between z1 and z2 to determine singe/dual touch
    if (z1)
      pRatio = 10 * (z2 - z1) / z1;
    else
      pRatio = 100;
    DBG("pRatio=%5d,%5d,%5d,%5d\n", pRatio, x_temp, z1, z2);

    // detect dual/single touch ...
    if (pRatio < 13 && ((X_LIMITS - dx) >= Tth || (Y_LIMITS - dy) >= Tth))
    {
      touch_mode = TWO_TOUCH;
    }
    else
    {
      touch_mode = ONE_TOUCH;
    }
  }
  else
  {
    touch_mode = ZERO_TOUCH;
  }

  pAdcArr[6] = touch_mode;

  DBG("LZ300-VAL=%04X %04X %04X %04X %04X %04X %04X %04X %04X\n",
      pAdcArr[0], pAdcArr[1], pAdcArr[2], pAdcArr[3],
      pAdcArr[4], pAdcArr[5], pAdcArr[6],
      X_LIMITS, Y_LIMITS);

  return touch_mode;
}

/**
 * report a touch point
 * @param ts
 * @param id
 * @param xAdc
 * @param yAdc
 * @param zAdc
 */
static void reportTouchPoint(struct lz300msf *ts, int id, int xAdc, int yAdc, int zAdc)
{
  int cal_x, cal_y;

  DBG("LZ300-TOUCH_ADC(%d), %04x, %04x, %04x\n", id, xAdc, yAdc, zAdc);

  input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
  if(zAdc)
  {
    // calibrate ADC value to absolute screen coordinates
    TouchPanelCalibrateAPoint(xAdc, yAdc, &cal_x, &cal_y);
    cal_x /= 4;
    cal_y /= 4;

    DBG("LZ300-TOUCHPOINT(%d), %4d, %4d, %4d\n", id, cal_x, cal_y, zAdc);

    // now report the touch event ...
    input_report_abs(ts->input, ABS_MT_PRESSURE,    zAdc ? 255 : 0);
    input_report_abs(ts->input, ABS_MT_POSITION_X,  cal_x);
    input_report_abs(ts->input, ABS_MT_POSITION_Y,  cal_y);
  }
  input_mt_sync(ts->input);
}

/**
 * read touch point(s) and report them
 * @param ts
 * @return
 */
static int lz300msf_read_point(struct lz300msf *ts)
{
  int ret = 0;
  int rx = 0, ry = 0;
  int z1 = 0, z2 = 0;
  int x1 = 0, y1 = 0;
  int x2 = 0, y2 = 0;
  u8 touch_mode = 0;
  u16 adcVal[16] = { 0 };

  // read ADC values
  touch_mode = lz300msf_read_adc(ts->client, adcVal);

  // get z-values (touch pressure)
  z1 = adcVal[2];
  z2 = adcVal[3];

  switch (touch_mode)
  {
  case TWO_TOUCH:
    // store current coordinates for 2-touch
    ts->x = adcVal[0];
    ts->y = adcVal[1];
    ts->rx = adcVal[4];
    ts->ry = adcVal[5];

    /*
     * calculate virtual touchpoints for dual touch
     */

    // calculate delta x/y
    rx = (X_LIMITS - ts->rx) * 12;
    ry = (Y_LIMITS - ts->ry) * 12;

    // virtual touch point 1
    x1 = ts->x - rx;
    y1 = ts->y - ry;
    reportTouchPoint(ts, 0, x1, y1, z1);

    // virtual touch point 2
    x2 = ts->x + rx;
    y2 = ts->y + ry;
    reportTouchPoint(ts, 1, x2, y2, z1);
    break;

  case ONE_TOUCH:
    // store current coordinates for 1-touch
    ts->x = adcVal[0];
    ts->y = adcVal[1];

    // report single touch point
    reportTouchPoint(ts, 0, ts->x, ts->y, z1);
    // ... report second finger lifted
    reportTouchPoint(ts, 1, ts->x, ts->y, 0);
    break;

  case ZERO_TOUCH:
  default:
    // report first finger lifted
    reportTouchPoint(ts, 0, ts->x, ts->y, 0);
    // ... report second finger lifted
    reportTouchPoint(ts, 1, ts->x, ts->y, 0);
    break;
  }
  // remember last touch mode
  ts->touchmode = touch_mode;
  // sync all reported events
  input_sync(ts->input);

  // if touch ist still active ...
  if (touch_mode != ZERO_TOUCH)
  {
    // queue next reading in 10ms
    queue_delayed_work(ts->wq, &ts->work, msecs_to_jiffies(40));
  }
  else
  {
    // re-enable IRQ
    enable_irq((ts->irq));
  }

  return ret;
}

/**
 * perform background work
 * @param work
 */
static void lz300msf_work(struct work_struct *work)
{
  struct lz300msf *ts =
    container_of(to_delayed_work(work), struct lz300msf, work);

  lz300msf_read_point(ts);
}

/**
 * handle touch IRQ
 * @param irq
 * @param handle
 * @return
 */
static irqreturn_t lz300msf_irq(int irq, void *handle)
{
  struct lz300msf *ts = handle;

  disable_irq_nosync(ts->irq);
  queue_delayed_work(ts->wq, &ts->work, 0);

  return IRQ_HANDLED;
}

static char cal_status = 0;

/**
 *
 * @param ts
 * @return
 */
static int set_mode(struct lz300msf *ts)
{
  int ret;
  ret = Init_LZ300MSF(ts->client, &ts->wid_threshold, &ts->hgt_threshold);

  if (!ret)
  {
    printk("set mode err\n");
    cal_status = 0;
  }
  else
  {
    printk("lz300 set mode ok\n");
    uncali_limit_x = X_LIMITS;
    uncali_limit_y = Y_LIMITS;
    printk("lz300 set mode ok\n");
    cal_status = 1;
  }
  return ret;
}

/**
 * TPCAL interface handler
 * @param kobj
 * @param attr
 * @param buf
 * @return
 */
ssize_t tp_cal_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

  if (cal_status)
    return sprintf(buf, "successful");
  else
    return sprintf(buf, "fail");
}

/**
 * TPCAL interface handler
 * @param kobj
 * @param attr
 * @param buf
 * @param count
 * @return
 */
ssize_t tp_cal_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
                     size_t count)
{

  if (!strncmp((char*) buf, "tp_cal", strlen("tp_cal")))
  {
    set_mode(gb_lz300msf);
  }

  return count;
}

struct kobj_attribute tp_cal_attrs =
{
  .attr = {
    .name = "tp_calibration",
    .mode = 0777
  },
  .show = tp_cal_show,
   .store= tp_cal_store,
  };

struct attribute *tp_attrs[] = { &tp_cal_attrs.attr, NULL };

static struct kobj_type tp_kset_ktype =
{
  .sysfs_ops = &kobj_sysfs_ops,
  .default_attrs = &tp_attrs[0],
};

/**
 *
 * @param ts_dev
 * @return
 */
static int tp_cal_add_attr(struct lz300msf *ts_dev)
{
  int result;
  struct kobject *parentkobject;
  struct kobject * me = kmalloc(sizeof(struct kobject), GFP_KERNEL);
  if (!me)
    return -ENOMEM;
  memset(me, 0, sizeof(struct kobject));
  kobject_init(me, &tp_kset_ktype);
  parentkobject = &ts_dev->input->dev.kobj;
  result = kobject_add(me, parentkobject->parent->parent->parent, "%s",
                       "tp_calibration");
  return result;
}

/**
 * set TP to sleep
 * @param client
 */
static void lz300_chip_sleep(struct i2c_client *client)
{
  LZ300MSF_IICWrite(REG_SetupCmd, SleepMode1);
  printk("\n<<<<<<<<<<<<<<<<<lz300 sleep >>>>>>>>>>>>>\n");
}

/**
 * wake up
 * @param client
 */
static void lz300_chip_wakeup(struct i2c_client *client)
{
  LZ300MSF_IICWrite(REG_SetupCmd, NormalMode);
  printk("\n<<<<<<<<<<<<<<<<<lz300 wakeup >>>>>>>>>>>>>\n");
}

/**
 * Set TP to suspend
 * @param h
 */
static void lz300_suspend(struct early_suspend *h)
{
  if (!gb_lz300msf)
    return;
  disable_irq(gb_lz300msf->irq);
  lz300_chip_sleep(gb_lz300msf->client);
}

/**
 * Set TP to resume
 * @param h
 */
static void lz300_resume(struct early_suspend *h)
{
  if (!gb_lz300msf)
    return;
  lz300_chip_wakeup(gb_lz300msf->client);
  msleep(10);
  enable_irq(gb_lz300msf->irq);
  msleep(10);
}

extern int screen_x[5];
extern int screen_y[5];
extern int uncali_x_default[5];
extern int uncali_y_default[5];

/**
 * Probe for LZ300 connection
 * @param client
 * @param id
 * @return
 */
static int lz300msf_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct lz300msf *ts;
  struct input_dev *input_dev;
  struct i2c_lz300_platform_data *pdata = client->dev.platform_data;
  int err;

  if (!pdata)
  {
    dev_dbg(&client->dev, "platform data is required!\n");
    return -EINVAL;
  }
  printk("---lz300msf start probe---\n");

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    return -EIO;

  ts = kzalloc(sizeof(struct lz300msf), GFP_KERNEL);

  input_dev = input_allocate_device();
  if (!ts || !input_dev)
  {
    err = -ENOMEM;
    goto err_free_ts;
  }
  ts->client = client;
  i2c_set_clientdata(client, ts);

  ts->input = input_dev;
  ts->irq = client->irq;
  ts->touchmode = ZERO_TOUCH;

  if (pdata->init_platform_hw)
    pdata->init_platform_hw();

  if (!ts->irq)
  {
    dev_dbg(&ts->client->dev, "no IRQ?\n");
    return -ENODEV;
  }
  else
  {
    ts->irq = gpio_to_irq(ts->irq);
  }
  snprintf(ts->phys, sizeof(ts->phys), "%s/input0", DEVICE_NAME);
  gb_lz300msf = ts;
  input_dev->name = "lz300msf Touchscreen";
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_I2C;

  input_dev->evbit[0] = BIT_MASK(EV_ABS);
  set_bit(EV_SYN, input_dev->evbit);

  /* register as multitouch device */
  input_set_abs_params(input_dev, ABS_X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE,   0, 255,   0, 0);
  input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);

  err = input_register_device(input_dev);
  if (err < 0)
  {
    dev_err(&client->dev, "irq %d busy?\n", ts->irq);
    goto err_free_ts;
  }
  Init_LZ300MSF(client,&ts->wid_threshold, &ts->hgt_threshold);
  printk("threshold value:(%d, %d)\n", ts->wid_threshold, ts->hgt_threshold);
  //threshold value:(2391, 2762)
  ts->m_dx = TP_DEF_DX;
  ts->m_dy = TP_DEF_DY;

  //setup_timer(&ts->timer, lz300msf_timer, (unsigned long)ts);

  ts->wq = create_workqueue("lz300msf");
  INIT_DELAYED_WORK(&ts->work, lz300msf_work);

  gpio_direction_input(LZ300MSF_IRQ);
  gpio_pull_updown(LZ300MSF_IRQ, GPIOPullUp);
  mdelay(10);
  err = request_irq(ts->irq, lz300msf_irq, IRQF_TRIGGER_FALLING, DEVICE_NAME,
                    ts);
  if (err < 0)
  {
    dev_err(&client->dev, "irq %d busy?\n", ts->irq);
    goto err_free_ts;
  }
  tp_cal_add_attr(gb_lz300msf);
  tp_calib_iface_init(screen_x, screen_y, uncali_x_default, uncali_y_default);
#ifdef CONFIG_HAS_EARLYSUSPEND
  lz300_early_suspend.suspend = lz300_suspend;
  lz300_early_suspend.resume = lz300_resume;
  lz300_early_suspend.level = 0x2;
  register_early_suspend(&lz300_early_suspend);
#endif
  lz300_chip_wakeup(gb_lz300msf->client);
  printk("---lz300msf end probe---\n");
  return 0;

err_free_ts:
  input_free_device(input_dev);
  kfree(ts);
  return err;
}

/**
 * remove driver registration
 * @param client
 * @return
 */
static int lz300msf_remove(struct i2c_client *client)
{
  struct lz300msf *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&lz300_early_suspend);
#endif
  del_timer(&ts->timer);
  free_irq(ts->irq, ts);
  input_unregister_device(ts->input);
  tp_calib_iface_exit();
  kfree(ts);
  return 0;
}

static struct i2c_device_id lz300msf_idtable[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

static struct i2c_driver lz300msf_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME
	},
	.id_table = lz300msf_idtable,
    .probe = lz300msf_probe,
    .remove = lz300msf_remove,
};

/**
 * Initialize LZ300 driver
 */
static int __init lz300msf_init(void)
{
  printk("lz300msf_init\n");
  gADPoint.x = 0;
  gADPoint.y = 0;

  return i2c_add_driver(&lz300msf_driver);
}

/**
 * exit driver
 */
static void __exit lz300msf_exit(void)
{
  i2c_del_driver(&lz300msf_driver);
}

module_init(lz300msf_init);
module_exit(lz300msf_exit);

MODULE_AUTHOR("elec@163.com");
MODULE_DESCRIPTION("lz300msf TouchScreen Driver");
MODULE_LICENSE("GPL");
