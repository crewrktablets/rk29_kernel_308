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
#define DEBUG 1
#if DEBUG==1
#define DBG(format, args...) printk(format, ##args)
#else
#define DBG(format, ...)
#endif

#if DEBUG==2
#define RDBG(format, args...) printk(format, ##args)
#else
#define RDBG(format, ...)
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
#define Tth 	               8	/* for comparing Two or One touch by Limits values, Two_Touch_Threshold===ok */
#define Sth 	               3 	/* Shift value Threshold which shift between two limits for Dual Touch(for ZoomIn and ZoomOut) */
/*Gesture Judgement Threshold*/
#define LineTh_length	     700	/* for line gesture for single touch */
#define LineTh_width	     300	/* for line gesture for single touch */
#define PanTh_length	     700	/* for Pan gesture for dual touch */
#define PanTh_width		     300	/* for Pan gesture for dual touch */
#define CWTh	            1500	/* for circular direction Judgement */
#define TapTh	               2	/* for Tap Judgement(times) */
#define JudgeDataMax	       3	/* for tow touch number of data to Judgement */

s16 X2_REF, Y2_REF;
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

s16 Temp_pressure = 0;
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

s16 Noise_Filter_Fast(s16* datatemp)
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
 * LZ300MSF Master Write
 * @param client
 * @param SubAddr
 * @param Command
 * @param data
 * @return
 */
s8 LZ300MSF_IICWrite1(struct i2c_client *client, u8 SubAddr, u8 Command, u8 data)
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
  { /* ddl@rock-chips.com :  Transfer again if transent is failed   */
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
 * LZ300 master read
 * @param client
 * @param SubAddr
 * @param readdata
 * @param nread
 * @return
 */
s8 LZ300MSF_IICRead1(struct i2c_client *client, u8 SubAddr, u8 *readdata, s8 nread)
{
  int err, cnt;

  struct i2c_msg msg[1];
  readdata[0] = SubAddr;
  msg[0].addr = 0x48;
  msg[0].flags = I2C_M_RD | I2C_M_REV_DIR_ADDR;
  msg[0].buf = readdata;
  msg[0].len = nread;
  msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED; /* ddl@rock-chips.com : 100kHz */
  msg[0].read_type = 2; /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

  cnt = 3;
  err = -EAGAIN;
  while ((cnt--) && (err < 0))
  { /* ddl@rock-chips.com :  Transfer again if transent is failed   */
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
 * Read ADC values
 * @param client
 * @param pAdcArr
 * @return
 */
s16 lz300msf_read_adc(struct i2c_client *client, s16 *pAdcArr)
{
  s8 touch_mode = 0;
  s16 z1, z2, pressure = 0;
  u8 icdata[12], TP_RETRY = 3, read_adc_retry = 0;
  u8 status[1], index = 0;
  s32 x_temp;

  do
  {
    LZ300MSF_IICWrite(REG_SequenceCmd, SeqMode_All_Scan | SampTimes_Value | ADConver_6_Times);
    index = 0;
    do
    {
      index++;
      LZ300MSF_IICRead(REG_Status, status, 1);
    }
    while ((status[0] != 0) && (index < 10));
    LZ300MSF_IICRead(REG_SeqData_X1, icdata, 12);

    RDBG("LZ300-I2C=%02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x\n",
        icdata[0],icdata[1],icdata[2],icdata[3],icdata[4],icdata[5],
        icdata[6],icdata[7],icdata[8],icdata[9],icdata[10],icdata[11]);

    pAdcArr[0] = icdata[0] << 4 | icdata[1] >> 4;
    pAdcArr[1] = icdata[2] << 4 | icdata[3] >> 4;

    z1 = icdata[4] << 4 | icdata[5] >> 4;
    z2 = icdata[6] << 4 | icdata[7] >> 4;

    X2_REF = icdata[8] << 4 | icdata[9] >> 4;
    Y2_REF = icdata[10] << 4 | icdata[11] >> 4;

    RDBG("LZ300-DEC=%04x %04x %04x %04x %04x %04x\n",
    		pAdcArr[0], pAdcArr[1], z1, z2, X2_REF, Y2_REF);

    x_temp = pAdcArr[0];

    if ((z1 > 0) && (z2 > z1))
    {
      pressure = ((x_temp) * (z2 - z1) / z1) >> 2; /* Rx*x/4096*(z2/z1 - 1), assume Rx=1024 */
    }
    else
    {
      pressure = R_MAX;
    }
    read_adc_retry++;
  }
  while ((pressure > R_Threshold_High || pressure < R_Threshold_invalide)
      && (read_adc_retry < TP_RETRY));

  DBG("pressure=%d,%d,%d,%d,%d\n",pressure,X2_REF,Y2_REF,z1,z2);

  Temp_pressure = pressure;
  if ((X_LIMITS - X2_REF) >= Tth || (Y_LIMITS - Y2_REF) >= Tth)
  {
    DBG("TWO_TOUCH\n");
    touch_mode = TWO_TOUCH;
  }
    else if (((X_LIMITS - X2_REF) < Tth) && ((Y_LIMITS - Y2_REF) < Tth))
  {
    DBG("ONE_TOUCH\n");
    touch_mode = ONE_TOUCH;
  }
  else
  {
    DBG("ZERO_TOUCH\n");
    touch_mode = ZERO_TOUCH;
  }

  if (touch_mode == TWO_TOUCH && pressure <= R_Threshold_Low
      && pressure > R_Threshold_invalide)
  {
    touch_mode = TWO_TOUCH;
  }
  else if (touch_mode == ONE_TOUCH && pressure < R_Threshold_High)
  {
    touch_mode = ONE_TOUCH;
  }
  else
  {
    touch_mode = ONE_TOUCH;
  }

  if (pressure == 0 || pressure == R_MAX)
  {
    DBG("ZERO_TOUCH1\n");
    touch_mode = ZERO_TOUCH;
  }DBG("touch_mode=%d\n",touch_mode);
  pAdcArr[2] = z1;
  pAdcArr[3] = z2;
  pAdcArr[4] = X2_REF;
  pAdcArr[5] = Y2_REF;
  pAdcArr[6] = touch_mode;

  DBG("LZ300-VAL=%04X %04X %04X %04X %04X %04X %04X %04X %04X\n",
      pAdcArr[0],pAdcArr[1],pAdcArr[2],pAdcArr[3],
      pAdcArr[4],pAdcArr[5],pAdcArr[6],
      X_LIMITS, Y_LIMITS);

  return touch_mode;
}

/**
 *
 * @param client
 * @param xlimits
 * @param ylimits
 * @return
 */
s8 lz300msf_GetLimits(struct i2c_client *client, s16 *xlimits, s16 *ylimits)
{
  u8 i, icdata[2];
  s16 x, y, TempDxy[NumberFilter];

  for (i = 0; i < NumberFilter; i++)
  {
    LZ300MSF_IICWrite(REG_SequenceCmd, SeqMode_X2_Scan | SampTimes_Value | ADConver_6_Times);
    LZ300MSF_IICRead(REG_SeqData_Start, icdata, 2);
    // icdata[0]: high byte, icdata[1]: low byte.
    TempDxy[i] = (((s16) icdata[0]) << 4 | ((s16) icdata[1]) >> 4);
  }
  x = Noise_Filter_Fast(TempDxy);

  for (i = 0; i < NumberFilter; i++)
  {
    LZ300MSF_IICWrite(REG_SequenceCmd, SeqMode_Y2_Scan | SampTimes_Value | ADConver_6_Times);
    LZ300MSF_IICRead(REG_SeqData_Start, icdata, 2);
    // icdata[0]: high byte, icdata[1]: low byte.
    TempDxy[i] = (((s16) icdata[0]) << 4 | ((s16) icdata[1]) >> 4);
  }
  y = Noise_Filter_Fast(TempDxy);

  *xlimits = x;
  *ylimits = y;
  //X_LIMITS =x;
  //Y_LIMITS =y;

  gADPoint.x = x;
  gADPoint.y = y;

  return 0;
}

s8
Init_LZ300MSF(struct i2c_client *client, s16 *xlimits, s16 *ylimits)
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

    enable_irq((gb_lz300msf->irq));
    return 1;
  }
  else
  {
    printk("------------LZ300MSF INIT ERROR ------------\n");
    //x=X_LIMITS_NUM;y=Y_LIMITS_NUM;
    //x=2835;y=2374;
    //*xlimits = x;
    //*ylimits = y;
    //X_LIMITS =x;
    //Y_LIMITS =y;
    enable_irq((gb_lz300msf->irq));
    return 0;
  }
}



#define FUZZ 8//30
#define FUZZ1 12//30
#define BACK_DATA(a,b,c,d) {a=c;b=d;}
#define FUZZ_MIN(a,b,c,d)  ((abs(a-c)<FUZZ)&&(abs(b-d)<FUZZ1))
#define FUZZ_MIN1(a,b,c,d)  ((abs(a-c)<FUZZ)&&(abs(b-d)<FUZZ1))

static int lz300msf_read_point(struct lz300msf *ts)
{
  int ret = 0, rx = 0, ry = 0, tmpVal = 0, z1 = 0, z2 = 0;
  s32 x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  s8 touch_mode = 0;
  s16 adcVal[16] = { 0 };
  int cal_x, cal_y;

  memset(adcVal, 0, sizeof(adcVal));

  touch_mode = lz300msf_read_adc(ts->client, adcVal);
  ts->x = adcVal[0];
  ts->y = adcVal[1];
  ts->rx = adcVal[4];
  ts->ry = adcVal[5];
  z1 = adcVal[2];
  z2 = adcVal[3];

  //printk("touch_mode,ts->x,ts->y,ts->rx,ts->ry,z1,z2(%d,%d,%d,%d,%d,%d,%d)\n", touch_mode, ts->x,ts->y,ts->rx,ts->ry,z1,z2);
  //(2,1851,2256,2366,2738,1080,2393)
  //(2,2120,2773,2720,2375,1802,3324)
  if (touch_mode == ONE_TOUCH)
  {
    if (ts->touchmode == TWO_TOUCH)
    {
      ts->touchmode = ZERO_TOUCH;
      ts->single_cnt = 0;
      ts->dual_cnt = 0;
      if ((__gpio_get_value(LZ300MSF_IRQ) == 0))
      {
        x1 = ts->x;
        y1 = ts->y;

        TouchPanelCalibrateAPoint(x1, y1, &cal_x, &cal_y);

        cal_x = cal_x / 4;
        cal_y = cal_y / 4;
        x1 = cal_x;
        y1 = cal_y;
        // report touch 0
        input_report_abs(ts->input, ABS_MT_POSITION_X, x1);
        input_report_abs(ts->input, ABS_MT_POSITION_Y, y1);
        input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, z1);
        input_report_abs(ts->input, ABS_MT_TRACKING_ID, 0);
        input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
        input_report_key(ts->input, BTN_TOUCH, 0);
        input_mt_sync(ts->input);
        // report touch 1
        input_report_abs(ts->input, ABS_MT_POSITION_X, x2);	// was x1 before
        input_report_abs(ts->input, ABS_MT_POSITION_Y, y2); // was y1 before
        input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, z2);
        input_report_abs(ts->input, ABS_MT_TRACKING_ID, 1);
        input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
        input_report_key(ts->input, BTN_TOUCH, 0);
        input_mt_sync(ts->input);
        input_sync(ts->input);

        DBG("up 11\n");
      }
    }
    else if ((++ts->single_cnt) >= 2)
    {
      ts->touchmode = ONE_TOUCH;
      ts->single_cnt = 0;
      x1 = ts->x;
      y1 = ts->y;

      gADPoint.x = x1;
      gADPoint.y = y1;

      //printk("Single>(%d,%d)\n", X_LIMITS, Y_LIMITS);

      TouchPanelCalibrateAPoint(x1, y1, &cal_x, &cal_y);

      cal_x = cal_x / 4;
      cal_y = cal_y / 4;
      x1 = cal_x;
      y1 = cal_y;

      DBG("Single>(%d,%d,%d)\n", x1, y1,z1);
      input_report_abs(ts->input, ABS_MT_POSITION_X, x1);
      input_report_abs(ts->input, ABS_MT_POSITION_Y, y1);
      input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, z1);
      input_report_abs(ts->input, ABS_MT_TRACKING_ID, 0);
      input_report_abs(ts->input, ABS_MT_PRESSURE, 255);
      input_report_key(ts->input, BTN_TOUCH, 1);
      input_mt_sync(ts->input);
      input_sync(ts->input);
    }
  }
  else if (touch_mode == TWO_TOUCH)
  {
    ts->tmp_x += ts->x;
    ts->tmp_y += ts->y;
    ts->tmp_rx += ts->rx;
    ts->tmp_ry += ts->ry;
    if (ts->touchmode == ONE_TOUCH)
    {
      ts->touchmode = ZERO_TOUCH;
      ts->single_cnt = 0;
      ts->dual_cnt = 0;
    }
    else if ((++ts->dual_cnt) >= 4)
    {
      ts->x = ts->tmp_x >> 2;
      ts->y = ts->tmp_y >> 2;
      ts->rx = ts->tmp_rx >> 2;
      ts->ry = ts->tmp_ry >> 2;
      ts->dual_cnt = 0;
      ts->touchmode = TWO_TOUCH;

      rx = X_LIMITS - ts->rx; // ts->wid_threshold  - ts->rx;
      ry = Y_LIMITS - ts->ry; //ts->hgt_threshold -  ts->ry;

      //printk("rx ry>1(%d,%d),\n", rx, ry);
      tmpVal = ts->x - (ts->m_dx * rx);
      x1 = tmpVal < 0 ? 0 : tmpVal;
      tmpVal = ts->y - (ts->m_dy * ry);
      y1 = tmpVal < 0 ? 0 : tmpVal;

      x2 = ts->x + (ts->m_dx * rx);
      y2 = ts->y + (ts->m_dy * ry);

      // calculate touch 1
      TouchPanelCalibrateAPoint(x1, y1, &cal_x, &cal_y);

      cal_x = cal_x / 4;
      cal_y = cal_y / 4;
      x1 = cal_x;
      y1 = cal_y;

      if (FUZZ_MIN( X1_BACK,Y1_BACK,x1,y1))
      {
        BACK_DATA(x1, y1, X1_BACK, Y1_BACK)
      }
      else
      {
        BACK_DATA( X1_BACK, Y1_BACK, x1, y1)
      }
      // report touch 1
      input_report_abs(ts->input, ABS_MT_POSITION_X, x1);
      input_report_abs(ts->input, ABS_MT_POSITION_Y, y1);
      input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, z1);
      input_report_abs(ts->input, ABS_MT_TRACKING_ID, 0);
      input_report_abs(ts->input, ABS_MT_PRESSURE, 255);
      input_report_key(ts->input, BTN_TOUCH, 1);
      input_mt_sync(ts->input);

      // calculate touch 2
      TouchPanelCalibrateAPoint(x2, y2, &cal_x, &cal_y);

      cal_x = cal_x / 4;
      cal_y = cal_y / 4;
      x2 = cal_x;
      y2 = cal_y;

      if (FUZZ_MIN1( X2_BACK,Y2_BACK,x2,y2))
      {
        BACK_DATA(x2, y2, X2_BACK, Y2_BACK)
      }
      else
      {
        BACK_DATA( X2_BACK, Y2_BACK, x2, y2)
      }DBG("Two>1(%d,%d),2(%d,%d)\n", x1, y1, x2, y2);
      // report touch 2
      input_report_abs(ts->input, ABS_MT_POSITION_X, x2);
      input_report_abs(ts->input, ABS_MT_POSITION_Y, y2);
      input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, z2);
      input_report_abs(ts->input, ABS_MT_TRACKING_ID, 1);
      input_report_abs(ts->input, ABS_MT_PRESSURE, 255);
      input_report_key(ts->input, BTN_TOUCH, 1);
      BACK_DATA( X2_BACK, Y2_BACK, x2, y2)
      input_mt_sync(ts->input);
      input_sync(ts->input);
    }
    //printk("Two>1(%d,%d),2(%d,%d)\n", x1, y1, x2, y2);
  }

  if (ts->dual_cnt == 0)
  {
    ts->tmp_x = 0;
    ts->tmp_y = 0;
    ts->tmp_rx = 0;
    ts->tmp_ry = 0;
  }

  if ((__gpio_get_value(LZ300MSF_IRQ) == 0) || (touch_mode != ZERO_TOUCH))
  {
    //mod_timer(&ts->timer, jiffies + msecs_to_jiffies(5));
    queue_delayed_work(ts->wq, &ts->work, msecs_to_jiffies(10));
    return ret;
  }
  else
  {
    ts->touchmode = ZERO_TOUCH;
    ts->single_cnt = 0;
    ts->dual_cnt = 0;

    input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
    input_report_key(ts->input, BTN_TOUCH, 0);
    input_report_abs(ts->input, ABS_MT_TRACKING_ID, 0);
    input_mt_sync(ts->input);

    input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(ts->input, ABS_MT_TRACKING_ID, 1);
    input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
    input_report_key(ts->input, BTN_TOUCH, 0);
    input_mt_sync(ts->input);
    input_sync(ts->input);

    DBG("up\n");
  }
  enable_irq((ts->irq));

  return ret;
}

static void lz300msf_work(struct work_struct *work)
{
	struct lz300msf *ts = container_of(to_delayed_work(work), struct lz300msf, work);

  lz300msf_read_point(ts);
}

static irqreturn_t lz300msf_irq(int irq, void *handle)
{
	struct lz300msf *ts = handle;

	if (__gpio_get_value(LZ300MSF_IRQ) == 1)
	{
		DBG("up1\n");
	}
	else
	{
		disable_irq_nosync(ts->irq);
		queue_delayed_work(ts->wq, &ts->work, 0);
	}
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
    .attr =
    {
        .name = "tp_calibration",
        .mode = 0777
    },
    .show = tp_cal_show,
    .store = tp_cal_store,
};

struct attribute *tp_attrs[] =
{
    &tp_cal_attrs.attr,
    NULL
};

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
  result = kobject_add(me, parentkobject->parent->parent->parent, "%s", "tp_calibration");
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
#if 1
  input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
  input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
  set_bit(EV_SYN, input_dev->evbit);

  /* register as multitouch device:
   set ABS_MT_TOUCH_MAJOR, ABS_MT_POSITION_X, ABS_MT_POSITION_Y*/
  input_set_abs_params(input_dev, ABS_X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_12BIT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
#else
  input_dev->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY)|BIT_MASK(EV_SYN);
  input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
  input_dev->keybit[BIT_WORD(BTN_2)] = BIT_MASK(BTN_2); //jaocbchen for dual
  input_set_abs_params(input_dev, ABS_X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
  input_set_abs_params(input_dev, ABS_HAT0X, 0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_HAT0Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X,0, TP_SIZE_WID, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TP_SIZE_HGT, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 2, 0, 0);

#endif
  err = input_register_device(input_dev);
  if (err < 0)
  {
    dev_err(&client->dev, "irq %d busy?\n", ts->irq);
    goto err_free_ts;
  }
  //Init_LZ300MSF_INIT(client,&ts->wid_threshold, &ts->hgt_threshold);
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
	err = request_irq(ts->irq, lz300msf_irq, IRQF_TRIGGER_FALLING,DEVICE_NAME, ts);
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

static struct i2c_device_id lz300msf_idtable[] =
{
    { "lz300msf", 0 },
    { }
};

static struct i2c_driver lz300msf_driver =
{
    .driver =
    {
        .owner = THIS_MODULE,
        .name  = DEVICE_NAME
    },
    .id_table = lz300msf_idtable,
    .probe    = lz300msf_probe,
    .remove   = lz300msf_remove,
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

