/* drivers/power/rk29_adc_battery.c
 *
 * battery detect driver for the rk2918 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include <linux/wakelock.h>

static struct wake_lock batt_wake_lock;

#if defined(CONFIG_POWER_SUPPLY_DEBUG)
#define DBG(...)   printk(__VA_ARGS__)
int rk29_battery_dbg_level = 1;
#else
#define DBG(...)
int rk29_battery_dbg_level = 0;
#endif

module_param_named(dbg_level, rk29_battery_dbg_level, int, 0644);

/*******************���²�������޸�******************************/
#define	 TIMER_MS_COUNTS				50	/**< ADC sample period, base time for work queue. */
//���²�����Ҫ���ʵ�ʲ��Ե���
#define SLOPE_SECOND_COUNTS	            15	/**< Loop time for update of averaged voltage value. */

/* AX: Timeout value calculation per percent.
 * Number of seconds the voltage has to be higher / lower than the last measured volt value before
 * increasing (charge) or decreasing (discharge) the percent value by one.
 */
/**< Seconds / percent while discharging */
#define DISCHARGE_MIN_SECOND	        70
/**< Seconds / percent while charging and voltage below [BATT_NORM_VOL_IDX] */
#define CHARGE_MIN_SECOND	            45
/**< Seconds / percent while charging and voltage below [BATT_MAX_VOL_IDX] */
#define CHARGE_MID_SECOND	            90
/**< Seconds / percent while charging and voltage above [BATT_MAX_VOL_IDX] */
#define CHARGE_MAX_SECOND	           250

#define CHARGE_FULL_DELAY_TIMES        10  //����������ʱ��
#define USBCHARGE_IDENTIFY_TIMES        5  //����USB������pcʶ����ʱ��

/* AX: Number of samples taken for average voltage calculation */
#define NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)		// AX: 300

/* AX: Convert seconds to 'ticks' counted in TIMER_MS_COUNTS */
#define NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)		// AX: 1400
#define NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    	// AX: 900
#define NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    	// AX: 1800
#define NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    	// AX: 5000
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	// �����״̬����ʱ�䳤��
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	// �����״̬����ʱ�䳤��

/* AX: Modded this to be the indexes of the array now supplied by the board-*.c file */
#define ADC_RESOLUTION			1024	/* 10-bit ADC type */
#define BATT_MAX_VOL_IDX		   0	/* Index of maxium battery voltage */
#define BATT_ZERO_VOL_IDX		   1	/* Index of discharge cut-off voltage */
#define BATT_NORM_VOL_IDX		   2	/* Index of standard voltage value */

// TODO: move this to header file of battery */
#define BAT_ADC_TABLE_LEN		  11	/* Voltage to percent conversion tables entries */

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

struct rk29_adc_battery_data
{
	int irq;

	struct timer_list timer; /**< ADC timer */
	struct work_struct timer_work;
	struct work_struct dcwakeup_work;
	struct work_struct resume_work;

	struct rk29_adc_battery_platform_data *pdata;

	int full_times;

	struct adc_client *client;
	int adc_val;
	int adc_samples[NUM_VOLTAGE_SAMPLE + 2];

	int bat_status;
	int bat_status_cnt;
	int bat_health;
	int bat_present;
	int bat_voltageNow;		/** Actual battery voltage in mV */
	int bat_voltageAvg;		/** Average battery voltage in mV */
	int bat_voltageMin;		/** Minimum battery voltage in mV */
	int bat_voltageMax;		/** Maximum battery voltage in mV */
	int bat_capacity;		/** Battery capacity in 0..100% */
	int bat_change;
};
static struct rk29_adc_battery_data *gBatteryData;

/* Battery parameters presented by driver.
 *
 */
enum
{
	BATTERY_STATUS = 0,
	BATTERY_HEALTH = 1,
	BATTERY_PRESENT = 2,
	BATTERY_CAPACITY = 3,
	BATTERY_AC_ONLINE = 4,
	BATTERY_STATUS_CHANGED = 5,
	AC_STATUS_CHANGED = 6,
	BATTERY_INT_STATUS = 7,
	BATTERY_INT_ENABLE = 8,
};

typedef enum
{
	CHARGER_BATTERY = 0,	/**< Battery discharging */
	CHARGER_USB, 			/**< Not sure if charging or just supplied by USB */
	CHARGER_AC				/**< Battery charging, running on PSY */
} charger_type_t;

/*****************************************************************
 * Battery Capacity File Handling.
 */
#define BATT_FILENAME "/data/bat_last_capacity.dat"
#include <linux/fs.h>

static void
rk29_adc_battery_capacity_samples(struct rk29_adc_battery_data *bat);
static int
rk29_adc_battery_voltage_to_capacity(struct rk29_adc_battery_data *bat,
		int BatVoltage);
static struct power_supply rk29_battery_supply;

static int rk29_adc_battery_load_capacity(void)
{
	char value[4];
	int* p = (int *) value;
	long fd = sys_open(BATT_FILENAME, O_RDONLY, 0);

	if (fd < 0)
	{
		printk(
				"rk29_adc_battery_load_capacity: open file /data/bat_last_capacity.dat failed\n");
		return -1;
	}

	sys_read(fd, (char __user *) value, 4);

	sys_close(fd);

	return (*p);
}

static void rk29_adc_battery_put_capacity(int loadcapacity)
{
	char value[4];
	int* p = (int *) value;
	long fd = sys_open(BATT_FILENAME, O_CREAT | O_RDWR, 0);

	if (fd < 0)
	{
		printk(
				"rk29_adc_battery_put_capacity: open file /data/bat_last_capacity.dat failed\n");
		return;
	}
	*p = loadcapacity;
	sys_write(fd, (const char __user *) value, 4);

	sys_close(fd);
}

/*****************************************************************
 * Battery Charge Conrtol.
 */

/* Enable Charging via GPIO if supported.
 *
 */
static void rk29_adc_battery_charge_enable(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO)
	{
		gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
	}
}

/* Disable Charging via GPIO if supported.
 *
 */
static void rk29_adc_battery_charge_disable(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO)
	{
		gpio_direction_output(pdata->charge_set_pin,
				1 - pdata->charge_set_level);
	}
}

/*****************************************************************
 * Battery Charge Conrtol.
 */

/* Check if charging is enabled.
 *
 * Check if either charger or USB is connected.
 * Charger has priority over USB charging.
 *
 * returns 1 if charging else 0.
 *
 */
extern int suspend_flag;
static int rk29_adc_battery_get_charge_level(struct rk29_adc_battery_data *bat)
{
	int charge_on = 0;
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;

#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
		if (gpio_get_value (pdata->dc_det_pin) == pdata->dc_det_level)
		{
			charge_on = 1;
			goto charge_out;
		}
	}
#endif

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	if (charge_on == 0)
	{
		if (suspend_flag) return;

		if (1 == dwc_vbus_status()) //��⵽USB���룬�����޷�ʶ���Ƿ��ǳ����
		{ //ͨ����ʱ���PCʶ���־�����ʱ��ⲻ����˵���ǳ��
			if (0 == get_msc_connect_flag())
			{ //��������ʱ�����һ��ʱ��֮�󣬿�ʼ������״̬
				if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES)
				{
					gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
					charge_on = 1;
					goto charge_out;
				}
			} //���򣬲�������ģʽ
		}
		else
		{
			gBatUsbChargeCnt = 0;
			if (2 == dwc_vbus_status())
			{
				charge_on = 1;
				goto charge_out;
			}
		}
	}
#endif

	charge_out: return charge_on;
}

/* Update current situation.
 *
 * This function updates current situation off battery.
 * It updates struct bat.
 *
 * return 1 if charging else 0.
 */
int old_charge_level;
static int rk29_adc_battery_status_samples(struct rk29_adc_battery_data *bat)
{
	int charge_level;
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;

	/* Read supply input state */
	charge_level = rk29_adc_battery_get_charge_level(bat);

	/* If changed, control charger logic adequately */
	if (charge_level != old_charge_level)
	{
		old_charge_level = charge_level;
		bat->bat_change = 1;

		if (charge_level)
		{
			rk29_adc_battery_charge_enable(bat);
		}
		else
		{
			rk29_adc_battery_charge_disable(bat);
		}
		/* Debounce change detection */
		bat->bat_status_cnt = 0;
	}

	if (charge_level == 0)
	{
		/* Not charging */
		bat->full_times = 0;
		bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	/* If there is no charge-full hardware pin... */
	if (pdata->charge_ok_pin == INVALID_GPIO)
	{
		/* simulate charge full detection by capacity */
		if (bat->bat_capacity == 100)
		{
			if (bat->bat_status != POWER_SUPPLY_STATUS_FULL)
			{
				bat->bat_status = POWER_SUPPLY_STATUS_FULL;
				bat->bat_change = 1;
			}
		}
		else
		{
			bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		goto out;
	}

	/* Read GPIO stating fully-charged */
	if (gpio_get_value(pdata->charge_ok_pin) != pdata->charge_ok_level)
	{
		/* Hardware logic still charging */
		bat->full_times = 0;
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
		/* Hardware finished charging. */
		if (bat->full_times < NUM_CHARGE_FULL_DELAY_TIMES)
		{
			bat->full_times++;
		}

		if ((bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES)
				&& (bat->bat_status != POWER_SUPPLY_STATUS_FULL))
		{
			bat->bat_status = POWER_SUPPLY_STATUS_FULL;
			bat->bat_capacity = 100;
			bat->bat_change = 1;
		}
	}
	out: return charge_level;
}

/* ADC Voltage sample function.
 *
 */
int AdcTestvalue = 0;
static int *pSamples; /**< Pointer to current sample memory for ADC */
static void rk29_adc_battery_voltage_samples(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	int value;
	int i, *pStart = bat->adc_samples, num = 0;

	value = bat->adc_val;
	AdcTestvalue = value;
	adc_async_read(bat->client);

	/* Calculate mV from ADC value */
	bat->bat_voltageNow = ((value * pdata->adc_vref * (pdata->adc_rset_high + pdata->adc_rset_low)) / (ADC_RESOLUTION * pdata->adc_rset_low));

	/* Limit battery voltage */
	if (bat->bat_voltageNow >= pdata->adc_bat_levels[BATT_MAX_VOL_IDX] + 300)
		bat->bat_voltageNow = pdata->adc_bat_levels[BATT_MAX_VOL_IDX] + 300;

	/* Add value to averaging ring buffer */
	*pSamples++ = bat->bat_voltageNow;

	/* Slow down status changes by counting... */
	bat->bat_status_cnt++;
	if (bat->bat_status_cnt > NUM_VOLTAGE_SAMPLE)
		bat->bat_status_cnt = NUM_VOLTAGE_SAMPLE + 1;

	num = pSamples - pStart;
	if (num >= NUM_VOLTAGE_SAMPLE)
	{
		pSamples = pStart;
		num = NUM_VOLTAGE_SAMPLE;
	}

	value = 0;
	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++)
	{
		if (bat->adc_samples[i]) {
			/* this ensures that on startup only valid samples are
			 * taken for averaging.
			 */
			value += bat->adc_samples[i];
		}
	}
	bat->bat_voltageAvg = value / i;

	if (num == 1)
	{
		/* Preset min/max memories */
		bat->bat_voltageMax = bat->bat_voltageNow;
		bat->bat_voltageMin = bat->bat_voltageNow;
	}
	else
	{
		/* track min/max values... we could detect battery health from this. */
		if (bat->bat_voltageNow > bat->bat_voltageMax)
			bat->bat_voltageMax = bat->bat_voltageNow;
		else if (bat->bat_voltageNow < bat->bat_voltageMin)
			bat->bat_voltageMin = bat->bat_voltageNow;
	}

}

int capacitytmp = 0;

/* Get capacity from voltage value.
 * This function uses the adc_raw_table_bat[] table to retreive
 * the capacity in 0..100% from the current battery voltage mV value.
 *
 * \param *bat pointer to battery struct.
 * \param BatWoltage Battery voltage in mV.
 * \return capacity in 0..100%
 */
static int rk29_adc_battery_voltage_to_capacity(
		struct rk29_adc_battery_data *bat, int BatVoltage)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	int i = 0;
	int capacity = 0;
	int *p = pdata->adc_raw_table_bat;

	DBG("Bat=%u ", BatVoltage);

	/* discharge table is preset, check if we're charging */
	if (rk29_adc_battery_get_charge_level(bat))
	{
		DBG("AC ");
	p = pdata->adc_raw_table_ac;
	}

	if (BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
	{
		/* if more or equal than last table entry: 100% */
		DBG("F100%%\n");
		capacity = 100;
		goto out;
	}

	if (BatVoltage <= p[0])
	{
		/* if less than first table entry: 0% */
		DBG("F0%%\n");
		capacity = 0;
		goto out;
	}

	/* calculate average between to table entries for 1% steps. */
	for (i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
	{
		if ((p[i] <= BatVoltage) && (BatVoltage < p[i + 1]))
		{
			capacity = i * 10
					+ ((BatVoltage - p[i]) * 10) / (p[i + 1] - p[i]);
			break;
		}
	}
	DBG("i=%u %u %u %u\n", i, p[i], p[i+1], capacity);

out:
	return capacity;
}

static int gBatCapacityDisChargeCnt = 0;
static int gBatCapacityChargeCnt = 0;
//static int rk29_adc_battery_get_capacity_ext(int BatVoltage)

/* Count up capacity.
 * This function counts up capacity while charging proceeds.
 * It is to avoid jumping capacity values during different load conditions.
 */
static void rk29_adc_bat_capacity_charge(struct rk29_adc_battery_data *bat, int capacity)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	int sampleEnd = NUM_CHARGE_MID_SAMPLE;

	/* TODO: Astralix: Battery calibration can be automated by saving the value
	 * at the point where charge_ok_pin flips.
	 */

	gBatCapacityDisChargeCnt = 0;
	gBatCapacityChargeCnt++;

	if ((pdata->charge_ok_pin != INVALID_GPIO)
			&& (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level))
	{
		/* This should fade capacity to 100% faster */
		sampleEnd = NUM_CHARGE_MIN_SAMPLE;
	}

	if (gBatCapacityChargeCnt >= sampleEnd)
	{
		if (bat->bat_capacity < 99)
		{
			bat->bat_capacity++;
			bat->bat_change = 1;
		}
		gBatCapacityChargeCnt = 0;
	}
}

/* Count down capacity.
 * This function counts down capacity while discharging proceeds.
 * It is to avoid jumping capacity values during different load conditions.
 */
static void rk29_adc_bat_capacity_discharge(struct rk29_adc_battery_data *bat,
		int capacity)
{
	if (capacity < bat->bat_capacity)
	{
		/* We detected x cycles of measured capacity is lower */
		if (++gBatCapacityDisChargeCnt >= NUM_DISCHARGE_MIN_SAMPLE)
		{
			gBatCapacityDisChargeCnt = 0;
			if (bat->bat_capacity > 0)
			{
				bat->bat_capacity--;
				bat->bat_change = 1;
			}
		}
	}
	else
	{
		gBatCapacityDisChargeCnt = 0;
	}

	gBatCapacityChargeCnt = 0;
}

static void rk29_adc_battery_capacity_samples(struct rk29_adc_battery_data *bat)
{
	int capacity = 0;

	/* First collect minimum number of samples needed for
	 * calculation of mean value
	 */
	if (bat->bat_status_cnt < NUM_VOLTAGE_SAMPLE)
	{
		gBatCapacityDisChargeCnt = 0;
		gBatCapacityChargeCnt = 0;
		return;
	}

	/* calculate capacity currently available in battery. */
	capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltageAvg);

	if (rk29_adc_battery_get_charge_level(bat))
	{
		/* This is executed if the battery is charging... */
		rk29_adc_bat_capacity_charge(bat, capacity);
	}
	else
	{
		/* Battery is discharging */
		rk29_adc_bat_capacity_discharge(bat, capacity);
	}

	/* save current capacity for comparisions in next run */
	capacitytmp = capacity;
}

/* Battery check at power-on.
 * This check is done only once at kernel bootup.
 */
static int poweron_check = 0; /**< for exec p-on functions only once */
static void rk29_adc_battery_poweron_capacity_check(void)
{
	int new_capacity, old_capacity;

	new_capacity = gBatteryData->bat_capacity;

	/* read previous capacity from filesystem */
	old_capacity = rk29_adc_battery_load_capacity();
	if ((old_capacity <= 0) || (old_capacity >= 100))
	{
		/* if old capacity is invalid, ignore it. */
		old_capacity = new_capacity;
	}

	if (gBatteryData->bat_status == POWER_SUPPLY_STATUS_FULL)
	{
		if (new_capacity > 80)
		{
			gBatteryData->bat_capacity = 100;
		}
	}
	else if (gBatteryData->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING)
	{
		//chargeing state
		//���⣺
		//1����ʱ��ػ���ú󣬿�����ȡ������ԶԶ����ʵ��������ô�죿
		//2���������������ʱ��ػ��ٿ���ǰ��������һ���ָ���ô�죿
		//3��һ�����ַ�ʽ���ʣ�
		//gBatteryData->bat_capacity = new_capacity;
		gBatteryData->bat_capacity =
				(new_capacity > old_capacity) ? new_capacity : old_capacity;
	}
	else
	{
		gBatteryData->bat_capacity =
				(new_capacity < old_capacity) ? new_capacity : old_capacity;
	}

	printk(
			"BAT: status = %d, capacity = %d, new_capacity = %d, old_capacity = %d\n",
			gBatteryData->bat_status, gBatteryData->bat_capacity, new_capacity,
			old_capacity);

	gBatteryData->bat_change = 1;
}

unsigned long AdcTestCnt = 0;
static void rk29_adc_battery_timer_work(struct work_struct *work)
{
	rk29_adc_battery_status_samples(gBatteryData);

	if (poweron_check)
	{
		/* First time capacity check after power-on. */
		poweron_check = 0;
		rk29_adc_battery_poweron_capacity_check();
	}

	/* Gather actual voltage value */
	rk29_adc_battery_voltage_samples(gBatteryData);
	/* Gather actual capacity value */
	rk29_adc_battery_capacity_samples(gBatteryData);

	/* Update battery parameter after ADC and capacity has been changed. */
	if (gBatteryData->bat_change)
	{
		gBatteryData->bat_change = 0;
		rk29_adc_battery_put_capacity(gBatteryData->bat_capacity);
		power_supply_changed(&rk29_battery_supply);
	}

	if (rk29_battery_dbg_level)
	{
		if (++AdcTestCnt >= 20)
		{
			AdcTestCnt = 0;
			printk(
					"Status = %d, RealAdcVal = %d, BatNow = %dmV, BatAvg = %dV, CalCap = %d, RealCapacity = %d, dischargecnt = %d, chargecnt = %d\n",
					gBatteryData->bat_status,
					AdcTestvalue,
					gBatteryData->bat_voltageNow, gBatteryData->bat_voltageAvg,
					capacitytmp, gBatteryData->bat_capacity,
					gBatCapacityDisChargeCnt, gBatCapacityChargeCnt);
		}
	}

}

/** Setup work timer for sampling of battery voltage.
 *
 * This function sets up the worker timer to scan the battery
 * ADC every TIMER_MS_COUNTS.
 */
static void rk29_adc_battery_scan_timer(unsigned long data)
{
	/* Schedule battery work queue every TIMER_MS_COUNTS */
	gBatteryData->timer.expires = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);

	schedule_work(&gBatteryData->timer_work);
}

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
static int rk29_adc_battery_get_usb_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	charger_type_t charger;
	charger = CHARGER_USB;

	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
		val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
		break;

		default:
		return -EINVAL;
	}

	return 0;

}

static enum power_supply_property rk29_adc_battery_usb_props[] =
{

	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk29_usb_supply =
{
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,

	.get_property = rk29_adc_battery_get_usb_property,

	.properties = rk29_adc_battery_usb_props,
	.num_properties = ARRAY_SIZE(rk29_adc_battery_usb_props),
};
#endif

#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
static irqreturn_t rk29_adc_battery_dc_wakeup(int irq, void *dev_id)
{
	schedule_work(&gBatteryData->dcwakeup_work);
	return IRQ_HANDLED;
}

static int rk29_adc_battery_get_ac_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger = CHARGER_USB;
	switch (psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			if (rk29_adc_battery_get_charge_level(gBatteryData))
			{
				val->intval = 1;
			}
			else
			{
				val->intval = 0;
			}
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;

		default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property rk29_adc_battery_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk29_ac_supply =
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,

	.get_property = rk29_adc_battery_get_ac_property,

	.properties = rk29_adc_battery_ac_props,
	.num_properties = ARRAY_SIZE(rk29_adc_battery_ac_props),
};

static void rk29_adc_battery_dcdet_delaywork(struct work_struct *work)
{
	int ret;
	struct rk29_adc_battery_platform_data *pdata = gBatteryData->pdata;
	int irq = gpio_to_irq(pdata->dc_det_pin);
	int irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	rk28_send_wakeup_key();

	free_irq(irq, NULL);
	ret = request_irq(irq, rk29_adc_battery_dc_wakeup, irq_flag, "rk29_adc_battery", NULL);
	if (ret)
	{
		free_irq(irq, NULL);
	}

	power_supply_changed(&rk29_ac_supply);

	gBatteryData->bat_status_cnt = 0; //״̬�仯��ʼ����

	wake_lock_timeout(&batt_wake_lock, 30 * HZ);

}

#endif

static int rk29_adc_battery_get_status(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_status);
}

static int rk29_adc_battery_get_health(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	return (bat->bat_voltageNow < pdata->adc_bat_levels[BATT_ZERO_VOL_IDX]) ? POWER_SUPPLY_HEALTH_DEAD : POWER_SUPPLY_HEALTH_GOOD;
}

static int rk29_adc_battery_get_present(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	return (bat->bat_voltageNow > pdata->adc_bat_levels[BATT_ZERO_VOL_IDX]) ? 1 : 0;
}

static int rk29_adc_battery_get_voltage_now(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltageNow);
}

static int rk29_adc_battery_get_voltage_avg(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltageAvg);
}

static int rk29_adc_battery_get_voltage_min(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltageMin);
}

static int rk29_adc_battery_get_voltage_max(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltageMax);
}

static int rk29_adc_battery_get_capacity(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_capacity);
}

static int rk29_adc_battery_get_level(struct rk29_adc_battery_data *bat)
{
	int retval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	if ( bat->bat_capacity < 1) {
		retval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	}
	else if (bat->bat_capacity < 10) {
		retval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	}
	else if (bat->bat_capacity < 90) {
		retval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
	else if (bat->bat_capacity < 95) {
		retval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	}
	else {
		retval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	}
	return retval;
}

static int rk29_adc_battery_get_max_design(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	return pdata->adc_bat_levels[BATT_MAX_VOL_IDX];
}

static int rk29_adc_battery_get_min_design(struct rk29_adc_battery_data *bat)
{
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	return pdata->adc_bat_levels[BATT_ZERO_VOL_IDX];
}

static int rk29_adc_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;

	switch (psp)
	{
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rk29_adc_battery_get_status(gBatteryData);
		DBG("gBatStatus=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = rk29_adc_battery_get_health(gBatteryData);
		DBG("gBatHealth=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = rk29_adc_battery_get_present(gBatteryData);
		DBG("gBatPresent=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = rk29_adc_battery_get_voltage_now(gBatteryData);
		DBG("gBatVoltageNow=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = rk29_adc_battery_get_voltage_avg(gBatteryData);
		DBG("gBatVoltageAvg=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = rk29_adc_battery_get_voltage_max(gBatteryData);
		DBG("gBatVoltageMax=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = rk29_adc_battery_get_voltage_min(gBatteryData);
		DBG("gBatVoltageMin=%d\n",val->intval);
		break;
//	case POWER_SUPPLY_PROP_CURRENT_NOW:
//		val->intval = 1100;
//		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = rk29_adc_battery_get_capacity(gBatteryData);
		DBG("gBatCapacity=%d%%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = rk29_adc_battery_get_level(gBatteryData);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = rk29_adc_battery_get_max_design(gBatteryData);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = rk29_adc_battery_get_min_design(gBatteryData);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 22;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk29_adc_battery_props[] =
{

		POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_PROP_PRESENT,
		POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_VOLTAGE_AVG,
		POWER_SUPPLY_PROP_VOLTAGE_MIN, POWER_SUPPLY_PROP_VOLTAGE_MAX,
		POWER_SUPPLY_PROP_TECHNOLOGY,
		POWER_SUPPLY_PROP_CAPACITY, POWER_SUPPLY_PROP_CAPACITY_LEVEL,
		POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
		POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN, };

static struct power_supply rk29_battery_supply =
{ .name = "battery", .type = POWER_SUPPLY_TYPE_BATTERY,

.get_property = rk29_adc_battery_get_property,

.properties = rk29_adc_battery_props, .num_properties = ARRAY_SIZE(
		rk29_adc_battery_props), };

#ifdef CONFIG_PM
int suspend_capacity = 0;
static void rk29_adc_battery_resume_check(struct work_struct *work)
{
	int i;
	int level,oldlevel;
	int new_capacity, old_capacity;
	struct rk29_adc_battery_data *bat = gBatteryData;

	old_charge_level = -1;
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client); //start adc sample
	level = oldlevel = rk29_adc_battery_status_samples(bat);//init charge status

	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++)//0.3 s
	{
		mdelay(1);
		rk29_adc_battery_voltage_samples(bat); //get voltage
		level = rk29_adc_battery_status_samples(bat);//check charge status
		if (oldlevel != level)
		{
			oldlevel = level; //if charge status changed, reset sample
			i = 0;
		}
	}
	new_capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltageAvg);
	old_capacity = suspend_capacity;

#if 0
	if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING)
	{
		//chargeing state
		bat->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}
	else
	{
		bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;
	}
#else
	bat->bat_capacity = new_capacity;
#endif
	printk("rk29_adc_battery_resume: status = %d, voltage = %d, capacity = %d, new_capacity = %d, old_capacity = %d\n",
			bat->bat_status, bat->bat_voltageNow, bat->bat_capacity, new_capacity, old_capacity);

	//start timer scan
	schedule_work(&bat->timer_work);
	bat->timer.expires = jiffies + 10;
	add_timer(&bat->timer);
}

static int rk29_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	suspend_capacity = gBatteryData->bat_capacity;
	del_timer(&gBatteryData->timer);
	//flush_scheduled_work();
	return 0;
}

static int rk29_adc_battery_resume(struct platform_device *dev)
{
	/* things may have changed while we were away */
	schedule_work(&gBatteryData->resume_work);
	return 0;
}
#else
#define rk29_adc_battery_suspend NULL
#define rk29_adc_battery_resume NULL
#endif

static int rk29_adc_battery_io_init(struct rk29_adc_battery_data *data,
		struct rk29_adc_battery_platform_data *pdata)
{
	int ret = 0;

	data->pdata = pdata;

	if (pdata->io_init)
	{
		pdata->io_init();
	}

	//charge control pin
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
		ret = gpio_request(pdata->charge_set_pin, NULL);
		if (ret)
		{
			printk("failed to request dc_det gpio\n");
			goto error;
		}
		gpio_direction_output(pdata->charge_set_pin,
				1 - pdata->charge_set_level);
	}

	//dc charge detect pin
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
		ret = gpio_request(pdata->dc_det_pin, NULL);
		if (ret)
		{
			printk("failed to request dc_det gpio\n");
			goto error;
		}

		gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp); //important
		ret = gpio_direction_input(pdata->dc_det_pin);
		if (ret)
		{
			printk("failed to set gpio dc_det input\n");
			goto error;
		}
	}

	//charge ok detect
	if (pdata->charge_ok_pin != INVALID_GPIO)
	{
		ret = gpio_request(pdata->charge_ok_pin, NULL);
		if (ret)
		{
			printk("failed to request charge_ok gpio\n");
			goto error;
		}

		gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp); //important
		ret = gpio_direction_input(pdata->charge_ok_pin);
		if (ret)
		{
			printk("failed to set gpio charge_ok input\n");
			goto error;
		}
	}

	return 0;
	error: return -1;
}

static void rk29_adc_battery_lowpower_check(struct rk29_adc_battery_data *bat)
{
	int i;
	int level, oldlevel;
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;

	printk("%s--%d:\n", __FUNCTION__, __LINE__);

	old_charge_level = -1;
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client); //start adc sample
	level = oldlevel = rk29_adc_battery_status_samples(bat); //init charge status

	/* Fill sample buffer with values */
	bat->full_times = 0;
	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++) //0.3 s
	{
		mdelay(1);
		rk29_adc_battery_voltage_samples(bat); //get voltage
		//level = rk29_adc_battery_status_samples(bat);       //check charge status
		level = rk29_adc_battery_get_charge_level(bat);
		if (oldlevel != level)
		{
			oldlevel = level; //if charge status changed, reset sample
			i = 0;
		}
	}

	bat->bat_capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltageAvg);

	bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (rk29_adc_battery_get_charge_level(bat))
	{
		bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
		if (pdata->charge_ok_pin != INVALID_GPIO)
		{
			if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level)
			{
				bat->bat_status = POWER_SUPPLY_STATUS_FULL;
				bat->bat_capacity = 100;
			}
		}
	}

#if 0
	rk29_adc_battery_poweron_capacity_check();
#else
	poweron_check = 1;
#endif

	/* Immediate power off for battery protection */
	if (bat->bat_voltageAvg <= (pdata->adc_bat_levels[BATT_ZERO_VOL_IDX] + 50))
	{
		printk("%umV -> low battery: powerdown (%u)\n", bat->bat_voltageAvg, pdata->adc_bat_levels[BATT_ZERO_VOL_IDX]+50);
		system_state = SYSTEM_POWER_OFF;
		pm_power_off();
	}
}

static void rk29_adc_battery_callback(struct adc_client *client, void *param,
		int result)
{
	gBatteryData->adc_val = result;
	return;
}

static int rk29_adc_battery_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	int irq_flag;
	struct adc_client *client;
	struct rk29_adc_battery_data *data;
	struct rk29_adc_battery_platform_data *pdata = pdev->dev.platform_data;

	printk("%s--%d:\n", __FUNCTION__, __LINE__);
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
	{
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gBatteryData = data;
	platform_set_drvdata(pdev, data);

	ret = rk29_adc_battery_io_init(data, pdata);
	if (ret)
	{
		goto err_io_init;
	}

	/* Register ADC memory for battery sampling and mean value calc. */
	memset(data->adc_samples, 0, sizeof(int) * (NUM_VOLTAGE_SAMPLE + 2));
	client = adc_register(0, rk29_adc_battery_callback, NULL);
	if (!client)
		goto err_adc_register_failed;

	//variable init
	data->client = client;
	data->adc_val = adc_sync_read(client);

	/* Setup timers for ADC and battery work queues. */
	setup_timer(&data->timer, rk29_adc_battery_scan_timer, (unsigned long)data);
	data->timer.expires = jiffies + 2000; /* First start delayed by 2s to finish init. */
	add_timer(&data->timer);

	INIT_WORK(&data->timer_work, rk29_adc_battery_timer_work);
	INIT_WORK(&data->resume_work, rk29_adc_battery_resume_check);

#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	//init dc dectet irq & delay work
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
		irq = gpio_to_irq(pdata->dc_det_pin);

		irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
		ret = request_irq(irq, rk29_adc_battery_dc_wakeup, irq_flag, "rk29_adc_battery", NULL);
		if (ret)
		{
			printk("failed to request dc det irq\n");
			goto err_dcirq_failed;
		}
		enable_irq_wake(irq);

		INIT_WORK(&data->dcwakeup_work, rk29_adc_battery_dcdet_delaywork);
	}
#endif

	/* Power on Battery detection. */
	rk29_adc_battery_lowpower_check(data);

	/* Power supply register. */
	wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");

	ret = power_supply_register(&pdev->dev, &rk29_battery_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}

#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk29_ac_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
#endif

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk29_usb_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif

	printk(KERN_INFO "rk29_adc_battery: driver initialized\n");

	return 0;

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	err_usb_failed:
	power_supply_unregister(&rk29_usb_supply);
#endif

	err_ac_failed:
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	power_supply_unregister(&rk29_ac_supply);
#endif

	err_battery_failed: power_supply_unregister(&rk29_battery_supply);

	err_dcirq_failed: free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	err_adc_register_failed: err_io_init: err_data_alloc_failed: kfree(data);

	printk("rk29_adc_battery: error!\n");

	return ret;
}

static int rk29_adc_battery_remove(struct platform_device *pdev)
{
	struct rk29_adc_battery_data *data = platform_get_drvdata(pdev);
	struct rk29_adc_battery_platform_data *pdata = pdev->dev.platform_data;

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	power_supply_unregister(&rk29_usb_supply);
#endif
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	power_supply_unregister(&rk29_ac_supply);
#endif
	power_supply_unregister(&rk29_battery_supply);

	free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	kfree(data);

	return 0;
}

static struct platform_driver rk29_adc_battery_driver =
{ .probe = rk29_adc_battery_probe, .remove = rk29_adc_battery_remove, .suspend =
		rk29_adc_battery_suspend, .resume = rk29_adc_battery_resume, .driver =
{ .name = "rk2918-battery", .owner = THIS_MODULE, } };

static int __init rk29_adc_battery_init(void)
{
	return platform_driver_register(&rk29_adc_battery_driver);
}

static void __exit rk29_adc_battery_exit(void)
{
	platform_driver_unregister(&rk29_adc_battery_driver);
}

subsys_initcall(rk29_adc_battery_init);
module_exit(rk29_adc_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");
