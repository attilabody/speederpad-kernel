/*
 * File:         al3010.c
 * Based on:
 * Author:       Leo Tsai <leohs.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Modification History:
 * Date(DD/MM/YY)     By       Summary
 * -------- -------- -------------------------------------------------------
 * 14/12/20  Leo        first release
 *
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include "al3010.h"

#include <misc/noah_eeprom_com.h>  //add by yxp  20210113 


#define ALS_AUTO_GAIN					1
#define ALS_AUTO_GAIN_MAX_DATA		65535
#define ALS_AUTO_GAIN_HTHRES			((3*ALS_AUTO_GAIN_MAX_DATA)/4)
#define ALS_AUTO_GAIN_LTHRES			((2*ALS_AUTO_GAIN_MAX_DATA)/16)

#define AL3010_DBG
#ifdef AL3010_DBG
#define LDBG(fmt, args...)    printk("%s [%d] : " fmt"\n", __func__, __LINE__, ##args)
#else
#define LDBG(format, ...)
#endif

/**********by add yanjb20210113***************/
static const char* hwinfo = "AL3010";
static int light_value = 0;
static int g_Parameter_Type = 0;
static int g_Parameter = 300;
static int g_Parameter_2 = 100;
static int g_Parameter_Correct = 0;

int alsps_debug = 0;

#ifdef CONFIG_YOUXUEPAI_BOARD
#define AL3010_AUTO_BL
#ifdef AL3010_AUTO_BL
#define al3010_REPORT_NUM 5
#endif
#endif
extern int noah_eeprom_setdata(int type,char * buf);
/********************************************/

static atomic_t l_flag;
static struct i2c_client *this_client;

struct al3010_data {
	struct input_dev *alssensor_input_dev;
	struct i2c_client *client;
	struct workqueue_struct *al3010_wq;
	struct delayed_work al3010_als_dwork;             /* for ALS polling */
	int irq_gpio_number;
	int als_en;
	int irq_en;
	int als_gain;
	int lux;
	bool als_auto_gain_trig;
	u64 als_auto_gain_trig_time;
	u64 als_auto_gain_msdelay;
	unsigned long als_poll_delay;	              /* needed for light sensor polling :
					                                                            * micro-second (us) */
};


static int al3010_write_data(unsigned char addr, unsigned char data);
static int al3010_read_data(unsigned char addr, unsigned char *data);
static int al3010_sw_reset(void);
static int al3010_get_wait_time(struct al3010_data *data);



static struct al3010_data *private_pl_data = NULL;

static int al3010_range[] = {77806, 19452, 4863, 1216};

enum al3010_regs {
	SYS_CONFIG = 0x00,
	INTERRUPT_STATUS = 0x01,
	ALS_DATA_LOW = 0x0c,
	ALS_DATA_HIGH = 0x0d,
	ALS_CONFIG = 0x10,
	ALS_LOW_THRES_L = 0x1a,
	ALS_LOW_THRES_H = 0x1b,
	ALS_HIGH_THRES_L = 0x1c,
	ALS_HIGH_THRES_H = 0x1d,
};

static u8 al3010_reg_array[][3] = {
	{SYS_CONFIG, 0x04, 1}, // SW reset
	{INTERRUPT_STATUS, 0x00, 0},
	{ALS_DATA_LOW, 0x00, 0},
	{ALS_DATA_HIGH, 0x00, 0},
	{ALS_CONFIG, 0x00, 0},
	{ALS_LOW_THRES_L, 0x00, 0},
	{ALS_LOW_THRES_H, 0x00, 0},
	{ALS_HIGH_THRES_L, 0x00, 0},
	{ALS_HIGH_THRES_H, 0x00, 0},
};

int cali = 100;

struct wakeup_source al3010_wake_source;

static int al3010_read_reg(u32 reg, uint8_t mask, uint8_t shift)
{
	unsigned char regvalue;
	al3010_read_data(reg, &regvalue);

	return (regvalue & mask) >> shift;
}

static int al3010_write_reg(u32 reg, uint8_t mask, uint8_t shift,
    uint8_t val)
{
	int ret = 0;
	u8 tmp;

	al3010_read_data(reg, &tmp);
	tmp &= ~mask;
	tmp |= val << shift;
	ret = al3010_write_data(reg, tmp);

	return ret;
}


static int al3010_get_mode(void)
{
	int ret;

	ret = al3010_read_reg(SYS_CONFIG, AL3010_REG_ALS_MODE_MASK,
	        AL3010_REG_ALS_MODE_SHIFT);

	return ret;
}

static int al3010_set_mode(int mode)
{
	struct al3010_data *data = i2c_get_clientdata(this_client);
	int ret = 0;
	int als_en = 0;

	ret = al3010_write_reg(SYS_CONFIG,
	        AL3010_REG_ALS_MODE_MASK, AL3010_REG_ALS_MODE_SHIFT, mode);

	msleep(10);

	atomic_set(&l_flag, mode & 0x01);

	als_en = atomic_read(&l_flag);

	if (als_en) {
		data->als_poll_delay = al3010_get_wait_time(data);
		data->als_auto_gain_msdelay = (u64)data->als_poll_delay * 2;

		queue_delayed_work(data->al3010_wq,
		    &data->al3010_als_dwork,
		    msecs_to_jiffies(data->als_poll_delay));
	} else {
		data->lux = 0;
		data->als_auto_gain_trig_time = 0;
		data->als_auto_gain_trig = false;
	}

	return ret;
}

static int al3010_get_wait_time(struct al3010_data *data)
{
	int val = 0;
	uint16_t conv_t[4] = {50, 200, 400, 800};

	val = al3010_read_reg(ALS_CONFIG, AL3010_REG_ALS_INTERRUPT_FILTER_MASK,
	        AL3010_REG_ALS_INTERRUPT_FILTER_SHIFT);

	LDBG("wait time = %d ms", conv_t[val]);

	return conv_t[val];
}

static uint16_t al3010_get_als_adc(struct al3010_data *data)
{
	s32 alsraw = 0;

	alsraw = i2c_smbus_read_word_data(data->client, ALS_DATA_LOW);

	if (alsraw < 0) {
		dev_err(&data->client->dev, "ch0 raw read err: %d\n", alsraw);
	}
    if(alsps_debug)
	LDBG("als raw=%d", (uint16_t)alsraw);

	return alsraw;
}

static int al3010_get_als_range(void)
{
	struct al3010_data *data = i2c_get_clientdata(this_client);

	u8 idx = al3010_read_reg(ALS_CONFIG,
	        AL3010_REG_ALS_GAIN_MASK, AL3010_REG_ALS_GAIN_SHIFT);

	data->als_gain = idx;
    if(alsps_debug)
	   LDBG("als range = %d", al3010_range[idx]);

	return al3010_range[idx];
}

static int al3010_set_als_range(int range)
{
	struct al3010_data *data = i2c_get_clientdata(this_client);

	range &= 0x07;
	data->als_gain = range;

	LDBG("als_gain = %d", data->als_gain);

	return al3010_write_reg(ALS_CONFIG, AL3010_REG_ALS_GAIN_MASK,
	        AL3010_REG_ALS_GAIN_SHIFT, (uint8_t)data->als_gain);
}

#if ALS_AUTO_GAIN
static bool al3010_als_auto_gain(struct al3010_data *data, uint16_t alsadc)
{
	int ret = 0;

	if ((alsadc >= ALS_AUTO_GAIN_HTHRES) && (data->als_gain > 0x00)) {
		data->als_auto_gain_trig_time = ktime_get_ns();
		LDBG("als raw = %d, ALS_AUTO_GAIN_HTHRES: %d", alsadc, ALS_AUTO_GAIN_HTHRES);
		data->als_gain = data->als_gain - 0x01;
		LDBG("next gain: 0x%02x", data->als_gain);
		ret = al3010_set_als_range(data->als_gain);

		if (ret < 0) {
			dev_err(&data->client->dev, "write ALS gain err: %d\n", ret);
		}
	} else if ((alsadc <= ALS_AUTO_GAIN_LTHRES) && (data->als_gain < 0x03)) {
		data->als_auto_gain_trig_time = ktime_get_ns();
		LDBG("als raw = %d, ALS_AUTO_GAIN_LTHRES: %d", alsadc, ALS_AUTO_GAIN_LTHRES);
		data->als_gain = data->als_gain + 0x01;
		LDBG("next gain: 0x%02x", data->als_gain);
		ret = al3010_set_als_range(data->als_gain);

		if (ret < 0) {
			dev_err(&data->client->dev, "write ALS gain err: %d\n", ret);
		}
	} else {
		data->als_auto_gain_trig_time = 0;
		return false;
	}

	return true;
}
#endif

/*******************************************************************************
* Function    :  al3010_get_lux_value
* Description :  get the lux value
* Parameters  :  void
* Return      :    value
*******************************************************************************/
static int al3010_get_lux_value(uint16_t alsraw)
{
	u64 range;
	u64 tmp;

	range = (u64)al3010_get_als_range();

	tmp = (alsraw * range) / 65535;

	tmp *= (u64)cali;
    if(alsps_debug)
	   LDBG("lux tmp: %llu, cali: %d", tmp, cali);

	return (int)(tmp / 100);
}

static u8 al3010_get_intstat(struct al3010_data *data)
{
	//struct al3010_data *data = i2c_get_clientdata(this_client);

	u8 val = al3010_read_reg(INTERRUPT_STATUS, 0x01, 0);

	LDBG("Interrupt Status = 0x%02x", val);

	return val;
}

static int al3010_set_althres(struct al3010_data *data, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & 0xFF;

	err = al3010_write_reg(ALS_LOW_THRES_L, 0xFF, 0, lsb);

	err = al3010_write_reg(ALS_LOW_THRES_H, 0xFF, 0, msb);

	return err;
}

static int al3010_set_ahthres(struct al3010_data *data, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & 0xFF;

	err = al3010_write_reg(ALS_HIGH_THRES_L, 0xFF, 0, lsb);

	err = al3010_write_reg(ALS_HIGH_THRES_H, 0xFF, 0, msb);

	return err;
}

//---------------------------------------------------------------------------------------
static ssize_t al3010_show_mode(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val = 0;

	val = al3010_get_mode() & 0x01;

	sprintf(buf, "0x%02x\n", val);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_store_mode(struct device *cd,
    struct device_attribute *attr,
    const char *buf, size_t len)
{
	unsigned long mode = simple_strtoul(buf, NULL, 10);

	LDBG("mode: %d", (int)mode);

	mode = mode & 0x01;

	al3010_set_mode((int)mode);

	return len;
}

static ssize_t al3010_show_int_st(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	ssize_t ret = 0;
	u8 val = 0;

	val = al3010_get_intstat(data);

	sprintf(buf, "0x%02x\n", val);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_show_als_raw(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	ssize_t ret = 0;
	u16 alsraw = 0;

	alsraw = al3010_get_als_adc(data);

	sprintf(buf, "als raw: %d\n", alsraw);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_show_lux(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	ssize_t ret = 0;
	u16 alsraw = 0, val;

	alsraw = al3010_get_als_adc(data);
	val = al3010_get_lux_value(alsraw);

	sprintf(buf, "%d\n", val);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_show_als_low_thr(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	ssize_t ret = 0;
	int val = 0;

	val = i2c_smbus_read_word_data(data->client, ALS_LOW_THRES_L);

	sprintf(buf, "%d\n", val);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_store_als_low_thr(struct device *cd,
    struct device_attribute *attr,
    const char *buf, size_t len)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	val = val & 0xffff;

	al3010_set_althres(data, val);

	return len;
}

static ssize_t al3010_show_als_high_thr(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	ssize_t ret = 0;
	int val = 0;

	val = i2c_smbus_read_word_data(data->client, ALS_HIGH_THRES_L);

	sprintf(buf, "%d\n", val);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t al3010_store_als_high_thr(struct device *cd,
    struct device_attribute *attr,
    const char *buf, size_t len)
{
	struct al3010_data *data =  dev_get_drvdata(cd);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	val = val & 0xffff;

	al3010_set_ahthres(data, val);

	return len;
}

static ssize_t al3010_show_em(struct device *cd,
    struct device_attribute *attr, char *buf)
{
	int i;
	unsigned char tmp;
	int tmp_index = 0;
	int reg_num = sizeof(al3010_reg_array) / sizeof(al3010_reg_array[0]);

	LDBG("DEBUG al3010_em_read..");

	for (i = 0; i < reg_num; i++) {
		al3010_read_data(al3010_reg_array[i][0], &tmp);
		tmp_index += sprintf(buf + tmp_index, "Reg[0x%02x] Val[0x%02x]\n",
		        al3010_reg_array[i][0], tmp);
		LDBG("Reg[0x%02x] Val[0x%02x]", al3010_reg_array[i][0], tmp);
	}

	return tmp_index;
}

static ssize_t al3010_store_em(struct device *cd,
    struct device_attribute *attr,
    const char *buf, size_t len)
{
	u32 addr, val;
	int ret = 0;

	sscanf(buf, "%x%x", &addr, &val);

	ret = al3010_write_reg(addr, 0xFF, 0, val);

	return len;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, al3010_show_mode,
    al3010_store_mode);
static DEVICE_ATTR(int_st, S_IRUGO, al3010_show_int_st, NULL);
static DEVICE_ATTR(als_raw, S_IRUGO, al3010_show_als_raw, NULL);
static DEVICE_ATTR(lux, S_IRUGO, al3010_show_lux, NULL);
static DEVICE_ATTR(als_low_thr, S_IRUGO | S_IWUSR, al3010_show_als_low_thr,
    al3010_store_als_low_thr);
static DEVICE_ATTR(als_high_thr, S_IRUGO | S_IWUSR, al3010_show_als_high_thr,
    al3010_store_als_high_thr);
static DEVICE_ATTR(em, S_IRUGO | S_IWUSR, al3010_show_em,
    al3010_store_em);

static struct attribute *al3010_attributes[] = {
	&dev_attr_mode.attr,
	&dev_attr_int_st.attr,
	&dev_attr_als_raw.attr,
	&dev_attr_lux.attr,
	&dev_attr_als_low_thr.attr,
	&dev_attr_als_high_thr.attr,
	&dev_attr_em.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
	.attrs = al3010_attributes,
};

static int al3010_create_sysfs(struct i2c_client *client)
{
	int err;

	LDBG("start");

	err = sysfs_create_group(&client->dev.kobj, &al3010_attr_group);

	return err;
}

/*******************************************************************************
* Function    :  al3010_write_data
* Description :  write data to IC
* Parameters  :  addr: register address, data: register data
* Return      :  none
*******************************************************************************/
static int al3010_write_data(unsigned char addr, unsigned char data)
{
	return i2c_smbus_write_byte_data(this_client, addr, data);
}

/*******************************************************************************
* Function    :  al3010_read_data
* Description :  read data from IC
* Parameters  :  addr: register address, data: read data
* Return      :    status
*******************************************************************************/
static int al3010_read_data(unsigned char addr, unsigned char *data)
{
	int ret = 0;

	*data = i2c_smbus_read_byte_data(this_client, addr);

	return ret;;
}

/*******************************************************************************
* Function    :  al3010_reg_init
* Description :  set al3010 registers
* Parameters  :  none
* Return      :  void
*******************************************************************************/
static int al3010_reg_init(struct al3010_data *data)
{
	int ret = 0;

	atomic_set(&l_flag, 0);

	data->als_auto_gain_trig = 0;
	data->als_auto_gain_trig_time = 0;

	return ret;
}
#ifdef CONFIG_YOUXUEPAI_BOARD
#ifdef AL3010_AUTO_BL
//yanjb 20210403 add automatic brightness adjustment
extern int backlight_bright_temp_yxp;
extern int bl_state;
int bl_state_flag = 0;
extern int disp_lcd_get_brightness_yxp(int bright_temp_yxp);
extern int al3010_bl_flag_yxp;
extern int al3010_bl_level;
static u16 average_value[al3010_REPORT_NUM]={0};
static int i=0;
static int j=0;
static int value_min=0;
static int value_max=0;
static int value_num=0;
static int value_virtual=0;
static int temp;
static int value_virtual_temp=0;
static int report_value=0;
static int x,y;
static int value_yxp=0;
//static int al3010_bl_level=0;
static int count=0;
static int average_value_frist=0;


static int al3010_set_automatic_brightness(int light_value)
{  
	if (alsps_debug)
	LDBG("==yy==light_value=%d	 al3010_bl_flag_yxp=%d	count=%d\n",light_value,al3010_bl_flag_yxp,count);

    count++;
	if(count<20)	
		return 0;
	   count=0;
			
	for(i=5;i>0;i--){
    average_value[i] = average_value[i-1];   
	}
    
	average_value[0] = light_value;
	if(bl_state_flag)
	average_value_frist = light_value;
	if (alsps_debug)
	printk("==yy== a=%d b=%d c=%d d=%d e=%d backlight_bright_temp_yxp=%d\n",average_value[0],average_value[1],average_value[2],average_value[3],average_value[4],backlight_bright_temp_yxp);	

    	value_max = average_value[0];
    	value_min = average_value[0];
    	report_value = 0;
    	for(i=0;i<4;i++){
    		if(value_max < average_value[i])
    			value_max = average_value[i];
    		if(value_min > average_value[i])
    			value_min = average_value[i];
    		report_value += average_value[i];
    	}
        //printk("==yy== report_value=%d value_max=%d value_min=%d\n",report_value,value_max,value_min);
    	value_virtual = (report_value - value_max - value_min)/2;  //去掉最大最小	
    	    
    	if (value_virtual_temp > value_virtual)
    	   value_num = value_virtual_temp - value_virtual;
    	else
    	   value_num = value_virtual - value_virtual_temp;
     	if (alsps_debug)   	
           printk("==yy==value_virtual=%d  value_virtual_temp=%d value_num=%d average_value_frist=%d light_value=%d al3010_bl_level=%d \n ",value_virtual,value_virtual_temp,value_num,average_value_frist,light_value,al3010_bl_level);
   if ((value_num >= 10)  || (average_value_frist == light_value)/* || first*/){            //
        bl_state_flag = 0;
		average_value_frist = 0;
		value_virtual_temp = value_virtual;
//光感亮度差异大于10执行自动调节亮度
        if(value_virtual_temp < 10){//当光感值在0-10之间，背光亮度设置为backlight_bright_temp_yxp = 3
        	 //printk("==yy==LLLLLLL\n");
    		 if(al3010_bl_level != 1){
        		 value_yxp = 3;
            	 al3010_bl_flag_yxp = 1;
        		 al3010_bl_level = 1;
    			}
        }else if((10 < value_virtual_temp) && (value_virtual_temp < 200)){//当光感值在0-200之间，背光亮度设置为backlight_bright_temp_yxp = 85
        	 //printk("==yy==BBBBBBB\n");
    		 if(al3010_bl_level != 2){
            	 value_yxp = 85;
            	 al3010_bl_flag_yxp = 1;
        		 al3010_bl_level = 2;
    		 	}
        }else if((200 <= value_virtual_temp) && (value_virtual_temp < 400)){//当光感值在200-400之间，背光亮度设置为backlight_bright_temp_yxp = 135
        	 //printk("==yy==DDDDD\n");
    		 if(al3010_bl_level != 3){
             	 value_yxp = 135;
             	 al3010_bl_flag_yxp = 1;
         		 al3010_bl_level = 3;
    		 	}
        }else if((400 <= value_virtual_temp) && (value_virtual_temp < 1000)){//当光感值在400-1000之间，背光亮度设置为backlight_bright_temp_yxp = 185
        	 //printk("==yy==FFFFF\n");
    		 if(al3010_bl_level != 4){
            	 value_yxp = 185;
    			 al3010_bl_flag_yxp = 1;
        		 al3010_bl_level = 4;
    		 	}
        }else if((1000 <= value_virtual_temp) && (value_virtual_temp < 3000)){//当光感值在1000-3000之间，背光亮度设置为backlight_bright_temp_yxp = 235
        	 //printk("==yy==HHHHH\n");
    		 if(al3010_bl_level != 5){
            	 value_yxp = 235;
    			 al3010_bl_flag_yxp = 1;
        		 al3010_bl_level = 5;
    		 	}
        }else if(value_virtual_temp > 3000){//当光感值在大于3000之间，背光亮度设置为backlight_bright_temp_yxp = 255
        	 //printk("==yy==JJJJJ\n");
    		 if(al3010_bl_level != 6){
            	 value_yxp = 255;
    			 al3010_bl_flag_yxp = 1;
        		 al3010_bl_level = 6;
    		 	}
        	}	
   
        if (al3010_bl_flag_yxp){
//            printk("==yy==KKKKKK\n");
            if(backlight_bright_temp_yxp == value_yxp){
            	al3010_bl_flag_yxp =0;
            	return 0;
            }else if(backlight_bright_temp_yxp < value_yxp){ 
            	for(x=backlight_bright_temp_yxp;x<(value_yxp+1);x++){
            		  disp_lcd_get_brightness_yxp(x);
            			}
            }else if(backlight_bright_temp_yxp > value_yxp){
            		for(x=backlight_bright_temp_yxp;x>(value_yxp-1);x--){
            			disp_lcd_get_brightness_yxp(x);
            			}
            	  }

        	}
   	}
return 0;
}
#endif
#endif
/*******************************************************************************
* Function    :  al3010_report_dls
* Description :  reg data,input dev
* Parameters  :  report light sensor data to input system
* Return      :  none
*******************************************************************************/
static void al3010_report_dls(int lux, struct input_dev *input)
{   
    if(alsps_debug)
	LDBG("lux=%d", lux);

	input_report_abs(input, ABS_MISC, lux);
	input_sync(input);
}

/*******************************************************************************
* Function    :  als_dwork_handler
* Description :  process the ls data through the polling mode
* Parameters  :  work
* Return      :    none
*******************************************************************************/
static void als_dwork_handler(struct work_struct *work)
{
	struct al3010_data *data = container_of(work,
	        struct al3010_data, al3010_als_dwork.work);
	u16 alsraw = 0;

	if (atomic_read(&l_flag)) {
#if ALS_AUTO_GAIN
		u64 now_time;

		if (data->als_auto_gain_trig) {
			now_time = ktime_get_ns();

			if ((now_time - data->als_auto_gain_trig_time) <
			    data->als_auto_gain_msdelay * 1000000)
				goto exit_als_auto_gain;
		}

#endif

		alsraw = al3010_get_als_adc(data);
        	
#if ALS_AUTO_GAIN
		data->als_auto_gain_trig = al3010_als_auto_gain(data, alsraw);

		if (data->als_auto_gain_trig)
			goto exit_als_auto_gain;

#endif

		data->lux = al3010_get_lux_value(alsraw);
        light_value = data->lux;   // get lux data
        light_value = light_value * g_Parameter / g_Parameter_2;     //add by  yxp  20210113
        if (alsps_debug)
		LDBG("==yy==lux: %d light_value: %d", data->lux,light_value);
#ifdef CONFIG_YOUXUEPAI_BOARD		
#ifdef AL3010_AUTO_BL
//yanjb 20210403
        if (bl_state)
            al3010_set_automatic_brightness(light_value);      	
#endif
#endif        	
		al3010_report_dls(light_value, data->alssensor_input_dev);

#if ALS_AUTO_GAIN
exit_als_auto_gain:
#endif

		/* restart timer */
		queue_delayed_work(data->al3010_wq,
		    &data->al3010_als_dwork,
		    msecs_to_jiffies(data->als_poll_delay));
	}
}
static int al3010_register_alssensor_device(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev;
	int rc;

	LDBG("allocating input device lsensor");

	input_dev = input_allocate_device();

	if (!input_dev) {
		dev_err(&client->dev,
		    "%s: could not allocate input device for lsensor\n", __func__);
		rc = -ENOMEM;
		goto done;
	}

	data->alssensor_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "AL3010 ALS";
	input_dev->dev.parent = &client->dev;
	input_dev->phys  = "ALS";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, ((1 << 16) - 1), 0, 0);

	rc = input_register_device(input_dev);

	if (rc < 0) {
		pr_err("%s: could not register input device for lsensor\n",
		    __func__);
	}

done:
	return rc;
}

static void al3010_unregister_alssensor_device(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	input_unregister_device(data->alssensor_input_dev);
}

static int al3010_sw_reset(void)
{
	int ret = 0;

	ret = al3010_write_data(SYS_CONFIG, 0x04);

	if (ret == 0) {
		LDBG("al3010_sw_reset success");
		msleep(30);
	} else {
		LDBG("al3010_sw_reset failed! ret = %d", ret);
	}

	return ret;
}
//yanjb20210113
/*----------------------------light sysfs-------------------------------------*/


static void para_eeprom_write(void)
{
	//struct file *filp = NULL;
	//mm_segment_t old_fs;	
	unsigned char buffer_tmp[8];
	unsigned char lux_checksum;
	
	/*filp = filp_open(EEPROM_NAME_EP, O_RDWR | O_NDELAY , 0);

	if (IS_ERR(filp))
	{
		printk("OPen file eeprom over!!!!\n");
		return ;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);*/
	buffer_tmp[2] = g_Parameter_2 & 0xff;
	buffer_tmp[3] = (g_Parameter_2 >> 8) & 0xff;
	buffer_tmp[0] = g_Parameter & 0xff;
	buffer_tmp[1] = (g_Parameter >> 8) & 0xff;

	lux_checksum = buffer_tmp[0];
	lux_checksum += buffer_tmp[1];
	lux_checksum += buffer_tmp[2];
	lux_checksum += buffer_tmp[3];
	buffer_tmp[4] = 'L';
	buffer_tmp[5] = 'U';
	buffer_tmp[6] = 'X';
	buffer_tmp[7] = lux_checksum;
	printk(KERN_ALERT"lux_checksum = %d\n",lux_checksum);
	noah_eeprom_setdata(NOAH_LIGHT_SENSOR_TYPE,buffer_tmp);
	printk("88888888888888888888888888888888");
	/*filp->f_op->llseek(filp, LS_PARA_ADDR_START, SEEK_SET);
	filp->f_op->write(filp, buffer_tmp, 8, &filp->f_pos);
	set_fs(old_fs);

	printk("xxxxxxxx:0x%x, 0x%x, 0x%x, 0x%x\n", buffer_tmp[0],buffer_tmp[1],buffer_tmp[2],buffer_tmp[3]);


	if (filp != NULL)
	{
		filp_close(filp, NULL);
		filp = NULL;
	}*/
}



static ssize_t light_parameter_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	unsigned char tmpbuffer[255]= { 0 };

	char *p1 = NULL, *p2 = NULL;
 
	p1 = (unsigned char *)buf;
	p2 = strchr(p1,  ' ');
	memset(tmpbuffer, 0, 255);
	memcpy(tmpbuffer, buf, p2-p1);

	p1 = p2 + 1;
	
	g_Parameter_Type = simple_strtoul(tmpbuffer, NULL, 10);
	printk("%s, %d, g_Parameter_Type:%d\n", __FILE__,  __LINE__, g_Parameter_Type);

	if (g_Parameter_Type == 0)
	{	
		g_Parameter = 100;
		g_Parameter_2 = 100;
		g_Parameter_Correct = 0;
	}
	else if (g_Parameter_Type == 1)
	{
		g_Parameter = 1902;
		g_Parameter_2 = 121;
		g_Parameter_Correct = 60;
	}
	else if (g_Parameter_Type == 2 )
	{
		g_Parameter = 1902;
		g_Parameter_2 = 121;
		g_Parameter_Correct = 60;
	}
	else
	{
		p2 = strchr(p1,  ' ');
		memset(tmpbuffer, 0, 255);
		memcpy(tmpbuffer, p1, p2-p1);
		g_Parameter = simple_strtoul(tmpbuffer, NULL, 10);

		p1 = p2 + 1;
		p2 = strchr(p1,  ' ');
		memset(tmpbuffer, 0, 255);
		memcpy(tmpbuffer, p1, p2-p1);
		g_Parameter_2 = simple_strtoul(tmpbuffer, NULL, 10);

		p1 = p2 + 1;
		p2 = strchr(p1,  ' ');
		memset(tmpbuffer, 0, 255);
		memcpy(tmpbuffer, p1, p2-p1);
		g_Parameter_Correct = simple_strtoul(tmpbuffer, NULL, 10);

		if (g_Parameter_Type == 3){
			para_eeprom_write();
			}
		
	}
	printk("%s, %d g_Parameter_Type:%d,  g_Parameter: %d,  g_Parameter_2: %d, g_Parameter_Correct: %d\n", 
		__FILE__,  __LINE__, 
		g_Parameter_Type, g_Parameter, g_Parameter_2, g_Parameter_Correct);
	return size;
}

static ssize_t light_parameter_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "g_Parameter_Type:%d, g_Parameter:%d, g_Parameter_2:%d, g_Parameter_Correct:%d\n",
		       g_Parameter_Type, g_Parameter, g_Parameter_2, g_Parameter_Correct);
}


static ssize_t light_hwinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret_len = 0;
	sprintf(buf, "%s\n", hwinfo);
	ret_len = strlen(buf) + 1;
	return ret_len;
}
static ssize_t light_lvalue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", light_value);
}

static ssize_t show_alsps_debug(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%u\n", alsps_debug);
}
static ssize_t store_alsps_debug(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	if (size > 0)
	{		
		alsps_debug = buf[0] -'0';
	}
	return size;
}
#ifdef CONFIG_YOUXUEPAI_BOARD
#ifdef AL3010_AUTO_BL
static ssize_t show_bl_state(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%u\n", bl_state);
}
static ssize_t store_bl_state(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	if (size > 0)
	{		
		bl_state = buf[0] -'0';
		bl_state_flag = 1;		
	}
	return size;
}
#endif
#endif

static DEVICE_ATTR(hwinfo, 0444, light_hwinfo_show, NULL);
static DEVICE_ATTR(lvalue, 0444,light_lvalue_show, NULL);
static DEVICE_NOAH_ATTR(light_parameter, S_IRUGO | S_IWUGO | S_IWGRP,light_parameter_show, light_parameter_store);
static DEVICE_NOAH_ATTR(alsps_debug, S_IRUGO | S_IWUGO | S_IWGRP,show_alsps_debug, store_alsps_debug);
#ifdef CONFIG_YOUXUEPAI_BOARD
#ifdef AL3010_AUTO_BL
static DEVICE_NOAH_ATTR(bl_state, S_IRUGO | S_IWUGO | S_IWGRP,show_bl_state, store_bl_state);
#endif
#endif

/*----------------------------------------------------------------------------*/


static struct kobject *android_light_proximity_sensor_kobj=NULL;

static int als_sensor_sysfs_init(void)
{
	int liRet ;

	android_light_proximity_sensor_kobj = kobject_create_and_add("android_light_proximity_sensor", NULL);
	if (android_light_proximity_sensor_kobj == NULL)
	{
		printk("\n light_proximity_sensor_sysfs_init:subsystem_register failed\n");
		liRet = -ENOMEM;
		goto kobject_create_failed;
	}

	liRet = sysfs_create_file(android_light_proximity_sensor_kobj, &dev_attr_hwinfo.attr); // "hwinfo"
	if (liRet) {
		printk("\n light_proximity_sensor_sysfs_init:/sys/android_light_proximity_sensor/hwinfo subsystem_register failed\n");
		goto sysfs_create_failed;
	}
	
	
	/*---------------------------light  sysfs---------------------------------*/

	liRet = sysfs_create_file(android_light_proximity_sensor_kobj, &dev_attr_alsps_debug.attr); // "alsps_debug" 
	if (liRet) {
		printk("\n light_proximity_sensor_sysfs_init:/sys/android_light_proximity_sensor/alsps_debug subsystem_register failed\n");
		goto sysfs_create_failed;
	}

	liRet = sysfs_create_file(android_light_proximity_sensor_kobj, &dev_attr_light_parameter.attr); // "light_parameter"
	if (liRet) {
		printk("\n light_proximity_sensor_sysfs_init:/sys/android_light_proximity_sensor/light_parameter subsystem_register failed\n");
		goto sysfs_create_failed;
	}


	liRet = sysfs_create_file(android_light_proximity_sensor_kobj, &dev_attr_lvalue.attr); // "lvalue"
	if (liRet) {
		printk("\n light_proximity_sensor_sysfs_init:/sys/android_light_proximity_sensor/lvalue subsystem_register failed\n");
		goto sysfs_create_failed;
	}
#ifdef CONFIG_YOUXUEPAI_BOARD		
#ifdef AL3010_AUTO_BL
	liRet = sysfs_create_file(android_light_proximity_sensor_kobj, &dev_attr_bl_state.attr); // "bl_state" 
	if (liRet) {
		printk("\n light_proximity_sensor_sysfs_init:/sys/android_light_proximity_sensor/alsps_debug subsystem_register failed\n");
		goto sysfs_create_failed;
	}
#endif
#endif	
	/*-------------------------------------------------------------------*/

	return 0 ;
	
sysfs_create_failed:
	kobject_del(android_light_proximity_sensor_kobj);
	
kobject_create_failed:
	return liRet ;
	
}

/*---------------------------add by yanjb 20210113  --------------------------------*/


extern char *noah_eeprom_readdata(int type);

static int ltr_read_light_calipara_from_eeprom(void)
{
	unsigned char buf[7]={0},sum=0;
	
	int i = 0;
	
	char *sys_light_sensor_data = noah_eeprom_readdata(NOAH_LIGHT_SENSOR_TYPE);

	if(NULL != sys_light_sensor_data){
		for(i = 0; i < 4 ; i++){
			buf[i] = sys_light_sensor_data[i];
			sum+=buf[i];			
		}
		buf[4] = sys_light_sensor_data[4];
		buf[5] = sys_light_sensor_data[5];
		buf[6] = sys_light_sensor_data[6];
		if(buf[4] != 'L' || buf[5] != 'U' || buf[6] != 'X' || sum != sys_light_sensor_data[7]){
			printk("noah_eeprom_readdata checked err!\n");
			return -1;
		}else{
			g_Parameter_2 = (buf[3] << 8) | buf[2];
			g_Parameter = (buf[1] <<8) | buf[0];
			printk("noah_eeprom_readdata : g_Parameter_2 = %d;g_Parameter= %d;\n",g_Parameter_2,g_Parameter);
		}
	}else{
		printk("noah_eeprom_readdata light sensor calipara failed!\n");
		return -1;
	}
	return 0;
}
/**********************************************************************************/

static int al3010_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *al3010;
	int err = 0;

	dev_info(&client->dev, "%s start\n", __func__);

	wakeup_source_init(&al3010_wake_source, "als_wake_source");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: functionality check failed\n", __func__);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	al3010 = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);

	if (!al3010) {
		dev_err(&client->dev, "%s: request memory failed\n", __func__);
		err = -ENOMEM;
		goto exit_request_memory_failed;
	}

	i2c_set_clientdata(client, al3010);
	al3010->client = client;
	this_client = client;

	LDBG("I2C addr=0x%x", client->addr);

	err = al3010_sw_reset();

	if (err)
		goto exit_device_init_failed;

	//init al3010
	if (al3010_reg_init(al3010) < 0) {
		dev_err(&client->dev, "%s: device init failed\n", __func__);
		err = -1;
		goto exit_device_init_failed;
	}

	err = al3010_register_alssensor_device(client);

	if (err) {
		dev_err(&client->dev, "failed to register_lsensor_device\n");
		goto exit_free_als_device;
	}

	//create work queue
	al3010->al3010_wq = create_singlethread_workqueue("als_wq");

	if (!al3010->al3010_wq) {
		dev_err(&client->dev, "create al3010 workqueue err\n");
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	/*--------------------------add by yanjb 20210113-----------------------*/
	als_sensor_sysfs_init();
				
	if(ltr_read_light_calipara_from_eeprom())
	   printk("read_light_calipara err......set default data\n");

	/*---------------------------------------------------------------------*/

	INIT_DELAYED_WORK(&al3010->al3010_als_dwork, als_dwork_handler);

	//create attribute files
	err = al3010_create_sysfs(client);

	if (err) {
		dev_err(&client->dev, "%s, create sysfs err: %d\n", __func__, err);
		goto remove_sysfs;
	}

	private_pl_data = al3010;

	dev_info(&client->dev, "%s: Probe Success!\n", __func__);

	return err;

remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);

err_create_wq_failed:
	destroy_workqueue(al3010->al3010_wq);

exit_free_als_device:
	al3010_unregister_alssensor_device(client);

exit_device_init_failed:
exit_request_memory_failed:
	kfree(al3010);

exit_check_functionality_failed:
	wakeup_source_trash(&al3010_wake_source);
	dev_err(&client->dev, "%s: Probe Fail!\n", __func__);
	return err;
}

static int al3010_remove(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s start\n", __func__);

	//remove queue
	flush_workqueue(data->al3010_wq);
	destroy_workqueue(data->al3010_wq);

	al3010_unregister_alssensor_device(client);

	//free input
	input_unregister_device(data->alssensor_input_dev);

	//free malloc
	kfree(data);
	data = NULL;
	this_client = NULL;

	wakeup_source_trash(&al3010_wake_source);

	return 0;
}

static int al3010_suspend(struct device *dev)
{
	int mode = 0;

	mode = al3010_get_mode();

	if (mode & AL3010_ALS) {
		al3010_write_reg(SYS_CONFIG, AL3010_REG_ALS_MODE_MASK,
		    AL3010_REG_ALS_MODE_SHIFT, 0x00);
	}

	/* Depend on use case */
#if 0
	al3010_power_init(ps_data, false);
	al3010_power_ctl(ps_data, false);
#endif

	return 0;
}

static int al3010_resume(struct device *dev)
{
	/* Depend on use case */
#if 0
	al3010_power_init(ps_data, true);
	al3010_power_ctl(ps_data, true);
	msleep(30);
#endif

	if (atomic_read(&l_flag)) {
		al3010_write_reg(SYS_CONFIG, AL3010_REG_ALS_MODE_MASK,
		    AL3010_REG_ALS_MODE_SHIFT, 0x01);
	}

	return 0;
}

static const struct dev_pm_ops al3010_pm_ops = {
	.suspend	    = al3010_suspend,
	.resume	    = al3010_resume,
};

static const struct i2c_device_id al3010_id[] = {
	{ AL3010_DEVICE, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, al3010_id);

#ifdef CONFIG_OF
static struct of_device_id al3010_match_table[] = {
	{.compatible = "di,al3010", },
	{},
};
#else
#define al3010_match_table NULL
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver al3010_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = AL3010_DEVICE,
#ifdef CONFIG_OF
		.of_match_table = al3010_match_table,
#endif
		.pm     = &al3010_pm_ops,
	},
	.probe      = al3010_probe,
	.remove     = al3010_remove,
	.id_table = al3010_id,
};
/*----------------------------------------------------------------------------*/

static int __init al3010_init(void)
{
	return i2c_add_driver(&al3010_driver);
}

static void __exit al3010_exit(void)
{
	i2c_del_driver(&al3010_driver);
}

module_init(al3010_init);
module_exit(al3010_exit);

MODULE_AUTHOR("Leo Tsai <leohs.tsai@dyna-image.com>");
MODULE_DESCRIPTION("Driver for AL3010 Proximity and Light Sensor");
MODULE_LICENSE("GPL v2");
