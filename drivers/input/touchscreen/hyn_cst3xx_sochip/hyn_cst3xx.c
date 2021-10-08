/* 
 * drivers/input/touchscreen/CST3XX.c
 *
 * hynitron TouchScreen driver. 
 *
 * Copyright (c) 2015  hynitron
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
 * VERSION      	DATE			AUTHOR
 *  1.0		    2015-10-12		    Tim
 *
 * note: only support mulititouch
 */
 
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pm_runtime.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/input/mt.h>

#include <linux/i2c.h>
#include <linux/input.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
//#include <linux/init-input.h>
#include "../../init-input.h"

#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/spinlock.h>


#include "hyn_cst3xx_fx_Ironman_7cun_fw.h"
#include "hyn_cst3xx_olm_Ironman_7cun_fw.h"
#include "HYN_CST1_1_0714.h"

struct hyn_fw_array {
	const char* name;
	const char* project_name;
	const int project_id;
	const unsigned char *fw;
} cst3xx_fw_grp[] = {
     { "hyn_cst3xx_fx_Ironman_7cun_fw",  "CPH_20210714_CST340_HYN_FXXC070G_0594_0701", 0x1737  , cst3_fw_v4 },         //0x1823 0x1737cst3_fw
	 { "hyn_cst3xx_olm_Ironman_7cun_fw", "CPH_2009004_CST340_ZT_OLM_070D4114_GG_7cun",0x1904  , cst3_fw_v1 },
	 { "HYN_CST1_1_0714_fw", "CPH_20210714_CST340_ZT_OLM_070D4114_GG_7cun",0x271C  , cst3_fw_v4 },
};
	 
struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
	.name = NULL,
	.int_number = 0,
};

#define CONFIG_TP_ESD_PROTECT        //yanjb 202103016  esd
#define ANDROID_TOOL_SURPORT
#define HYN_SYSFS_NODE_EN            //yanjb 20210114
#define TRANSACTION_LENGTH_LIMITED
//#define ICS_SLOT_REPORT
#define SLEEP_RESUME_CLEAR_POINT
//#define HAVE_TOUCH_KEY
#define GTP_DEBUG_FUNC_ON
#define HYN_UPDATE_FIRMWARE_ENABLE    //open update_fw   yanjb
//#define HYN_UPDATE_FIRMWARE_FORCE



#define PRESS_MAX    				255
#define MAX_FINGERS 				5 
#define MAX_CONTACTS 				5

#define CST3XX_I2C_NAME 	        "cst3xx"
#define CTP_IRQ_NUMBER              (config_info.int_number)
#define CTP_IRQ_MODE			    (IRQF_TRIGGER_FALLING)
#define CTP_NAME			         CST3XX_I2C_NAME
#define CST3XX_I2C_ADDR   			 0x1A
int hyn_tp_flag = 0;


						 
#define CST3XX_USED     "\n \
												  \n+++++++++++++++++++++++++++++++++ \
												  \n++++++ CST3XX new used +++++++++ \
												  \n+++++++++++++++++++++++++++++++++ \
												  \n"
													 

static unsigned int   g_cst3xx_ic_version  = 0;
static unsigned int   g_cst3xx_ic_checksum = 0;
static unsigned int   g_cst3xx_ic_checkcode =0;
static unsigned int   g_cst3xx_ic_project_id  = 0;
static unsigned int   g_cst3xx_ic_type  = 0;
static int 			  fw_index = -1;
static int 			  screen_max_x = 0;
static int 			  screen_max_y = 0;
static int            revert_x_flag = 0;
static int            revert_y_flag = 0;
static int			  exchange_x_y_flag = 0;
static char* 		  fwname;
static __u32 		  twi_id    = 0;
struct i2c_client    *cst3xx_i2c;
static unsigned char *pcst3xx_update_firmware = (unsigned char *)cst3_fw ; //the updating firmware  cst3_fw


#define SCREEN_MAX_X		        (screen_max_x)
#define SCREEN_MAX_Y		        (screen_max_y)


#ifdef ANDROID_TOOL_SURPORT
static unsigned short g_unnormal_mode = 0;
static unsigned short g_cst3xx_abs_x_max = 0;
static unsigned short g_cst3xx_abs_y_max = 0;
static unsigned char  g_cst3xx_key_num = 0;
static unsigned short g_cst3xx_tx = 14;
static unsigned short g_cst3xx_rx = 26;
static unsigned char *p_online_firmware = NULL ; //the updating firmware
static unsigned short g_online_mode = 0;
#ifdef HYN_SYSFS_NODE_EN
static struct kobject* k_obj = NULL;
static struct mutex g_device_mutex;
static DEFINE_MUTEX (g_device_mutex);
#endif

#endif




#ifdef CONFIG_TP_ESD_PROTECT
#define SWITCH_ESD_OFF                  0
#define SWITCH_ESD_ON                   1
static struct workqueue_struct *cst3xx_esd_workqueue;
static int esd_work_cycle = 1000;
static struct delayed_work esd_check_work;
static int esd_running;
static struct mutex esd_lock;
static void cst3xx_esd_check_func(struct work_struct *);
static unsigned int pre_counter = 0;
static unsigned int cur_counter = 0;
static char i2c_lock_flag = 0;

#endif


 

#ifdef HAVE_TOUCH_KEY       
static u16 key = 0;
static int key_state_flag = 0;
struct key_data {
	u16 key;
	u16 x;
	u16 y;	
};
const u16 key_array[]={
      KEY_BACK,
      KEY_HOME,
      KEY_MENU,
      KEY_SEARCH,
}; 
#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))
struct key_data hyn_key_data[MAX_KEY_NUM] = {
	{KEY_BACK,   850, 120},
	{KEY_HOME,   850, 240},	
	{KEY_MENU,   850, 360},
	{KEY_SEARCH, 850, 480},
};
#endif

struct hyn_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct hyn_ts_data *dd;
	u8 *touch_data;
	u8 device_id;
	u8 prev_touches;
	bool is_suspended;
	int is_enable;
	bool int_pending;
	struct mutex sus_lock;
	int irq;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

};
struct hyn_ts *ts_init;   //yanjb20210121

#ifdef	GTP_DEBUG_FUNC_ON
#define HYN_DBG(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#define HYN_INF(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#define HYN_ERR(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#else
#define HYN_DBG(fmt, arg...)  
#define HYN_INF(fmt, arg...) 
#define HYN_ERR(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#endif


/* Addresses to scan */
static const unsigned short normal_i2c[2] = {CST3XX_I2C_ADDR, I2C_CLIENT_END};
static int cst3xx_firmware_info(struct i2c_client * client);
static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata);
#ifdef CONFIG_TP_ESD_PROTECT
void cst3xx_esd_switch(s32 on);
#endif

static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while(retries < 2) { 
		ret = i2c_master_recv(client, buf, len); 
		if(ret<=0) retries++;
        else  break; 
	} 
	return ret; 
} 

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while(retries < 2) 
	{ 
		ret = i2c_master_send(client, buf, len); 
		if(ret<=0) retries++;
        else  break; 
	} 
	
	return ret; 
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
    
    ret = cst3xx_i2c_write(client, buf, 2);

    ret = cst3xx_i2c_read(client, buf, len);
	
    return ret; 
} 
static int test_i2c(struct i2c_client *client)
{
	int retry = 0;
	int ret;
	u8  buf[4];

	buf[0] = 0xD1;
	buf[1] = 0x06;
	while(retry++<2) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return ret;
		mdelay(2);		
	}
    if(retry==5) HYN_ERR("cst3xx test error.ret:%d;\n", ret);
	return ret;
}
static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)){
		        HYN_INF("==yy==i2c_check  fail\n");
                return -ENODEV;
    	}
    HYN_INF("==yy== adapter->nr = %d\n",adapter->nr);
	if(twi_id == adapter->nr){
    	HYN_INF("%s: addr= %x\n",__func__,client->addr);
        ret = test_i2c(client);
        if(ret<0){
        	HYN_INF("%s:iic connection might be something wrong \n",__func__);
        	return -ENODEV;
        }else{      
            HYN_INF("iic connection sucess!\n");
            strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
			HYN_INF("%s", CST3XX_USED);
    		return 0;	
	    }
	}else{
		return -ENODEV;
	}
}

/**
 * ctp_print_info - sysconfig print function
 * return value:
 *
 */
void ctp_print_info(struct ctp_config_info info)
{	
	HYN_INF("info.ctp_used:%d\n",info.ctp_used);
	HYN_INF("info.ctp_name:%s\n",info.name);
	HYN_INF("info.twi_id:%d\n",info.twi_id);
	HYN_INF("info.screen_max_x:%d\n",info.screen_max_x);
	HYN_INF("info.screen_max_y:%d\n",info.screen_max_y);
	HYN_INF("info.revert_x_flag:%d\n",info.revert_x_flag);
	HYN_INF("info.revert_y_flag:%d\n",info.revert_y_flag);
	HYN_INF("info.exchange_x_y_flag:%d\n",info.exchange_x_y_flag);
	HYN_INF("info.irq_gpio_number:%d\n",info.irq_gpio.gpio);
	HYN_INF("info.wakeup_gpio_number:%d\n",info.wakeup_gpio.gpio);	
}

/**
 * hard_reset_chip - function
 *
 */

int hard_reset_chip(int ms)
{
	HYN_INF("***CTP*** %s,ms = %d\n",__func__,ms); 
	unsigned char buf[4];
	if(0){
		buf[0] = 0xD1;
		buf[1] = 0x0E;	 
		cst3xx_i2c_write(cst3xx_i2c, buf, 2);

	}else{
		__gpio_set_value(config_info.wakeup_gpio.gpio, 0);
		mdelay(10);
		__gpio_set_value(config_info.wakeup_gpio.gpio, 1);
	}
	mdelay(ms);
	return 0;
}

static void cst3xx_enter_sleep(struct i2c_client *client)
{
	int ret;
	int retry = 0;
	unsigned char buf[2];

    buf[0] = 0xD1;
	buf[1] = 0x05;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return;
		mdelay(2);
	}
	
	return;
}

static s32 irq_is_disable = 0;
spinlock_t irq_lock;
void cst3xx_irq_enable(void)
{
	unsigned long irqflags = 0;
	int ret;
	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable) {
		ret = input_set_int_enable(&(config_info.input_type), 1);
		if (ret < 0)
		HYN_INF(" %s irq disable failed\n", __func__);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void cst3xx_irq_disable(void)
{
	unsigned long irqflags;
	int ret;
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		ret = input_set_int_enable(&(config_info.input_type), 0);
		if (ret < 0)
		HYN_INF(" %s irq disable failed\n", __func__);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}
#define CST3XX_BIN_SIZE    (24*1024 + 24)

static int cst3xx_check_checksum(struct i2c_client * client)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	const unsigned char *pData;

	for(i=0; i<5; i++)
	{
		buf[0] = 0xA0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if(buf[0]!=0)
			break;
		else
		mdelay(2);
	}
    mdelay(2);

	if(buf[0]==0x01)
	{
		buf[0] = 0xA0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(client, buf, 4);	
		if(ret < 0)	return -1;		
		// read chip checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);
        pData=(unsigned char  *)pcst3xx_update_firmware +24*1024+16;  
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);
        HYN_INF("hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\n", checksum, bin_checksum);   
        if(checksum!=bin_checksum)
		{
			HYN_ERR("cst3xx hyn check sum error.\n");			
			return -1;		
		}	
	}
	else
	{
		HYN_ERR(" cst3xx hyn No checksum.\n");
		return -1;
	}	
	return 0;
}

static int cst3xx_into_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[4];
	
	buf[0] = 0xA0;
	buf[1] = 0x01;	
	buf[2] = 0xAA;	//set cmd to enter program mode		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)  return -1;

	mdelay(2);
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	//check whether into program mode
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1; //
	
	return 0;

}

static int cst3xx_exit_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x06;
	buf[2] = 0xEE;
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)
		return -1;
	
	mdelay(10);	//wait for restart

	return 0;
}

static int cst3xx_erase_program_area(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	
	buf[2] = 0x00;		//set cmd to erase main area		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0) return -1;
	
	mdelay(5);
	
	buf[0] = 0xA0;
	buf[1] = 0x03;
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1;

	return 0;
}

static int cst3xx_write_program_data(struct i2c_client * client,
		const unsigned char *pdata)
{
	int i, ret;
	unsigned char *i2c_buf;
	unsigned short eep_addr;
	int total_kbyte;
#ifdef TRANSACTION_LENGTH_LIMITED
	unsigned char temp_buf[8];
	unsigned short iic_addr;
	int  j;

#endif

	i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
	if (i2c_buf == NULL) 
		return -1;
	
	//make sure fwbin len is N*1K
	//total_kbyte = len / 1024;
	total_kbyte = 24;
	for (i=0; i<total_kbyte; i++) {
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x14;
		eep_addr = i << 10;		//i * 1024
		i2c_buf[2] = eep_addr;
		i2c_buf[3] = eep_addr>>8;
		ret = cst3xx_i2c_write(client, i2c_buf, 4);
		if (ret < 0)
			goto error_out;

#ifdef TRANSACTION_LENGTH_LIMITED
		memcpy(i2c_buf, pdata + eep_addr, 1024);
		for(j=0; j<256; j++) {
			iic_addr = (j<<2);
    	temp_buf[0] = (iic_addr+0xA018)>>8;
    	temp_buf[1] = (iic_addr+0xA018)&0xFF;
		temp_buf[2] = i2c_buf[iic_addr+0];
		temp_buf[3] = i2c_buf[iic_addr+1];
		temp_buf[4] = i2c_buf[iic_addr+2];
		temp_buf[5] = i2c_buf[iic_addr+3];
    	ret = cst3xx_i2c_write(client, temp_buf, 6);
    		if (ret < 0)
    			goto error_out;		
		}
#else
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x18;
		memcpy(i2c_buf + 2, pdata + eep_addr, 1024);
		ret = cst3xx_i2c_write(client, i2c_buf, 1026);
		if (ret < 0)
			goto error_out;
#endif

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x04;
		i2c_buf[2] = 0xEE;
		ret = cst3xx_i2c_write(client, i2c_buf, 3);
		if (ret < 0)
			goto error_out;
		
		mdelay(60);	
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x05;
		ret = cst3xx_i2c_read_register(client, i2c_buf, 1);
		if (ret < 0)
			goto error_out;
		
		if (i2c_buf[0] != 0x55)
			goto error_out;

	}

	i2c_buf[0] = 0xA0;
	i2c_buf[1] = 0x03;
	i2c_buf[2] = 0x00;
	ret = cst3xx_i2c_write(client, i2c_buf, 3);
	if (ret < 0)
		goto error_out;
	
	mdelay(8);	
	
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}

	return 0;
	
error_out:
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}
	return -1;
}

static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata)
{
	int ret;
	int retry = 0;
	
	HYN_INF(" cst3xx----------upgrade cst3xx begain------------\n");

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	cst3xx_irq_disable();

	mdelay(20);
START_FLOW:	

	
	hard_reset_chip(5+retry);	
	
	ret = cst3xx_into_program_mode(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]into program mode failed.\n");
		goto err_out;
	}

	ret = cst3xx_erase_program_area(client);
	if (ret) {
		HYN_ERR(" cst3xx[cst3xx]erase main area failed.\n");
		goto err_out;
	}

	ret = cst3xx_write_program_data(client, pdata);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]write program data into cstxxx failed.\n");
		goto err_out;
	}

    ret =cst3xx_check_checksum(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx] after write program cst3xx_check_checksum failed.\n");
		goto err_out;
	}

	ret = cst3xx_exit_program_mode(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]exit program mode failed.\n");
		goto err_out;
	}
	mdelay(10);
	hard_reset_chip(30);	

	HYN_INF(" cst3xx hyn----------cst3xx_update_firmware ok end------------\n");

	cst3xx_irq_enable();
	
	return 0;
	
err_out:
	if (retry < 30) {
		retry++;
		mdelay(30);
		goto START_FLOW;
	} 
	else {	

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
	
	cst3xx_irq_enable();
	return -1;
	}
}

static int cst3xx_update_judge( unsigned char *pdata, int strict)
{
	unsigned short ic_type, project_id;
	unsigned int fw_checksum, fw_version;
	const unsigned int *p;
	int i;
	unsigned char *pBuf;
		
	fw_checksum = 0x55;
	p = (const unsigned int *)pdata;
	for (i=0; i<(CST3XX_BIN_SIZE-4); i+=4) {
		fw_checksum += (*p);
		p++;
	}
	
	if (fw_checksum != (*p)) {
		HYN_ERR(" cst3xx[cst3xx]calculated checksum error:0x%x not equal 0x%x.\n", fw_checksum, *p);
		return -1;	//bad fw, so do not update
	}
	
	pBuf = &pdata[CST3XX_BIN_SIZE-16];
	
	project_id = pBuf[1];
	project_id <<= 8;
	project_id |= pBuf[0];

	ic_type = pBuf[3];
	ic_type <<= 8;
	ic_type |= pBuf[2];

	fw_version = pBuf[7];
	fw_version <<= 8;
	fw_version |= pBuf[6];
	fw_version <<= 8;
	fw_version |= pBuf[5];
	fw_version <<= 8;
	fw_version |= pBuf[4];

	fw_checksum = pBuf[11];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[10];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[9];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[8];	
	
	HYN_INF(" cst3xx[cst3xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,version:0x%x,checksum:0x%x\n",
			project_id, ic_type, fw_version, fw_checksum);
#ifndef HYN_UPDATE_FIRMWARE_ENABLE
	HYN_INF("[cst3xx] HYN_UPDATE_FIRMWARE_ENABLE is not open.\n");
   return -1;
#endif

	if((strict > 0)&&(g_cst3xx_ic_project_id == project_id)){
		
		if (g_cst3xx_ic_checksum != fw_checksum){
			
#ifdef HYN_UPDATE_FIRMWARE_FORCE
    		HYN_INF("[cst3xx] update firmware online force.\n");		
    		return 0;
#endif 		
			HYN_ERR("[cst3xx]fw version(0x%x), ic version(0x%x).\n",fw_version, g_cst3xx_ic_version);
			if (g_cst3xx_ic_version != fw_version){      //�ж���������    yanjb			
				return 0;
			}else{
				return -1;
			}
			
		}else{
			HYN_ERR("[cst3xx]fw checksum(0x%x), ic checksum(0x%x).\n",fw_checksum, g_cst3xx_ic_checksum);
			return -1;
		}
	}	
	
	return 0;
}


static int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
//	unsigned short ic_type, project_id;

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	
	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
	
	mdelay(10);

	buf[0] = 0xD1;
	buf[1] = 0xFC;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;

	//0xCACA0000
	g_cst3xx_ic_checkcode = buf[3];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[2];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[1];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[0];
	
	HYN_INF(" cst3xx [cst3xx] the chip g_cst3xx_ic_checkcode:0x%x.\r\n",g_cst3xx_ic_checkcode);
	if((g_cst3xx_ic_checkcode&0xffff0000)!=0xCACA0000){
		HYN_INF(" cst3xx [cst3xx] cst3xx_firmware_info read error .\r\n");
		return -1;
	}

	mdelay(10);

	buf[0] = 0xD2;
	buf[1] = 0x04;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;
	g_cst3xx_ic_type = buf[3];
	g_cst3xx_ic_type <<= 8;
	g_cst3xx_ic_type |= buf[2];

	g_cst3xx_ic_project_id = buf[1];
	g_cst3xx_ic_project_id <<= 8;
	g_cst3xx_ic_project_id |= buf[0];

	HYN_INF(" cst3xx [cst3xx] the chip ic g_cst3xx_ic_type :0x%x, g_cst3xx_ic_project_id:0x%x\r\n",
		g_cst3xx_ic_type, g_cst3xx_ic_project_id);

	mdelay(2);
	
	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;	

	g_cst3xx_ic_version = buf[3];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[2];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[1];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[0];

	g_cst3xx_ic_checksum = buf[7];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[6];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[5];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[4];	

	HYN_INF(" cst3xx [cst3xx] the chip ic version:0x%x, checksum:0x%x\r\n",
		g_cst3xx_ic_version, g_cst3xx_ic_checksum);

	if(g_cst3xx_ic_version==0xA5A5A5A5)
	{
		HYN_ERR(" cst3xx [cst3xx] the chip ic don't have firmware. \n");
		return -1;
	}

	buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
	
	return 0;
}

static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata)
{
	int ret;
	int retry = 0;
	
	ret = cst3xx_update_judge(pdata, 1);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx] no need to update firmware.\n");
		return 0;
	}
	ret = cst3xx_update_firmware(client, pdata);
	if (ret < 0){
		HYN_ERR(" cst3xx [cst3xx] update firmware failed.\n");
		return -1;
	}
    mdelay(50);
	ret = cst3xx_firmware_info(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx [cst3xx] after update read version and checksum fail.\n");
		return -1;
	}

	return 0;
}

static int cst3xx_find_fw_idx(int project_id)
{
	int i = 0;
//yanjb 20210102
//	if (NULL != name) {
		for (i=0; i<ARRAY_SIZE(cst3xx_fw_grp); i++) {
			if(project_id==cst3xx_fw_grp[i].project_id)
				return i;
//		}
	}
	return -1;

}
//yanjb20210102

static int hyn_boot_update_fw(struct i2c_client * client)
{
	unsigned char *ptr_fw;
	int ret;
	int retry=0;
	int proj_id=0;
	
	while (retry++ < 3) {
		ret = cst3xx_firmware_info(client);
		if (ret == 0) {
			break;
		}
	}
	if(ret==0){
		if(g_online_mode==0){
			proj_id=cst3xx_find_fw_idx(g_cst3xx_ic_project_id);
			HYN_INF("cst3xx_firmware_info proj_id:%d.\r\n",proj_id);
			if(proj_id<0){
				HYN_INF("cst3xx: not find matched TP firmware,update default firmware !\n");
				pcst3xx_update_firmware=cst3xx_fw_grp[0].fw;
				//return 0;
			}else{
				HYN_INF("find matched TP firmware(%s)!\n", cst3xx_fw_grp[proj_id].name);
				HYN_INF("find matched TP firmware(%s)!\n", cst3xx_fw_grp[proj_id].project_name);
				pcst3xx_update_firmware=cst3xx_fw_grp[proj_id].fw;
			}
		}
		ptr_fw = pcst3xx_update_firmware;
		ret = cst3xx_boot_update_fw(client, ptr_fw);
	    return ret;	
	}else{
		HYN_INF("ic have no firmware, please return to module supplier!\n");
		return -1;	
	}

}
static void cst3xx_touch_down(struct input_dev *input_dev,s32 id,s32 x,s32 y,s32 w)
{
    s32 temp_w = (w>>1);
    s32 temp;
    
    temp_w += (temp & 0x0007);

#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
#else
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_key(input_dev, BTN_TOUCH, 1);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(input_dev);
#endif
}

static void cst3xx_touch_up(struct input_dev *input_dev, int id)
{
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif

}
#ifdef CONFIG_TP_ESD_PROTECT

void cst3xx_init_esd_protect(void)
{
    esd_work_cycle = 1 * HZ;	/*HZ: clock ticks in 1 second generated by system*/
	printk("Clock ticks for an esd cycle: %d", esd_work_cycle);
	INIT_DELAYED_WORK(&esd_check_work, cst3xx_esd_check_func);
	mutex_init(&esd_lock);

}

void cst3xx_esd_switch(s32 on)
{
	mutex_lock(&esd_lock);
	if (SWITCH_ESD_ON == on) {	/* switch on esd check */
		if (!esd_running) {
			esd_running = 1;
			pre_counter = 0;
			cur_counter = 0;
			printk("Esd protector started!");
			queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
		}
	} else {		/* switch off esd check */
		if (esd_running) {
			esd_running = 0;
			printk("Esd protector stopped!");
			cancel_delayed_work(&esd_check_work);
		}
	}
	mutex_unlock(&esd_lock);
}

static void cst3xx_esd_check_func(struct work_struct *work)
{

    int retry = 0;
	int ret;
	int check_sum;
	unsigned char buf[8];

	if (!esd_running) {
	printk("Esd protector suspended!");
	return;
	}

	if((i2c_lock_flag != 0)||(g_online_mode!=0))
		goto END;
	else
		i2c_lock_flag = 1;

	buf[0] = 0xD1;
	buf[1] = 0x09;
	cst3xx_i2c_write(cst3xx_i2c, buf, 2);

	while(retry++ < 5) {
		buf[0] = 0xD0;
		buf[1] = 0x40;
		ret = cst3xx_i2c_read_register(cst3xx_i2c, buf, 6);
		if (ret > 0){
			check_sum = buf[0]+buf[1]+buf[2]+buf[3]+0xA5;
			if(check_sum != ( (buf[4]<<8)+ buf[5])){
		          printk("[HYN]cst3xx_esd_check_func read check_sum error. check_sum = 0x%x.\n", check_sum);
			}else{
//			      printk("[HYN]==yy==esd  cst3xx_esd_check_func success \n");      //yanjb esd 20210315
					goto END; 
			}
		}
		mdelay(4);
	}

    if((retry==6) || (ret<0)){
		goto hyn_esd_check_init;
    }

	cur_counter = buf[3]+(buf[2]<<8)+(buf[1]<<16)+(buf[0]<<24);
	if(((cur_counter<pre_counter) || ((cur_counter-pre_counter)<20)) && (pre_counter>100))
	{
           printk("[HYN]cst3xx_esd_check_func cur_counter is :0x%x. pre_counter is:0x%x.------\n",cur_counter,pre_counter);
           goto hyn_esd_check_init;
	}
	goto END;

hyn_esd_check_init:
	printk("[HYN]cst3xx_esd_check_func reset .\n");		
	hard_reset_chip(30);
	cur_counter=0;
	pre_counter=0;
	

END:
	i2c_lock_flag = 0;
	pre_counter=cur_counter;
	mutex_lock(&esd_lock);
	if (esd_running)
		queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
	else
		printk("Esd protector suspended!");
	mutex_unlock(&esd_lock);
}

#endif


#ifdef ANDROID_TOOL_SURPORT   //debug tool support
#define CST1XX_PROC_DIR_NAME    "cst1xx_ts"
#define CST1XX_PROC_FILE_NAME   "cst1xx-update"
static struct proc_dir_entry* g_proc_dir, *g_update_file;
static int CMDIndex = 0;

static struct file* cst3xx_open_fw_file (char* path, mm_segment_t* old_fs_p)
{
    struct file* filp = NULL;
    int ret;
    *old_fs_p = get_fs();
    //set_fs(KERNEL_DS);
    filp = filp_open (path, O_RDONLY, 0);

    if (IS_ERR (filp)) {
        ret = PTR_ERR (filp);
        HYN_ERR ("cst3xx_open_fw_file:filp_open error.\n");
        return NULL;
    }

    filp->f_op->llseek (filp, 0, 0);
    return filp;
}

static void cst3xx_close_fw_file(struct file * filp,mm_segment_t old_fs)
{
	//set_fs(old_fs);
	if(filp)
	    filp_close(filp,NULL);
}

static int cst3xx_read_fw_file (unsigned char* filename, unsigned char* pdata, int* plen)
{
    struct file* fp;
    int ret = -1;
    loff_t pos;
    off_t fsize;
    struct inode* inode;
    unsigned long magic;
    mm_segment_t old_fs;

    HYN_DBG ("cst3xx_read_fw_file enter.\n");

    if ((pdata == NULL) || (strlen (filename) == 0)) {
        HYN_ERR ("cst3xx file name is null.\n");
        return ret;
    }

    fp = cst3xx_open_fw_file (filename, &old_fs);
    if (fp == NULL) {
        HYN_ERR ("cst3xxOpen bin file faild.path:%s.\n", filename);
        goto clean;
    }
	
#if 0
    length = fp->f_op->llseek (fp, 0, SEEK_END);
    fp->f_op->llseek (fp, 0, 0);
    size = fp->f_op->read (fp, pdata, length, &fp->f_pos);
    if (size == length) {
        ret = 0;
        *plen = length;
    } else
        HYN_DBG ("read bin file length fail****size:%d*******length:%d .\n", size, length);
#else
    if (IS_ERR (fp)) {
        HYN_ERR ("error occured while opening file %s.\n", filename);
        return -EIO;
    }

    inode = fp->f_inode;
    magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    old_fs = get_fs();
    set_fs (KERNEL_DS);
    pos = 0;
    ret = vfs_read (fp, pdata, fsize, &pos);
    if (ret == fsize)
        HYN_DBG ("vfs_read success.ret:%d.\n", ret);
    else
        HYN_DBG ("vfs_read fail.ret:%d.\n", ret);

    filp_close (fp, NULL);
    set_fs (old_fs);
    HYN_DBG ("vfs_read done.\n");
#endif
clean:
    cst3xx_close_fw_file (fp, old_fs);
    return ret;
}


static int cst3xx_apk_fw_dowmload (struct i2c_client* client,
                                   unsigned char* pdata, int length)
{
    HYN_DBG ("cst3xx_apk_fw_dowmload enter.\n");
	p_online_firmware=pdata;
	g_online_mode=1;
	cst3xx_boot_update_fw(client,pdata);
	g_online_mode=0;
    return 0;
}

static ssize_t cst3xx_proc_read_foobar (struct file* page, char __user* user_buf, size_t count, loff_t* data)
{
    unsigned char buf[1024];
    int len = 0;
    int ret;

    HYN_DBG ("cst3xx is entering cst3xx_proc_read_foobar.\n");
	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

    if (CMDIndex == 0) {
        sprintf (buf, "Hynitron touchscreen driver 1.0\n");
        //strcpy(page,buf);
        len = strlen (buf);
        ret = copy_to_user (user_buf, buf, len);
    } else if (CMDIndex == 1) {
        buf[0] = 0xD1;
        buf[1] = 0x01;
        ret = cst3xx_i2c_write (cst3xx_i2c, buf, 2);
        if (ret < 0)
            return -1;

        mdelay (10);

        buf[0] = 0xD1;
        buf[1] = 0xE8;
        ret = cst3xx_i2c_read_register (cst3xx_i2c, buf, 4);
        if (ret < 0)
            return -1;

        g_cst3xx_tx = buf[0];
        g_cst3xx_rx = buf[2];
		
		HYN_DBG (" cst3xx_proc_read_foobar:g_cst3xx_tx:%d,g_cst3xx_rx:%d.\n", g_cst3xx_tx, g_cst3xx_rx);

        buf[0] = g_cst3xx_rx;
        buf[1] = g_cst3xx_tx;
        ret = copy_to_user (user_buf, buf, 2);
        len = 2;
        buf[0] = 0xD1;
        buf[1] = 0x09;
        ret = cst3xx_i2c_write (cst3xx_i2c, buf, 2);
    }

	 if (CMDIndex == 2 || CMDIndex == 3 || CMDIndex == 4) {
        unsigned short rx, tx;
        int data_len;
        rx = g_cst3xx_rx;
        tx = g_cst3xx_tx;
        data_len = rx * tx * 2 + 4 + (tx + rx) * 2 + rx + rx;

        if (CMDIndex == 2) { //read diff
            buf[0] = 0xD1;
            buf[1] = 0x0D;
        } else if (CMDIndex == 3) {   //rawdata
            buf[0] = 0xD1;
            buf[1] = 0x0A;
        } else if (CMDIndex == 4) {     //factory test
            buf[0] = 0xD1;
            buf[1] = 0x19;
            data_len = rx * tx * 4 + (4 + tx + rx) * 2;
        }

        ret = i2c_master_send (cst3xx_i2c, buf, 2);
        if (ret < 0) {
            HYN_ERR (" cst3xxWrite command raw/diff mode failed.error:%d.\n", ret);
            goto END;
        }
		g_unnormal_mode = 1;
		mdelay(14);
		
 		while(!gpio_get_value(CTP_IRQ_NUMBER));
        
        buf[0] = 0x80;
        buf[1] = 0x01;
        ret = cst3xx_i2c_write (cst3xx_i2c, buf, 2);
        if (ret < 0) {
            HYN_ERR (" cst3xxWrite command(0x8001) failed.error:%d.\n", ret);
            goto END;
        }

        ret = cst3xx_i2c_read (cst3xx_i2c, &buf[2], data_len);
        if (ret < 0) {
            HYN_ERR (" cst3xxRead raw/diff data failed.error:%d.\n", ret);
            goto END;
        }

        mdelay (1);

        buf[0] = 0xD1;
        buf[1] = 0x09;
        ret = cst3xx_i2c_write (cst3xx_i2c, buf, 2);
        if (ret < 0) {
            HYN_ERR (" cst3xxWrite command normal mode failed.error:%d.\n", ret);
            goto END;
        }	
		
		buf[0] = rx;
		buf[1] = tx;	
    	ret = copy_to_user(user_buf,buf,data_len + 2);
    	len = data_len + 2;

	}	

END:	

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	g_unnormal_mode = 0;
	CMDIndex = 0;		
	return len;
}

static ssize_t cst3xx_proc_write_foobar (struct file* file, const char __user* buffer, size_t count, loff_t* data)
{
    unsigned char cmd[128];
    unsigned char* pdata = NULL;
    int len;
    int ret;
    int length = 24 * 1024;

    if (count > 128)
        len = 128;
    else
        len = count;

    HYN_DBG ("cst3xx is entering cst3xx_proc_write_foobar .\n");

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif


    if (copy_from_user (cmd, buffer, len)) {
        HYN_ERR ("cst3xx copy data from user space failed.\n");
        return -EFAULT;
    }

    HYN_DBG ("cmd:%d......%d.......len:%d\r\n", cmd[0], cmd[1], len);

    if (cmd[0] == 0) {
        pdata = kzalloc (sizeof (char) * length, GFP_KERNEL);
        if (pdata == NULL) {
            HYN_DBG ("cst3xx zalloc GFP_KERNEL memory fail.\n");
            return -ENOMEM;
        }
		ret = cst3xx_read_fw_file (&cmd[1], pdata, &length);
        if (ret < 0) {
            if (pdata != NULL) {
                kfree (pdata);
                pdata = NULL;
            }
            return -EPERM;
        }

        ret = cst3xx_apk_fw_dowmload (cst3xx_i2c, pdata, length);
        if (ret < 0) {
            HYN_ERR (" cst3xx update firmware failed.\n");
            if (pdata != NULL) {
                kfree (pdata);
                pdata = NULL;
            }
            return -EPERM;
        }

        mdelay (50);
        cst3xx_firmware_info (cst3xx_i2c);

        if (pdata != NULL) {
            kfree (pdata);
            pdata = NULL;
        }
    }
	else if (cmd[0] == 2) {
        //cst3xx_touch_release();
        CMDIndex = cmd[1];
    } else if (cmd[0] == 3)
        CMDIndex = 0;
	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

    return count;
}


static const struct file_operations proc_tool_debug_fops = {
    .owner      = THIS_MODULE,
    .read           = cst3xx_proc_read_foobar,
    .write      = cst3xx_proc_write_foobar,
};

static int cst3xx_proc_fs_init (void)
{
    int ret;

    g_proc_dir = proc_mkdir (CST1XX_PROC_DIR_NAME, NULL);
    if (g_proc_dir == NULL) {
        ret = -ENOMEM;
        goto out;
    }

#if 0
    g_update_file = proc_create (CST1XX_PROC_FILE_NAME, 0777, g_proc_dir, &proc_tool_debug_fops);
    if (g_update_file == NULL) {
        ret = -ENOMEM;
        HYN_DBG ("proc_create CST1XX_PROC_FILE_NAME failed.\n");
        goto no_foo;
    }
#else
    g_update_file = proc_create_data (CST1XX_PROC_FILE_NAME, 0777 | S_IFREG, g_proc_dir, &proc_tool_debug_fops, (void*)cst3xx_i2c);
    if (NULL == g_update_file) {
        ret = -ENOMEM;
        HYN_ERR ("proc_create_data CST1XX_PROC_FILE_NAME failed.\n");
        goto no_foo;
    }

#endif
    return 0;

no_foo:
    remove_proc_entry (CST1XX_PROC_FILE_NAME, g_proc_dir);
out:
    return ret;
}


#ifdef HYN_SYSFS_NODE_EN
static ssize_t gtp_hyn_type_show (struct device* dev, struct device_attribute* attr, char* buf)
{
	char * devname = NULL;
	if (hyn_tp_flag == 1)
		devname  = "hyn_cst3xx";
	else
		devname  = "unknow";
    return snprintf(buf,PAGE_SIZE , "%s\n", devname);  


}

static ssize_t hyn_tpfwver_show (struct device* dev, struct device_attribute* attr, char* buf)
{
    ssize_t num_read_chars = 0;
    u8 buf1[20];
    int ret = -1;
    unsigned int firmware_version, module_version, project_version, chip_type, checksum;
    //struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    memset ((u8*)buf1, 0, 20);
    mutex_lock (&g_device_mutex);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

    firmware_version = 0;
    module_version = 0;
    project_version = 0;
    chip_type = 0;
    checksum = 0;
    buf1[0] = 0xD1;
    buf1[1] = 0x01;
    ret = cst3xx_i2c_write (cst3xx_i2c, buf1, 2);
    if (ret < 0)
        return -1;

    mdelay (10);
    buf1[0] = 0xD2;
    buf1[1] = 0x04;
    ret = cst3xx_i2c_read_register (cst3xx_i2c, buf1, 12);
    if (ret < 0)
        return -1;

    chip_type = buf1[3];
    chip_type <<= 8;
    chip_type |= buf1[2];
    project_version |= buf1[1];
    project_version <<= 8;
    project_version |= buf1[0];
    firmware_version = buf1[7];
    firmware_version <<= 8;
    firmware_version |= buf1[6];
    firmware_version <<= 8;
    firmware_version |= buf1[5];
    firmware_version <<= 8;
    firmware_version |= buf1[4];
    checksum = buf1[11];
    checksum <<= 8;
    checksum |= buf1[10];
    checksum <<= 8;
    checksum |= buf1[9];
    checksum <<= 8;
    checksum |= buf1[8];
    buf1[0] = 0xD1;
    buf1[1] = 0x09;
    ret = cst3xx_i2c_write (cst3xx_i2c, buf1, 2);
    num_read_chars = snprintf (buf, 128, "firmware_version: 0x%02X,module_version:0x%02X,project_version:0x%02X,chip_type:0x%02X,checksum:0x%02X .\n", firmware_version, module_version, project_version, chip_type, checksum);

    mutex_unlock (&g_device_mutex);
	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

    return num_read_chars;
}

static ssize_t hyn_tpfwver_store (struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    /*place holder for future use*/
    return -EPERM;
}


static ssize_t hyn_tprwreg_show (struct device* dev, struct device_attribute* attr, char* buf)
{
    /*place holder for future use*/
    return -EPERM;
}

static ssize_t hyn_tprwreg_store (struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    //struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    ssize_t num_read_chars = 0;
    int retval;
    long unsigned int wmreg = 0;
    u16 regaddr = 0xff;
    u8 valbuf[10] = {0};

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

    memset (valbuf, 0, sizeof (valbuf));
    mutex_lock (&g_device_mutex);
    num_read_chars = count - 1;
    if (num_read_chars != 2) {
        if (num_read_chars != 4) {
            HYN_ERR ("please input 2 or 4 character\n");
            goto error_return;
        }
    }

    memcpy (valbuf, buf, num_read_chars);
    retval = kstrtoul (valbuf, 16, &wmreg);

    if (0 != retval) {
        HYN_ERR ("%s() - ERROR: The given input was: \"%s\"\n", __func__, buf);
        goto error_return;
    }

    if (2 == num_read_chars) {
        /*read register*/
        regaddr = valbuf[0] << 8;
        regaddr |= valbuf[1];

        if (regaddr == 0x3838) //88-ascll
            //disable_irq (ts->irq);
            cst3xx_irq_disable();
        else if (regaddr == 0x3939) //99-ascll
            //enable_irq (ts->irq);
            cst3xx_irq_disable();
        else if (regaddr == 0x3737)
            hard_reset_chip(10);

        if (cst3xx_i2c_read_register (cst3xx_i2c, valbuf, num_read_chars) < 0)
            HYN_ERR ("Could not read the register(0x%02x).\n", regaddr);
        else
            HYN_ERR ("the register(0x%02x) is 0x%02x\n", regaddr, valbuf[0] );
    } else {
        regaddr = valbuf[0] << 8;
        regaddr |= valbuf[1];

        if (cst3xx_i2c_read_register (cst3xx_i2c, valbuf, num_read_chars) < 0)
            HYN_ERR ("Could not write the register(0x%02x)\n", regaddr);
        else
            HYN_ERR ("Write 0x%02x into register(0x%02x) successful\n", regaddr, valbuf[0]);
    }

error_return:
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

    mutex_unlock (&g_device_mutex);
    return count;
}

static ssize_t hyn_fwupdate_show (struct device* dev, struct device_attribute* attr, char* buf)
{
    /* place holder for future use */
    return -EPERM;
}

/*upgrade from *.i*/
static ssize_t hyn_fwupdate_store (struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    //struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    HYN_DBG ("hyn_fwupdate_store enter.\n");
    mutex_lock (&g_device_mutex);
    hyn_boot_update_fw (cst3xx_i2c);
    mutex_unlock (&g_device_mutex);
    return count;
}

static ssize_t hyn_fwupgradeapp_show (struct device* dev, struct device_attribute* attr, char* buf)
{
    /*place holder for future use*/
    return -EPERM;
}

/*upgrade from app.bin*/
static ssize_t hyn_fwupgradeapp_store (struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    char fwname[256];
    int ret;
    unsigned char* pdata = NULL;
    int length = 24 * 1024;
    //struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    HYN_DBG ("hyn_fwupgradeapp_store enter.\n");

    memset (fwname, 0, sizeof (fwname));
    sprintf (fwname, "/mnt/%s", buf);
    fwname[count - 1 + 5] = '\0';

    HYN_DBG ("fwname:%s.\n", fwname);

    pdata = kzalloc (sizeof (char) * length, GFP_KERNEL);
    if (pdata == NULL) {
        HYN_ERR ("hyn_fwupgradeapp_store GFP_KERNEL memory fail.\n");
        return -ENOMEM;
    }

    mutex_lock (&g_device_mutex);
    ret = cst3xx_read_fw_file (fwname, pdata, &length);
    if (ret < 0) {
        HYN_ERR ("cst3xx_read_fw_file fail.\n");
        if (pdata != NULL) {
            kfree (pdata);
            pdata = NULL;
        }
    }else {
        ret = cst3xx_apk_fw_dowmload (cst3xx_i2c, pdata, length);
        if (ret < 0) {
            HYN_ERR ("cst3xx_apk_fw_dowmload failed.\n");
            if (pdata != NULL) {
                kfree (pdata);
                pdata = NULL;
            }
        }
    }

    mutex_unlock (&g_device_mutex);

    HYN_DBG ("hyn_fwupgradeapp_store exit.\n");
    return count;
}

static ssize_t hyn_enable_show (struct device* dev, struct device_attribute* attr, char* buf)
{
    struct hyn_ts *ts = i2c_get_clientdata(cst3xx_i2c);
	printk("FILE:%s, FUNC:%s, LINE:%d, ts:%p\n", __FILE__, __func__, __LINE__, ts);

	return snprintf(buf, PAGE_SIZE , "%d\n", ts->is_enable);
}

/*upgrade from app.bin*/
static ssize_t hyn_enable_store (struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
	struct hyn_ts *ts = i2c_get_clientdata(cst3xx_i2c);
	printk("FILE:%s, FUNC:%s, LINE:%d, ts:%p\n", __FILE__, __func__, __LINE__, ts);
	int val, retval;

    retval = simple_strtoul(buf, NULL, 10);    
    
	printk("FILE:%s, FUNC:%s, LINE:%d, retval:%d\n", __FILE__, __func__, __LINE__, retval);
	if(ts->is_suspended == true){
		printk("FILE:%s, FUNC:%s, LINE:%d, is suspended break;\n", __FILE__, __func__, __LINE__);
		return count;
	}	

	if(ts->is_enable == !!retval){
		return count;
	}
	ts->is_enable = !!retval;
	printk("FILE:%s, FUNC:%s, LINE:%d, is_enable:%d\n", __FILE__, __func__, __LINE__, ts->is_enable);

	if(ts->is_enable){
		printk("FILE:%s, FUNC:%s, LINE:%d, enable\n", __FILE__, __func__, __LINE__);
			
		hard_reset_chip(30);
		cst3xx_irq_enable();
		
	#ifdef CONFIG_TP_ESD_PROTECT
		if(cst3xx_esd_workqueue!=NULL)
		cst3xx_esd_switch(SWITCH_ESD_ON);
	#endif

	}else{
		printk("FILE:%s, FUNC:%s, LINE:%d, disable\n", __FILE__, __func__, __LINE__);
		cst3xx_irq_disable();

		cancel_work_sync(&ts->work);
		flush_workqueue(ts->wq);
		mdelay(5);	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif
		ts->is_enable = 0;

	}
	return count;
}

/*sysfs */
/*get the fw version
 *example:cat hyntpfwver
 */
static DEVICE_ATTR (gtp_type, S_IRUGO , gtp_hyn_type_show,
						NULL);

static DEVICE_ATTR (hyntpfwver, S_IRUGO | S_IWUSR, hyn_tpfwver_show,
                    hyn_tpfwver_store);

/*upgrade from *.i
 *example: echo 1 > hynfwupdate
 */
static DEVICE_ATTR (hynfwupdate, S_IRUGO | S_IWUSR, hyn_fwupdate_show,
                    hyn_fwupdate_store);

/*read and write register
 *read example: echo 88 > hyntprwreg ---read register 0x88
 *write example:echo 8807 > hyntprwreg ---write 0x07 into register 0x88
 *
 *note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
 */
static DEVICE_ATTR (hyntprwreg, S_IRUGO | S_IWUSR, hyn_tprwreg_show,
                    hyn_tprwreg_store);


/*upgrade from app.bin
 *example:echo "*_app.bin" > hynfwupgradeapp
 */
static DEVICE_ATTR (hynfwupgradeapp, S_IRUGO | S_IWUSR, hyn_fwupgradeapp_show,
                    hyn_fwupgradeapp_store);


static DEVICE_ATTR (enable, S_IRUGO | S_IWUSR, hyn_enable_show,
                    hyn_enable_store);

/*add your attr in here*/
static struct attribute* hyn_attributes[] = {
    &dev_attr_gtp_type.attr,
    &dev_attr_hyntpfwver.attr,
    &dev_attr_hynfwupdate.attr,
    &dev_attr_hyntprwreg.attr,
    &dev_attr_hynfwupgradeapp.attr,
    &dev_attr_enable.attr,
    NULL
};

static struct attribute_group hyn_attribute_group = {
    .attrs = hyn_attributes
};
/*create sysfs for debug*/

int hyn_create_sysfs (struct i2c_client* client)
{
    int err;
    cst3xx_i2c = client;

    if ((k_obj = kobject_create_and_add ("gtp_hynitron", NULL)) == NULL )
        HYN_DBG ("hynitron_debug sys node create error.\n");

    err = sysfs_create_group (k_obj, &hyn_attribute_group);
    if (0 != err) {
        HYN_ERR ("%s() - ERROR: sysfs_create_group() failed.\n", __func__);
        sysfs_remove_group (k_obj, &hyn_attribute_group);
        return -EIO;
    } else {
        mutex_init (&g_device_mutex);
        HYN_DBG ("cst3xx:%s() - sysfs_create_group() succeeded.\n", __func__);
    }

    return err;
}

void hyn_release_sysfs (struct i2c_client* client)
{
    sysfs_remove_group (k_obj, &hyn_attribute_group);
    mutex_destroy (&g_device_mutex);
}
#endif //end of HYN_SYSFS_NODE_EN

#endif//end of ANDROID_TOOL_SURPORT


int report_flag = 0;
static void hyn_ts_xy_worker(struct work_struct *work)
{
	u8  buf[30];
	u8  i2c_buf[8];
	u8  key_status, key_id, finger_id, sw;
	s32 input_x = 0; 
	s32 input_y = 0; 
	s32 temp = 0; 
	s32 input_w = 0;
    s32  cnt_up, cnt_down;
	int  i, ret, idx; 
	int  cnt, i2c_len, len_1, len_2;
	
	struct hyn_ts *ts = container_of(work, struct hyn_ts,work);

	//HYN_INF("hyn_ts_xy_worker enter.\r\n");
#ifdef CONFIG_TP_ESD_PROTECT
	if(i2c_lock_flag!=0) return;
	else i2c_lock_flag=1;
#endif

	cst3xx_irq_disable();
    //start read iic touch data
	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst3xx_i2c_read_register(cst3xx_i2c, buf, 7);
	if(ret < 0){
		HYN_INF("cst3xx_i2c_read_register:0xD000 failed.\r\n");
		goto OUT_PROCESS;
	}
	
	if(buf[0] == 0xAB) {
		HYN_INF("data is not valid,buf[0]:%d \r\n",buf[0]);
		goto OUT_PROCESS;
	}
		
	if(buf[6] != 0xAB) {
		HYN_INF("data is not valid,buf[6]:%d \r\n",buf[6]);
		goto OUT_PROCESS;
	}
	
	cnt = buf[5] & 0x7F;
	if(cnt > MAX_FINGERS) goto OUT_PROCESS;
	else if(cnt==0)       goto CLR_POINT;

	if(buf[5] == 0x80) {
		key_status = buf[0];
		key_id = buf[1];		
		goto KEY_PROCESS;
	} 
	else if(cnt == 0x01){
		goto FINGER_PROCESS;
	} 
	else 
	{
		#ifdef TRANSACTION_LENGTH_LIMITED
		if((buf[5]&0x80) == 0x80) //key
		{
			i2c_len = (cnt - 1)*5 + 3;
			len_1   = i2c_len;
			for(idx=0; idx<i2c_len; idx+=6)
			{
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6)
				{
					len_2  = 6;
					len_1 -= 6;
				}
				else 
				{
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(cst3xx_i2c, i2c_buf, len_2);
    			if(ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++)
				{
                   buf[5+idx+i] = i2c_buf[i];
				}
			}
			
			i2c_len   += 5;
			key_status = buf[i2c_len - 3];
			key_id     = buf[i2c_len - 2];
		} 
		else 
		{			
			i2c_len = (cnt - 1)*5 + 1;
			len_1   = i2c_len;
			
			for(idx=0; idx<i2c_len; idx+=6)
			{
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6)
				{
					len_2  = 6;
					len_1 -= 6;
				}
				else 
				{
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(cst3xx_i2c, i2c_buf, len_2);
    			if (ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++)
				{
                   buf[5+idx+i] = i2c_buf[i];
				}
			}			
			i2c_len += 5;
		}
		#else
		if ((buf[5]&0x80) == 0x80) 
		{
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 3;
			ret = cst3xx_i2c_read_register(cst3xx_i2c, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
			key_status = buf[i2c_len - 3];
			key_id = buf[i2c_len - 2];
		} 
		else 
		{			
			buf[5] = 0xD0;
			buf[6] = 0x07;			
			i2c_len = (cnt - 1)*5 + 1;
			ret = cst3xx_i2c_read_register(cst3xx_i2c, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
		}
		#endif

		if (buf[i2c_len - 1] != 0xAB) 
		{
			goto OUT_PROCESS;
		}
	}	

	if((cnt>0)&&(key_status&0x80))  //both key and point
	{
        if(report_flag==0xA5) goto KEY_PROCESS; 
	}
	
FINGER_PROCESS:
	
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(cst3xx_i2c, i2c_buf, 3);
	if (ret < 0)
	{
		HYN_ERR("hyn send read touch info ending failed.\r\n"); 
		hard_reset_chip(20);
	}
	
	idx = 0;
       cnt_up = 0;
       cnt_down = 0;
	for (i = 0; i < cnt; i++) 
	{
		input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
		input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));	
		input_w = (unsigned int)(buf[idx + 4]);
		sw = (buf[idx] & 0x0F) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0F;
//		if(input_x > 106)
//			input_x -= 106;     //yanjb 20210104  Reporting from 0
//		else
//			input_x = 0;
		if(1 == exchange_x_y_flag){
            temp=input_x;
			input_x=input_y;
			input_y=temp;
        }
		
        if(1 == revert_x_flag){
            input_x = SCREEN_MAX_X - input_x;
        }
        if(1 == revert_y_flag){
            input_y = SCREEN_MAX_Y - input_y;
        }
		
        //HYN_INF("==yy==Point x:%d, y:%d, id:%d, sw:%d.\r\n", input_x, input_y, finger_id, sw);

		if (sw == 0x03) 
        {
			cst3xx_touch_down(ts->input, finger_id, input_x, input_y, input_w);
            cnt_down++;
        }
		else 
        {
            cnt_up++;
            #ifdef ICS_SLOT_REPORT
			cst3xx_touch_up(ts->input, finger_id);
            #endif
        }
		idx += 5;
	}
    
    #ifndef ICS_SLOT_REPORT
    if((cnt_up>0) && (cnt_down==0))
        cst3xx_touch_up(ts->input, 0);
    #endif

	if(cnt_down==0)  report_flag = 0;
	else report_flag = 0xCA;

    input_sync(ts->input);
	goto END;

KEY_PROCESS:
	
	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(cst3xx_i2c, i2c_buf, 3);
	if (ret < 0)
	{
		HYN_INF("hyn send read touch info ending failed.\r\n"); 
		hard_reset_chip(20);
	}

    #ifdef TPD_HAVE_BUTTON
	if(key_status&0x80)
	{
        if((key_status&0x7F)==0x03) 
        {
    	    i = (key_id>>4)-1;
    		cst3xx_touch_down(ts->input, hyn_key_data[i].x, hyn_key_data[i].y, 0, 0);
			report_flag = 0xA5;
		}
    	else
    	{
            cst3xx_touch_up(ts->input, 0);
			report_flag = 0;			
    	}
	}
	#endif
	
	input_sync(ts->input);
	
	goto END;

CLR_POINT:
	
#ifdef SLEEP_RESUME_CLEAR_POINT
    #ifdef ICS_SLOT_REPORT
    for(i=0; i<=MAX_CONTACTS; i++)
    {	
        input_mt_slot(ts->input, i);
        input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
        input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
    }
    #else	
        input_mt_sync(ts->input);
    #endif
        input_sync(ts->input);
#endif	

OUT_PROCESS:
	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0xAB;
	ret = cst3xx_i2c_write(cst3xx_i2c, buf, 3);
	if (ret < 0)
	{
		HYN_INF("hyn send read touch info ending failed.\r\n"); 
		hard_reset_chip(20);
	}
	
END:
		
schedule:
#ifdef CONFIG_TP_ESD_PROTECT
	i2c_lock_flag = 0;
i2c_lock_schedule:
#endif


	cst3xx_irq_enable();

}




irqreturn_t hyn_ts_irq(int irq, void *dev_id)
{
	struct hyn_ts *ts = (struct hyn_ts *)dev_id;
	//HYN_INF(DEBUG_INT_INFO,"==========hyn_ts_irq============\n");
	queue_work(ts->wq, &ts->work);
	return IRQ_HANDLED;
}


static int hyn_ts_init_ts(struct i2c_client *client, struct hyn_ts *ts)
{
	struct input_dev *input_device;
	int  ret = 0;
#ifdef HAVE_TOUCH_KEY
	int i= 0;
#endif
	HYN_INF("hyn enter %s\n", __func__);

	ts->prev_touches = 0;

	input_device = input_allocate_device();
	if (!input_device) {
		ret = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = CST3XX_I2C_NAME;
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

#ifdef ICS_SLOT_REPORT
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_REP, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_mt_init_slots(input_device, (MAX_CONTACTS+1));
#else

	input_set_abs_params(input_device,ABS_MT_TRACKING_ID, 0, (MAX_CONTACTS+1), 0, 0);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_device->propbit);
	input_device->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#ifdef HAVE_TOUCH_KEY
		//input_device->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 0; i < MAX_KEY_NUM; i++)
		set_bit(key_array[i] & KEY_MAX, input_device->keybit);

#endif

	set_bit(ABS_MT_POSITION_X, input_device->absbit);
	set_bit(ABS_MT_POSITION_Y, input_device->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_device->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_device->absbit);

	input_set_abs_params(input_device,ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_device,ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_device,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_device,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	ts->wq = create_singlethread_workqueue("cst3xx_touch_workqueue");
	if (!ts->wq) {
//		HYN_ERR(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}
	flush_workqueue(ts->wq);

	INIT_WORK(&ts->work, hyn_ts_xy_worker);

	ret = input_register_device(input_device);
	if (ret<0)
		goto error_unreg_device;

	config_info.dev = &(ts_init->input->dev);
	ret = input_request_int(&(config_info.input_type), hyn_ts_irq,
				CTP_IRQ_MODE, ts_init);
	if (ret<0) {
		HYN_INF("cst3xx_init_events: request irq failed\n");
	}
	ts_init->is_suspended = false;
	ts_init->is_enable = 1;
	cst3xx_irq_disable();
	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	return ret;
}

#ifdef CONFIG_PM
static int hyn_ts_suspend(struct device *dev)
{

    //struct i2c_client *client;
	//printk("FILE:%s, FUNC:%s, LINE:%d, cst3xx_i2c:%p\n", __FILE__, __func__, __LINE__, cst3xx_i2c);
    struct hyn_ts *ts = i2c_get_clientdata(cst3xx_i2c);

	HYN_INF("hyn is entering hyn_ts_suspend \n");
	cst3xx_irq_disable();
		//printk("FILE:%s, FUNC:%s, LINE:%d, ts:%p\n", __FILE__, __func__, __LINE__, ts);

#ifndef CONFIG_HAS_EARLYSUSPEND
    ts->is_suspended = true;
#endif
	cst3xx_enter_sleep(cst3xx_i2c);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif
	//printk("FILE:%s, FUNC:%s, LINE:%d, ts:%p\n", __FILE__, __func__, __LINE__, ts);
    return 0;
	
}

static int hyn_ts_resume(struct device *dev)
{
	//struct i2c_client *client;
	//printk("FILE:%s, FUNC:%s, LINE:%d, cst3xx_i2c:%p\n", __FILE__, __func__, __LINE__, cst3xx_i2c);

	struct hyn_ts *ts = i2c_get_clientdata(cst3xx_i2c);
#ifndef CONFIG_HAS_EARLYSUSPEND
		//printk("FILE:%s, FUNC:%s, LINE:%d, ts:%p\n", __FILE__, __func__, __LINE__, ts);

	ts->is_suspended = false;
#endif
    HYN_INF("hyn is entering hyn_ts_resume \n");
		
	hard_reset_chip(30);
	cst3xx_irq_enable();
	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
	
	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hyn_ts_early_suspend(struct early_suspend *h)
{
	struct hyn_ts *ts = container_of(h, struct hyn_ts, early_suspend);
    HYN_INF("hyn is entering hyn_ts_early_suspend \n");

	if(!ts->is_enable){
		printk("FILE:%s, FUNC:%s, LINE:%d, is disable\n", __FILE__, __func__, __LINE__);
		ts->is_suspended = true;
		return;
	}
	cst3xx_irq_disable();

	if(ts->is_suspended == false){
		cancel_work_sync(&ts->work);
		flush_workqueue(ts->wq);
		mdelay(5);	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif
		ts->is_suspended = true;
	}
}

static void hyn_ts_late_resume(struct early_suspend *h)
{
	struct hyn_ts *ts = container_of(h, struct hyn_ts, early_suspend);
    HYN_INF("hyn is entering hyn_ts_late_resume \n");
	
	hard_reset_chip(30);
	cst3xx_irq_enable();
	
	if(ts->is_suspended == true){

	cancel_work_sync(&ts->work);
	flush_workqueue(ts->wq);
	mdelay(10); 
	
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

#ifdef SLEEP_RESUME_CLEAR_POINT
	mdelay(10); 
	#ifdef ICS_SLOT_REPORT
	int i;
		for(i =0;i<=MAX_CONTACTS;i++)
		{	
			input_mt_slot(ts->input, i);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		}
	#else	
		input_mt_sync(ts->input);
	#endif
		input_sync(ts->input);	
#endif
		ts->is_suspended = false;
	}
}
#endif


static int  hyn_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hyn_ts *ts;
	int ret = 0;

	HYN_INF("hyn_ts_probe enter %s\n", __func__);
	input_set_power_enable(&(config_info.input_type),1);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		HYN_INF("I2C functionality not supported\n");
		return -ENODEV;
	}
 
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts){
	        HYN_ERR("allocate data fail!\n");
		return -ENOMEM;
	}	
	cst3xx_i2c = client;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->device_id = id->driver_data;
	ts->is_suspended = false;
	ts->int_pending = false;
	mutex_init(&ts->sus_lock);
	ts_init = ts;
	hard_reset_chip(20);

	ret = hyn_boot_update_fw(cst3xx_i2c);
	if(ret < 0){
		HYN_ERR(" hyn_boot_update_fw failed.\n");
		return -1;
	}	
	ret = hyn_ts_init_ts(client, ts);
	if (ret < 0) {
		HYN_ERR( "hyn_ts_init_ts failed\n");
		goto error_mutex_destroy;
	}
	device_enable_async_suspend(&client->dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = hyn_ts_early_suspend;
	ts->early_suspend.resume = hyn_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef ANDROID_TOOL_SURPORT
    cst3xx_i2c = client;
    ret = cst3xx_proc_fs_init();
    if (ret < 0)
        HYN_ERR (" cst3xx hyn create cst3xx proc fs failed.\n");

#ifdef HYN_SYSFS_NODE_EN
    hyn_create_sysfs (client);
#endif

#endif


#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_esd_workqueue = create_singlethread_workqueue("cst3xx_esd_workqueue");
	if (cst3xx_esd_workqueue == NULL)
		printk("  cst3xxcreate cst2xx_esd_workqueue failed!");
	else{
		cst3xx_init_esd_protect();
		cst3xx_esd_switch(SWITCH_ESD_ON);
	}
#endif
	cst3xx_irq_enable();
    hyn_tp_flag=1;
	HYN_INF("[CST3XX] End %s\n", __func__);
	return 0;
  	
error_mutex_destroy:
	mutex_destroy(&ts->sus_lock);
	input_free_device(ts->input);
	kfree(ts);
	return ret;
}

static int  hyn_ts_remove(struct i2c_client *client)
{
	struct hyn_ts *ts = i2c_get_clientdata(client);
	HYN_INF("==hyn_ts_remove=\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	input_free_int(&(config_info.input_type), ts);
	destroy_workqueue(ts->wq);
#ifdef CONFIG_TP_ESD_PROTECT
	destroy_workqueue(cst3xx_esd_workqueue);
	cancel_work_sync(&ts_init->work);   //esd_check_work
#endif
	input_unregister_device(ts->input);
	mutex_destroy(&ts->sus_lock);
	//input_set_power_enable(&(config_info.input_type),0);
	kfree(ts->touch_data);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id hyn_ts_id[] = {
	{CST3XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, hyn_ts_id);

//yanjb 20210102
static const struct dev_pm_ops cst3xx_pm_ops = {
	.suspend = hyn_ts_suspend,
	.resume = hyn_ts_resume,
	};
#define CST3XX_PM_OPS (&cst3xx_pm_ops)
/*****************************/

static struct i2c_driver hyn_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = CST3XX_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = CST3XX_PM_OPS,
	},
	.probe		= hyn_ts_probe,
	.remove		= hyn_ts_remove,
	.id_table		= hyn_ts_id,
	.address_list	= normal_i2c,
	.detect   = ctp_detect,
#ifdef CONFIG_PM
//	.suspend    = hyn_ts_suspend,
//	.resume	    = hyn_ts_resume,
#endif
};

static int ctp_get_system_config(void)
{  
    ctp_print_info(config_info);

	fwname = config_info.name;
	HYN_INF("%s:fwname:%s\n",__func__,fwname);	 		
    twi_id = config_info.twi_id;
    screen_max_x = config_info.screen_max_x;
    screen_max_y = config_info.screen_max_y;
    revert_x_flag = config_info.revert_x_flag;
    revert_y_flag = config_info.revert_y_flag;
    exchange_x_y_flag = config_info.exchange_x_y_flag;
	if((screen_max_x == 0) || (screen_max_y == 0)){
           HYN_INF("%s:read config error!\n",__func__);
           return 0;
    }
    return 1;
}
static int __init hyn_ts_init(void)
{

	int ret = -1;
	HYN_INF("******************hyn_ts_init********************************\n");
#if 0	
	if (input_fetch_sysconfig_para(&(config_info.input_type))) {
		HYN_INF("%s: ctp_fetch_sysconfig_para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(config_info.input_type));
		if (0 != ret) {
			HYN_INF("%s:ctp_ops.init_platform_resource err. \n", __func__);    
		}
	}
#endif
//yanjb20210102
	if (!input_sensor_startup(&(config_info.input_type))) {
		ret = input_sensor_init(&(config_info.input_type));
		if (ret != 0) {
			pr_err("%s:ctp_ops.input_sensor_init err.\n", __func__);
			goto init_err;
		}
		input_set_power_enable(&(config_info.input_type), 1);
	} else {
		pr_err("%s: input_ctp_startup err.\n", __func__);
		return 0;
	}
//
	if (config_info.ctp_used == 0) {
		HYN_INF("*** ctp_used set to 0 !\n");
		HYN_INF("*** if use ctp,please put the sys_config.fex ctp_used set to 1. \n");
		return 0;
	}
	if (!ctp_get_system_config()) {
		HYN_INF("%s:read config fail!\n",__func__);
		return ret;
	}
	
	input_set_power_enable(&(config_info.input_type),1);
	mdelay(20); 
	hard_reset_chip(20);   //yanjb20210212 reset
	
	hyn_ts_driver.detect = ctp_detect;
	ret = i2c_add_driver(&hyn_ts_driver);
	HYN_INF("****************************************************************\n");
	return ret;
//yanjb20210102
init_err:
	input_set_power_enable(&(config_info.input_type), 0);
	return ret;

}

static void __exit hyn_ts_exit(void)
{
	HYN_DBG("==hyn_ts_exit==\n");
	i2c_del_driver(&hyn_ts_driver);
//	input_free_platform_resource(&(config_info.input_type));
//	input_set_power_enable(&(config_info.input_type), 0);
	input_sensor_free(&(config_info.input_type));                         //yanjb20210102
	input_set_power_enable(&(config_info.input_type), 0);
	return;
}

late_initcall(hyn_ts_init);
module_exit(hyn_ts_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CST3XX touchscreen controller driver");
MODULE_AUTHOR("Tim.Tan");
MODULE_ALIAS("platform:hyn_ts");

