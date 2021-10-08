/*
* Copyright (C) 2019 MediaTek Inc.
*
* Version: v1.0.0
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>

/* device tree should be defined in flashlight-dt.h */
#ifndef AW36514_DTNAME
#define AW36514_DTNAME "awinic,aw36514"
#endif
#ifndef AW36514_DTNAME_I2C
#define AW36514_DTNAME_I2C "awinic,aw36514"
#endif
#define AW36514_NAME "aw36514"

#define AW36514_VERSION "v1.0.0"

#define AW36514_LEVEL_TORCH          7

/*define ioctl*/
#define FLASH_IOC_SET_ON	1
#define FLASH_IOC_SET_OFF	0

/* define mutex and work queue */
static DEFINE_MUTEX(aw36514_mutex);

/* define torch_enable_gpio*/
static int torch_enable_gpio,gpio_flash;

/* define i2c */
static struct i2c_client *aw36514_i2c_client;
static struct cdev *aw36514_cdev;
static dev_t aw36514_devid;
static struct class *aw36514_class;
static struct device *aw36514_dev;

unsigned char get_brightness(char *b );
int aw36514_init(unsigned char brightness);
unsigned char get_brightness_from_fs(char *b );

/* platform data
* torch_pin_enable: TX1/TORCH pin isa hardware TORCH enable
* pam_sync_pin_enable: TX2 Mode The ENVM/TX2 is a PAM Sync. on input
* thermal_comp_mode_enable: LEDI/NTC pin in Thermal Comparator Mode
* strobe_pin_disable: STROBE Input disabled
* vout_mode_enable: Voltage Out Mode enable
*/
struct aw36514_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

/* aw36514 chip data */
struct aw36514_chip_data {
	struct i2c_client *client;
	struct aw36514_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

/* i2c wrapper function */
static int aw36514_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw36514_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

static int aw36514_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw36514_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

/***************************************************************************/
/*AW36514 fs test*/
/***************************************************************************/
#ifdef CONFIG_COWBOY_BOARD

static ssize_t show_led1_brightness(struct device *dev,
                  struct device_attribute *attr, char *buf)   
{
	
	int val;
	if(torch_enable_gpio > 0){
		val = aw36514_read_reg(aw36514_i2c_client,0x05);
	}

    return sprintf(buf, "%d\n", val);
}

static ssize_t show_led2_brightness(struct device *dev,
  struct device_attribute *attr, char *buf)   
{
	
	int val;
	if(torch_enable_gpio > 0){
		val = aw36514_read_reg(aw36514_i2c_client,0x06);
	}
	return sprintf(buf, "%d\n", val);
}

static ssize_t set_led1_brightness(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t len)  
{
	//pr_info("set_my_brightness[%d]:%s \n",strlen(buf),buf);
	unsigned char value =128 ;
	sscanf(buf,"%d",&value);

	pr_info("set led1 brightness = %d\n",value);
	if(torch_enable_gpio > 0){
		aw36514_write_reg(aw36514_i2c_client,0x05,value);
	}
    return len;
}

 static ssize_t set_led2_brightness(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)	
 {
	 //pr_info("set_my_brightness[%d]:%s \n",strlen(buf),buf);
	 unsigned char value =128 ;
	 sscanf(buf,"%d",&value);

	 pr_info("set led2 brightness = %d\n",value);
	 if(torch_enable_gpio > 0){
		 aw36514_write_reg(aw36514_i2c_client,0x06,value);
	 }
	 return len;
 }

#endif

static ssize_t show_my_switch(struct device *dev,
                  struct device_attribute *attr, char *buf)   //cat命令时,将会调用该函数
{
	//pr_info("show_my_switch:\n");

	int val;
	if(torch_enable_gpio > 0){
		val = gpio_get_value(torch_enable_gpio);
	}

    return sprintf(buf, "%d\n", val);
}

#ifdef CONFIG_COWBOY_BOARD			  
extern int scan_leds_init(struct scan_leds_chip_data *chip, int enable);
#endif
static ssize_t set_my_switch(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t len)   
{
	printk("set_my_brightness[%d]:%s \n",strlen(buf),buf);

	if(buf[0] == '0'){
	#ifdef CONFIG_COWBOY_BOARD	
		scan_leds_init(NULL,0);//close LED output (IO Driver);
		return len;
	#endif
		gpio_direction_output(torch_enable_gpio, 0); //1-enable,0-disable
		aw36514_write_reg(aw36514_i2c_client,0x01,0x00);//close all output
		pr_info("aw36514 colse output torch_enable_gpio =%d\n",torch_enable_gpio);
		
	}
	else if(buf[0] == '1'){
		#ifdef CONFIG_COWBOY_BOARD	
			scan_leds_init(NULL,1);//open LED output (IO Driver);
			return len;
		#endif
		if(torch_enable_gpio > 0){
			aw36514_init(1);
		}
	}
    return len;
}

 static ssize_t show_my_brightness(struct device *dev,
				   struct device_attribute *attr, char *buf)   //cat命令时,将会调用该函数
 {
	 //pr_info("show_my_brightness:\n");
 
	 int val;
	 if(torch_enable_gpio > 0){
		 val = aw36514_read_reg(aw36514_i2c_client,0x06);
	 }
 
	 return sprintf(buf, "%d\n", val);
 }


				   
 static ssize_t set_my_brightness(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)	 //echo命令时,将会调用该函数
 {
	 //pr_info("set_my_brightness[%d]:%s \n",strlen(buf),buf);
 
	 unsigned char b = get_brightness_from_fs(buf); 
	 

	 #ifdef CONFIG_COWBOY_BOARD
	 printk("set brightness is %d\n",b);	
	 if(b > 0){
	 	//scan_leds_init(NULL, b);		//old  1:R   2:W  0:close
		scan_leds_init(NULL, b >=2 ? 1 : 2);           //new  2:R   1:W  0:close
	 }else {
		scan_leds_init(NULL,0);
  	 }
	 return len;
	 #endif

	 if(b>1){
	 			b=1;	
	 }
	 if(torch_enable_gpio > 0){
		 aw36514_init(b);
	 }
	 return len;
 }


#ifdef CONFIG_COWBOY_BOARD	

 static ssize_t set_led1_switch(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned char value =0,temp;
	sscanf(buf,"%d",&value);

	temp = aw36514_read_reg(aw36514_i2c_client,0x01);
	pr_info("set_led1_switch = %d\n",value);
	aw36514_write_reg(aw36514_i2c_client,0x01,value?((0x01 << 0) |temp ):(((~(0x01 << 0)) & temp)));
	return len;
}
				  
static ssize_t set_led2_switch(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned char value =0,temp;
	sscanf(buf,"%d",&value);

	temp = aw36514_read_reg(aw36514_i2c_client,0x01);
	pr_info("set_led2_switch = %d\n",value);
	aw36514_write_reg(aw36514_i2c_client,0x01,value?((0x01 << 1) |temp ):(((~(0x01 << 1)) & temp)));
	return len;
}


ssize_t set_flash_led_switch(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned char value =0,temp;
	sscanf(buf,"%d",&value);

	printk("FILE:%s, FUNC:%s, LINE:%d, value:%d, gpio_flash:%d\n", __FILE__, __func__, __LINE__, value, gpio_flash);
	if(value){
		if(gpio_flash > 0)
			gpio_direction_output(gpio_flash, 1); 
	}else
	{
		if(gpio_flash > 0)
			gpio_direction_output(gpio_flash, 0);
	}
	return len;
}
EXPORT_SYMBOL(set_flash_led_switch);
#endif

static DEVICE_ATTR(switch, 0660, show_my_switch, set_my_switch);				
static DEVICE_ATTR(brightness, 0660, show_my_brightness, set_my_brightness);	
#ifdef CONFIG_COWBOY_BOARD	
static DEVICE_ATTR(led_switch1, 0660, NULL, set_led1_switch);
static DEVICE_ATTR(led_switch2, 0660, NULL, set_led2_switch);
static DEVICE_ATTR(flash_led, 0660, NULL, set_flash_led_switch);//
static DEVICE_ATTR(led_brightness1, 0660, show_led1_brightness, set_led1_brightness);
static DEVICE_ATTR(led_brightness2, 0660, show_led2_brightness, set_led2_brightness);
#endif


static struct attribute *aw_attributes[] = {
	&dev_attr_switch.attr,
	&dev_attr_brightness.attr,
#ifdef CONFIG_COWBOY_BOARD	
	&dev_attr_led_switch1.attr,
	&dev_attr_led_switch2.attr,
	&dev_attr_led_brightness1.attr,
	&dev_attr_led_brightness2.attr,
	&dev_attr_flash_led.attr,
#endif
	NULL
};

static struct attribute_group aw_attribute_group = {
	.name = "attr",
	.attrs = aw_attributes
};

/* flashlight init */
int aw36514_init(unsigned char brightness)
{
	int ret,val;

#ifdef CONFIG_COWBOY_BOARD 
	//pr_info("no use this aw36514 driver return\n");
	return 0;
#endif
	//usleep_range(2000, 2500);
	pr_info("aw36514_init brightness %d\n",brightness);
	if( brightness > 10 )
	{
		pr_info("aw36514_init brightness oversize!\n");
		return -1;
	}	
	/*init*/
	gpio_direction_output(torch_enable_gpio, 0); //1-enable,0-disable 
#ifdef CONFIG_COWBOY_BOARD	
	ret = aw36514_write_reg(aw36514_i2c_client,0x01,0x0B);//0x0B:1011 10:torch mode  11： LED1 LED2： ON
#else
	ret = aw36514_write_reg(aw36514_i2c_client,0x01,0x02);//warning: 0x01 reg must set 0x02 for ailabs_cowboy
#endif	
	ret = aw36514_write_reg(aw36514_i2c_client,0x05,0xff&brightness);
	ret = aw36514_write_reg(aw36514_i2c_client,0x06,0xff&brightness);

	val = aw36514_read_reg(aw36514_i2c_client,0x09);
	ret = aw36514_write_reg(aw36514_i2c_client,0x09,val &~ (0x01<<0));
	//val = aw36514_read_reg(aw36514_i2c_client,0x09);

	val = aw36514_read_reg(aw36514_i2c_client,0x01);
	ret = aw36514_write_reg(aw36514_i2c_client,0x01,val | (0x01<<4));
	//val = aw36514_read_reg(aw36514_i2c_client,0x01);

	gpio_direction_output(torch_enable_gpio, 1); //1-enable,0-disable 
	//pr_info("aw36514_read_regb: %d \n", aw36514_read_reg(aw36514_i2c_client,0x0b));
	return 0;
}

unsigned char get_brightness_from_fs(char *b )
{
	unsigned char brightness = 0;
	int length = strlen(b) -1;

	//pr_info("length: %d \n",length);

	if(length == 1){
		brightness = b[0]-48;
	}else if(length == 2){
		brightness = (b[0]-48)*10 + (b[1]-48);
	}else if(length == 3 && b[0] < 51 && b[1] < 53 && b[2] < 53){
		brightness = (b[0]-48)*100 + (b[1]-48)*10 + (b[2] -48);
	}
	//pr_info("get brightness= %d \n",brightness);
	return brightness;
}

unsigned char get_brightness(char *b )
{
	unsigned char brightness = 0;
	int length = strlen(b);

	//pr_info("length: %d \n",length);
	if(length == 1){
		brightness = b[0]-48;
	}else if(length == 2){
		brightness = (b[0]-48)*10 + (b[1]-48);
	}else if(length == 3 && b[0] < 51 && b[1] < 53 && b[2] < 53){
		brightness = (b[0]-48)*100 + (b[1]-48)*10 + (b[2] -48);
	}
	//pr_info("get brightness= %d \n",brightness);
	return brightness;
}
/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static long aw36514_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char *p = (char *)arg;
	printk("iotctl data %s\n",p);
	switch (cmd) {
	case FLASH_IOC_SET_ON:
		//pr_info("FLASH_IOC_SET_ON\n");
		if(torch_enable_gpio > 0)	
			aw36514_init(1);
		break;
	case FLASH_IOC_SET_OFF:
		//pr_info("FLASH_IOC_SET_OFF\n");
		if(torch_enable_gpio > 0)
			gpio_direction_output(torch_enable_gpio, 0);
		break;
	default:
		//pr_info("No such command(%d):\n",cmd);
		return -ENOTTY;
	}

	return 0;
}

/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36514_chip_init(struct aw36514_chip_data *chip)
{
	/* NOTE: Chip initialication move to
	*"set driver" operation for power saving issue.
	* aw36514_init();
	*/
	aw36514_init(1);
	return 0;
}

/***************************************************************************/
/*AW36514 Debug file */
/***************************************************************************/
static ssize_t
aw36514_get_reg(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;

	for (i = 0; i < 0x0E; i++) {
		reg_val = aw36514_read_reg(aw36514_i2c_client, i);
		len += snprintf(buf+len, PAGE_SIZE-len,
			"reg0x%2X = 0x%2X\n", i, reg_val);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36514_set_reg(struct device *cd,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw36514_write_reg(aw36514_i2c_client, databuf[0], databuf[1]);
	return len;
}

static DEVICE_ATTR(reg, 0660, aw36514_get_reg, aw36514_set_reg);

static int aw36514_open(struct inode *inode, struct file *file)
{
	/* Actual behavior move to set driver function */
	/* since power saving issue */
	//pr_info("aw36514_open\n");
	if (aw36514_i2c_client)
		file->private_data = aw36514_i2c_client;
	else {
		pr_info("aw36514_i2c_client null\n");
		return -ENODEV;
	}
	return 0;
}
static int aw36514_release(struct inode *inode, struct file *file)
{
	/* uninit chip and clear usage count */
	//pr_info("Release\n");
	gpio_direction_output(torch_enable_gpio, 0);
	return 0;
}
static const struct file_operations aw36514_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= aw36514_ioctl,
	.open		= aw36514_open,
	.release	= aw36514_release,
};

static int
aw36514_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36514_chip_data *chip;
	struct aw36514_platform_data *pdata = client->dev.platform_data;
	struct device_node *node = client->dev.of_node;
	int err,ret;
	unsigned int gpio;

	printk("%s Probe start.\n", __func__);

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	if (!node)
		return -EINVAL;

	ret = of_gpio_count(node);
	if (ret == 0){
		pr_err(KERN_EMERG "yslu | of_gpio_count 0");
		return -EINVAL;
	}

	printk("get gpio count = %d\n",ret);
	
	if(ret > 0){
		gpio = of_get_gpio(node, 0);
		if (gpio < 0) {
			pr_err(KERN_EMERG "yslu | Unable to get gpio\n");
			return	-EINVAL;
		}
		ret = devm_gpio_request_one(&client->dev, gpio, GPIOF_DIR_OUT, client->name);
		if (ret < 0) {

			pr_err(KERN_EMERG "yslu | Unable to re quest GPIO %d: %d\n",
                      gpio, ret);
			return -EINVAL;
		}

		gpio_direction_output(gpio, 0); //设置输出的电平

		// <<<<<<<<<<  add bykfliu
		#ifdef CONFIG_COWBOY_BOARD	
		gpio_flash = of_get_gpio(node, 1);
		if (gpio_flash < 0) {
			pr_err(KERN_EMERG "yslu | Unable to get gpio\n");
			return	-EINVAL;
		}
		ret = devm_gpio_request_one(&client->dev, gpio_flash, GPIOF_DIR_OUT, client->name);
		if (ret < 0) {

			pr_err(KERN_EMERG "yslu | Unable to re quest GPIO %d: %d\n",
                      gpio_flash, ret);
			return -EINVAL;
		}
		gpio_direction_output(gpio_flash, 0); //设置输出的电平
		#endif 
		///>>>>>>>>>>>>>>>>>>>>>>>>>>>
	}
	//pr_info("of_get_gpio.%d\n", gpio);
	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36514_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;
	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata =
		kzalloc(sizeof(struct aw36514_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	torch_enable_gpio = gpio;
	i2c_set_clientdata(client, chip);
	aw36514_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init chip hw */
	//aw36514_chip_init(chip);

	ret = sysfs_create_group(&aw36514_dev->kobj, &aw_attribute_group);
	if (ret)
		pr_err("%s sysfs_create_group fail!\n",__func__);	
	
	printk("%s Probe done.\n", __func__);
	return 0;

err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw36514_i2c_remove(struct i2c_client *client)
{
	struct aw36514_chip_data *chip = i2c_get_clientdata(client);

	//pr_info("Remove start.\n");

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	sysfs_remove_group(&aw36514_dev->kobj, &aw_attribute_group);
	//pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36514_i2c_id[] = {
	{AW36514_NAME, 0},
	{}
};

static const struct of_device_id aw36514_i2c_of_match[] = {
	{.compatible = AW36514_DTNAME_I2C},
	{},
};

static struct i2c_driver aw36514_i2c_driver = {
	.driver = {
		   .name = AW36514_NAME,
		   .of_match_table = aw36514_i2c_of_match,
		   },
	.probe = aw36514_i2c_probe,
	.remove = aw36514_i2c_remove,
	.id_table = aw36514_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36514_probe(struct platform_device *dev)
{
	//pr_info("%s Probe start.\n", __func__);

	if (i2c_add_driver(&aw36514_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	//pr_info("%s Probe done.\n", __func__);

	return 0;
}

static int aw36514_remove(struct platform_device *dev)
{
	//pr_info("Remove start.\n");

	i2c_del_driver(&aw36514_i2c_driver);

	//pr_info("Remove done.\n");

	return 0;
}


static const struct of_device_id aw36514_of_match[] = {
	{.compatible = AW36514_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36514_of_match);


static struct platform_device aw36514_platform_device[] = {
	{
		.name = AW36514_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36514_platform_device);


static struct platform_driver aw36514_platform_driver = {
	.probe = aw36514_probe,
	.remove = aw36514_remove,
	.driver = {
		.name = AW36514_NAME,
		.owner = THIS_MODULE,
		.of_match_table = aw36514_of_match,
	},
};

static int __init flashlight_aw36514_init(void)
{
	int ret;

	//pr_info("%s driver version %s.\n", __func__, AW36514_VERSION);

	printk("flashlight_aw36514_init start\n");

	alloc_chrdev_region(&aw36514_devid, 0, 1, "aw36514");
	aw36514_cdev = cdev_alloc();
	cdev_init(aw36514_cdev, &aw36514_fops);
	aw36514_cdev->owner = THIS_MODULE;
	ret = cdev_add(aw36514_cdev, aw36514_devid, 1);
	if (ret) {
		pr_err("%s cdev_add fail\n",__func__);
		return -1;
	}
	aw36514_class = class_create(THIS_MODULE, "aw36514");
	if (IS_ERR(aw36514_class)) {
		pr_err("%s class_create fail\n",__func__);
		return -1;
	}

	aw36514_dev = device_create(aw36514_class, NULL, aw36514_devid, NULL, "aw36514");


	//pr_info("platform_device_register\n");
	ret = platform_device_register(aw36514_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
	//pr_info("platform_driver_register\n");
	ret = platform_driver_register(&aw36514_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	//pr_info("flashlight_aw36514 Init done.\n");
	printk("flashlight_aw36514_init END");
	return 0;
}

static void __exit flashlight_aw36514_exit(void)
{
	//pr_info("flashlight_aw36514-Exit start.\n");

	platform_driver_unregister(&aw36514_platform_driver);
	device_destroy(aw36514_class, aw36514_devid);
	class_destroy(aw36514_class);
	cdev_del(aw36514_cdev);
	
	//pr_info("flashlight_aw36514 Exit done.\n");
}
/*-------------------------------------------------------------------------*/

module_init(flashlight_aw36514_init);
module_exit(flashlight_aw36514_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <yslu@iflytek.com>");
MODULE_DESCRIPTION("AW Flashlight AW36514 Driver");

