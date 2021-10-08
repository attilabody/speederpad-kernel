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
#include <linux/pwm.h>
#include <linux/sunxi-gpio.h>
#include <linux/regulator/consumer.h>

/* device tree should be defined in flashlight-dt.h */
#define SCAN_LEDS_DTNAME "flashlights,scan-leds"
#define SCAN_LEDS_NAME "scan-leds"

#define SCAN_LEDS_VERSION "v1.0.0"

/*define ioctl*/
#define FLASH_IOC_SET_ON	1
#define FLASH_IOC_SET_OFF	0

#define LEDS_GPIO_NUM 4
#define REGULATOR_NAME_SIZE  128
/* platform data
* torch_pin_enable: TX1/TORCH pin isa hardware TORCH enable
* pam_sync_pin_enable: TX2 Mode The ENVM/TX2 is a PAM Sync. on input
* thermal_comp_mode_enable: LEDI/NTC pin in Thermal Comparator Mode
* strobe_pin_disable: STROBE Input disabled
* vout_mode_enable: Voltage Out Mode enable
*/
struct scan_leds_platform_data {
	u8 torch_pin_enable;
	u8 pam_sync_pin_enable;
	u8 thermal_comp_mode_enable;
	u8 strobe_pin_disable;
	u8 vout_mode_enable;
};

/* scan leds chip data */
struct scan_leds_chip_data {
	struct device *dev;
	struct scan_leds_platform_data *pdata;
	struct mutex lock;
	int leds_gpio_en[LEDS_GPIO_NUM];
	char regulator_name[REGULATOR_NAME_SIZE];
	struct regulator *leds_regulator;
	int leds_voltage;
	int times_index;
	u8 last_flag;
	u8 no_pdata;
	int enable;
	u8  change_io_driver;
};

static struct scan_leds_chip_data *p_chip_data;

/* define mutex and work queue */
static DEFINE_MUTEX(scan_leds_mutex);


/* define i2c */
static struct cdev *scan_leds_cdev;
static dev_t scan_leds_devid;
static struct class *scan_leds_class;
static struct device *scan_leds_dev;

int scan_leds_init(struct scan_leds_chip_data *chip, int enable);

int set_leds_power(struct scan_leds_chip_data *chip, int enable){
	int ret = -1;

	chip->leds_regulator = regulator_get_optional(chip->dev, chip->regulator_name);
//	printk("FILE:%s, FUNC:%s, LINE:%d, enable:%d\n", __FILE__, __func__, __LINE__, enable);
	if (!IS_ERR(chip->leds_regulator)) {
		if (enable) {
			ret = regulator_set_voltage(chip->leds_regulator, chip->leds_voltage, chip->leds_voltage);
			if (ret < 0) {
				dev_err(chip->dev, "set regulator leds_regulator voltage failed!\n");
				regulator_put(chip->leds_regulator);
				return ret;
			}

			ret = regulator_enable(chip->leds_regulator);
			if (ret < 0) {
				dev_err(chip->dev, "regulator leds_regulator enable failed\n");
				regulator_put(chip->leds_regulator);
				return ret;
			}

			ret = regulator_get_voltage(chip->leds_regulator);
			if (ret < 0) {
				dev_err(chip->dev, "regulator leds_regulator get voltage failed\n");
				regulator_put(chip->leds_regulator);
				return ret;
			}
			//dev_info(chip->dev, "check wlan leds_regulator voltage: %d\n", ret);
		} else {
			if(regulator_is_enabled(chip->leds_regulator)){
				ret = regulator_disable(chip->leds_regulator);
				if (ret < 0) {
					dev_err(chip->dev, "regulator leds_regulator disable failed\n");
					regulator_put(chip->leds_regulator);
					return ret;
				}
			}
		}
		regulator_put(chip->leds_regulator);
		return 0;
	}	
	return -1;
}

static ssize_t show_led_enable(struct device *dev,
                  struct device_attribute *attr, char *buf)   //cat命令时,将会调用该函数
{
	//pr_info("show_my_switch:\n");
    return sprintf(buf, "%d\n", p_chip_data->enable);
}

static ssize_t set_led_enable(struct device *dev,
                 struct device_attribute *attr,
                 const char *buf, size_t len)   //echo命令时,将会调用该函数
{
    unsigned int data;
    int error;
	//printk("FILE:%s, FUNC:%s, LINE:%d\n", __FILE__, __func__, __LINE__);
    error = kstrtouint(buf, 10, &data);	
	//printk("FILE:%s, FUNC:%s, LINE:%d, data:%d\n", __FILE__, __func__, __LINE__, data);
	scan_leds_init(p_chip_data, data);
    return len;
}


 static ssize_t show_power_voltage(struct device *dev,
				   struct device_attribute *attr, char *buf)
 {
	 //pr_info("show_my_switch:\n");
	 return sprintf(buf, "%d\n", p_chip_data->leds_voltage);
 }

static ssize_t set_power_voltage(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)   
{
   unsigned int data;
   int error;
   error = kstrtouint(buf, 10, &data); 
   p_chip_data->leds_voltage = data;
   return len;
}

static ssize_t show_change_io_driver(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	//pr_info("show_my_switch:\n");
	return sprintf(buf, "%d\n", p_chip_data->change_io_driver);
}


static ssize_t set_change_io_driver(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)	 
{
	 unsigned int data;
	 int error;
	 error = kstrtouint(buf, 10, &data); 
	 p_chip_data->change_io_driver = data;
	 return len;
}


static DEVICE_ATTR(led_enable, 0660, show_led_enable, set_led_enable);
static DEVICE_ATTR(power_voltage, 0660, show_power_voltage, set_power_voltage);
static DEVICE_ATTR(change_io_driver, 0660, show_change_io_driver, set_change_io_driver);


static struct attribute *aw_attributes[] = {
	&dev_attr_led_enable.attr,
	&dev_attr_power_voltage.attr,
	&dev_attr_change_io_driver.attr,
	NULL
};

static struct attribute_group aw_attribute_group = {
	.name = "attr",
	.attrs = aw_attributes
};

/* flashlight init */
int scan_leds_init(struct scan_leds_chip_data *chip, int enable)
{
	if(enable){
		if(enable == 1 && p_chip_data->change_io_driver == 0){
			gpio_set_value(p_chip_data->leds_gpio_en[0], 1);
			gpio_set_value(p_chip_data->leds_gpio_en[1], 1);
			gpio_set_value(p_chip_data->leds_gpio_en[2], 0);
			gpio_set_value(p_chip_data->leds_gpio_en[3], 0);
		}else{
			gpio_set_value(p_chip_data->leds_gpio_en[0], 0);
			gpio_set_value(p_chip_data->leds_gpio_en[1], 0);
			gpio_set_value(p_chip_data->leds_gpio_en[2], 1);
			gpio_set_value(p_chip_data->leds_gpio_en[3], 1);
		}
		set_leds_power(p_chip_data, 1);
	}else{
		set_leds_power(p_chip_data, 0);
		gpio_set_value(p_chip_data->leds_gpio_en[0], 1);
		gpio_set_value(p_chip_data->leds_gpio_en[1], 0);
		gpio_set_value(p_chip_data->leds_gpio_en[2], 1);
		gpio_set_value(p_chip_data->leds_gpio_en[3], 0);

	}
	p_chip_data->enable = enable;

	return 0;
}
EXPORT_SYMBOL(scan_leds_init);


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static long scan_leds_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char *p = (char *)arg;
	switch (cmd) {
	case FLASH_IOC_SET_ON:
		//pr_info("FLASH_IOC_SET_ON\n");
		scan_leds_init(p_chip_data, 1);
		break;
	case FLASH_IOC_SET_OFF:
		//pr_info("FLASH_IOC_SET_OFF\n");
		scan_leds_init(p_chip_data, 0);
		usleep_range(4000, 5000);			
		break;
	default:
		//pr_info("No such command(%d):\n",cmd);
		return -ENOTTY;
	}

	return 0;
}

static int scan_leds_open(struct inode *inode, struct file *file)
{
	/* Actual behavior move to set driver function */
	/* since power saving issue */
	return 0;
}
static int scan_leds_release(struct inode *inode, struct file *file)
{
	/* uninit chip and clear usage count */
	//pr_info("Release\n");
	//gpio_set_value(p_chip_data->pwm_gpio, 0);
	return 0;
}
static const struct file_operations scan_leds_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= scan_leds_ioctl,
	.open		= scan_leds_open,
	.release	= scan_leds_release,
};

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int scan_leds_probe(struct platform_device *pdev)
{
	struct scan_leds_chip_data *chip;
	struct scan_leds_platform_data *pdata;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int err,ret;
	struct gpio_config config;
	int val, index;
	char tmp_buf[128];
	const char *regulator_name = NULL;

	printk("FILE:%s, FUNC:%s, LINE:%d.\n", __FILE__, __func__, __LINE__);
	/* init chip private data */
	chip = kzalloc(sizeof(struct scan_leds_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	p_chip_data = chip;
	for(index = 0; index < LEDS_GPIO_NUM; ++index){
		memset(tmp_buf, 0, sizeof(tmp_buf));
		sprintf(tmp_buf, "led_gpio_en%d", index + 1);
		//printk("FILE:%s, FUNC:%s, LINE:%d, tmp_buf:%s\n", __FILE__, __func__, __LINE__, tmp_buf);

		chip->leds_gpio_en[index] = of_get_named_gpio_flags(node, tmp_buf, 0, (enum of_gpio_flags *)&config);
		//printk("FILE:%s, FUNC:%s, LINE:%d, flash_gpio:%d\n", __FILE__, __func__, __LINE__, chip->flash_gpio);
		if (!gpio_is_valid(chip->leds_gpio_en[index])) {
			dev_err(dev, "get gpio flash gpio %d failed\n", index);
		}else{
			ret = devm_gpio_request(dev, chip->leds_gpio_en[index], tmp_buf);
			if (ret < 0) {
				dev_err(dev, "can't request flash_gpio %d %d\n",
					index, chip->leds_gpio_en[index]);
			}
			ret = gpio_direction_output(chip->leds_gpio_en[index], 0);
			if (ret < 0) {
				dev_err(dev, "can't request output direction flash_gpio %d %d\n",
					index, chip->leds_gpio_en[index]);
			}	
			
		}	
		//printk("FILE:%s, FUNC:%s, LINE:%d, pwm_gpio:%d\n", __FILE__, __func__, __LINE__, chip->pwm_gpio);
	}
	chip->leds_voltage = 3300000;
	if (!of_property_read_u32(node, "leds_voltage", &val)) {
		chip->leds_voltage = val;
		dev_err(dev, "leds voltage (%u)\n", val);
	}	

	//printk("FILE:%s, FUNC:%s, LINE:%d, leds_voltage:%d\n", __FILE__, __func__, __LINE__, chip->leds_voltage);	
	if (of_property_read_string(node, "leds_regulator", &regulator_name)) {
		dev_warn(dev, "Missing leds_regulator.\n");
	} else {
		memset(chip->regulator_name, 0 , sizeof(chip->regulator_name));
		strncpy(chip->regulator_name, regulator_name, REGULATOR_NAME_SIZE - 1);
		//printk("FILE:%s, FUNC:%s, LINE:%d, leds_regulator:%p\n", __FILE__, __func__, __LINE__, chip->leds_regulator);
		//set_leds_power(chip, 0);
	}
	//printk("FILE:%s, FUNC:%s, LINE:%d, regulator_name:%s\n", __FILE__, __func__, __LINE__, chip->regulator_name);

	//gpio_set_value(chip->leds_gpio_en[0], 1);
	//gpio_set_value(chip->leds_gpio_en[1], 0);
	//gpio_set_value(chip->leds_gpio_en[2], 1);
	//gpio_set_value(chip->leds_gpio_en[3], 0);
	chip->enable = 0;
	chip->dev = dev;
	//printk("FILE:%s, FUNC:%s, LINE:%d.\n", __FILE__, __func__, __LINE__);
	/* init platform data */
	pdata = kzalloc(sizeof(struct scan_leds_platform_data), GFP_KERNEL);
	if (!pdata) {
		err = -ENOMEM;
		goto err_init_pdata;
	}

	//printk("FILE:%s, FUNC:%s, LINE:%d.\n", __FILE__, __func__, __LINE__);

	chip->pdata = pdata;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);
	//printk("FILE:%s, FUNC:%s, LINE:%d.\n", __FILE__, __func__, __LINE__);

	/* init chip hw */
	scan_leds_init(chip, 0);

	alloc_chrdev_region(&scan_leds_devid, 0, 1, "scan-leds");
	scan_leds_cdev = cdev_alloc();
	cdev_init(scan_leds_cdev, &scan_leds_fops);
	scan_leds_cdev->owner = THIS_MODULE;
	ret = cdev_add(scan_leds_cdev, scan_leds_devid, 1);
	if (ret) {
		pr_err("%s cdev_add fail\n",__func__);
		return -1;
	}
	scan_leds_class = class_create(THIS_MODULE, "scan-leds");
	if (IS_ERR(scan_leds_class)) {
		pr_err("%s class_create fail\n",__func__);
		return -1;
	}

	scan_leds_dev = device_create(scan_leds_class, NULL, scan_leds_devid, NULL, "scan-leds");

	ret = sysfs_create_group(&scan_leds_dev->kobj, &aw_attribute_group);
	if (ret)
		pr_err("%s sysfs_create_group fail!\n",__func__);	


	p_chip_data->change_io_driver=0;//0 :normal  1: need chang led driver logic
	
	printk("FILE:%s, FUNC:%s, LINE:%d, Probe done.\n", __FILE__, __func__, __LINE__);
	return 0;

err_init_pdata:
	kfree(chip);
err_out:
	return err;
}

static int scan_leds_remove(struct platform_device *dev)
{
	//pr_info("Remove start.\n");
	//pr_info("Remove done.\n");

	return 0;
}


static const struct of_device_id scan_leds_of_match[] = {
	{.compatible = SCAN_LEDS_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, scan_leds_of_match);

static struct platform_driver scan_leds_platform_driver = {
	.probe = scan_leds_probe,
	.remove = scan_leds_remove,
	.driver = {
		.name = SCAN_LEDS_NAME,
		.owner = THIS_MODULE,
		.of_match_table = scan_leds_of_match,
	},
};

module_platform_driver(scan_leds_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <yslu@iflytek.com>");
MODULE_DESCRIPTION("AW Flashlight scan leds Driver");

