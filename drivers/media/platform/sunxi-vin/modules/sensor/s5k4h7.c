/*
 * A V4L2 driver for s5k4h7yx Raw cameras.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *    Liang WeiJie <liangweijie@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>

#include "camera.h"
#include "sensor_helper.h"

MODULE_AUTHOR("bjt");
MODULE_DESCRIPTION("A low-level driver for s5k4h7yx sensors");
MODULE_LICENSE("GPL");

#define MCLK              (24*1000*1000)
#define V4L2_IDENT_SENSOR 0x487b

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30

/*
 * The s5k4h7yx i2c address
 */
#define I2C_ADDR 0x5a
#define SENSOR_NAME "s5k4h7"

#define EEPROM_WRITE_ID 0xA0

#define AW_OTP_OFFSET 0x1000

#define AW_OTP_AWB_Start 0x0009
#define AW_OTP_MSC_Start 0x0011
#define AW_OTP_DPC_Start 0x0613
#define AW_OTP_CHECK_Start 0x061d

#define OTP_SIZE 1567
static int sensor_otp_enable; /* 0: init, -1: disable, 1: enable */
static unsigned char sensor_otp_info[OTP_SIZE];

#define LSC_INFO_SIZE (16 * 16 * 3)
#define AWB_INFO_SIZE 8
static unsigned short sensor_otp_lscinfo[LSC_INFO_SIZE + AWB_INFO_SIZE];

/*
 * The default register settings
 */

static struct regval_list sensor_default_regs[] = {
	{0X0100, 0X00},
	{0X0000, 0X12},
	{0X0000, 0X48},
	{0X0A02, 0X15},

	{0X0100, 0X00},
	{0X0B05, 0X01},
	{0X3074, 0X06},
	{0X3075, 0X2F},
	{0X308A, 0X20},
	{0X308B, 0X08},
	{0X308C, 0X0B},
	{0X3081, 0X07},
	{0X307B, 0X85},
	{0X307A, 0X0A},
	{0X3079, 0X0A},
	{0X306E, 0X71},
	{0X306F, 0X28},
	{0X301F, 0X20},
	{0X306B, 0X9A},
	{0X3091, 0X1F},
	{0X30C4, 0X06},
	{0X3200, 0X09},
	{0X306A, 0X79},
	{0X30B0, 0XFF},
	{0X306D, 0X08},
	{0X3080, 0X00},
	{0X3929, 0X3F},
	{0X3084, 0X16},
	{0X3070, 0X0F},
	{0X3B45, 0X01},
	{0X30C2, 0X05},
	{0X3069, 0X87},

	{0X3924, 0X7F},
	{0X3925, 0XFD},
	{0X3C08, 0XFF},
	{0X3C09, 0XFF},
	{0X3C31, 0XFF},
	{0X3C32, 0XFF},
	{0X0100, 0X00},
};

static struct regval_list sensor_8M_30fps_regs[] = {
	{0x0100, 0x00},// software stanby
	{0X0136, 0X18},
	{0X0137, 0X00},
	{0X0305, 0X06},
	{0X0306, 0X00},
	{0X0307, 0X8C},
	{0X030D, 0X06},
	{0X030E, 0X00},
	{0X030F, 0XAF},
	{0X3C1F, 0X00},
	{0X3C17, 0X00},
	{0X3C1C, 0X05},
	{0X3C1D, 0X15},
	{0X0301, 0X04},
	{0X0820, 0X02},
	{0X0821, 0XBC},
	{0X0822, 0X00},
	{0X0823, 0X00},
	{0X0112, 0X0A},
	{0X0113, 0X0A},
	{0X0114, 0X03},
	{0X3906, 0X04},
	{0X0344, 0X00},
	{0X0345, 0X08},  ///0x00
	{0X0346, 0X00},
	{0X0347, 0X08},  ///0x00
	{0X0348, 0X0C},
	{0X0349, 0XC7},  ///0xCF
	{0X034A, 0X09},
	{0X034B, 0X97},   ///0x9F
	{0X034C, 0X0C},
	{0X034D, 0XC0},   ///0xD0
	{0X034E, 0X09},
	{0X034F, 0X90},   ///0xA0
	{0X0900, 0X00},
	{0X0901, 0X00},
	{0X0381, 0X01},
	{0X0383, 0X01},
	{0X0385, 0X01},
	{0X0387, 0X01},
	{0X0101, 0X03}, ///mirror
	{0X0340, 0X09}, //frame_length_lines = 2530 Frame_Length --- VTS
	{0X0341, 0XE2},
	{0X0342, 0X0E}, //line_length_pck = 3688 Line Length --- HTS
	{0X0343, 0X68},
	{0X0200, 0X0D},
	{0X0201, 0XD8},
	{0X0202, 0X02},   ///0x00
	{0X0203, 0X08},   ///0x02
	{0x3400, 0x01},
	{0x0100, 0x01},
};

static struct regval_list sensor_8M_bining_30fps_regs[] = {
	{0x0100, 0x00},// software stanby
	{0X0136, 0X18},
	{0X0137, 0X00},
	{0X0305, 0X06},
	{0X0306, 0X00},
	{0X0307, 0X8C},
	{0X030D, 0X06},
	{0X030E, 0X00},
	{0X030F, 0XAF},
	{0X3C1F, 0X00},
	{0X3C17, 0X00},
	{0X3C1C, 0X05},
	{0X3C1D, 0X15},
	{0X0301, 0X04},
	{0X0820, 0X02},
	{0X0821, 0XBC},
	{0X0822, 0X00},
	{0X0823, 0X00},
	{0X0112, 0X0A},
	{0X0113, 0X0A},
	{0X0114, 0X03},
	{0X3906, 0X00},   //0X00
	{0X0344, 0X00},
	{0X0345, 0X08},  ///0x00
	{0X0346, 0X00},
	{0X0347, 0X08},  ///0x00
	{0X0348, 0X0C},
	{0X0349, 0XC7},  ///0xCF
	{0X034A, 0X09},
	{0X034B, 0X97},   ///0x9F
	{0X034C, 0X06},
	{0X034D, 0X60},   ///0xD0
	{0X034E, 0X04},
	{0X034F, 0Xc8},   ///0xA0
	{0X0900, 0X01},
	{0X0901, 0X22},
	{0X0381, 0X01},
	{0X0383, 0X01},
	{0X0385, 0X01},
	{0X0387, 0X03},
	{0X0101, 0X03}, ///mirror
	{0X0340, 0X09}, //24fps-->0X0C    30fps-->0X09
	{0X0341, 0XE2}, //24fps-->0X5A    30fps-->0XE2
	{0X0342, 0X0E}, //line_length_pck = 3688 Line Length --- HTS
	{0X0343, 0X68},
	{0X0200, 0X0D},
	{0X0201, 0XD8},
	{0X0202, 0X02},   ///0x00
	{0X0203, 0X08},   ///0x02
	{0x3400, 0x01},
	{0x0100, 0x01},
};

static int sensor_read_byte(struct v4l2_subdev *sd, unsigned short reg,
	unsigned char *value)
{
	int ret = 0, cnt = 0;

	if (!sd || !sd->entity.use_count) {
		sensor_print("%s error! sensor is not used!\n", __func__);
		return -1;
	}

	ret = cci_read_a16_d8(sd, reg, value);
	while ((ret != 0) && (cnt < 2)) {
		ret = cci_read_a16_d8(sd, reg, value);
		cnt++;
	}
	if (cnt > 0)
		pr_info("%s sensor read retry = %d\n", sd->name, cnt);

	return ret;
}

static int sensor_write_byte(struct v4l2_subdev *sd, unsigned short reg,
	unsigned char value)
{
	int ret = 0, cnt = 0;

	if (!sd || !sd->entity.use_count) {
		sensor_print("%s error! sensor is not used!\n", __func__);
		return -1;
	}

	ret = cci_write_a16_d8(sd, reg, value);
	while ((ret != 0) && (cnt < 2)) {
		ret = cci_write_a16_d8(sd, reg, value);
		cnt++;
	}
	if (cnt > 0)
		pr_info("%s sensor write retry = %d\n", sd->name, cnt);

	return ret;
}

static int s5k4h7yx_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int i = 0, ret = 0;

	if (!regs)
		return -EINVAL;

	while (i < array_size) {
		if (regs->addr == REG_DLY) {
			usleep_range(regs->data * 1000, regs->data * 1000 + 100);
		} else {
            ret = sensor_write_byte(sd, regs->addr, regs->data);
			if (ret < 0) {
				sensor_print("%s sensor write array error, array_size %d!\n", sd->name, array_size);
				return -1;
			}
		}
		i++;
		regs++;
	}
	return 0;
}

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */

static struct regval_list sensor_fmt_raw[] = {

};

/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */
static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp;
	sensor_dbg("sensor_get_exposure = %d\n", info->exp);

	return 0;
}

static int s5k4h7yx_sensor_vts;
static int s5k4h7yx_sensor_hts;
static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned int exp_coarse;
	unsigned short exp_fine;
	struct sensor_info *info = to_state(sd);

	if (exp_val > 0xffffff)
		exp_val = 0xfffff0;
	if (exp_val < 16)
		exp_val = 16;

	if (info->exp == exp_val)
		return 0;

	exp_coarse = exp_val >> 4;//rounding to 1
	//exp_fine = (exp_val - exp_coarse * 16) * info->current_wins->hts / 16;
	exp_fine = (unsigned short) (((exp_val - exp_coarse * 16) * s5k4h7yx_sensor_hts) / 16);

	//sensor_write(sd, 0x0200, exp_fine);
	sensor_write(sd, 0x0202, (unsigned short)exp_coarse);

	sensor_dbg("%s exp_val %d s5k4h7yx_sensor_hts %d\n", __func__, exp_val, s5k4h7yx_sensor_hts);
	info->exp = exp_val;

	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->gain;
	sensor_dbg("sensor_get_gain = %d\n", info->gain);

	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	//	unsigned short gain;
	int ana_gain = 0, digi_gain = 0;
	unsigned short GR_GAIN, R_GAIN, B_GAIN, GB_GAIN;

	if (info->gain == gain_val)
		return 0;

	if (gain_val <= 1 * 16) {
		ana_gain = 16;
		digi_gain = 256;
	} else if (gain_val > 16 && gain_val <= (16 * 16 - 1)) {
		ana_gain = gain_val;
		digi_gain = 256;
	} else if (gain_val > (16 * 16 - 1) && gain_val < (32 * 16 - 1)) {
		ana_gain = 16 * 16 - 1;
		digi_gain = (gain_val - 255 + 16) * 1 + 256;
	} else {
		ana_gain = 16 * 16 - 1;
		digi_gain = 512;
	}
	ana_gain *= 2;//shift to 1/32 step
	GR_GAIN = (unsigned short)digi_gain;
	R_GAIN  = (unsigned short)digi_gain;
	B_GAIN  = (unsigned short)digi_gain;
	GB_GAIN = (unsigned short)digi_gain;
//	gain = gain_val * 2;//shift to 1/32 step

//	sensor_write(sd, 0x0204, gain);
	sensor_write(sd, 0x0204, (unsigned short)ana_gain);
	sensor_write(sd, 0x020e, (unsigned short)GR_GAIN);
	sensor_write(sd, 0x0210, (unsigned short)R_GAIN);
	sensor_write(sd, 0x0212, (unsigned short)B_GAIN);
	sensor_write(sd, 0x0214, (unsigned short)GB_GAIN);

	sensor_dbg("%s info->gain %d\n", __func__, gain_val);

	info->gain = gain_val;

	return 0;
}


static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	int exp_val, gain_val, shutter, frame_length;
	struct sensor_info *info = to_state(sd);

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	shutter = exp_val >> 4;

	if (shutter  > s5k4h7yx_sensor_vts - 4)
		frame_length = shutter + 4;
	else
		frame_length = s5k4h7yx_sensor_vts;

	sensor_write_byte(sd, 0x0104, 0x01);
	sensor_write(sd, 0x0340, (unsigned short)frame_length);
	sensor_s_gain(sd, gain_val);
	sensor_s_exp(sd, exp_val);
	sensor_write_byte(sd, 0x0104, 0x00);

	info->exp = exp_val;
	info->gain = gain_val;

	return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	unsigned char rdval;

	ret = sensor_read_byte(sd, 0x0100, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == STBY_ON)
		ret = sensor_write_byte(sd, 0x0100, rdval & 0xfe);
	else
		ret = sensor_write_byte(sd, 0x0100, rdval | 0x01);

	return ret;
}

/*
 * Stuff that knows about the sensor.
 */

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret;

	ret = 0;
	switch (on) {
	case STBY_ON:
		ret = sensor_s_sw_stby(sd, STBY_ON);
		if (ret < 0)
			sensor_err("soft stby falied!\n");
		usleep_range(10000, 12000);

		cci_lock(sd);
		/* inactive mclk after stadby in */
		vin_set_mclk(sd, OFF);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		cci_lock(sd);

		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);

		cci_unlock(sd);
		ret = sensor_s_sw_stby(sd, STBY_OFF);
		if (ret < 0)
			sensor_err("soft stby off falied!\n");
		usleep_range(10000, 12000);
		break;
	case PWR_ON:
		sensor_print("PWR_ON!\n");

		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);

		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(1000, 1200);

		vin_set_mclk(sd, ON);
		usleep_range(100, 120);

		vin_set_mclk_freq(sd, MCLK);
		usleep_range(100, 120);

		vin_set_pmu_channel(sd, CAMERAVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		usleep_range(1000, 1200);

		vin_set_pmu_channel(sd, DVDD, ON);
		usleep_range(1000, 1200);

		vin_set_pmu_channel(sd, IOVDD, ON);
		usleep_range(1000, 1200);

		vin_set_pmu_channel(sd, AFVDD, ON);

		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		usleep_range(5000, 5200);

		cci_unlock(sd);
		break;
	case PWR_OFF:
		sensor_print("PWR_OFF!\n");
		cci_lock(sd);

		vin_set_mclk(sd, OFF);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);

		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		vin_set_pmu_channel(sd, AFVDD, OFF);

		vin_gpio_set_status(sd, PWDN, 0);

		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(100, 120);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(100, 120);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type rdval = 0;

	sensor_read(sd, 0x0000, &rdval);
	if (rdval != V4L2_IDENT_SENSOR) {
		sensor_err("read reg 0x000 return 0x%x\n", rdval);
		return -ENODEV;
	}

	return 0;
}

static int sensor_read_block_otp_a16_d8(struct v4l2_subdev *sd,
					unsigned char dev_addr,
					unsigned short eeprom_addr,
					unsigned char *buf, int len)
{
	unsigned char reg[3];
	struct i2c_msg msg[2];
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!sd || !buf)
		return -1;

	reg[0] = (eeprom_addr & 0xff00) >> 8;
	reg[1] = eeprom_addr & 0xff;
	reg[2] = 0xee;

	msg[0].addr = dev_addr >> 1;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg;

	msg[1].addr = dev_addr >> 1;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	return (i2c_transfer(client->adapter, msg, 2) == 2);
}

static int check_otp_checksum(unsigned char *data, int len)
{
	unsigned int checksum = 0;
	unsigned short otp_check;
	int index = 0;
	int ret = 0;

	if (!data || len <= AW_OTP_CHECK_Start)
		return -1;

	for (index = 0; index < AW_OTP_CHECK_Start; index++)
		checksum += data[index];

	otp_check = (data[AW_OTP_CHECK_Start] >> 8) | data[AW_OTP_CHECK_Start + 1];
	if (otp_check != (checksum % 0xff + 1)) {
		sensor_err("readchecksum 0x%x cal checksum 0x%x\n", otp_check, checksum);
		return -1;
	}

	return ret;
}

static void conver_Otp2AwInfo(unsigned char *sensor_otp_src,
			      unsigned short *aw_info_dst)
{
	unsigned char *src;
	unsigned short *dst;
	unsigned int i;
	unsigned short val, high, low;

	/* Get MSC table */
	src = &sensor_otp_src[AW_OTP_MSC_Start];
	dst = &aw_info_dst[0];
	for (i = 0; i < LSC_INFO_SIZE; i++) {
		high = *src;
		src++;
		low = *src;
		src++;
		val = (high << 8) + low;
		*dst = val;
		dst++;
	}

	/* Get AWB Data */
	src = &sensor_otp_src[AW_OTP_AWB_Start];
	dst = &aw_info_dst[LSC_INFO_SIZE];
	for (i = 0; i < AWB_INFO_SIZE; i++) {
		*dst = *src;
		dst++;
		src++;
	}
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	sensor_dbg("sensor_init\n");

	/*Make sure it is a target sensor */
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = 3264;
	info->height = 2448;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->exp = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;	/* 30fps */

	if (sensor_otp_enable == 0) {
		memset(sensor_otp_info, 0, sizeof(sensor_otp_info));
		ret = sensor_read_block_otp_a16_d8(sd, EEPROM_WRITE_ID,
						   AW_OTP_OFFSET,
						   sensor_otp_info,
						   sizeof(sensor_otp_info));
		if (ret < 0) {
			sensor_otp_enable = -1;
			sensor_print("sensor_otp_disable ret =%d [0x0006]=%x\n",
				     ret, sensor_otp_info[0x0006]);
			return 0;
		}

		sensor_otp_enable = 1;
		ret = check_otp_checksum(sensor_otp_info,
					 sizeof(sensor_otp_info));
		if (ret < 0) {
			sensor_err("sensor check OTP error!\n");
			sensor_otp_enable = -1;
		} else
			sensor_print("sensor check OTP success\n");

		conver_Otp2AwInfo(&sensor_otp_info[0], &sensor_otp_lscinfo[0]);
	}

	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins) {
			memcpy(arg, info->current_wins,
				sizeof(struct sensor_win_size));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		ret = 0;
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		ret = sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_SET_FPS:
		ret = 0;
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	case VIDIOC_VIN_ACT_INIT:
		ret = actuator_init(sd, (struct actuator_para *)arg);
		break;
	case VIDIOC_VIN_ACT_SET_CODE:
		ret = actuator_set_code(sd, (struct actuator_ctrl *)arg);
		break;
	case VIDIOC_VIN_FLASH_EN: {
		struct flash_para *flash_para = (struct flash_para *)arg;

		/* close flash --snake edit 2021 0502*/
		//if (flash_para->mode == V4L2_FLASH_LED_MODE_NONE)
			flash_en(sd, (struct flash_para *)arg);

		ret = 0;
		break;
	}
	case VIDIOC_VIN_GET_SENSOR_OTP_INFO: {
		if (sensor_otp_enable == 1) {
			memcpy(arg, sensor_otp_lscinfo, sizeof(sensor_otp_lscinfo));
			sensor_print("user copy otp info success\n");
		} else {
			ret = -EFAULT;
		}
		break;
	}
	default:
		return -EINVAL;
	}
	return ret;
}

/*
 * Store information about the video data format.
 */
static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
	{
		.width         = 3264,
		.height        = 2448,
		.hoffset       = 0,
		.voffset       = 0,
		.hts           = 3688,
		.vts           = 2530,
		.pclk          = 280*1000*1000,
		.mipi_bps      = 660*1000*1000,
		.fps_fixed     = 30,
		.bin_factor    = 1,
		.intg_min      = 16,
		.intg_max      = (2530-4)<<4,
		.gain_min      = 16,
		.gain_max      = (32<<4),
		.regs          = sensor_8M_30fps_regs,
		.regs_size     = ARRAY_SIZE(sensor_8M_30fps_regs),
		.set_size      = NULL,
	},
	{
		.width         = 1632,
		.height        = 1224,
		.hoffset       = 0,
		.voffset       = 0,
		.hts           = 3688,
		.vts           = 2530,
		.pclk          = 280*1000*1000,
		.mipi_bps      = 660*1000*1000,
		.fps_fixed     = 30,
		.bin_factor    = 1,
		.intg_min      = 16,
		.intg_max      = (2530-4)<<4,
		.gain_min      = 16,
		.gain_max      = (32<<4),
		.regs          = sensor_8M_bining_30fps_regs,
		.regs_size     = ARRAY_SIZE(sensor_8M_bining_30fps_regs),
		.set_size      = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;

	cfg->flags = 0 | V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{

	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static int sensor_reg_init(struct sensor_info *info)
{
	int ret;
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;

	ret = s5k4h7yx_write_array(sd, sensor_default_regs,
				 ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}

	sensor_dbg("sensor_reg_init\n");

	s5k4h7yx_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

	if (wsize->regs)
		s5k4h7yx_write_array(sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(sd);

	info->width = wsize->width;
	info->height = wsize->height;
	s5k4h7yx_sensor_vts = wsize->vts;
	s5k4h7yx_sensor_hts = wsize->hts;

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_dbg("%s on = %d, %d*%d fps: %d code: %x\n", __func__, enable,
		     info->current_wins->width, info->current_wins->height,
		     info->current_wins->fps_fixed, info->fmt->mbus_code);

	if (!enable)
		return 0;

	return sensor_reg_init(info);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.s_stream = sensor_s_stream,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,
	.data_width = CCI_BITS_16,
};


static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 2);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
			      256 * 1600, 1, 1 * 1600);
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0,
			      65536 * 16, 1, 0);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;

	return ret;
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	sd = &info->sd;

	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);
	sensor_init_controls(sd, &sensor_ctrl_ops);

	mutex_init(&info->lock);

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->combo_mode = CMB_PHYA_OFFSET1 | MIPI_NORMAL_MODE;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 0;

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

		sd = cci_dev_remove_helper(client, &cci_drv);

	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   },
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
