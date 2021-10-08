/*
 * drivers/usb/sunxi_usb/include/sunxi_usb_board.h
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * javen, 2010-12-20, create this file
 *
 * usb board config.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef __SUNXI_USB_BOARD_H__
#define __SUNXI_USB_BOARD_H__

#include <linux/sunxi-gpio.h>
#include <linux/of_gpio.h>

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
#include <linux/usb/class-dual-role.h>
#endif

#define  SET_USB_PARA				"usb_para"
#define  SET_USB0				"usbc0"
#define  SET_USB1				"usbc1"
#define  SET_USB2				"usbc2"

#define  KEY_USB_GLOBAL_ENABLE			"usb_global_enable"
#define  KEY_USBC_NUM				"usbc_num"

#define  KEY_USB_ENABLE				"usbc0_used"
#define  KEY_USB_PORT_TYPE			"usb_port_type"
#define  KEY_USB_DET_MODE			"usb_detect_mode"
#define  KEY_USB_ID_GPIO			"usb_id_gpio"
#define  KEY_USB_DETVBUS_GPIO			"usb_det_vbus_gpio"

#define  KEY_USB_DRVVBUS_GPIO			"usb_drv_vbus_gpio"
#define  KEY_USB_REGULATOR_IO			"usb_regulator_io"
#define  KEY_USB_REGULATOR_IO_VOL		"usb_regulator_vol"
#define  KEY_USB_REGULATOR_ID_VBUS		"usb_regulator_id_vbus"
#define  KEY_USB_REGULATOR_ID_VBUS_VOL		"usb_regulator_id_vbus_vol"

#define  KEY_USB_WAKEUP_SUSPEND		        "usb_wakeup_suspend"

/* USB config info */
enum usb_gpio_group_type {
	GPIO_GROUP_TYPE_PIO = 0,
	GPIO_GROUP_TYPE_POWER,
};

/* 0: device only; 1: host only; 2: otg */
enum usb_port_type {
	USB_PORT_TYPE_DEVICE = 0,
	USB_PORT_TYPE_HOST,
	USB_PORT_TYPE_OTG,
};

/* 0: dp/dm detect, 1: vbus/id detect */
enum usb_detect_type {
	USB_DETECT_TYPE_DP_DM = 0,
	USB_DETECT_TYPE_VBUS_ID,
};

/* 0: thread scan mode; 1: gpio interrupt mode */
enum usb_detect_mode {
	USB_DETECT_MODE_THREAD = 0,
	USB_DETECT_MODE_INTR,
};

enum usb_det_vbus_type {
	USB_DET_VBUS_TYPE_NULL = 0,
	USB_DET_VBUS_TYPE_GPIO,
	USB_DET_VBUS_TYPE_AXP,
};

enum usb_id_type {
	USB_ID_TYPE_NULL = 0,
	USB_ID_TYPE_GPIO,
	USB_ID_TYPE_AXP,
	USB_ID_TYPE_EXTCON,
};

/* pio info */
typedef struct usb_gpio {
	/* pio valid, 1 - valid, 0 - invalid */
	__u32 valid;
	struct gpio_config gpio_set;
} usb_gpio_t;

typedef struct usb_port_info {
	__u32 enable;				/* port valid */

	__u32 port_no;				/* usb port number */
	enum usb_port_type port_type;		/* usb port type */
	enum usb_detect_type detect_type;	/* usb detect type */
	enum usb_detect_mode detect_mode;	/* usb detect mode */
	enum usb_det_vbus_type det_vbus_type;
	enum usb_id_type id_type;
	const char *det_vbus_name;
	const char *id_name;
	usb_gpio_t id;				/* usb id pin info */
	usb_gpio_t det_vbus;			/* usb vbus pin info */
	usb_gpio_t drv_vbus;			/* usb drv_vbus pin info */
	usb_gpio_t restrict_gpio_set;		/* usb drv_vbus pin info */
	__u32 usb_restrict_flag;		/* usb port number(?) */
	__u32 voltage;				/* usb port number(?) */
	__u32 capacity;				/* usb port number(?) */

	int id_irq_num;				/* id gpio irq num */

#if !defined(CONFIG_AW_AXP) && defined(CONFIG_POWER_SUPPLY)
	struct power_supply *psy;
#endif

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc dr_desc;
#endif
} usb_port_info_t;

typedef struct usb_cfg {
	u32 usb_global_enable;
	u32 usbc_num;
	struct platform_device *pdev;

	struct usb_port_info port;
} usb_cfg_t;

#endif /* __SUNXI_USB_BOARD_H__ */
