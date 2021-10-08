#ifndef __AL3010_ALS_H__
#define __AL3010_ALS_H__

#include <linux/types.h>
#include <linux/ioctl.h>


#define AL3010_DEVICE 		"al3010"

#define AL3010_REG_ALS_MODE_MASK		0x07
#define AL3010_REG_ALS_MODE_SHIFT		0
#define AL3010_REG_ALS_INTERRUPT_FILTER_MASK		0x03
#define AL3010_REG_ALS_INTERRUPT_FILTER_SHIFT		0
#define AL3010_REG_ALS_GAIN_MASK		0x70
#define AL3010_REG_ALS_GAIN_SHIFT		4


#define AL3010_ALS_ACTIVE		0x01
#define AL3010_ALS_DEACTIVE	0x00

typedef enum _SENSOR_TYPE {
	AL3010_ALS = 1,
} SENSOR_TYPE;

#endif


