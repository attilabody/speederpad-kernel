/*
 * stk8xxx.c - Linux driver for sensortek stk8xxx accelerometer
 * Copyright (C) 2017 Sensortek
 */

#include <asm/uaccess.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif
#include <linux/sunxi-gpio.h>
//#include <linux/stk8xxx.h>

/*****************************************************************************
 * Global variable
 *****************************************************************************/
//#define STK_INTERRUPT_MODE
#define STK_POLLING_MODE
//#define STK_AMD /* Turn ON any motion */
//#define STK_STEP_COUNTER /* Turn on step counter */
#define STK_CALI /* Turn on sensortek calibration feature */
//#define STK_CALI_FILE /* Write cali data to file */
#define STK_FIR /* low-pass mode */
//#define STK_ZG
    /* enable Zero-G simulation.
     * This feature only works when both of STK_FIR and STK_ZG are turn ON. */
//#define STK_QUALCOMM
#define STK_SPREADTRUM /* Must work with STK_POLLING_MODE */
//#define STK_AUTOK /* Auto cali */

#ifdef STK_QUALCOMM
    #undef STK_SPREADTRUM
    #include <linux/sensors.h>
#elif defined STK_SPREADTRUM
    #include <linux/limits.h>
    #include <linux/version.h>
    #undef STK_INTERRUPT_MODE
    #define STK_POLLING_MODE
#endif /* STK_QUALCOMM or STK_SPREADTRUM */

#ifdef STK_AUTOK
    #define STK_CALI
    #define STK_CALI_FILE
#endif /* STK_AUTOK */

/* Any motion only works under either STK_INTERRUPT_MODE or STK_POLLING_MODE */
#ifdef STK_AMD
    #if !defined STK_INTERRUPT_MODE && !defined STK_POLLING_MODE
        #undef STK_AMD
    #endif /* !defined STK_INTERRUPT_MODE && !defined STK_POLLING_MODE*/
#endif /* STK_AMD */
#ifdef STK_AMD
    #define IN_DEV_AMD_NAME "any motion"
#endif /* STK_AMD */

#ifdef STK_FIR
    #define STK_FIR_LEN         2
    #define STK_FIR_LEN_MAX     32
    struct data_fir
    {
        s16 xyz[STK_FIR_LEN_MAX][3];
        int sum[3];
        int idx;
        int count;
    };
#endif /* STK_FIR */

#if (defined STK_FIR && defined STK_ZG)
    #define ZG_FACTOR   0
#endif /* defined STK_FIR && defined STK_ZG */

#define STK_ACC_DRIVER_VERSION "1.0.0"
#define STK8XXXX_I2C_NAME "stk8xxx"
#define IN_DEV_ACCEL_NAME "accelerometer"
#ifdef STK_INTERRUPT_MODE
    #define STK_IRQ_INT1_LABEL "STK_ACCEL_INT1"
    #define STK_IRQ_INT1_NAME "stk8xxx_int1"
#endif /* STK_INTERRUPT_MODE */

/*****************************************************************************
 * stk8xxx register, start
 *****************************************************************************/
#define STK8XXX_REG_CHIPID          0x00
#define STK8XXX_REG_XOUT1           0x02
#define STK8XXX_REG_XOUT2           0x03
#define STK8XXX_REG_YOUT1           0x04
#define STK8XXX_REG_YOUT2           0x05
#define STK8XXX_REG_ZOUT1           0x06
#define STK8XXX_REG_ZOUT2           0x07
#define STK8XXX_REG_INTSTS1         0x09
#define STK8XXX_REG_INTSTS2         0x0A
#define STK8XXX_REG_FIFOSTS         0x0C
#define STK8XXX_REG_STEPOUT1        0x0D
#define STK8XXX_REG_STEPOUT2        0x0E
#define STK8XXX_REG_RANGESEL        0x0F
#define STK8XXX_REG_BWSEL           0x10
#define STK8XXX_REG_POWMODE         0x11
#define STK8XXX_REG_SWRST           0x14
#define STK8XXX_REG_INTEN1          0x16
#define STK8XXX_REG_INTEN2          0x17
#define STK8XXX_REG_INTMAP1         0x19
#define STK8XXX_REG_INTMAP2         0x1A
#define STK8XXX_REG_INTCFG1         0x20
#define STK8XXX_REG_INTCFG2         0x21
#define STK8XXX_REG_SLOPEDLY        0x27
#define STK8XXX_REG_SLOPETHD        0x28
#define STK8XXX_REG_SIGMOT1         0x29
#define STK8XXX_REG_SIGMOT2         0x2A
#define STK8XXX_REG_SIGMOT3         0x2B
#define STK8XXX_REG_STEPCNT1        0x2C
#define STK8XXX_REG_STEPCNT2        0x2D
#define STK8XXX_REG_STEPTHD         0x2E
#define STK8XXX_REG_STEPDEB         0x2F
#define STK8XXX_REG_STEPMAXTW       0x31
#define STK8XXX_REG_INTFCFG         0x34
#define STK8XXX_REG_OFSTCOMP1       0x36
#define STK8XXX_REG_OFSTX           0x38
#define STK8XXX_REG_OFSTY           0x39
#define STK8XXX_REG_OFSTZ           0x3A
#define STK8XXX_REG_CFG1            0x3D
#define STK8XXX_REG_CFG2            0x3E
#define STK8XXX_REG_FIFOOUT         0x3F

/* STK8XXX_REG_CHIPID */
#define STK8BA50_R_ID                       0x86
#define STK8BA53_ID                         0x87
#define STK8323_ID                          0x23 /* include for STK8321 */
#define STK8325_ID                          0x25
static const u8 STK_ID[] = {STK8BA50_R_ID, STK8BA53_ID, STK8323_ID, STK8325_ID};

/* STK8XXX_REG_INTSTS1 */
#define STK8XXX_INTSTS1_SIG_MOT_STS         0x1
#define STK8XXX_INTSTS1_ANY_MOT_STS         0x4

/* STK8XXX_REG_INTSTS2 */
#define STK8XXX_INTSTS2_FWM_STS_MASK        0x40

/* STK8XXX_REG_FIFOSTS */
#define STK8XXX_FIFOSTS_FIFOOVER            0x80
#define STK8XXX_FIFOSTS_FIFO_FRAME_CNT_MASK 0x7F

/* STK8XXX_REG_RANGESEL */
#define STK8XXX_RANGESEL_2G                 0x3
#define STK8XXX_RANGESEL_4G                 0x5
#define STK8XXX_RANGESEL_8G                 0x8
#define STK8XXX_RANGESEL_BW_MASK            0xF
#define STK8XXX_RANGESEL_DEF                STK8XXX_RANGESEL_2G
typedef enum
{
    STK_2G = STK8XXX_RANGESEL_2G,
    STK_4G = STK8XXX_RANGESEL_4G,
    STK_8G = STK8XXX_RANGESEL_8G
} stk_rangesel;

#if 0
/* STK8XXX_REG_BWSEL */
#define STK8XXX_BWSEL_BW_7_81               0x08    /* ODR = BW x 2 = 15.62Hz */
#define STK8XXX_BWSEL_BW_15_63              0x09    /* ODR = BW x 2 = 31.26Hz */
#define STK8XXX_BWSEL_BW_31_25              0x0A    /* ODR = BW x 2 = 62.5Hz */
#define STK8XXX_BWSEL_BW_62_5               0x0B    /* ODR = BW x 2 = 125Hz */
#define STK8XXX_BWSEL_BW_125                0x0C    /* ODR = BW x 2 = 250Hz */
#define STK8XXX_BWSEL_BW_250                0x0D    /* ODR = BW x 2 = 500Hz */
#define STK8XXX_BWSEL_BW_500                0x0E    /* ODR = BW x 2 = 1000Hz */
#else
/* STK8XXX_REG_BWSEL */
#define STK8XXX_BWSEL_BW_7_81               0x09    /* ODR = BW x 2 = 15.62Hz */
#define STK8XXX_BWSEL_BW_15_63              0x0A    /* ODR = BW x 2 = 31.26Hz */
#define STK8XXX_BWSEL_BW_31_25              0x0B    /* ODR = BW x 2 = 62.5Hz */
#define STK8XXX_BWSEL_BW_62_5               0x0C    /* ODR = BW x 2 = 125Hz */
#define STK8XXX_BWSEL_BW_125                0x0D    /* ODR = BW x 2 = 250Hz */
#define STK8XXX_BWSEL_BW_250                0x0E    /* ODR = BW x 2 = 500Hz */
#define STK8XXX_BWSEL_BW_500                0x0F    /* ODR = BW x 2 = 1000Hz */
#endif

/* STK8XXX_REG_POWMODE */
#define STK8XXX_PWMD_SUSPEND                0x80
#define STK8XXX_PWMD_LOWPOWER               0x40
#define STK8XXX_PWMD_NORMAL                 0x00
#define STK8XXX_PWMD_SLP_MASK               0x3E

/* STK8XXX_REG_SWRST */
#define STK8XXX_SWRST_VAL                   0xB6

/* STK8XXX_REG_INTEN1 */
#define STK8XXX_INTEN1_SLP_EN_XYZ           0x07

/* STK8XXX_REG_INTEN2 */
#define STK8XXX_INTEN2_DATA_EN              0x10
#define STK8XXX_INTEN2_FWM_EN               0x40

/* STK8XXX_REG_INTMAP1 */
#define STK8XXX_INTMAP1_SIGMOT2INT1         0x01
#define STK8XXX_INTMAP1_ANYMOT2INT1         0x04

/* STK8XXX_REG_INTMAP2 */
#define STK8XXX_INTMAP2_DATA2INT1           0x01
#define STK8XXX_INTMAP2_FWM2INT1            0x02
#define STK8XXX_INTMAP2_FWM2INT2            0x40

/* STK8XXX_REG_INTCFG1 */
#define STK8XXX_INTCFG1_INT1_ACTIVE_H       0x01
#define STK8XXX_INTCFG1_INT1_OD_PUSHPULL    0x00
#define STK8XXX_INTCFG1_INT2_ACTIVE_H       0x04
#define STK8XXX_INTCFG1_INT2_OD_PUSHPULL    0x00

/* STK8XXX_REG_INTCFG2 */
#define STK8XXX_INTCFG2_NOLATCHED           0x00
#define STK8XXX_INTCFG2_LATCHED             0x0F
#define STK8XXX_INTCFG2_INT_RST             0x80

/* STK8XXX_REG_SLOPETHD */
#define STK8XXX_SLOPETHD_DEF                0x14

/* STK8XXX_REG_SIGMOT1 */
#define STK8XXX_SIGMOT1_SKIP_TIME_3SEC      0x96    /* default value */

/* STK8XXX_REG_SIGMOT2 */
#define STK8XXX_SIGMOT2_SIG_MOT_EN          0x02
#define STK8XXX_SIGMOT2_ANY_MOT_EN          0x04

/* STK8XXX_REG_SIGMOT3 */
#define STK8XXX_SIGMOT3_PROOF_TIME_1SEC     0x32    /* default value */

/* STK8XXX_REG_STEPCNT2 */
#define STK8XXX_STEPCNT2_RST_CNT            0x04
#define STK8XXX_STEPCNT2_STEP_CNT_EN        0x08

/* STK8XXX_REG_INTFCFG */
#define STK8XXX_INTFCFG_I2C_WDT_EN          0x04

/* STK8XXX_REG_OFSTCOMP1 */
#define STK8XXX_OFSTCOMP1_OFST_RST          0x80

/* STK8XXX_REG_CFG1 */
/* the maximum space for FIFO is 32*3 bytes */
#define STK8XXX_CFG1_XYZ_FRAME_MAX          32

/* STK8XXX_REG_CFG2 */
#define STK8XXX_CFG2_FIFO_MODE_BYPASS       0x0
#define STK8XXX_CFG2_FIFO_MODE_FIFO         0x1
#define STK8XXX_CFG2_FIFO_MODE_SHIFT        5
#define STK8XXX_CFG2_FIFO_DATA_SEL_XYZ      0x0
#define STK8XXX_CFG2_FIFO_DATA_SEL_X        0x1
#define STK8XXX_CFG2_FIFO_DATA_SEL_Y        0x2
#define STK8XXX_CFG2_FIFO_DATA_SEL_Z        0x3
#define STK8XXX_CFG2_FIFO_DATA_SEL_MASK     0x3

/* STK8XXX_REG_OFSTx */
#define STK8XXX_OFST_LSB                    128     /* 8 bits for +-1G */
/*****************************************************************************
 * stk8xxx register, end
 *****************************************************************************/

typedef struct {
    uint8_t         regBwsel;
    unsigned int    sample_rate_us;
} _stkODRMap;

const _stkODRMap stkODRTable[] = {
    /* 0: ODR=31.25 */
    {
        .regBwsel       = STK8XXX_BWSEL_BW_15_63,
        .sample_rate_us = 32000,
    },
    /* 1: ODR=62.5 */
    {
        .regBwsel       = STK8XXX_BWSEL_BW_31_25,
        .sample_rate_us = 16000,
    },
    /* 2: ODR=125 */
    {
        .regBwsel       = STK8XXX_BWSEL_BW_62_5,
        .sample_rate_us = 8000,
    },
    /* 3: ODR=250 */
    {
        .regBwsel       = STK8XXX_BWSEL_BW_125,
        .sample_rate_us = 4000,
    },
    /* 4: ODR=500 */
    {
        .regBwsel       = STK8XXX_BWSEL_BW_250,
        .sample_rate_us = 2000,
    },
};

#define STK_MIN_DELAY_US 2000   /* stkODRTable[4].sample_rate_us */
#define STK_MAX_DELAY_US 32000  /* stkODRTable[0].sample_rate_us */

#ifdef STK_CALI
/* calibration parameters */
#define STK_CALI_SAMPLE_NO          10
#ifdef STK_CALI_FILE
#define STK_CALI_VER0               0x18
#define STK_CALI_VER1               0x03
#define STK_CALI_END                '\0'
#define STK_CALI_FILE_PATH          "/data/misc/stkacccali.conf"
#define STK_CALI_FILE_SIZE          25
#endif /* STK_CALI_FILE */
/* parameter for cali_status/atomic_t and cali file */
#define STK_K_SUCCESS_FILE          0x01
/* parameter for cali_status/atomic_t */
#define STK_K_FAIL_WRITE_OFST       0xF2
#define STK_K_FAIL_I2C              0xF8
#define STK_K_FAIL_W_FILE           0xFB
#define STK_K_FAIL_VERIFY_CALI      0xFD
#define STK_K_RUNNING               0xFE
#define STK_K_NO_CALI               0xFF
#endif /* STK_CALI */

/* selftest usage */
#define STK_SELFTEST_SAMPLE_NUM             100
#define STK_SELFTEST_RESULT_NA              0
#define STK_SELFTEST_RESULT_RUNNING         (1 << 0)
#define STK_SELFTEST_RESULT_NO_ERROR        (1 << 1)
#define STK_SELFTEST_RESULT_DRIVER_ERROR    (1 << 2)
#define STK_SELFTEST_RESULT_FAIL_X          (1 << 3)
#define STK_SELFTEST_RESULT_FAIL_Y          (1 << 4)
#define STK_SELFTEST_RESULT_FAIL_Z          (1 << 5)
#define STK_SELFTEST_RESULT_NO_OUTPUT       (1 << 6)
static inline int stk_selftest_offset_factor(int sen)
{
    return sen * 3 / 10;
}
static inline int stk_selftest_noise_factor(int sen)
{
    return sen / 10;
}

struct stk_data
{
    /* platform related */
    struct i2c_client           *client;
    /* device tree */
    int                         direction;
    int                         interrupt_int1_pin;
#ifdef STK_QUALCOMM	
    struct sensors_classdev     accel_cdev;
#endif	
    /* chip informateion */
    int                         pid;
    bool                        fifo;               /* Support FIFO or not */
    /* system operation */
    atomic_t                    enabled;            /* chip is enabled or not */
#ifdef STK_CALI
    s16                         cali_sw[3];
    atomic_t                    cali_status;        /* cali status */
#ifdef STK_AUTOK
    atomic_t                    first_enable;
    u8                          offset[3];          /* offset value for STK8XXX_REG_OFSTX~Z */
#endif /* STK_AUTOK */
#endif /* STK_CALI */
    atomic_t                    recv;               /* recv data. DEVICE_ATTR(recv, ...) */
    struct mutex                reg_lock;           /* mutex lock for register R/W */
    u8                          power_mode;
    struct input_dev            *input_dev_accel;   /* accel data */
#ifdef STK_AMD
    struct input_dev            *input_dev_amd;     /* any motion data */
#endif /* STK_AMD */
    bool                        temp_enable;        /* record current power status. For Suspend/Resume used. */
    int                         sensitivity;        /* sensitivity, bit number per G */
    int                         sr_no;              /* Serial number of stkODRTable */
    int                         latency_us;         /* latency in usec */
    u64                         fifo_start_ns;
    s16                         xyz[3];             /* The latest data of xyz */
    ktime_t                     timestamp;
    atomic_t                    selftest;           /* selftest result */
#ifdef STK_STEP_COUNTER
    int                         steps;              /* The latest step counter value */
#endif /* STK_STEP_COUNTER */
#ifdef STK_INTERRUPT_MODE
    int                         irq1;               /* for all data usage(DATA, FIFO, ANYMOTION) */
    struct workqueue_struct     *alldata_workqueue; /* all data workqueue for int1. (DATA, FIFO, ANYMOTION) */
    struct work_struct          alldata_work;       /* all data work for int1. (DATA, FIFO, ANYMOTION) */
#elif defined STK_POLLING_MODE
    struct delayed_work         accel_delaywork;
    struct hrtimer              accel_timer;
    ktime_t                     poll_delay;
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
#ifdef STK_FIR
    struct data_fir             fir;
    /*
     * fir_len
     * 0: turn OFF FIR operation
     * 1 ~ STK_FIR_LEN_MAX: turn ON FIR operation
     */
    atomic_t                    fir_len;
#endif /* STK_FIR */
};

#ifdef STK_QUALCOMM
/* Accelerometer information read by HAL */
static struct sensors_classdev stk_cdev = {
    .name = "stk8xxx",
    .vendor = "Sensortek",
    .version = 1,
    .handle = SENSORS_ACCELERATION_HANDLE,
    .type = SENSOR_TYPE_ACCELEROMETER,
    .max_range = "39.24", /* 4G mode: 4.0f*9.81f=39.24f */
	.resolution = "0.01916", /* 4G mode,12-bit resolution: 9.81f/512.f=0.01916f */
	.sensor_power = "0.138",
	.min_delay = STK_MIN_DELAY_US,
	.max_delay = STK_MAX_DELAY_US,
	.delay_msec = 16,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
};
#endif /* STK_QUALCOMM */

static struct stk_data *stk_data_ptr;
#ifdef STK_AUTOK
static void stk_autok(struct stk_data *stk);
#endif /* STK_AUTOK */

/*	direction settings	*/
static const int coordinate_trans[8][3][3] = {
	/* x_after, y_after, z_after */
	{{0,-1,0}, {1,0,0}, {0,0,1}},
	{{1,0,0}, {0,1,0}, {0,0,1}},
	{{0,1,0}, {-1,0,0}, {0,0,1}},
	{{-1,0,0}, {0,-1,0}, {0,0,1}},
	{{0,1,0}, {1,0,0}, {0,0,-1}},
	{{-1,0,0}, {0,1,0}, {0,0,-1}},
	{{0,-1,0}, {-1,0,0}, {0,0,-1}},
	{{1,0,0}, {0,-1,0}, {0,0,-1}},
};

struct stk8xxx_platform_data
{
    unsigned char   direction;
    int             interrupt_int1_pin;
};

static struct stk8xxx_platform_data stk_plat_data =
{
    .direction              = 1,
    .interrupt_int1_pin     = 117,
};

/*------------------add by yxp 20210111------------------------*/

//static const char* hwinfo = "BMA250E";
#include <misc/noah_eeprom_com.h>

#define XX_MAX_PERM_VALUE 80
#define XX_MAX_NUM_PER_GROUP 20
#define XX_MAX_NUM_OF_GROUP 10000
static int x_offset_cal = 0;
static int y_offset_cal = 0;
static int z_offset_cal = 0;
static int x_sum = 0;
static int y_sum = 0;
static int z_sum = 0;
static int n_index = 0;
static int m_index = 0;
static struct kobject *android_gsensor_kobj;
static rwlock_t cal_rwlock;
static char cal_lock_flag = 0;
static int acc_debug = 0;

/*------------------------------------------------*/

/********************* Functions **********************/
#ifdef CONFIG_OF
/*
 * @brief: Parse data in device tree
 *
 * @param[in] dev: struct device *
 * @param[in/out] pdata: struct stk8xxx_platform_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_parse_dt(struct device *dev,
                            struct stk8xxx_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    const int *p;
    struct gpio_config int_gpio;
    p = of_get_property(np, "stk,direction", NULL);

    if (p)
        pdata->direction = be32_to_cpu(*p);
#if 1
    pdata->interrupt_int1_pin = of_get_named_gpio_flags(np,
                                "stk8xxx,irq-gpio", 0, (enum of_gpio_flags *)&int_gpio);

    if (pdata->interrupt_int1_pin < 0)
    {
        dev_err(dev, "%s: Unable to read stk8xxx,irq-gpio\n", __func__);
#ifdef STK_INTERRUPT_MODE
        return pdata->interrupt_int1_pin;
#else /* no STK_INTERRUPT_MODE */
        return 0;
#endif /* STK_INTERRUPT_MODE */
    }
#endif
    return 0; /* SUCCESS */
}
#else
static int stk_parse_dt(struct device *dev,
                            struct stk8xxx_platform_data *pdata)
{
    return -ENODEV
}
#endif /* CONFIG_OF */

/*
 * stk8xxx register write
 * @brief: Register writing via I2C
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] reg: Register address
 * @param[in] val: Data, what you want to write.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_reg_write(struct stk_data *stk, u8 reg, u8 val)
{
    int error = 0;
    mutex_lock(&stk->reg_lock);
    error = i2c_smbus_write_byte_data(stk->client, reg, val);
    mutex_unlock(&stk->reg_lock);

    if (error)
        dev_err(&stk->client->dev,
                "%s: failed to write reg:0x%x with val:0x%x\n",
                __func__, reg, val);

    return error;
}

/*
 * stk8xxx register read
 * @brief: Register reading via I2C
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] reg: Register address
 * @param[in] len: 0, for normal usage. Others, read length (FIFO used).
 * @param[out] val: Data, the register what you want to read.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_reg_read(struct stk_data *stk, u8 reg, int len, u8 *val)
{
    int error = 0;
    struct i2c_msg msgs[2] = {
        {
            .addr = stk->client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg
        },
        {
            .addr = stk->client->addr,
            .flags = I2C_M_RD,
            .len = (0 >= len) ? 1 : len,
            .buf = val
        }
    };

    mutex_lock(&stk->reg_lock);
    error = i2c_transfer(stk->client->adapter, msgs, 2);
    mutex_unlock(&stk->reg_lock);

    if (2 == error)
        error = 0;
    else if (0 > error)
    {
        dev_err(&stk->client->dev, "transfer failed to read reg:0x%x with len:%d, error=%d\n", reg, len, error);
    }
    else
    {
        dev_err(&stk->client->dev, "size error in reading reg:0x%x with len:%d, error=%d\n", reg, len, error);
        error = -1;
    }

    return error;
}

/*
 * @brief: Get platform data
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int get_platform_data(struct stk_data *stk)
{
    int error = 0;
    struct stk8xxx_platform_data *stk_platdata;

    if (stk->client->dev.of_node)
    {
        dev_info(&stk->client->dev,
                 "%s: probe with device tree\n", __func__);
        stk_platdata = devm_kzalloc(&stk->client->dev,
                                    sizeof(struct stk8xxx_platform_data), GFP_KERNEL);

        if (!stk_platdata)
        {
            dev_err(&stk->client->dev,
                    "Failed to allocate memory\n");
            return -ENOMEM;
        }

        error = stk_parse_dt(&stk->client->dev, stk_platdata);

        if (error)
        {
            dev_err(&stk->client->dev,
                    "%s: stk_parse_dt ret=%d\n", __func__, error);
            return error;
        }
    }
    else
    {
        if (NULL != stk->client->dev.platform_data)
        {
            dev_info(&stk->client->dev,
                     "%s: probe with platform data\n", __func__);
            stk_platdata = stk->client->dev.platform_data;
        }
        else
        {
            dev_info(&stk->client->dev,
                     "%s: probe with private platform data\n", __func__);
            stk_platdata = &stk_plat_data;
        }
    }

    stk->interrupt_int1_pin = stk_platdata->interrupt_int1_pin;
    stk->direction = stk_platdata->direction;
    return 0;
}

/*
 * @brief: Change power mode
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] pwd_md: power mode for STK8XXX_REG_POWMODE
 *              STK8XXX_PWMD_SUSPEND
 *              STK8XXX_PWMD_LOWPOWER
 *              STK8XXX_PWMD_NORMAL
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_change_power_mode(struct stk_data *stk, u8 pwd_md)
{
    if (pwd_md != stk->power_mode)
    {
        int error = 0;
        u8 val = 0;
        error = stk_reg_read(stk, STK8XXX_REG_POWMODE, 0, &val);

        if (error)
            return error;

        val &= STK8XXX_PWMD_SLP_MASK;
        error = stk_reg_write(stk, STK8XXX_REG_POWMODE, (val | pwd_md));

        if (error)
            return error;

        stk->power_mode = pwd_md;
    }
    else
        dev_info(&stk->client->dev,
                 "%s: Same as original power mode: 0x%X\n",
                 __func__, stk->power_mode);

    return 0;
}

/*
 * @brief: Get sensitivity. Set result to stk_data.sensitivity.
 *          sensitivity = number bit per G (LSB/g)
 *          Example: RANGESEL=8g, 12 bits for STK8xxx full resolution
 *          Ans: number bit per G = 2^12 / (8x2) = 256 (LSB/g)
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_get_sensitivity(struct stk_data *stk)
{
    u8 val = 0;
    stk->sensitivity = 0;

    if ( 0 == stk_reg_read(stk, STK8XXX_REG_RANGESEL, 0, &val))
    {
        val &= STK8XXX_RANGESEL_BW_MASK;

        switch (val)
        {
            case STK8XXX_RANGESEL_2G:
                stk->sensitivity = 1024;
                break;

            case STK8XXX_RANGESEL_4G:
                stk->sensitivity = 512;
                break;

            case STK8XXX_RANGESEL_8G:
                stk->sensitivity = 256;
                break;

            default:
                dev_err(&stk->client->dev, "%s: got wrong RANGE: 0x%X\n", __func__, val);
                break;
        }

        if (STK8BA50_R_ID == stk->pid) {
            stk->sensitivity /= 4;
        }
    }
}

/*
 * @brief: Set range
 *          1. Setting STK8XXX_REG_RANGESEL
 *          2. Calculate sensitivity and store to stk_data.sensitivity
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] range: range for STK8XXX_REG_RANGESEL
 *              STK_2G
 *              STK_4G
 *              STK_8G
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_range_selection(struct stk_data *stk, stk_rangesel range)
{
    int result = 0;
    result = stk_reg_write(stk, STK8XXX_REG_RANGESEL, range);

    if (result)
        return result;

    stk_get_sensitivity(stk);
    return 0;
}

#ifdef STK_AMD
/*
 * @brief: report any motion result to /sys/class/input/inputX/capabilities/abs
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] flag: ANY MOTION status
 *              true: set 1 to /sys/class/input/inputX/capabilities/abs
 *              false: set 0 to /sys/class/input/inputX/capabilities/abs
 */
static void stk_report_ANYMOTION(struct stk_data *stk, bool flag)
{
    if (!stk->input_dev_amd)
    {
        dev_err(&stk->client->dev,
                "%s: No input device for ANY motion\n", __func__);
        return;
    }

    if (flag)
    {
        input_report_abs(stk->input_dev_amd, ABS_MISC, 0x1);
        dev_info(&stk->client->dev, "%s: trigger\n", __func__);
    }
    else
    {
        input_report_abs(stk->input_dev_amd, ABS_MISC, 0x0);
        dev_info(&stk->client->dev, "%s: no ANY motion\n", __func__);
    }

    input_sync(stk->input_dev_amd);
}

/*
 * @brief: Read any motion data, then report to userspace.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_read_anymotion_data(struct stk_data *stk)
{
    u8 data = 0;

    if (stk_reg_read(stk, STK8XXX_REG_INTSTS1, 0, &data))
        return;

    if (STK8XXX_INTSTS1_ANY_MOT_STS & data)
    {
        stk_report_ANYMOTION(stk, true);
    }
    else
    {
        stk_report_ANYMOTION(stk, false);
    }
}

/*
 * @brief: Trigger INT_RST for latched STS
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_reset_latched_int(struct stk_data *stk)
{
    u8 data = 0;

    if (stk_reg_read(stk, STK8XXX_REG_INTCFG2, 0, &data))
        return;

    if (stk_reg_write(stk, STK8XXX_REG_INTCFG2,
                          (data | STK8XXX_INTCFG2_INT_RST)))
        return;
}
#endif /* STK_AMD */

/*
 * stk_set_enable
 * @brief: Turn ON/OFF the power state of stk8xxx.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] en: turn ON/OFF
 *              0 for suspend mode;
 *              1 for normal mode.
 */
static void stk_set_enable(struct stk_data *stk, char en)
{
#ifdef STK_AUTOK
    if (atomic_read(&stk->first_enable)) {
        dev_info(&stk->client->dev, "%s, stk_autok\n", __func__);
        atomic_set(&stk->first_enable, 0);
        stk_autok(stk);
    }
#endif /* STK_AUTOK */

    if (en == atomic_read(&stk->enabled))
        return;

    if (en)
    {
        if (stk_change_power_mode(stk, STK8XXX_PWMD_NORMAL))
        {
            dev_err(&stk->client->dev, "%s: failed to change power mode, en:%d\n", __func__, en);
        }
        //dev_info(&stk->client->dev, "%s: STK8XXX_PWMD_NORMAL\n", __func__);

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
        hrtimer_start(&stk->accel_timer, stk->poll_delay, HRTIMER_MODE_REL);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    }
    else
    {
        if (stk_change_power_mode(stk, STK8XXX_PWMD_SUSPEND))
        {
            dev_err(&stk->client->dev, "%s: failed to change power mode, en:%d\n", __func__, en);
        }
        //dev_info(&stk->client->dev, "%s: STK8XXX_PWMD_SUSPEND\n", __func__);

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
        hrtimer_cancel(&stk->accel_timer);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    }

#ifdef STK_AMD
    stk_report_ANYMOTION(stk, false);
#endif /* STK_AMD */
    atomic_set(&stk->enabled, en);
}

/*
 * @brief: Get delay
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: delay in usec
 */
static int stk_get_delay(struct stk_data *stk)
{
    return stkODRTable[stk->sr_no].sample_rate_us;
}

/*
 * @brief: Set delay
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_set_delay(struct stk_data *stk, int delay_us)
{
    int error = 0;
    bool enable = false;
    int sr_no;
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============delay_us:%d.\n", __FILE__, __func__, __LINE__, delay_us);

    for (sr_no = 0; sr_no < sizeof(stkODRTable)/sizeof(stkODRTable[0]); sr_no++) {
        if (delay_us >= stkODRTable[sr_no].sample_rate_us)
            break;
    }
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============sr_no:%d, stkODRTable[sr_no].sample_rate_us:%d\n", __FILE__, __func__, __LINE__, sr_no, stkODRTable[sr_no].sample_rate_us);

    if (sr_no >= sizeof(stkODRTable)/sizeof(stkODRTable[0]))
    {
        sr_no = sizeof(stkODRTable)/sizeof(stkODRTable[0]) - 1;
    }
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============sr_no:%d\n", __FILE__, __func__, __LINE__, sr_no);

    stk->sr_no = sr_no;
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============sr_no:%d\n", __FILE__, __func__, __LINE__, sr_no);

    if (atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 0);
        enable = true;
    }
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);

    error = stk_reg_write(stk, STK8XXX_REG_BWSEL, stkODRTable[sr_no].regBwsel);
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);

    if (error)
        dev_err(&stk->client->dev, "%s, failed to change ODR\n", __func__);

    if (enable)
    {
        stk_set_enable(stk, 1);
    }
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);

#ifdef STK_INTERRUPT_MODE
        /* do nothing */
#elif defined STK_POLLING_MODE
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);
    stk->poll_delay = ns_to_ktime(stkODRTable[sr_no].sample_rate_us * NSEC_PER_USEC);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);

    if (stk->fifo) {
        stk->latency_us = stkODRTable[sr_no].sample_rate_us;
    }

    return error;
}

#ifdef STK_FIR
/*
 * @brief: low-pass filter operation
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in/out] xyz: s16 *
 */
static void stk_low_pass_fir(struct stk_data *stk, s16 *xyz)
{
    int firlength = atomic_read(&stk->fir_len);
#ifdef STK_ZG
    s16 avg;
    int jitter_boundary = stk->sensitivity / 128;
#if 0

    if (0 == jitter_boundary)
        jitter_boundary = 1;

#endif
#endif /* STK_ZG */

    if (0 == firlength)
    {
        /* stk_data.fir_len == 0: turn OFF FIR operation */
        return;
    }

    if (firlength > stk->fir.count)
    {
        stk->fir.xyz[stk->fir.idx][0] = xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = xyz[2];
        stk->fir.sum[0] += xyz[0];
        stk->fir.sum[1] += xyz[1];
        stk->fir.sum[2] += xyz[2];
        stk->fir.count++;
        stk->fir.idx++;
    }
    else
    {
        if (firlength <= stk->fir.idx)
            stk->fir.idx = 0;

        stk->fir.sum[0] -= stk->fir.xyz[stk->fir.idx][0];
        stk->fir.sum[1] -= stk->fir.xyz[stk->fir.idx][1];
        stk->fir.sum[2] -= stk->fir.xyz[stk->fir.idx][2];
        stk->fir.xyz[stk->fir.idx][0] = xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = xyz[2];
        stk->fir.sum[0] += xyz[0];
        stk->fir.sum[1] += xyz[1];
        stk->fir.sum[2] += xyz[2];
        stk->fir.idx++;
#ifdef STK_ZG
        avg = stk->fir.sum[0] / firlength;

        if (abs(avg) <= jitter_boundary)
            xyz[0] = avg * ZG_FACTOR;
        else
            xyz[0] = avg;

        avg = stk->fir.sum[1] / firlength;

        if (abs(avg) <= jitter_boundary)
            xyz[1] = avg * ZG_FACTOR;
        else
            xyz[1] = avg;

        avg = stk->fir.sum[2] / firlength;

        if (abs(avg) <= jitter_boundary)
            xyz[2] = avg * ZG_FACTOR;
        else
            xyz[2] = avg;

#else /* STK_ZG */
        xyz[0] = stk->fir.sum[0] / firlength;
        xyz[1] = stk->fir.sum[1] / firlength;
        xyz[2] = stk->fir.sum[2] / firlength;
#endif /* STK_ZG */
    }
}
#endif /* STK_FIR */

/**
 * @brief: read accel raw data from register.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_read_accel_rawdata(struct stk_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk_reg_read(stk, STK8XXX_REG_XOUT1, 0, &dataL))
        return;

    if (stk_reg_read(stk, STK8XXX_REG_XOUT2, 0, &dataH))
        return;

    stk->xyz[0] = dataH << 8 | dataL;
    stk->xyz[0] >>= 4;

    if (stk_reg_read(stk, STK8XXX_REG_YOUT1, 0, &dataL))
        return;

    if (stk_reg_read(stk, STK8XXX_REG_YOUT2, 0, &dataH))
        return;

    stk->xyz[1] = dataH << 8 | dataL;
    stk->xyz[1] >>= 4;

    if (stk_reg_read(stk, STK8XXX_REG_ZOUT1, 0, &dataL))
        return;

    if (stk_reg_read(stk, STK8XXX_REG_ZOUT2, 0, &dataH))
        return;

    stk->xyz[2] = dataH << 8 | dataL;
    stk->xyz[2] >>= 4;

    if (STK8BA50_R_ID == stk->pid) {
        stk->xyz[0] >>= 2;
        stk->xyz[1] >>= 2;
        stk->xyz[2] >>= 2;
    }
}


/*----------------------------add by yxp 20210111----------------------------*/
static void stk_CalData(struct stk_data *stk)
{
	int x,y,z;
	if (cal_lock_flag != 1) 
		return;//不在校准状态
		
	x = stk->xyz[0];
	y = stk->xyz[1];
	z = stk->xyz[2];

	if (abs(x) > XX_MAX_PERM_VALUE 
	|| abs(y) > XX_MAX_PERM_VALUE 
	//|| abs(z) > XX_MAX_PERM_VALUE
	)
	{
		return;//偏差太大，直接忽略
	}

	x_sum += x;
	y_sum += y;
	z_sum += z;

	n_index++;
	if (n_index > XX_MAX_NUM_PER_GROUP)
	{
		m_index++;
	}

	if (n_index > XX_MAX_NUM_PER_GROUP)
	{
		x_offset_cal = ((x_offset_cal*(m_index-1))+(x_sum/n_index))/m_index;
		y_offset_cal = ((y_offset_cal*(m_index-1))+(y_sum/n_index))/m_index;
		z_offset_cal = ((z_offset_cal*(m_index-1))+(z_sum/n_index))/m_index;
	
		printk("[BMA250_CalData:]x_offset_cal:%d,y_offset_cal:%d,z_offset_cal:%d===\n",x_offset_cal,y_offset_cal,z_offset_cal);
	}

	
	if (n_index > XX_MAX_NUM_PER_GROUP)
	{
		x_sum = 0;
		y_sum = 0;
		z_sum = 0;
		n_index = 0;
	}
	if (m_index > XX_MAX_NUM_OF_GROUP)
	{
		m_index = 10;//此处理为避免数值无限变大
	}

}

/*------------------------------------------------*/


/*
 * @brief: read accel raw data from register.
 *          Store result to stk_data.xyz[].
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_read_accel_data(struct stk_data *stk)
{
    int ii = 0;
    s16 coor_trans[3] = {0};
    static int times1_ = 0, times2_ = 0;

    stk_read_accel_rawdata(stk);
    stk->timestamp = ktime_get_boottime();
    
    if(times1_++ >= 1000){
        times1_ = 0;
        dev_info(&stk->client->dev, "times1_:%d\n", times1_);
        dev_info(&stk->client->dev, "%s: xyz before coordinate trans %d %d %d with direction:%d\n",
            __func__, stk->xyz[0], stk->xyz[1], stk->xyz[2], stk->direction);
    }

    for (ii = 0; ii < 3; ii++)
    {
        coor_trans[0] += stk->xyz[ii] * coordinate_trans[stk->direction][0][ii];
        coor_trans[1] += stk->xyz[ii] * coordinate_trans[stk->direction][1][ii];
        coor_trans[2] += stk->xyz[ii] * coordinate_trans[stk->direction][2][ii];
    }
    stk->xyz[0] = coor_trans[0];
    stk->xyz[1] = coor_trans[1];
    stk->xyz[2] = coor_trans[2];
    
		if(cal_lock_flag == 1){
			write_lock(&cal_rwlock);
			stk_CalData(stk);	
			write_unlock(&cal_rwlock);
					
		//	printk("[no_cali data] x=%d, y=%d, z=%d \n",
		//		obj->data[BMA250_AXIS_X],obj->data[BMA250_AXIS_Y],obj->data[BMA250_AXIS_Z]);
			
		}else{
			
			/*-------------------modify by yxp --z平放时z方向是最大值，不能通过简单偏移来校准-----*/
			stk->xyz[0] -= x_offset_cal;
			stk->xyz[1] -= y_offset_cal;
		}	
    
    
    

    if(times2_++ >= 1000){
        times2_ = 0;
        dev_info(&stk->client->dev, "times2_:%d\n", times2_);
        dev_info(&stk->client->dev, "%s: xyz after coordinate trans %d %d %d\n",
            __func__, stk->xyz[0], stk->xyz[1], stk->xyz[2]);
    }
#ifdef STK_FIR
    stk_low_pass_fir(stk, stk->xyz);
#endif /* STK_FIR */
}

#ifdef STK_STEP_COUNTER
/**
 * @brief: read step counter value from register.
 *          Store result to stk_data.steps.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_read_step_data(struct stk_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk_reg_read(stk, STK8XXX_REG_STEPOUT1, 0, &dataL))
        return;

    if (stk_reg_read(stk, STK8XXX_REG_STEPOUT2, 0, &dataH))
        return;

    stk->steps = dataH << 8 | dataL;
}

/**
 * @brief: Turn ON/OFF step count.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] turn: true to turn ON step count; false to turn OFF.
 */
static void stk_turn_step_counter(struct stk_data *stk, bool turn)
{
    if (turn)
    {
        if (stk_reg_write(stk, STK8XXX_REG_STEPCNT2,
                              STK8XXX_STEPCNT2_RST_CNT | STK8XXX_STEPCNT2_STEP_CNT_EN))
            return;
    }
    else
    {
        if (stk_reg_write(stk, STK8XXX_REG_STEPCNT2, 0))
            return;
    }

    stk->steps = 0;
}
#endif /* STK_STEP_COUNTER */

/*
 * @brief: Read all register (0x0 ~ 0x3F)
 *
 * @param[in/out] stk: struct stk_data *
 * @param[out] show_buffer: record all register value
 *
 * @return: buffer length or fail
 *          positive value: return buffer length
 *          -1: Fail
 */
static int stk_show_all_reg(struct stk_data *stk, char *show_buffer)
{
    int reg;
    int len = 0;
    u8 data = 0;

    if (NULL == show_buffer)
        return -1;

    for (reg = 0; reg <= 0x3F; reg++)
    {
        if (stk_reg_read(stk, reg, 0, &data))
        {
            len = -1;
            goto exit;
        }

        if (0 >= (PAGE_SIZE - len))
        {
            dev_err(&stk->client->dev,
                    "%s: print string out of PAGE_SIZE\n", __func__);
            goto exit;
        }

        len += scnprintf(show_buffer + len, PAGE_SIZE - len,
                         "[0x%2X]=0x%2X, ", reg, data);

        if (4 == reg % 5) {
            len += scnprintf(show_buffer + len, PAGE_SIZE - len,
                    "\n");
        }
    }

    len += scnprintf(show_buffer + len, PAGE_SIZE - len, "\n");
exit:

    return len;
}

/*
 * @brief: Get offset
 *
 * @param[in/out] stk: struct stk_data *
 * @param[out] offset: offset value read from register
 *                  STK8XXX_REG_OFSTX,  STK8XXX_REG_OFSTY, STK8XXX_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_get_offset(struct stk_data *stk, u8 offset[3])
{
    if (stk_reg_read(stk, STK8XXX_REG_OFSTX, 0, &offset[0]))
    {
        return -1;
    }

    if (stk_reg_read(stk, STK8XXX_REG_OFSTY, 0, &offset[1]))
    {
        return -1;
    }

    if (stk_reg_read(stk, STK8XXX_REG_OFSTZ, 0, &offset[2]))
    {
        return -1;
    }

    return 0;
}

/*
 * @brief: Set offset
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] offset: offset value write to register
 *                  STK8XXX_REG_OFSTX,  STK8XXX_REG_OFSTY, STK8XXX_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_set_offset(struct stk_data *stk, u8 offset[3])
{
    int error = 0;
    bool enable = false;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk_reg_write(stk, STK8XXX_REG_OFSTX, offset[0]))
    {
        error = -1;
        goto exit;
    }

    if (stk_reg_write(stk, STK8XXX_REG_OFSTY, offset[1]))
    {
        error = -1;
        goto exit;
    }

    if (stk_reg_write(stk, STK8XXX_REG_OFSTZ, offset[2]))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return error;
}

/*
 * @brief: Read PID and write to stk_data.pid.
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_get_pid(struct stk_data *stk)
{
    int error = 0;
    u8 val = 0;
    error = stk_reg_read(stk, STK8XXX_REG_CHIPID, 0, &val);

    if (error)
        dev_err(&stk->client->dev,
                "%s: failed to read PID\n", __func__);
    else
        stk->pid = (int)val;

    switch (stk->pid) {
        case STK8323_ID:
        case STK8325_ID:
            stk->fifo = true;
            break;
        case STK8BA50_R_ID:
        case STK8BA53_ID:
        default:
            stk->fifo = false;
            break;
    }

    return error;
}

/*
 * @brief: Updata fifo_start_ns
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_set_fifo_start_time(struct stk_data *stk)
{
    struct timespec ts;

    get_monotonic_boottime(&ts);
    stk->fifo_start_ns = timespec_to_ns(&ts);
}

/*
 * @brief: Read FIFO data
 *
 * @param[in/out] stk: struct stk_data *
 * @param[out] fifo: FIFO data
 * @param[in] len: FIFO size what you want to read
 */
static void stk_fifo_reading(struct stk_data *stk, u8 fifo[], int len)
{
    static int time1_ = 0, time2_ = 0;
    if (!stk->fifo) {
        dev_err(&stk->client->dev, "%s: No fifo data\n", __func__);
        return;
    }
    if(time1_++ >= 1000){
        time1_ = 0;
        /* Reject all register R/W to protect FIFO data reading */
        dev_info(&stk->client->dev, "%s: Start to read FIFO data\n", __func__);
    }

    if (stk_reg_read(stk, STK8XXX_REG_FIFOOUT, len, fifo))
    {
        dev_err(&stk->client->dev, "%s: Break to read FIFO data\n", __func__);
    }
    if(time2_++ >= 1000){
        time2_ = 0;
        dev_info(&stk->client->dev, "%s: Done for reading FIFO data\n", __func__);
    }
}

/*
 * @brief: Change FIFO status
 *          If wm = 0, change FIFO to bypass mode.
 *          STK8XXX_CFG1_XYZ_FRAME_MAX >= wm, change FIFO to FIFO mode +
 *                                          STK8XXX_CFG2_FIFO_DATA_SEL_XYZ.
 *          Do nothing if STK8XXX_CFG1_XYZ_FRAME_MAX < wm.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] wm: water mark
 *
 * @return: Success or fail
 *          0: Success
 *          Others: Fail
 */
static int stk_change_fifo_status(struct stk_data *stk, u8 wm)
{
    int error = 0;

    if (!stk->fifo) {
        return -1;
    }

    if (STK8XXX_CFG1_XYZ_FRAME_MAX < wm)
    {
        dev_err(&stk->client->dev,
                "%s: water mark out of range(%d).\n", __func__, wm);
        return -1;
    }

#ifdef STK_INTERRUPT_MODE
    u8 regIntmap2 = 0, regInten2 = 0;

    error = stk_reg_read(stk, STK8XXX_REG_INTMAP2, 0, &regIntmap2);
    if (error)
        return error;

    error = stk_reg_read(stk, STK8XXX_REG_INTEN2, 0, &regInten2);
    if (error)
        return error;
#endif /* STK_INTERRUPT_MODE */

    if (wm)
    {
        /* FIFO settings: FIFO mode + XYZ per frame */
        error = stk_reg_write(stk, STK8XXX_REG_CFG2,
                                  (STK8XXX_CFG2_FIFO_MODE_FIFO << STK8XXX_CFG2_FIFO_MODE_SHIFT)
                                  | STK8XXX_CFG2_FIFO_DATA_SEL_XYZ);
        if (error)
            return error;

#ifdef STK_INTERRUPT_MODE
        error = stk_reg_write(stk, STK8XXX_REG_INTMAP2, regIntmap2 | STK8XXX_INTMAP2_FWM2INT1);
        if (error)
            return error;

        error = stk_reg_write(stk, STK8XXX_REG_INTEN2, regInten2 | STK8XXX_INTEN2_FWM_EN);
        if (error)
            return error;
#endif /* STK_INTERRUPT_MODE */
    }
    else
    {
        /* FIFO settings: bypass mode */
        error = stk_reg_write(stk, STK8XXX_REG_CFG2,
                                  STK8XXX_CFG2_FIFO_MODE_BYPASS << STK8XXX_CFG2_FIFO_MODE_SHIFT);
        if (error)
            return error;

#ifdef STK_INTERRUPT_MODE
        error = stk_reg_write(stk, STK8XXX_REG_INTMAP2, regIntmap2 & (~STK8XXX_INTMAP2_FWM2INT1));
        if (error)
            return error;

        error = stk_reg_write(stk, STK8XXX_REG_INTEN2, regInten2 & (~STK8XXX_INTEN2_FWM_EN));
        if (error)
            return error;
#endif /* STK_INTERRUPT_MODE */
    }

    error = stk_reg_write(stk, STK8XXX_REG_CFG1, wm);

    if (error)
        return error;

    stk_set_fifo_start_time(stk);

    return 0;
}

/*
 * @brief: SW reset for stk8xxx
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_sw_reset(struct stk_data *stk)
{
    int error = 0;
    error = stk_reg_write(stk, STK8XXX_REG_SWRST, STK8XXX_SWRST_VAL);

    if (error)
        return error;

    usleep_range(1000, 2000);
    stk->power_mode = STK8XXX_PWMD_NORMAL;
    atomic_set(&stk->enabled, 1);
    return 0;
}

/*
 * @brief: stk8xxx register initialize
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] range: range for STK8XXX_REG_RANGESEL
 *              STK_2G
 *              STK_4G
 *              STK_8G
 * @param[in] sr_no: Serial number of stkODRTable.
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_reg_init(struct stk_data *stk, stk_rangesel range, int sr_no)
{
    int error = 0;
    /* SW reset */
    error = stk_sw_reset(stk);

    if (error)
        return error;

    /* INT1, push-pull, active high. */
    error = stk_reg_write(stk, STK8XXX_REG_INTCFG1,
                              STK8XXX_INTCFG1_INT1_ACTIVE_H | STK8XXX_INTCFG1_INT1_OD_PUSHPULL);

    if (error)
        return error;

#ifdef STK_INTERRUPT_MODE
    /* map new accel data interrupt to int1 */
    error = stk_reg_write(stk, STK8XXX_REG_INTMAP2, STK8XXX_INTMAP2_DATA2INT1);

    if (error)
        return error;

    /* enable new data interrupt for both new accel data */
    error = stk_reg_write(stk, STK8XXX_REG_INTEN2, STK8XXX_INTEN2_DATA_EN);

    if (error)
        return error;
#endif /* STK_INTERRUPT_MODE */

#ifdef STK_AMD

    /* enable new data interrupt for any motion */
    error = stk_reg_write(stk, STK8XXX_REG_INTEN1,
                              STK8XXX_INTEN1_SLP_EN_XYZ);

    if (error)
        return error;

    /* map any motion interrupt to int1 */
    error = stk_reg_write(stk, STK8XXX_REG_INTMAP1,
                              STK8XXX_INTMAP1_ANYMOT2INT1);

    if (error)
        return error;

    /* SIGMOT2 */
    error = stk_reg_write(stk, STK8XXX_REG_SIGMOT2,
                              STK8XXX_SIGMOT2_ANY_MOT_EN);

    if (error)
        return error;

    /*
     * latch int
     * In interrupt mode + significant mode, both of them share the same INT.
     * Set latched to make sure we can get ANY data(ANY_MOT_STS) before signal fall down.
     * Read ANY flow:
     * Get INT --> check INTSTS1.ANY_MOT_STS status -> INTCFG2.INT_RST(relese all latched INT)
     * Read FIFO flow:
     * Get INT --> check INTSTS2.FWM_STS status -> INTCFG2.INT_RST(relese all latched INT)
     * In latch mode, echo interrupt(SIT_MOT_STS/FWM_STS) will cause all INT(INT1/INT2)
     * rising up.
     */
    error = stk_reg_write(stk, STK8XXX_REG_INTCFG2, STK8XXX_INTCFG2_LATCHED);

    if (error)
        return error;
#else /* STK_AMD */

    /* non-latch int */
    error = stk_reg_write(stk, STK8XXX_REG_INTCFG2, STK8XXX_INTCFG2_NOLATCHED);

    if (error)
        return error;

    /* SIGMOT2 */
    error = stk_reg_write(stk, STK8XXX_REG_SIGMOT2, 0);

    if (error)
        return error;
#endif /* STK_AMD */

    /* SLOPE DELAY */
    error = stk_reg_write(stk, STK8XXX_REG_SLOPEDLY, 0x00);

    if (error)
        return error;

    /* SLOPE THRESHOLD */
    error = stk_reg_write(stk, STK8XXX_REG_SLOPETHD, STK8XXX_SLOPETHD_DEF);

    if (error)
        return error;

    /* SIGMOT1 */
    error = stk_reg_write(stk, STK8XXX_REG_SIGMOT1,
                              STK8XXX_SIGMOT1_SKIP_TIME_3SEC);

    if (error)
        return error;

    /* SIGMOT3 */
    error = stk_reg_write(stk, STK8XXX_REG_SIGMOT3,
                              STK8XXX_SIGMOT3_PROOF_TIME_1SEC);

    if (error)
        return error;

    /* According to STK_DEF_DYNAMIC_RANGE */
    error = stk_range_selection(stk, range);

    if (error)
        return error;

    /* ODR */
    error = stk_reg_write(stk, STK8XXX_REG_BWSEL, stkODRTable[sr_no].regBwsel);

    if (error)
        return error;
    stk->sr_no = sr_no;

    if (stk->fifo) {
        stk->latency_us = stkODRTable[sr_no].sample_rate_us;
    }

    if (stk->fifo) {
        stk_change_fifo_status(stk, 0);
    }
    /* i2c watchdog enable */
    error = stk_reg_write(stk, STK8XXX_REG_INTFCFG,
                              STK8XXX_INTFCFG_I2C_WDT_EN);

    /* power to suspend mode */
    error = stk_reg_write(stk, STK8XXX_REG_POWMODE, STK8XXX_PWMD_SUSPEND);
    if (error)
        return error;
    atomic_set(&stk->enabled, 0);

    if (error)
        return error;

    return 0;
}

#ifdef STK_CALI
/*
 * @brief: Verify offset.
 *          Read register of STK8XXX_REG_OFSTx, then check data are the same as
 *          what we wrote or not.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] offset: offset value to compare with the value in register
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 */
static int stk_verify_offset(struct stk_data *stk, u8 offset[3])
{
    int axis;
    u8 offset_from_reg[3] = {0, 0, 0};

    if (stk_get_offset(stk, offset_from_reg))
        return STK_K_FAIL_I2C;

    for (axis = 0; axis < 3; axis++)
    {
        if (offset_from_reg[axis] != offset[axis])
        {
            dev_err(&stk->client->dev,
                    "%s: set OFST failed! offset[%d]=%d, read from reg[%d]=%d\n",
                    __func__, axis, offset[axis], axis, offset_from_reg[axis]);
            atomic_set(&stk->cali_status, STK_K_FAIL_WRITE_OFST);
            return STK_K_FAIL_WRITE_OFST;
        }
    }

    return 0;
}

#ifdef STK_CALI_FILE
/*
 * @brief: Write calibration config file to STK_CALI_FILE_PATH.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] w_buf: cali data what want to write to STK_CALI_FILE_PATH.
 * @param[in] buf_size: size of w_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_write_to_file(struct stk_data *stk,
                             char *w_buf, int8_t buf_size)
{
    struct file *cali_file;
    char r_buf[buf_size];
    mm_segment_t fs;
    ssize_t ret;
    int i;
    cali_file = filp_open(STK_CALI_FILE_PATH, O_CREAT | O_RDWR, 0666);

    if (IS_ERR(cali_file))
    {
        dev_err(&stk->client->dev,
                "%s: err=%ld, failed to open %s\n",
                __func__, PTR_ERR(cali_file), STK_CALI_FILE_PATH);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->write(cali_file, w_buf, buf_size,
                                     &cali_file->f_pos);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: write error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        cali_file->f_pos = 0x0;
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: read error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        set_fs(fs);

        for (i = 0; i < buf_size; i++)
        {
            if (r_buf[i] != w_buf[i])
            {
                dev_err(&stk->client->dev,
                        "%s: read back error! r_buf[%d]=0x%X, w_buf[%d]=0x%X\n",
                        __func__, i, r_buf[i], i, w_buf[i]);
                filp_close(cali_file, NULL);
                return -1;
            }
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/*
 * @brief: Get calibration config file from STK_CALI_FILE_PATH.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[out] r_buf: cali data what want to read from STK_CALI_FILE_PATH.
 * @param[in] buf_size: size of r_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_get_from_file(struct stk_data *stk,
                             char *r_buf, int8_t buf_size)
{
    struct file *cali_file;
    mm_segment_t fs;
    ssize_t ret;
    cali_file = filp_open(STK_CALI_FILE_PATH, O_RDONLY, 0);

    if (IS_ERR(cali_file))
    {
        dev_err(&stk->client->dev,
                "%s: err=%ld, failed to open %s\n",
                __func__, PTR_ERR(cali_file), STK_CALI_FILE_PATH);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);
        set_fs(fs);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: read error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/*
 * @brief: Write calibration data to config file
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] offset: offset value
 * @param[in] status: status
 *                  STK_K_SUCCESS_FILE
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_write_cali_to_file(struct stk_data *stk,
                                  u8 offset[3], u8 status)
{
    char file_buf[STK_CALI_FILE_SIZE];
    memset(file_buf, 0, sizeof(file_buf));
    file_buf[0] = STK_CALI_VER0;
    file_buf[1] = STK_CALI_VER1;
    file_buf[3] = offset[0];
    file_buf[5] = offset[1];
    file_buf[7] = offset[2];
    file_buf[8] = status;
    file_buf[STK_CALI_FILE_SIZE - 2] = '\0';
    file_buf[STK_CALI_FILE_SIZE - 1] = STK_CALI_END;

    if (stk_write_to_file(stk, file_buf, STK_CALI_FILE_SIZE))
        return -1;

    return 0;
}

/*
 * @brief: Get calibration data and status.
 *          Set cali status to stk_data.cali_status.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[out] r_buf: cali data what want to read from STK_CALI_FILE_PATH.
 * @param[in] buf_size: size of r_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static void stk_get_cali(struct stk_data *stk)
{
    char stk_file[STK_CALI_FILE_SIZE];

    if (stk_get_from_file(stk, stk_file, STK_CALI_FILE_SIZE) == 0)
    {
        if (STK_CALI_VER0 == stk_file[0]
            && STK_CALI_VER1 == stk_file[1]
            && STK_CALI_END == stk_file[STK_CALI_FILE_SIZE - 1])
        {
            atomic_set(&stk->cali_status, (int)stk_file[8]);
            stk->offset[0] = (u8)stk_file[3];
            stk->offset[1] = (u8)stk_file[5];
            stk->offset[2] = (u8)stk_file[7];
            dev_info(&stk->client->dev, "%s: offset:%d,%d,%d, mode=0x%X\n",
                     __func__, stk_file[3], stk_file[5], stk_file[7],
                     stk_file[8]);
            dev_info(&stk->client->dev, "%s: variance=%u,%u,%u\n", __func__,
                     (stk_file[9] << 24 | stk_file[10] << 16 | stk_file[11] << 8 | stk_file[12]),
                     (stk_file[13] << 24 | stk_file[14] << 16 | stk_file[15] << 8 | stk_file[16]),
                     (stk_file[17] << 24 | stk_file[18] << 16 | stk_file[19] << 8 | stk_file[20]));
        }
        else
        {
            int i;
            dev_err(&stk->client->dev, "%s: wrong cali version number\n",
                    __func__);

            for (i = 0; i < STK_CALI_FILE_SIZE; i++)
                dev_info(&stk->client->dev, "%s:cali_file[%d]=0x%X\n",
                         __func__, i, stk_file[i]);
        }
    }
}
#endif /* STK_CALI_FILE */

/*
 * @brief: Get sample_no of samples then calculate average
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] delay_ms: delay in msec
 * @param[in] sample_no: amount of sample
 * @param[out] acc_ave: XYZ average
 */
static void stk_calculate_average(struct stk_data *stk,
                                  unsigned int delay_ms, int sample_no, int acc_ave[3])
{
    int i;

    for (i = 0; i < sample_no; i++)
    {
        msleep(delay_ms);
        stk_read_accel_data(stk);
        acc_ave[0] += stk->xyz[0];
        acc_ave[1] += stk->xyz[1];
        acc_ave[2] += stk->xyz[2];
    }

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (i = 0; i < 3; i++)
    {
        if ( 0 <= acc_ave[i])
            acc_ave[i] = (acc_ave[i] + sample_no / 2) / sample_no;
        else
            acc_ave[i] = (acc_ave[i] - sample_no / 2) / sample_no;
    }

    /*
     * For Z-axis
     * Pre-condition: Sensor be put on a flat plane, with +z face up.
     */
    if (0 < acc_ave[2])
        acc_ave[2] -= stk->sensitivity;
    else
        acc_ave[2] += stk->sensitivity;
}

/*
 * @brief: Align STK8XXX_REG_OFSTx sensitivity with STK8XXX_REG_RANGESEL
 *  Description:
 *  Example:
 *      RANGESEL=0x3 -> +-2G / 12bits for STK8xxx full resolution
 *              number bit per G = 2^12 / (2x2) = 1024 (LSB/g)
 *              (2x2) / 2^12 = 0.97 mG/bit
 *      OFSTx: There are 8 bits to describe OFSTx for +-1G
 *              number bit per G = 2^8 / (1x2) = 128 (LSB/g)
 *              (1x2) / 2^8 = 7.8125mG/bit
 *      Align: acc_OFST = acc * 128 / 1024
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in/out] acc: accel data
 *
 */
static void stk_align_offset_sensitivity(struct stk_data *stk, int acc[3])
{
    int axis;

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (axis = 0; axis < 3; axis++)
    {
        if (acc[axis] > 0)
        {
            acc[axis] = (acc[axis] * STK8XXX_OFST_LSB + stk->sensitivity / 2)
                        / stk->sensitivity;
        }
        else
        {
            acc[axis] = (acc[axis] * STK8XXX_OFST_LSB - stk->sensitivity / 2)
                        / stk->sensitivity;
        }
    }
}

/*
 * @brief: Calibration action
 *          1. Calculate calibration data
 *          2. Write data to STK8XXX_REG_OFSTx
 *          3. Check calibration well-done with chip register
 *          4. Write calibration data to file
 *          Pre-condition: Sensor be put on a flat plane, with +z face up.
 *
 * @param[in/out] stk: struct stk_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 *          STK_K_FAIL_W_FILE: fail during writing cali to file
 */
static int stk_cali_do(struct stk_data *stk, int delay_us)
{
    int error = 0;
    int acc_ave[3] = {0, 0, 0};
    unsigned int delay_ms = delay_us / 1000;
    u8 offset[3] = {0, 0, 0};
    int acc_verify[3] = {0, 0, 0};
    const unsigned char verify_diff = stk->sensitivity / 10;
    int axis;

    stk_calculate_average(stk, delay_ms, STK_CALI_SAMPLE_NO, acc_ave);
    stk_align_offset_sensitivity(stk, acc_ave);

#ifdef STK_AUTOK
    dev_info(&stk->client->dev, "%s: acc_ave:%d,%d,%d\n",
            __func__, acc_ave[0], acc_ave[1], acc_ave[2]);
    for (axis = 0; axis < 3; axis++) {
        int threshold = STK8XXX_OFST_LSB * 3 / 10; // 300mG
        if (abs(acc_ave[axis] > threshold)) {
            dev_err(&stk->client->dev, "%s: abs(%d) > %d(300mG)\n",
                    __func__, acc_ave[axis], threshold);
            return STK_K_FAIL_VERIFY_CALI;
        }
    }
#endif /* STK_AUTOK */

    stk->cali_sw[0] = (s16)acc_ave[0];
    stk->cali_sw[1] = (s16)acc_ave[1];
    stk->cali_sw[2] = (s16)acc_ave[2];

    for (axis = 0; axis < 3; axis++)
        offset[axis] = -acc_ave[axis];

    dev_info(&stk->client->dev, "%s: New offset for XYZ: %d, %d, %d\n",
             __func__, acc_ave[0], acc_ave[1], acc_ave[2]);
    error = stk_set_offset(stk, offset);

    if (error)
        return STK_K_FAIL_I2C;

    /* Read register, then check OFSTx are the same as we wrote or not */
    error = stk_verify_offset(stk, offset);

    if (error)
        return error;

    /* verify cali */
    stk_calculate_average(stk, delay_ms, 3, acc_verify);

    if (verify_diff < abs(acc_verify[0]) || verify_diff < abs(acc_verify[1])
        || verify_diff < abs(acc_verify[2]))
    {
        dev_err(&stk->client->dev, "%s: Check data x:%d, y:%d, z:%d. Check failed!\n",
                __func__, acc_verify[0], acc_verify[1], acc_verify[2]);
        return STK_K_FAIL_VERIFY_CALI;
    }

#ifdef STK_CALI_FILE
    /* write cali to file */
    error = stk_write_cali_to_file(stk, offset, STK_K_SUCCESS_FILE);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: failed to stk_write_cali_to_file, error=%d\n",
                __func__, error);
        return STK_K_FAIL_W_FILE;
    }
#endif /* STK_CALI_FILE */

    atomic_set(&stk->cali_status, STK_K_SUCCESS_FILE);
    return 0;
}

/*
 * @brief: Set calibration
 *          1. Change delay to 8000msec
 *          2. Reset offset value by trigger OFST_RST
 *          3. Calibration action
 *          4. Change delay value back
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_set_cali(struct stk_data *stk)
{
    int error = 0;
    bool enable;
    int org_delay_us, real_delay_us;
    atomic_set(&stk->cali_status, STK_K_RUNNING);
    org_delay_us = stk_get_delay(stk);
    /* Use several samples (with ODR:125) for calibration data base */
    error = stk_set_delay(stk, 8000);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: failed to stk_set_delay, error=%d\n", __func__, error);
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        return;
    }

    real_delay_us = stk_get_delay(stk);

    /* SW reset before getting calibration data base */
    if (atomic_read(&stk->enabled))
    {
        enable = true;
        stk_set_enable(stk, 0);
    }
    else {
        enable = false;
    }

    stk_set_enable(stk, 1);
    error = stk_reg_write(stk, STK8XXX_REG_OFSTCOMP1,
                              STK8XXX_OFSTCOMP1_OFST_RST);

    if (error)
    {
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        goto exit_for_OFST_RST;
    }

    /* Action for calibration */
    error = stk_cali_do(stk, real_delay_us);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: failed to stk_cali_do, error=%d\n",
                __func__, error);
        atomic_set(&stk->cali_status, error);
        goto exit_for_OFST_RST;
    }

    dev_info(&stk->client->dev, "%s: successful calibration\n", __func__);
exit_for_OFST_RST:

    if (!enable) {
        stk_set_enable(stk, 0);
    }

    stk_set_delay(stk, org_delay_us);
}

#ifdef STK_AUTOK
/*
 * @brief: Auto cali if there is no any cali files.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_autok(struct stk_data *stk) {
    dev_info(&stk->client->dev, "%s\n", __func__);
#ifdef STK_CALI_FILE
    stk_get_cali(stk);
#endif /* STK_CALI_FILE */
    if (STK_K_SUCCESS_FILE == atomic_read(&stk->cali_status)) {
        dev_info(&stk->client->dev, "%s: get offset from cali: %d %d %d\n",
                __func__, stk->offset[0], stk->offset[1], stk->offset[2]);
        atomic_set(&stk->first_enable, 0);
        stk_set_offset(stk, stk->offset);
    } else {
        stk_set_cali(stk);
    }
}
#endif /* STK_AUTOK */

/*
 * @brief: Get calibration status
 *          Send calibration status to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);

#ifdef STK_CALI_FILE
    if (STK_K_RUNNING != atomic_read(&stk->cali_status))
        stk_get_cali(stk);
#endif /* STK_CALI_FILE */

    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", atomic_read(&stk->cali_status));
}

/*
 * @brief: Trigger to calculate calibration data
 *          Get 1 from userspace, then start to calculate calibration data.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);

    if (sysfs_streq(buf, "1"))
        stk_set_cali(stk);
    else
    {
        dev_err(&stk->client->dev, "%s: invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }

    return count;
}
#endif /* STK_CALI */

/**
 * @brief: Selftest for XYZ offset and noise.
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static char stk_testOffsetNoise(struct stk_data *stk)
{
    int read_delay_ms = 8; /* 125Hz = 8ms */
    int acc_ave[3] = {0, 0, 0};
    int acc_min[3] = {INT_MAX, INT_MAX, INT_MAX};
    int acc_max[3] = {INT_MIN, INT_MIN, INT_MIN};
    int noise[3] = {0, 0, 0};
    int sn = 0, axis = 0;
    int thresholdOffset, thresholdNoise;
    u8 localResult = 0;

    if (stk_sw_reset(stk))
        return -1;

    atomic_set(&stk->enabled, 1);

    if (stk_reg_write(stk, STK8XXX_REG_BWSEL, 0x0B)) /* ODR = 125Hz */
        return -1;

    if (stk_range_selection(stk, STK8XXX_RANGESEL_2G))
        return -1;

    thresholdOffset = stk_selftest_offset_factor(stk->sensitivity);
    thresholdNoise = stk_selftest_noise_factor(stk->sensitivity);

    for (sn = 0; sn < STK_SELFTEST_SAMPLE_NUM; sn++)
    {
        msleep(read_delay_ms);
        stk_read_accel_rawdata(stk);
        dev_info(&stk->client->dev, "%s: acc = %d, %d, %d\n", __func__,
                stk->xyz[0], stk->xyz[1], stk->xyz[2]);

        for (axis = 0; axis < 3; axis++)
        {
            acc_ave[axis] += stk->xyz[axis];

            if (stk->xyz[axis] > acc_max[axis])
                acc_max[axis] = stk->xyz[axis];

            if (stk->xyz[axis] < acc_min[axis])
                acc_min[axis] = stk->xyz[axis];
        }
    }

    for (axis = 0; axis < 3; axis++)
    {
        acc_ave[axis] /= STK_SELFTEST_SAMPLE_NUM;
        noise[axis] = acc_max[axis] - acc_min[axis];
    }

    dev_info(&stk->client->dev, "%s: acc_ave=%d, %d, %d, noise=%d, %d, %d\n",
            __func__, acc_ave[0], acc_ave[1], acc_ave[2], noise[0], noise[1], noise[2]);
    dev_info(&stk->client->dev, "%s: offset threshold=%d, noise threshold=%d\n",
            __func__, thresholdOffset, thresholdNoise);

    if (0 < acc_ave[2])
        acc_ave[2] -= stk->sensitivity;
    else
        acc_ave[2] += stk->sensitivity;

    if (0 == acc_ave[0] && 0 == acc_ave[1] && 0 == acc_ave[2])
        localResult |= STK_SELFTEST_RESULT_NO_OUTPUT;

    if (thresholdOffset <= abs(acc_ave[0])
            || 0 == noise[0] || thresholdNoise <= noise[0])
        localResult |= STK_SELFTEST_RESULT_FAIL_X;

    if (thresholdOffset <= abs(acc_ave[1])
            || 0 == noise[1] || thresholdNoise <= noise[1])
        localResult |= STK_SELFTEST_RESULT_FAIL_Y;

    if (thresholdOffset <= abs(acc_ave[2])
            || 0 == noise[2] || thresholdNoise <= noise[2])
        localResult |= STK_SELFTEST_RESULT_FAIL_Z;

    if (0 == localResult)
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NO_ERROR);
    else
        atomic_set(&stk->selftest, localResult);
    return 0;
}

/**
 * @brief: SW selftest function.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_selftest(struct stk_data *stk)
{
    int i = 0;
    u8 data = 0;

    dev_info(&stk->client->dev, "%s\n", __func__);

    atomic_set(&stk->selftest, STK_SELFTEST_RESULT_RUNNING);

    /* Check PID */
    if (stk_get_pid(stk))
    {
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
        return;
    }

    dev_info(&stk->client->dev, "%s: PID 0x%x\n", __func__, stk->pid);

    for (i = 0; i < (sizeof(STK_ID)/sizeof(STK_ID[0])); i++) {
        if (STK_ID[i] == stk->pid) {
            break;
        }
        if (sizeof(STK_ID)/sizeof(STK_ID[0])-1 == i) {
            atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
            return;
        }
    }

    /* Touch all register */
    for (i = 0; i <= 0x3F; i++)
    {
        if (stk_reg_read(stk, i, 0, &data))
        {
            atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
            return;
        }

        dev_info(&stk->client->dev, "%s: [0x%2X]=0x%2X\n", __func__, i, data);
    }

    if (stk_testOffsetNoise(stk))
    {
        atomic_set(&stk->selftest, STK_SELFTEST_RESULT_DRIVER_ERROR);
    }

    stk_reg_init(stk, STK8XXX_RANGESEL_DEF, 1);
}

/*
 * @brief: Get power status
 *          Send 0 or 1 to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    char en;
    en = atomic_read(&stk->enabled);
    return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

/*
 * @brief: Set power status
 *          Get 0 or 1 from userspace, then set stk8xxx power status.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    unsigned int data;
    int error;
    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoul failed, error=%d\n",
                __func__, error);
        return error;
    }

    if ((1 == data) || (0 == data))
        stk_set_enable(stk, data);
    else
        dev_err(&stk->client->dev, "%s: invalid argument, en=%d\n",
                __func__, data);

    return count;
}

/*
 * @brief: Get accel data
 *          Send accel data to userspce.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_value_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    stk_read_accel_data(stk);
    return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
                     stk->xyz[0], stk->xyz[1], stk->xyz[2]);
}

/*
 * @brief: Get delay value in usec
 *          Send delay in usec to userspce.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)stk_get_delay(stk) * 1000);
}

/*
 * @brief: Set delay value in usec
 *          Get delay value in usec from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_store(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    long long data;
    int error;
    error = kstrtoll(buf, 10, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoul failed, error=%d\n",
                __func__, error);
        return error;
    }
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);
#ifdef STK_SPREADTRUM
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);
    stk_set_delay(stk, (int)(data * 1000));
#else /* not STK_SPREADTRUM */
    dev_info(&stk->client->dev, "FILE:%s, FUNC:%s, LINE:%d:============\n", __FILE__, __func__, __LINE__);
    stk_set_delay(stk, (int)(data / 1000));
#endif /* STK_SPREADTRUM */
    return count;
}

/*
 * @brief: Get offset value
 *          Send X/Y/Z offset value to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    u8 offset[3] = {0, 0, 0};
    stk_get_offset(stk, offset);
    return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X\n",
                     offset[0], offset[1], offset[2]);
}

/*
 * @brief: Set offset value
 *          Get X/Y/Z offset value from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    char *token[10];
    u8 r_offset[3];
    int error, data, i;

    for (i = 0; i < 3; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[0] = (u8)data;
    error = kstrtoint(token[1], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[1] = (u8)data;
    error = kstrtoint(token[2], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[2] = (u8)data;
    dev_info(&stk->client->dev, "%s: offset=0x%X, 0x%X, 0x%X\n", __func__,
             r_offset[0], r_offset[1], r_offset[2]);
    stk_set_offset(stk, r_offset);
    return count;
}

/*
 * @brief: Register writting
 *          Get address and content from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_send_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    char *token[10];
    int addr, cmd, error, i;
    bool enable = false;

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &addr);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    error = kstrtoint(token[1], 16, &cmd);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    dev_info(&stk->client->dev, "%s, write reg[0x%X]=0x%X\n",
             __func__, addr, cmd);

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk_reg_write(stk, (u8)addr, (u8)cmd))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/*
 * @brief: Read stk_data.recv(from stk_recv_store), then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "0x%X\n", atomic_read(&stk->recv));
}

/*
 * @brief: Get the read address from userspace, then store the result to
 *          stk_data.recv.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int addr, error;
    u8 data = 0;
    bool enable = false;
    error = kstrtoint(buf, 16, &addr);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk_reg_read(stk, (u8)addr, 0, &data))
    {
        error = -1;
        goto exit;
    }

    atomic_set(&stk->recv, data);
    dev_info(&stk->client->dev, "%s: read reg[0x%X]=0x%X\n",
             __func__, addr, data);
exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/*
 * @brief: Read all register value, then send result to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_allreg_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int result;
    result = stk_show_all_reg(stk, buf);

    if (0 >  result)
        return result;

    return (ssize_t)result;
}

/*
 * @brief: Check PID, then send chip number to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_chipinfo_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);

    switch (stk->pid) {
        case STK8BA50_R_ID:
            return scnprintf(buf, PAGE_SIZE, "stk8ba50-r\n");
        case STK8BA53_ID:
            return scnprintf(buf, PAGE_SIZE, "stk8ba53\n");
        case STK8323_ID:
            return scnprintf(buf, PAGE_SIZE, "stk8321/8323\n");
        case STK8325_ID:
            return scnprintf(buf, PAGE_SIZE, "stk8325\n");
        default:
            return scnprintf(buf, PAGE_SIZE, "unknown\n");
    }

    return scnprintf(buf, PAGE_SIZE, "unknown\n");
}

/*
 * @brief: Read FIFO data, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    u8 fifo_wm = 0;
    u8 frame_unit = 0;
    int fifo_len, len = 0;

    if (!stk->fifo) {
        return scnprintf(buf, PAGE_SIZE , "No fifo data\n");
    }

    if (stk_reg_read(stk, STK8XXX_REG_FIFOSTS, 0, &fifo_wm))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO cnt\n");

    fifo_wm &= STK8XXX_FIFOSTS_FIFO_FRAME_CNT_MASK;
    if (0 == fifo_wm)
        return scnprintf(buf, PAGE_SIZE , "no fifo data yet\n");

    if (stk_reg_read(stk, STK8XXX_REG_CFG2, 0, &frame_unit))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");

    frame_unit &= STK8XXX_CFG2_FIFO_DATA_SEL_MASK;

    if (0 == frame_unit)
        fifo_len = fifo_wm * 6; /* xyz * 2 bytes/axis */
    else
        fifo_len = fifo_wm * 2; /* single axis * 2 bytes/axis */

    {
        u8 *fifo = NULL;
        int i;
        /* vzalloc: allocate memory and set to zero. */
        fifo = vzalloc(sizeof(u8) * fifo_len);

        if (!fifo)
        {
            dev_err(&stk->client->dev, "%s: memory allocation error\n",
                    __func__);
            return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");
        }

        stk_fifo_reading(stk, fifo, fifo_len);

        for (i = 0; i < fifo_wm; i++)
        {
            if (0 == frame_unit)
            {
                s16 x, y, z;
                x = fifo[i * 6 + 1] << 8 | fifo[i * 6];
                x >>= 4;
                y = fifo[i * 6 + 3] << 8 | fifo[i * 6 + 2];
                y >>= 4;
                z = fifo[i * 6 + 5] << 8 | fifo[i * 6 + 4];
                z >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth x:%d, y:%d, z:%d\n", i, x, y, z);
            }
            else
            {
                s16 xyz;
                xyz = fifo[i * 2 + 1] << 8 | fifo[i * 2];
                xyz >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth fifo:%d\n", i, xyz);
            }

            if ( 0 >= (PAGE_SIZE - len))
            {
                dev_err(&stk->client->dev,
                        "%s: print string out of PAGE_SIZE\n", __func__);
                break;
            }
        }

        vfree(fifo);
    }
    return len;
}

/*
 * @brief: Read water mark from userspace, then send to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int wm, error;

    if (!stk->fifo) {
        dev_err(&stk->client->dev, "%s: not support fifo\n", __func__);
        return count;
    }

    error = kstrtoint(buf, 10, &wm);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (stk_change_fifo_status(stk, (u8)wm))
    {
        return -1;
    }

    return count;
}

/**
 *
 */
static ssize_t stk_selftest_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    u8 result = atomic_read(&stk->selftest);

    if (STK_SELFTEST_RESULT_NA == result)
        return scnprintf(buf, PAGE_SIZE, "No result\n");
    if (STK_SELFTEST_RESULT_RUNNING == result)
        return scnprintf(buf, PAGE_SIZE, "selftest is running\n");
    else if (STK_SELFTEST_RESULT_NO_ERROR == result)
        return scnprintf(buf, PAGE_SIZE, "No error\n");
    else
        return scnprintf(buf, PAGE_SIZE, "Error code:0x%2X\n", result);
}

/**
 *
 */
static ssize_t stk_selftest_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);

    stk_selftest(stk);

    return count;
}

#ifdef STK_STEP_COUNTER
/*
 * @brief: Read step counter data, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */

static ssize_t stk_step_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    /*
        bool enable = true;

        if (!atomic_read(&stk->enabled))
        {
            stk_set_enable(stk, 1);
            enable = false;
        }
    */
    stk_read_step_data(stk);
    /*
        if (!enable)
            stk_set_enable(stk, 0);
    */
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk->steps);
}

/*
 * @brief: Read step counter setting from userspace, then send to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_step_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int step, error;
    error = kstrtoint(buf, 10, &step);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (step)
        stk_turn_step_counter(stk, true);
    else
        stk_turn_step_counter(stk, false);

    return count;
}
#endif /* STK_STEP_COUNTER */

#ifdef STK_FIR
/*
 * @brief: Get FIR parameter, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int len = atomic_read(&stk->fir_len);

    if (len)
    {
        dev_info(&stk->client->dev, "FIR count=%2d, idx=%2d\n",
                 stk->fir.count, stk->fir.idx);
        dev_info(&stk->client->dev, "sum = [\t%d \t%d \t%d]\n",
                 stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2]);
        dev_info(&stk->client->dev, "avg = [\t%d \t%d \t%d]\n",
                 stk->fir.sum[0] / len, stk->fir.sum[1] / len, stk->fir.sum[2] / len);
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

/*
 * @brief: Get FIR length from userspace, then write to stk_data.fir_len.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk_data *stk = dev_get_drvdata(dev);
    int firlen, error;
    error = kstrtoint(buf, 10, &firlen);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (STK_FIR_LEN_MAX < firlen)
        dev_err(&stk->client->dev, "%s: maximum FIR length is %d\n",
                __func__, STK_FIR_LEN_MAX);
    else
    {
        memset(&stk->fir, 0, sizeof(struct data_fir));
        atomic_set(&stk->fir_len, firlen);
    }

    return count;
}
#endif /* STK_FIR */

static DEVICE_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DEVICE_ATTR(value, 0444, stk_value_show, NULL);
static DEVICE_ATTR(delay, 0664, stk_delay_show, stk_delay_store);
static DEVICE_ATTR(offset, 0664, stk_offset_show, stk_offset_store);
static DEVICE_ATTR(send, 0220, NULL, stk_send_store);
static DEVICE_ATTR(recv, 0664, stk_recv_show, stk_recv_store);
static DEVICE_ATTR(allreg, 0444, stk_allreg_show, NULL);
static DEVICE_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DEVICE_ATTR(fifo, 0664, stk_fifo_show, stk_fifo_store);
static DEVICE_ATTR(selftest, 0664, stk_selftest_show, stk_selftest_store);
#ifdef STK_CALI
    static DEVICE_ATTR(cali, 0664, stk_cali_show, stk_cali_store);
#endif /* STK_CALI */
#ifdef STK_STEP_COUNTER
    static DEVICE_ATTR(stepcount, 0644, stk_step_show, stk_step_store);
#endif /* STK_STEP_COUNTER */
#ifdef STK_FIR
    static DEVICE_ATTR(firlen, 0664, stk_firlen_show, stk_firlen_store);
#endif /* STK_FIR */

static struct attribute *stk_attribute_accel[] =
{
    &dev_attr_enable.attr,
    &dev_attr_value.attr,
    &dev_attr_delay.attr,
    &dev_attr_offset.attr,
    &dev_attr_send.attr,
    &dev_attr_recv.attr,
    &dev_attr_allreg.attr,
    &dev_attr_chipinfo.attr,
    &dev_attr_fifo.attr,
    &dev_attr_selftest.attr,
#ifdef STK_CALI
    &dev_attr_cali.attr,
#endif /* STK_CALI */
#ifdef STK_STEP_COUNTER
    &dev_attr_stepcount.attr,
#endif /* STK_STEP_COUNTER */
#ifdef STK_FIR
    &dev_attr_firlen.attr,
#endif /* STK_FIR */
    NULL
};

static struct attribute_group stk_attribute_accel_group =
{
    .name = "driver",
    .attrs = stk_attribute_accel,
};

#if defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE
/*
 * @brief: Report accel data to /sys/class/input/inputX/capabilities/rel
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_report_accel_data(struct stk_data *stk)
{
    if (!stk->input_dev_accel)
    {
        dev_err(&stk->client->dev,
                "%s: No input device for accel data\n", __func__);
        return;
    }

    input_report_abs(stk->input_dev_accel, ABS_X, stk->xyz[0]);
    input_report_abs(stk->input_dev_accel, ABS_Y, stk->xyz[1]);
    input_report_abs(stk->input_dev_accel, ABS_Z, stk->xyz[2]);
	
/*	
    input_event(stk->input_dev_accel, EV_SYN, SYN_TIME_SEC,
            ktime_to_timespec(stk->timestamp).tv_sec);
    input_event(stk->input_dev_accel, EV_SYN, SYN_TIME_NSEC,
            ktime_to_timespec(stk->timestamp).tv_nsec);
*/			
			
    input_sync(stk->input_dev_accel);
}

/*
 * @brief: 1. Read FIFO data.
 *         2. Report to /sys/class/input/inputX/capabilities/rel
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_read_then_report_fifo_data(struct stk_data *stk)
{
    u8 wm = 0;

    if (!stk->fifo) {
        return;
    }

    if (stk_reg_read(stk, STK8XXX_REG_FIFOSTS, 0, &wm))
        return;

    wm &= STK8XXX_FIFOSTS_FIFO_FRAME_CNT_MASK;
    if (wm)
    {
        u8 *fifo = NULL;
        int i, fifo_len;

        fifo_len = (int)wm * 6; /* xyz * 2 bytes/axis */
        /* vzalloc: allocate memory and set to zero. */
        fifo = vzalloc(sizeof(u8) * fifo_len);

        if (!fifo)
        {
            dev_err(&stk->client->dev, "%s: memory allocation error\n",
                    __func__);
            return;
        }

        stk_fifo_reading(stk, fifo, fifo_len);

        for (i = 0; i < (int)wm; i++) {
            s16 *xyz;
            int ii;
            s16 coor_trans[3] = {0};
            u64 sec, ns;

            xyz = vzalloc(sizeof(s16) * 3);
            xyz[0] = fifo[i*6 + 1] << 8 | fifo[i*6];
            xyz[0] >>= 4;
            xyz[1] = fifo[i*6 + 3] << 8 | fifo[i*6 + 2];
            xyz[1] >>= 4;
            xyz[2] = fifo[i*6 + 5] << 8 | fifo[i*6 + 4];
            xyz[2] >>= 4;

            for (ii = 0; ii < 3; ii++) {
                coor_trans[0] += xyz[ii] * coordinate_trans[stk->direction][0][ii];
                coor_trans[1] += xyz[ii] * coordinate_trans[stk->direction][1][ii];
                coor_trans[2] += xyz[ii] * coordinate_trans[stk->direction][2][ii];
            }

            xyz[0] = coor_trans[0];
            xyz[1] = coor_trans[1];
            xyz[2] = coor_trans[2];

#ifdef STK_FIR
            stk_low_pass_fir(stk, xyz);
#endif /* STK_FIR */

            stk->fifo_start_ns += stkODRTable[stk->sr_no].sample_rate_us * NSEC_PER_USEC;
            sec = stk->fifo_start_ns;
            ns = do_div(sec, NSEC_PER_SEC);
            input_report_abs(stk->input_dev_accel, ABS_X, xyz[0]);
            input_report_abs(stk->input_dev_accel, ABS_Y, xyz[1]);
            input_report_abs(stk->input_dev_accel, ABS_Z, xyz[2]);
//           input_event(stk->input_dev_accel, EV_SYN, SYN_TIME_SEC, (int)sec);
//           input_event(stk->input_dev_accel, EV_SYN, SYN_TIME_NSEC, (int)ns);
            input_sync(stk->input_dev_accel);
            vfree(xyz);

        }
        vfree(fifo);
    }

    stk_set_fifo_start_time(stk);
}
#endif /* defined STK_INTERRUPT_MODE || defined STK_POLLING_MODE */

#ifdef STK_INTERRUPT_MODE
/*
 * @brief: Queue work list.
 *          1. Read accel data, then report to userspace.
 *          2. Read FIFO data.
 *          3. Read ANY MOTION data.
 *          4. Reset latch status.
 *          5. Enable IRQ.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_data_irq_work(struct work_struct *work)
{
    struct stk_data *stk =
        container_of(work, struct stk_data, alldata_work);

    stk_read_accel_data(stk);
    stk_report_accel_data(stk);

    if (stk->fifo) {
        u8 data = 0;

        if (stk_reg_read(stk, STK8XXX_REG_INTSTS2, 0, &data)) {
            dev_err(&stk->client->dev, "%s: cannot read register INTSTS2\n", __func__);
        } else {
            if (STK8XXX_INTSTS2_FWM_STS_MASK & data)
            {
                stk_read_then_report_fifo_data(stk);
            }
        }
    }

#ifdef STK_AMD
    stk_read_anymotion_data(stk);
    stk_reset_latched_int(stk);
#endif /* STK_AMD */
    enable_irq(stk->irq1);
}

/*
 * @brief: IRQ handler. This function will be trigger after receiving IRQ.
 *          1. Disable IRQ without waiting.
 *          2. Send work to quque.
 *
 * @param[in] irq: irq number
 * @param[in] data: void *
 *
 * @return: IRQ_HANDLED
 */
static irqreturn_t stk_all_data_handler(int irq, void *data)
{
    struct stk_data *stk = data;
    disable_irq_nosync(irq);
    queue_work(stk->alldata_workqueue, &stk->alldata_work);
    return IRQ_HANDLED;
}

/*
 * @brief: IRQ setup
 *          1. Set GPIO as input direction.
 *          2. Allocate an interrupt resource, enable the interrupt, and IRQ
 *              handling.
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return:
 *          IRQC_IS_HARDIRQ or IRQC_IS_NESTED: Success
 *          Negative value: Fail
 */
static int stk_irq_alldata_setup(struct stk_data *stk)
{
    int irq = 0;
    gpio_direction_input(stk->interrupt_int1_pin);
    irq = gpio_to_irq(stk->interrupt_int1_pin);

    if (0 > irq)
    {
        dev_err(&stk->client->dev, "%s: gpio_to_irq(%d) failed\n",
                __func__, stk->interrupt_int1_pin);
        return -1;
    }

    stk->irq1 = irq;
    dev_info(&stk->client->dev, "%s: irq #=%d, interrupt pin=%d\n",
             __func__, irq, stk->interrupt_int1_pin);
#if 0
    irq = request_any_context_irq(stk->irq1, stk_all_data_handler,
                                  IRQF_TRIGGER_RISING, STK_IRQ_INT1_NAME, stk);

    if (0 > irq)
    {
        dev_err(&stk->client->dev,
                "%s: request_any_context_irq(%d) failed for %d\n",
                __func__, stk->irq1, irq);
        return -1;
    }
#else
    irq = request_irq(stk->irq1, stk_all_data_handler,
                                  IRQF_TRIGGER_RISING, STK_IRQ_INT1_NAME, stk);

    if (0 > irq)
    {
        dev_err(&stk->client->dev,
                "%s: request_irq(%d) failed for %d\n",
                __func__, stk->irq1, irq);
        return -1;
    }
#endif

    return irq;
}
#elif defined STK_POLLING_MODE
/*
 * @brief: Queue delayed_work list.
 *          1. Read accel data, then report to userspace.
 *          2. Read ANY MOTION data.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_accel_delay_work(struct work_struct *work)
{
    struct stk_data *stk =
        container_of(work, struct stk_data, accel_delaywork.work);
    stk_read_accel_data(stk);
    stk_report_accel_data(stk);
    if (stk->fifo) {
        stk_read_then_report_fifo_data(stk);
    }
#ifdef STK_AMD
    stk_read_anymotion_data(stk);
    stk_reset_latched_int(stk);
#endif /* STK_AMD */
}

/*
 * @brief: This function will send delayed_work to queue.
 *          This function will be called regularly with period:
 *          stk_data.poll_delay.
 *
 * @param[in] timer: struct hrtimer *
 *
 * @return: HRTIMER_RESTART.
 */
static enum hrtimer_restart stk_accel_timer_func(struct hrtimer *timer)
{
    struct stk_data *stk =
        container_of(timer, struct stk_data, accel_timer);
    schedule_delayed_work(&stk->accel_delaywork, 0);
    hrtimer_forward_now(&stk->accel_timer, stk->poll_delay);
    return HRTIMER_RESTART;
}
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

/*
 * @brief: Initialize some data in stk_data.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_data_initialize(struct stk_data *stk)
{
    atomic_set(&stk->enabled, 0);
#ifdef STK_CALI
    memset(stk->cali_sw, 0x0, sizeof(stk->cali_sw));
    atomic_set(&stk->cali_status, STK_K_NO_CALI);
#ifdef STK_AUTOK
    atomic_set(&stk->first_enable, 1);
    stk->offset[0] = 0;
    stk->offset[1] = 0;
    stk->offset[2] = 0;
#endif /* STK_AUTOK */
#endif /* STK_CALI */
    atomic_set(&stk->selftest, STK_SELFTEST_RESULT_NA);
    atomic_set(&stk->recv, 0);
    stk->power_mode = STK8XXX_PWMD_SUSPEND;
    stk->sr_no = 1;
    stk->latency_us = stkODRTable[stk->sr_no].sample_rate_us;
    stk_set_fifo_start_time(stk);
    stk->temp_enable = false;
#ifdef STK_FIR
    memset(&stk->fir, 0, sizeof(struct data_fir));
    atomic_set(&stk->fir_len, STK_FIR_LEN);
#endif /* STK_FIR */
    dev_info(&stk->client->dev, "%s: done\n", __func__);
}

/*
 * @brief: File system setup for accel and any motion
 *
 * @param[in/out] stk: struct stk_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_input_setup(struct stk_data *stk)
{
    int error = 0;
    /* input device: setup for accel */
    stk->input_dev_accel = input_allocate_device();

    if (!stk->input_dev_accel)
    {
        dev_err(&stk->client->dev,
                "%s: input_allocate_device for accel failed\n", __func__);
        return -ENOMEM;
    }

    stk->input_dev_accel->name = IN_DEV_ACCEL_NAME;
    stk->input_dev_accel->id.bustype = BUS_I2C;
    input_set_capability(stk->input_dev_accel, EV_ABS, ABS_MISC);
    input_set_abs_params(stk->input_dev_accel, ABS_X, 32767, -32768, 0, 0);
    input_set_abs_params(stk->input_dev_accel, ABS_Y, 32767, -32768, 0, 0);
    input_set_abs_params(stk->input_dev_accel, ABS_Z, 32767, -32768, 0, 0);
    stk->input_dev_accel->dev.parent = &stk->client->dev;
    input_set_drvdata(stk->input_dev_accel, stk);
    error = input_register_device(stk->input_dev_accel);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: Unable to register input device: %s\n",
                __func__, stk->input_dev_accel->name);
        input_free_device(stk->input_dev_accel);
        return error;
    }

#ifdef STK_AMD
    /* input device: setup for any motion */
    stk->input_dev_amd = input_allocate_device();

    if (!stk->input_dev_amd)
    {
        dev_err(&stk->client->dev,
                "%s: input_allocate_device for ANY MOTION failed\n", __func__);
        input_free_device(stk->input_dev_accel);
        input_unregister_device(stk->input_dev_accel);
        return -ENOMEM;
    }

    stk->input_dev_amd->name = IN_DEV_AMD_NAME;
    stk->input_dev_amd->id.bustype = BUS_I2C;
    input_set_capability(stk->input_dev_amd, EV_ABS, ABS_MISC);
    stk->input_dev_amd->dev.parent = &stk->client->dev;
    input_set_drvdata(stk->input_dev_amd, stk);
    error = input_register_device(stk->input_dev_amd);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: Unable to register input device: %s\n",
                __func__, stk->input_dev_amd->name);
        input_free_device(stk->input_dev_amd);
        input_free_device(stk->input_dev_accel);
        input_unregister_device(stk->input_dev_accel);
        return error;
    }
#endif /* STK_AMD */

    return 0;
}

#ifdef STK_QUALCOMM
/*
 * @brief: The handle for enable and disable sensor.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] enabled:
 */
static int stk_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
        unsigned int enabled)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    if (0 == enabled) {
        stk_set_enable(stk, 0);
    } else if (1 == enabled) {
        stk_set_enable(stk, 1);
    } else {
        dev_err(&stk->client->dev, "%s: Invalid vlaue of input, input=%d\n",
                __func__, enabled);
        return -EINVAL;
    }

    return 0;
}

/*
 * @brief: The handle for set the sensor polling delay time.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] delay_msec:
 */
static int stk_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
        unsigned int delay_msec)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    stk_set_delay(stk, delay_msec * 1000UL);

    return 0;
}

/*
 * @brief: 
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] enable:
 */
static int stk_cdev_sensors_enable_wakeup(struct sensors_classdev *sensors_cdev,
        unsigned int enable)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    dev_info(&stk->client->dev, "%s: enable=%d\n", __func__, enable);

    return 0;
}

/*
 * @brief: Set the max report latency of the sensor.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] max_latency: msec
 */
static int stk_cdev_sensors_set_latency(struct sensors_classdev *sensors_cdev,
        unsigned int max_latency)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    if (!stk->fifo) {
        return 0;
    }

    if (max_latency * 1000 > stkODRTable[stk->sr_no].sample_rate_us) {
        stk->latency_us = max_latency * 1000;
    }

    if (stk->latency_us != stkODRTable[stk->sr_no].sample_rate_us) {
        u8 wm = stk->latency_us / stkODRTable[stk->sr_no].sample_rate_us;

        if (0 != stk->latency_us % stkODRTable[stk->sr_no].sample_rate_us) {
            wm++;
        }

        if (STK8XXX_CFG1_XYZ_FRAME_MAX < wm) {
            wm = STK8XXX_CFG1_XYZ_FRAME_MAX;
            dev_err(&stk->client->dev,
                    "%s: wm out of range. latency(us)=%d, odr(us)=%d\n",
                    __func__, stk->latency_us, stkODRTable[stk->sr_no].sample_rate_us);
        }

        if (stk_change_fifo_status(stk, wm)) {
            dev_err(&stk->client->dev, "%s: failed\n", __func__);
        } else {
#ifdef STK_POLLING_MODE
            hrtimer_cancel(&stk->accel_timer);
            stk->poll_delay = ns_to_ktime(stk->latency_us * NSEC_PER_USEC);
            hrtimer_start(&stk->accel_timer, stk->poll_delay, HRTIMER_MODE_REL);
#endif /* STK_POLLING_MODE */
        }
    }

    return 0;
}

/*
 * @brief: Flush sensor events in FIFO and report it to user space.
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 */
static int stk_cdev_sensors_flush(struct sensors_classdev *sensors_cdev)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    if (!stk->fifo) {
        return 0;
    }

    stk_read_then_report_fifo_data(stk);

    return 0;
}

#ifdef STK_CALI
/*
 * @brief: Self calibration. 
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] axis
 * @param[in] apply_now
 */
static int stk_cdev_sensors_calibrate(struct sensors_classdev *sensors_cdev,
        int axis, int apply_now)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    stk_set_cali(stk);

    return 0;
}

/*
 * @brief: 
 *          include/linux/sensors.h
 *
 * @param[in] *sensors_cdev: struct sensors_classdev
 * @param[in] *cal_result:
 */
static int stk_cdev_sensors_cal_params(struct sensors_classdev *sensors_cdev,
        struct cal_result_t *cal_result)
{
    struct stk_data *stk = container_of(sensors_cdev, struct stk_data, accel_cdev);

    stk->cali_sw[0] = cal_result->offset_x;
    stk->cali_sw[1] = cal_result->offset_y;
    stk->cali_sw[2] = cal_result->offset_z;

    return 0;
}
#endif /* STK_CALI */

/*
 * @brief: Parameters initialize for sensors_classdev.
 *
 * @param[in/out] stk: struct stk_data *
 */
static void stk_classdev_init(struct stk_data *stk)
{
    stk->accel_cdev = stk_cdev;

    if (STK8BA50_R_ID == stk->pid) {
        stk->accel_cdev.name = "stk8ba50r";
    } else if (STK8BA53_ID == stk->pid) {
        stk->accel_cdev.name = "stk8ba53";
    } else if (STK8323_ID == stk->pid || STK8325_ID == stk->pid) {
        stk->accel_cdev.name = "stk832x";
        stk->accel_cdev.fifo_reserved_event_count = 30,
        stk->accel_cdev.fifo_max_event_count = 32,
        stk->accel_cdev.sensors_set_latency = stk_cdev_sensors_set_latency;
    }

#if (STK8XXX_RANGESEL_2G == STK8XXX_RANGESEL_DEF)
    stk->accel_cdev.max_range = "19.62"; /* 2G mode: 2.0f*9.81f=19.62f */
    if (STK8BA50_R_ID == stk->pid) {
        stk->accel_cdev.resolution = "0.03832";
    } else {
        /* 2G mode,12-bit resolution: 9.81f/1024.f=0.00958f */
        stk->accel_cdev.resolution = "0.00958";
    }
#elif (STK8XXX_RANGESEL_4G == STK8XXX_RANGESEL_DEF)
    stk->accel_cdev.max_range = "39.24"; /* 4G mode: 4.0f*9.81f=39.24f */
    if (STK8BA50_R_ID == stk->pid) {
        stk->accel_cdev.resolution = "0.07664";
    } else {
        /* 4G mode,12-bit resolution: 9.81f/512.f=0.01916f */
        stk->accel_cdev.resolution = "0.01916";
    }
#elif (STK8XXX_RANGESEL_8G == STK8XXX_RANGESEL_DEF)
    stk->accel_cdev.max_range = "78.48"; /* 8G mode: 8.0f*9.81f=78.48f */
    if (STK8BA50_R_ID == stk->pid) {
        stk->accel_cdev.resolution = "0.15328";
    } else {
        /* 8G mode,12-bit resolution: 9.81f/256.f=0.03832f */
        stk->accel_cdev.resolution = "0.03832";
    }
#endif /* mapping for STK8XXX_RANGESEL_DEF */

    stk->accel_cdev.sensors_enable = stk_cdev_sensors_enable;
    stk->accel_cdev.sensors_poll_delay = stk_cdev_sensors_poll_delay;
    stk->accel_cdev.sensors_enable_wakeup = stk_cdev_sensors_enable_wakeup;
    stk->accel_cdev.sensors_flush = stk_cdev_sensors_flush;
#ifdef STK_CALI
    stk->accel_cdev.sensors_calibrate = stk_cdev_sensors_calibrate;
    stk->accel_cdev.sensors_write_cal_params = stk_cdev_sensors_cal_params;
#else /* no STK_CALI */
    stk->accel_cdev.sensors_calibrate = NULL;
    stk->accel_cdev.sensors_write_cal_params = NULL;
#endif /* STK_CALI */
}
#endif /* STK_QUALCOMM */




/*---------------------------add by yxp 20210111----------------------------*/

static void set_cal_flag(char val)
{
	//struct i2c_client *client = bma250_i2c_client;
	write_lock(&cal_rwlock);
	cal_lock_flag = val;
	if (cal_lock_flag == 1)
	{
		x_offset_cal = 0;
		y_offset_cal = 0;
		z_offset_cal = 0;
		x_sum = 0;
		y_sum = 0;
		z_sum = 0;
		n_index = 0;
		m_index = 0;		
	}
	
	write_unlock(&cal_rwlock);	
}

static char get_cal_flag(void)
{
	char rtn;
	read_lock(&cal_rwlock);
	rtn = cal_lock_flag;
	read_unlock(&cal_rwlock);
	return rtn;
}

static ssize_t gsensor_calipara_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	read_lock(&cal_rwlock);
	x_offset_cal = (x_offset_cal/4)*4;
	y_offset_cal = (y_offset_cal/4)*4;
	z_offset_cal = (z_offset_cal/4)*4;
	ret = snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", x_offset_cal/4,y_offset_cal/4,z_offset_cal/4);
	read_unlock(&cal_rwlock);
	printk("====gsensor_calipara_show:%d,%d,%d\n",x_offset_cal,y_offset_cal,z_offset_cal);
	return ret;
}

static ssize_t
gsensor_calipara_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int  x,y,z;
	write_lock(&cal_rwlock);
	sscanf(buf, "%d,%d,%d", &x,&y,&z);
	if (abs(x) > XX_MAX_PERM_VALUE 
	|| abs(y) > XX_MAX_PERM_VALUE 
	|| abs(z) > XX_MAX_PERM_VALUE)
	{
		//偏差太大，直接忽略
		printk("invalid paras!!the max abs value is %d.\n",XX_MAX_PERM_VALUE);
	}
	else
	{
		x_offset_cal = x*4;
		y_offset_cal = y*4;
		z_offset_cal = z*4;
		printk("====gsensor_calipara_write:%d,%d,%d\n",x,y,z);
	}
	write_unlock(&cal_rwlock);
	return count;
}

static ssize_t gsensor_calistatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%d\n", get_cal_flag());
	return ret;
}

static ssize_t
gsensor_calistatus_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count > 0)
	{
		int tmpStatus = 0;
		tmpStatus = buf[0] -'0';
		set_cal_flag(tmpStatus);
	}
	return count;
}

static ssize_t gsensor_hwinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret_len = 0;
	sprintf(buf, "%s\n", "STK8321");
	ret_len = strlen(buf) + 1;

	return ret_len;
}

static ssize_t show_acc_debug(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%u\n", acc_debug);
}

static ssize_t store_acc_debug(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t size)
{
	if (size > 0)
	{		
		acc_debug = buf[0] -'0';
	}
	return size;
}



static DEVICE_NOAH_ATTR(acc_debug, S_IRUGO | S_IWUGO | S_IWGRP,
		   show_acc_debug, store_acc_debug);

static DEVICE_NOAH_ATTR(hwinfo, 0444, gsensor_hwinfo_show, NULL);

static DEVICE_NOAH_ATTR(calipara,
		   S_IRWXUGO | S_IRWXUGO,
		   gsensor_calipara_show, gsensor_calipara_write);

static DEVICE_NOAH_ATTR(calistatus,
		   S_IRWXUGO | S_IRWXUGO,
		   gsensor_calistatus_show, gsensor_calistatus_write);

static int gsensor_sysfs_init(void)
{
	int liRet ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL)
	{
		printk("BMA250e gsensor_sysfs_init:subsystem_register failed\n");
		liRet = -ENOMEM;
		goto kobject_create_failed;
	}

	liRet = sysfs_create_file(android_gsensor_kobj, &dev_attr_calipara.attr);; 
	if (liRet) {
		printk(KERN_ERR "BMA250e gsensor_sysfs_init:calipara sysfs_create failed\n");
		goto sysfs_create_failed;
	}

	liRet = sysfs_create_file(android_gsensor_kobj, &dev_attr_calistatus.attr);; 
	if (liRet) {
		printk(KERN_ERR "BMA250e gsensor_sysfs_init: calistatus sysfs_create failed\n");
		goto sysfs_create_failed;
	}

	liRet = sysfs_create_file(android_gsensor_kobj, &dev_attr_hwinfo.attr);; 
	if (liRet) {
		printk(KERN_ERR "BMA250e gsensor_sysfs_init:hwinfo sysfs_create failed\n");
		goto sysfs_create_failed;
	}
	
	liRet = sysfs_create_file(android_gsensor_kobj, &dev_attr_acc_debug.attr);; 
	if (liRet) {
		printk(KERN_ERR "BMA250e gsensor_sysfs_init:acc_debug sysfs_create failed\n");
		goto sysfs_create_failed;
	}
	printk("[gsensor_sysfs_init] sucess \n");
	
	return 0;
	
sysfs_create_failed:	
	kobject_del(android_gsensor_kobj);
kobject_create_failed:
	return liRet;
	
}

#if 1
extern char *noah_eeprom_readdata(int type);
static int bma250_read_calicalipara_from_eeprom(void)
{

	char sum = 0;
	signed char x=0,y=0,z=0;
	
	char *sys_gsensor_data = noah_eeprom_readdata(NOAH_GSENSOR_TYPE);
	if(NULL == sys_gsensor_data)
		return -1;
	x = sys_gsensor_data[0];
	y = sys_gsensor_data[1];
	z = sys_gsensor_data[2];
	sum = x+y+z;
	printk(KERN_INFO"[%s--------%d:%d,%d,%d,%d]\n",
			__func__,__LINE__,x,y,z,sum);
	if(sum != sys_gsensor_data[3])
		return -1;
	if (abs(x) > XX_MAX_PERM_VALUE 
	|| abs(y) > XX_MAX_PERM_VALUE 
	|| abs(z) > XX_MAX_PERM_VALUE)
	{
		//偏差太大，直接忽略
		printk("invalid paras!!the max abs value is %d.\n",XX_MAX_PERM_VALUE);
	}
	else
	{
		x_offset_cal = x*4;
		y_offset_cal = y*4;
		z_offset_cal = z*4;
	}
	
	return 0;
}

#endif
/*---------------------------add by yxp end--------------------------------*/



/*
 * @brief: Proble function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] id: struct i2c_device_id *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk8xxx_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int error = 0;
    struct stk_data *stk;
    dev_info(&client->dev, "%s: driver version:%s\n",
             __func__, STK_ACC_DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        error = i2c_get_functionality(client->adapter);
        dev_err(&client->dev,
                "%s: i2c_check_functionality error, functionality=0x%x\n",
                __func__, error);
        return -EIO;
    }

    /* kzalloc: allocate memory and set to zero. */
    stk = kzalloc(sizeof(struct stk_data), GFP_KERNEL);

    if (!stk)
    {
        dev_err(&client->dev, "%s: memory allocation error\n", __func__);
        return -ENOMEM;
    }

    stk_data_ptr = stk;
    stk->client = client;
    i2c_set_clientdata(client, stk);
    mutex_init(&stk->reg_lock);

    if (get_platform_data(stk))
        goto err_free_mem;

    if (stk_get_pid(stk))
        goto err_free_mem;

    dev_info(&client->dev, "%s: PID 0x%x\n", __func__, stk->pid);
    stk_data_initialize(stk);
#ifdef STK_INTERRUPT_MODE

    if (gpio_request(stk->interrupt_int1_pin, STK_IRQ_INT1_LABEL))
    {
        dev_err(&client->dev, "%s: gpio_request failed\n", __func__);
        goto err_free_mem;
    }

    stk->alldata_workqueue = create_singlethread_workqueue("stk_int1_wq");

    if (stk->alldata_workqueue)
        INIT_WORK(&stk->alldata_work, stk_data_irq_work);
    else
    {
        dev_err(&stk->client->dev, "%s: create_singlethread_workqueue error\n",
                __func__);
        error = -EPERM;
        goto exit_int1_create_singlethread_workqueue;
    }

    error = stk_irq_alldata_setup(stk);

    if (0 > error)
        goto exit_irq_int1_setup_error;

#elif defined STK_POLLING_MODE
    /* polling accel data */
    INIT_DELAYED_WORK(&stk->accel_delaywork, stk_accel_delay_work);
    hrtimer_init(&stk->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    stk->poll_delay = ns_to_ktime(stkODRTable[stk->sr_no].sample_rate_us * NSEC_PER_USEC);
    stk->accel_timer.function = stk_accel_timer_func;
    if (stk->fifo) {
        stk->latency_us = stkODRTable[stk->sr_no].sample_rate_us;
    }
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */

    if (stk_reg_init(stk, STK8XXX_RANGESEL_DEF, stk->sr_no))
    {
        dev_err(&client->dev, "%s: stk8xxx initialization failed\n", __func__);
        goto exit_stk_init_error;
    }

    if (stk_input_setup(stk))
    {
        dev_err(&client->dev, "%s: failed\n", __func__);
        goto exit_stk_init_error;
    }

    /* sysfs: create file system */
    error = sysfs_create_group(&stk->input_dev_accel->dev.kobj,
                               &stk_attribute_accel_group);

    if (error)
    {
        dev_err(&client->dev, "%s: sysfs_create_group failed for accel\n",
                __func__);
        goto exit_sysfs_create_accelgroup_error;
    }

#ifdef STK_QUALCOMM
    stk_classdev_init(stk);
#endif /* STK_QUALCOMM */


//----------------add by yxp 20210111----------------------

	if(gsensor_sysfs_init())
		printk("[gsensor_sysfs_init err......]\n");
/*------从eeprom驱动拷贝在内存中的数据中读取重感校准参数---*/

#if 1
	if(bma250_read_calicalipara_from_eeprom())
		printk("read gsensor calicalipara failed,set defalut data\n");

#endif 	
//----------------add by yxp end--------------------------	

    dev_info(&client->dev, "%s: Successfully\n", __func__);
    return 0;

exit_sysfs_create_accelgroup_error:
#ifdef STK_AMD
    input_unregister_device(stk->input_dev_amd);
#endif /* STK_AMD */
    input_unregister_device(stk->input_dev_accel);
exit_stk_init_error:
#ifdef STK_INTERRUPT_MODE
    free_irq(stk->irq1, stk);
exit_irq_int1_setup_error:
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
exit_int1_create_singlethread_workqueue:
    gpio_free(stk->interrupt_int1_pin);
#elif defined STK_POLLING_MODE
    hrtimer_try_to_cancel(&stk->accel_timer);
    cancel_delayed_work_sync(&stk->accel_delaywork);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
err_free_mem:
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    return error;
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
static int stk8xxx_remove(struct i2c_client *client)
{
    struct stk_data *stk = i2c_get_clientdata(client);
    sysfs_remove_group(&stk->input_dev_accel->dev.kobj,
                       &stk_attribute_accel_group);
#ifdef STK_AMD
    input_unregister_device(stk->input_dev_amd);
#endif /* STK_AMD */
    input_unregister_device(stk->input_dev_accel);
#ifdef STK_INTERRUPT_MODE
    free_irq(stk->irq1, stk);
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
    gpio_free(stk->interrupt_int1_pin);
#elif defined STK_POLLING_MODE
    hrtimer_try_to_cancel(&stk->accel_timer);
    cancel_delayed_work_sync(&stk->accel_delaywork);
#endif /* STK_INTERRUPT_MODE, STK_POLLING_MODE */
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    stk = NULL;
    return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * @brief: Suspend function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk8xxx_suspend(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct stk_data *stk = i2c_get_clientdata(client);

    if (atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 0);
        stk->temp_enable = true;
    }
    else
        stk->temp_enable = false;

    return 0;
}

/*
 * @brief: Resume function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk8xxx_resume(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct stk_data *stk = i2c_get_clientdata(client);

    if (stk->temp_enable)
        stk_set_enable(stk, 1);

    stk->temp_enable = false;
    return 0;
}

static const struct dev_pm_ops stk8xxx_pm_ops =
{
    .suspend = stk8xxx_suspend,
    .resume = stk8xxx_resume,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id stk8xxx_acpi_id[] =
{
    {"STK8XXXX", 0},
    {}
};
MODULE_DEVICE_TABLE(acpi, stk8xxx_acpi_id);
#endif /* CONFIG_ACPI */

#ifdef CONFIG_OF
static struct of_device_id stk8xxx_match_table[] =
{
    { .compatible = "stk,stk8xxx", },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id stk8xxx_i2c_id[] =
{
    {STK8XXXX_I2C_NAME, 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, stk8xxx_i2c_id);

static struct i2c_driver stk8xxx_i2c_driver =
{
    .probe      = stk8xxx_probe,
    .remove     = stk8xxx_remove,
    .id_table   = stk8xxx_i2c_id,
    .class      = I2C_CLASS_HWMON,
    .driver = {
        .owner  = THIS_MODULE,
        .name   = STK8XXXX_I2C_NAME,
#ifdef CONFIG_PM_SLEEP
        .pm     = &stk8xxx_pm_ops,
#endif
#ifdef CONFIG_ACPI
        .acpi_match_table = ACPI_PTR(stk8xxx_acpi_id),
#endif /* CONFIG_ACPI */
#ifdef CONFIG_OF
        .of_match_table = stk8xxx_match_table,
#endif /* CONFIG_OF */
    }
};

//module_i2c_driver(stk8xxx_i2c_driver);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk8xxx 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(STK_ACC_DRIVER_VERSION);
