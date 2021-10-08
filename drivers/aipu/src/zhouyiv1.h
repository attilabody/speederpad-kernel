/**********************************************************************************
 * This file is CONFIDENTIAL and any use by you is subject to the terms of the
 * agreement between you and Arm China or the terms of the agreement between you
 * and the party authorised by Arm China to disclose this file to you.
 * The confidential and proprietary information contained in this file
 * may only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm China.
 *
 *        (C) Copyright 2020 Arm Technology (China) Co. Ltd.
 *                    All rights reserved.
 *
 * This entire notice must be reproduced on all copies of this file and copies of
 * this file may only be made by a person if such person is permitted to do so
 * under the terms of a subsisting license agreement from Arm China.
 *

 *********************************************************************************/

/**
 * @file zhouyiv1.h
 * Header of the zhouyiv1 AIPU hardware control and interrupt handle operations
 */

#ifndef _AIPU_ZHOUYIV1_H_
#define _AIPU_ZHOUYIV1_H_

#include <linux/device.h>
#include "aipu_core.h"
#include "aipu_sunxi_platform.h"

#if ((defined BUILD_ZHOUYIV1_KMD_DEFAULT) && (BUILD_ZHOUYIV1_KMD_DEFAULT == 1))
#include "config/zhouyiv1_default_cfg.h"
#endif

/**
 * Zhouyi-v1 AIPU Core Interrupts
 */
#define ZHOUYIV1_IRQ_NONE   0x0
#define ZHOUYIV1_IRQ_QEMPTY 0x1
#define ZHOUYIV1_IRQ_DONE   0x2
#define ZHOUYIV1_IRQ_EXCEP  0x4
#define ZHOUYIV1_IRQ        ( ZHOUYIV1_IRQ_QEMPTY | ZHOUYIV1_IRQ_DONE | ZHOUYIV1_IRQ_EXCEP )
//#define ZHOUYIV1_IRQ              ZHOUYIV1_IRQ_DONE
#define ZHOUYIV1_IRQ_ENABLE_FLAG  ZHOUYIV1_IRQ
#define ZHOUYIV1_IRQ_DISABLE_FLAG ZHOUYIV1_IRQ_NONE

#define ZHOUYIV1_PWOFF_AIPU_FLAG 0x0
#define ZHOUYIV1_PWON_AIPU_FLAG  0x1
#define ZHOUYI_AIPU_IDLE_STATUS  0x70000

/**
 * Zhouyi-v1 AIPU Core Host Control Register Map
 */
#define AIPU_CTRL_REG_OFFSET                0x0
#define AIPU_STAT_REG_OFFSET                0x4
#define AIPU_START_PC_REG_OFFSET            0x8
#define AIPU_INTR_PC_REG_OFFSET             0xC
#define AIPU_IPI_CTRL_REG_OFFSET            0x10
#define AIPU_DATA_ADDR_0_REG_OFFSET         0x14
#define AIPU_DATA_ADDR_1_REG_OFFSET         0x18
#define AIPU_INTR_CAUSE_REG_OFFSET          0x20
#define AIPU_INTR_STAT_REG_OFFSET           0x24
#define AIPU_INTR_BACKUP_STAT_REG_OFFSET    0x28
#define AIPU_INTR_BACKUP_PC_REG_OFFSET      0x2C
#define AIPU_DBG_ERR_CAUSE_REG_OFFSET       0x30
#define AIPU_DBG_DATA_REG_0_OFFSET          0x34
#define AIPU_DBG_DATA_REG_1_OFFSET          0x38
#define AIPU_CLK_CTRL_REG_OFFSET            0x3C
#define AIPU_MAX_REG_OFFSET                 0x3C
#define AIPU_ISA_VERSION_REG_OFFSET         0x40
#define AIPU_TPC_FEATURE_REG_OFFSET         0x44
#define AIPU_SPU_FEATURE_REG_OFFSET         0x48
#define AIPU_HWA_FEATURE_REG_OFFSET         0x4C
#define AIPU_REVISION_ID_REG_OFFSET         0x50
#define AIPU_MEM_FEATURE_REG_OFFSET         0x54
#define AIPU_INST_RAM_FEATURE_REG_OFFSET    0x58
#define AIPU_LOCAL_SRAM_FEATURE_REG_OFFSET  0x5C
#define AIPU_GLOBAL_SRAM_FEATURE_REG_OFFSET 0x60
#define AIPU_INST_CACHE_FEATURE_REG_OFFSET  0x64
#define AIPU_DATA_CACHE_FEATURE_REG_OFFSET  0x68
#define AIPU_L2_CACHE_FEATURE_REG_OFFSET    0x6C


#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
#define AIPU_RESET_REG_OFFSET               0x104
#define AIPU_WORK_STAT_START_REQ            0x140
#define AIPU_WORK_STAT_END_REQ              0x144
#define AIPU_ALL_RDATA_TOT_LSB              0x200
#define AIPU_ALL_RDATA_TOT_MSB              0x204
#define AIPU_ALL_WDATA_TOT_LSB              0x208
#define AIPU_ALL_WDATA_TOT_MSB              0x20C
#define AIPU_TOT_CYCLE_LSB                  0x220
#define AIPU_TOT_CYCLE_MSB                  0x224
#define AIPU_ID_LATENCY_MAX_MSB             0x280
#define AIPU_ID_LATENCY_MAX_LSB             0x284
#define AIPU_ID_LATENCY_SINGLE              0x288
#define AIPU_DMA_LATENCY_TOT_MSB            0x290
#define AIPU_DMA_LATENCY_TOT_LSB            0x294
#define AIPU_DMA_RDATA_TOT_MSB              0x298
#define AIPU_DMA_RDATA_TOT_LSB              0x29C
#define AIPU_DMA_AR_HANDSHAKE_MSB           0x2A0
#define AIPU_DMA_AR_HANDSHAKE_LSB           0x2A4
#define AIPU_MAX_OUTSTAND                   0x2A8
#define AIPU_LATENCY_TOT_ID0_MSB            0x300
#define AIPU_LATENCY_TOT_ID0_LSB            0x304
#define AIPU_LATENCY_TOT_ID1_MSB            0x308
#define AIPU_LATENCY_TOT_ID1_LSB            0x30C
#define AIPU_LATENCY_TOT_ID2_MSB            0x310
#define AIPU_LATENCY_TOT_ID2_LSB            0x314
#define AIPU_LATENCY_TOT_ID3_MSB            0x318
#define AIPU_LATENCY_TOT_ID3_LSB            0x31C
#define AIPU_LATENCY_TOT_ID4_MSB            0x320
#define AIPU_LATENCY_TOT_ID4_LSB            0x324
#define AIPU_LATENCY_TOT_ID5_MSB            0x328
#define AIPU_LATENCY_TOT_ID5_LSB            0x32C
#define AIPU_LATENCY_TOT_ID6_MSB            0x330
#define AIPU_LATENCY_TOT_ID6_LSB            0x334
#define AIPU_LATENCY_TOT_ID7_MSB            0x338
#define AIPU_LATENCY_TOT_ID7_LSB            0x33C
#define AIPU_LATENCY_TOT_ID8_MSB            0x340
#define AIPU_LATENCY_TOT_ID8_LSB            0x344
#define AIPU_LATENCY_TOT_ID9_MSB            0x348
#define AIPU_LATENCY_TOT_ID9_LSB            0x34C
#define AIPU_LATENCY_TOT_ID10_MSB           0x350
#define AIPU_LATENCY_TOT_ID10_LSB           0x354
#define AIPU_LATENCY_TOT_ID11_MSB           0x358
#define AIPU_LATENCY_TOT_ID11_LSB           0x35C
#define AIPU_LATENCY_TOT_ID12_MSB           0x360
#define AIPU_LATENCY_TOT_ID12_LSB           0x364
#define AIPU_LATENCY_TOT_ID13_MSB           0x368
#define AIPU_LATENCY_TOT_ID13_LSB           0x36C
#define AIPU_LATENCY_TOT_ID14_MSB           0x370
#define AIPU_LATENCY_TOT_ID14_LSB           0x374
#define AIPU_LATENCY_TOT_ID15_MSB           0x378
#define AIPU_LATENCY_TOT_ID15_LSB           0x37C
#define AIPU_LATENCY_MAX_ID0                0x3C0
#define AIPU_LATENCY_MAX_ID1                0x3C4
#define AIPU_LATENCY_MAX_ID2                0x3C8
#define AIPU_LATENCY_MAX_ID3                0x3CC
#define AIPU_LATENCY_MAX_ID4                0x3D0
#define AIPU_LATENCY_MAX_ID5                0x3D4
#define AIPU_LATENCY_MAX_ID6                0x3D8
#define AIPU_LATENCY_MAX_ID7                0x3DC
#define AIPU_LATENCY_MAX_ID8                0x3E0
#define AIPU_LATENCY_MAX_ID9                0x3E4
#define AIPU_LATENCY_MAX_ID10               0x3E8
#define AIPU_LATENCY_MAX_ID11               0x3EC
#define AIPU_LATENCY_MAX_ID12               0x3F0
#define AIPU_LATENCY_MAX_ID13               0x3F4
#define AIPU_LATENCY_MAX_ID14               0x3F8
#define AIPU_LATENCY_MAX_ID15               0x3FC
#endif

/**
 * struct zhouyiv1_aipu - stores hardware config and resources like io and irq
 *        There should be exactly one zhouyiv1_aipu object instance in KMD.
 *
 * @core0: AIPU core description struct
 * @scc: SCC I/O region, for platforms like Juno
 * @freq: AIPU main working frequency in MHz
 */
typedef struct zhouyiv1_aipu {
    aipu_core_t core0;
    io_region_t scc;
    u32         freq_in_MHz;
} zhouyiv1_aipu_t;

/**
 * @brief create AIPU core object instance
 *
 * @param irq_num: interrupt number from platform resource; input -1 if get from platform failed;
 * @param core_phys_base: core io physcal address
 * @param core_size: core io region size
 * @param scc_phys_base: scc io physcal address
 * @param scc_size: scc io region size
 * @param freq: AIPU main working frequency in MHz
 *
 * @return zhouyiv1_aipu struct pointer if successful; NULL if failed.
 */
zhouyiv1_aipu_t  *zhouyi_create_aipu(int irqnum, u32 core_phys_base, u32 core_size,
    u32 scc_phys_base, u32 scc_size, u32 freq, struct device *dev);
/**
 * @brief create AIPU core object instance
 *
 * @param aipu: zhouyiv1_aipu struct pointer to be destroyed;
 *
 * @return void
 */
void zhouyi_destroy_aipu(zhouyiv1_aipu_t *aipu);
/**
 * @brief reset Zhouyi AIPU
 *
 * @param aipu: zhouyiv1_aipu struct pointer
 *
 * @return void
 */
void zhouyi_ctrl_reset(zhouyiv1_aipu_t *aipu);
/**
 * @brief enable AIPU interrupts
 *
 * @param aipu: zhouyiv1_aipu struct pointer
 *
 * @return void
 */
void zhouyi_ctrl_enable_interrupt(zhouyiv1_aipu_t *aipu);
/**
 * @brief print HW ID information
 *
 * @param aipu: zhouyiv1_aipu struct pointer
 *
 * @return void
 */
void zhouyi_ctrl_print_hw_info(zhouyiv1_aipu_t *aipu);

#endif //_AIPU_ZHOUYIV1_H_
