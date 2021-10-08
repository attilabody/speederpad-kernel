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
 * @file aipu_ctrl.c
 * Zhouyiv1 AIPU control interfaces implementation file
 */

#include <linux/delay.h>
#include "uk_interface/aipu_errcode.h"
#include "zhouyiv1.h"
#include "aipu_ctrl.h"
#include "aipu_io.h"
#include "log.h"

int aipu_ctrl_ispwoff(io_region_t *scc)
{
#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return 0;
    }

    /* TBD */
#endif  //KMD_USE_SCC_REGION
	return 1;
}

void aipu_ctrl_pwoff(io_region_t *scc)
{
#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return;
    }
/*
    aipu_write32(scc, AIPU_RESET_REG_OFFSET, ZHOUYIV1_PWOFF_AIPU_FLAG);
    mdelay(500);
*/
#endif //KMD_USE_SCC_REGION
}

void aipu_ctrl_pwon(io_region_t* scc)
{
#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return;
    }
/*
    aipu_write32(scc, AIPU_RESET_REG_OFFSET, ZHOUYIV1_PWON_AIPU_FLAG);
    mdelay(500);
*/
#endif //KMD_USE_SCC_REGION
}

void aipu_ctrl_reset(io_region_t* scc)
{
#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return;
    }

    aipu_ctrl_pwoff(scc);
    aipu_ctrl_pwon(scc);
    LOG(LOG_INFO, "AIPU hardware is reset.");
#endif //KMD_USE_SCC_REGION
}

int aipu_ctrl_status_check(io_region_t* io)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* commented because open should not rely on this status */
    /* hw need to add another status indicating normal state regardless of working status */
/*
    if (ZHOUYI_AIPU_IDLE_STATUS != aipu_read32(io, AIPU_STAT_REG_OFFSET))
    {
        ret = AIPU_ERRCODE_INVALID_INT_STAT;
    }
*/

finish:
    return ret;
}

void aipu_ctrl_enable_interrupt(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return;
    }
    aipu_write32(io, AIPU_CTRL_REG_OFFSET, ZHOUYIV1_IRQ_ENABLE_FLAG);
}

void aipu_ctrl_disable_interrupt(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return;
    }
    aipu_write32(io, AIPU_CTRL_REG_OFFSET, ZHOUYIV1_IRQ_DISABLE_FLAG);
}

int aipu_ctrl_trigger_run(io_region_t *io, _aipu_const_ user_job_desc_t *udesc, int tid)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    unsigned int phys_addr = 0;
    unsigned int phys_addr0 = 0;
    unsigned int phys_addr1 = 0;
    unsigned int start_pc = 0;

    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* Load data addr 0 register */
    phys_addr0 = (unsigned int)udesc->data_0_addr;
    aipu_write32(io, AIPU_DATA_ADDR_0_REG_OFFSET, phys_addr0);

    /* Load data addr 1 register */
    phys_addr1 = (unsigned int)udesc->data_1_addr;
    aipu_write32(io, AIPU_DATA_ADDR_1_REG_OFFSET, phys_addr1);

    /* Load interrupt PC */
    aipu_write32(io, AIPU_INTR_PC_REG_OFFSET, (unsigned int)udesc->intr_handler_addr);

    /* Load start PC register */
    /* use write back and invalidate DCache because HW does not implement invalidate option in Zhouyi-z1 */
    phys_addr = (unsigned int)udesc->start_pc_addr;
    start_pc = phys_addr | 0xD;
    aipu_write32(io, AIPU_START_PC_REG_OFFSET, start_pc);

    LOG(LOG_INFO, "[Thread %u] trigger AIPU Job 0x%x running done: start pc = 0x%x, dreg0 = 0x%x, dreg1 = 0x%x.",
        tid, udesc->job_id, start_pc, phys_addr0, phys_addr1);

finish:
    return ret;
}

int aipu_ctrl_read_startpc_reg(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return 0;
    }
    return aipu_read32(io, AIPU_START_PC_REG_OFFSET);
}

bool aipu_ctrl_is_queue_empty(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return 0;
    }
    return AIPU_BIT(aipu_read32(io, AIPU_STAT_REG_OFFSET), 16);
}

bool aipu_ctrl_is_idle(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return 0;
    }
    return AIPU_BIT(aipu_read32(io, AIPU_STAT_REG_OFFSET), 17);
}

int aipu_ctrl_read_status_reg(io_region_t* io)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return 0;
    }
    return aipu_read32(io, AIPU_STAT_REG_OFFSET);
}

void aipu_ctrl_clear_intrrupt_flag(io_region_t* io, unsigned int flag)
{
    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return;
    }
    aipu_write32(io, AIPU_STAT_REG_OFFSET, flag);
}

void aipu_ctrl_get_hw_id_info(io_region_t* io)
{
    int ret = 0;

    if (NULL == io)
    {
        LOG(LOG_ERR, "invalid input args io to be NULL!");
        return;
    }

    ret = aipu_read32(io, AIPU_STAT_REG_OFFSET);
    LOG(LOG_INFO, "AIPU Initial Status: 0x%x.", ret);
    LOG(LOG_INFO, "AIPU_START_PC_REG_OFFSET 0x%x: 0x%x",AIPU_START_PC_REG_OFFSET, aipu_read32(io, AIPU_START_PC_REG_OFFSET));
    LOG(LOG_INFO, "AIPU_INTR_PC_REG_OFFSET 0x%x: 0x%x", AIPU_INTR_PC_REG_OFFSET,aipu_read32(io, AIPU_INTR_PC_REG_OFFSET));
    LOG(LOG_INFO, "AIPU_IPI_CTRL_REG_OFFSET 0x%x: 0x%x", AIPU_IPI_CTRL_REG_OFFSET,aipu_read32(io, AIPU_IPI_CTRL_REG_OFFSET));
    LOG(LOG_INFO, "AIPU_DATA_ADDR_0_REG_OFFSET 0x%x : 0x%x",AIPU_DATA_ADDR_0_REG_OFFSET, aipu_read32(io, AIPU_DATA_ADDR_0_REG_OFFSET));
    LOG(LOG_INFO, "AIPU_DATA_ADDR_1_REG_OFFSET 0x%x : 0x%x",AIPU_DATA_ADDR_1_REG_OFFSET, aipu_read32(io, AIPU_DATA_ADDR_1_REG_OFFSET));
    LOG(LOG_INFO, "AIPU_CLK_CTRL_REG_OFFSET 0x%x: 0x%x",AIPU_CLK_CTRL_REG_OFFSET, aipu_read32(io, AIPU_CLK_CTRL_REG_OFFSET));

    LOG(LOG_DEFAULT, "###### AIPU HARDWARE IP ID INFORMATION ######");
    LOG(LOG_DEFAULT, "# ISA Version Register: 0x%x", aipu_read32(io, AIPU_ISA_VERSION_REG_OFFSET));
    LOG(LOG_DEFAULT, "# TPC Feature Register: 0x%x", aipu_read32(io, AIPU_TPC_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# SPU Feature Register: 0x%x", aipu_read32(io, AIPU_SPU_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# HWA Feature Register: 0x%x", aipu_read32(io, AIPU_HWA_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Revision ID Register: 0x%x", aipu_read32(io, AIPU_REVISION_ID_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Memory Hierarchy Feature Register: 0x%x", aipu_read32(io, AIPU_MEM_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Instruction RAM Feature Register:  0x%x", aipu_read32(io, AIPU_INST_RAM_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# TEC Local SRAM Feature Register:   0x%x", aipu_read32(io, AIPU_LOCAL_SRAM_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Global SRAM Feature Register:      0x%x", aipu_read32(io, AIPU_GLOBAL_SRAM_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Instruction Cache Feature Register:0x%x", aipu_read32(io, AIPU_INST_CACHE_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# Data Cache Feature Register:       0x%x", aipu_read32(io, AIPU_DATA_CACHE_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "# L2 Cache Feature Register:         0x%x", aipu_read32(io, AIPU_L2_CACHE_FEATURE_REG_OFFSET));
    LOG(LOG_DEFAULT, "#############################################");
}

int aipu_ctrl_query_cap(io_region_t *io, aipu_cap_t *cap)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if ((NULL == io) || (NULL == cap))
    {
        LOG(LOG_ERR, "invalid input args io or cap to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    cap->isa_version = aipu_read32(io, AIPU_ISA_VERSION_REG_OFFSET);
    cap->tpc_feature = aipu_read32(io, AIPU_TPC_FEATURE_REG_OFFSET);
    cap->hwa_feature = aipu_read32(io, AIPU_HWA_FEATURE_REG_OFFSET);
    cap->revision_id = aipu_read32(io, AIPU_REVISION_ID_REG_OFFSET);

    /* success */
    cap->errcode = AIPU_ERRCODE_NO_ERROR;

finish:
    return ret;
}

int aipu_ctrl_read_profiling_data(io_region_t* scc, profiling_data_t *pdata)
{
    if ((NULL == scc) || (NULL == pdata))
    {
        LOG(LOG_ERR, "invalid input args scc or pdata to be NULL!");
        return 0;
    }

#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    pdata->rdata_tot_msb = aipu_read32(scc, AIPU_ALL_RDATA_TOT_MSB);
    pdata->rdata_tot_lsb = aipu_read32(scc, AIPU_ALL_RDATA_TOT_LSB);
    pdata->wdata_tot_msb = aipu_read32(scc, AIPU_ALL_WDATA_TOT_MSB);
    pdata->wdata_tot_lsb = aipu_read32(scc, AIPU_ALL_WDATA_TOT_LSB);
    pdata->tot_cycle_msb = aipu_read32(scc, AIPU_TOT_CYCLE_MSB);
    pdata->tot_cycle_lsb = aipu_read32(scc, AIPU_TOT_CYCLE_LSB);
    pdata->id_latency_max_msb = aipu_read32(scc, AIPU_ID_LATENCY_MAX_MSB);
    pdata->id_latency_max_lsb = aipu_read32(scc, AIPU_ID_LATENCY_MAX_LSB);
    pdata->id_latency_single = aipu_read32(scc, AIPU_ID_LATENCY_SINGLE);
    pdata->all_latency_total_msb = aipu_read32(scc, AIPU_DMA_LATENCY_TOT_MSB);
    pdata->all_latency_total_lsb = aipu_read32(scc, AIPU_DMA_LATENCY_TOT_LSB);
    pdata->all_rdata_total_msb = aipu_read32(scc, AIPU_DMA_RDATA_TOT_MSB);
    pdata->all_rdata_total_lsb = aipu_read32(scc, AIPU_DMA_RDATA_TOT_LSB);
    pdata->all_ar_handshake_msb = aipu_read32(scc, AIPU_DMA_AR_HANDSHAKE_MSB);
    pdata->all_ar_handshake_lsb = aipu_read32(scc, AIPU_DMA_AR_HANDSHAKE_LSB);
    pdata->max_outstand = aipu_read32(scc, AIPU_MAX_OUTSTAND);
#endif

    return 0;
}

void aipu_ctrl_start_profiling(io_region_t* scc)
{
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return;
    }

#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    aipu_write32(scc, AIPU_WORK_STAT_START_REQ, 1);
#endif
}

void aipu_ctrl_end_profiling(io_region_t* scc)
{
    if (NULL == scc)
    {
        LOG(LOG_ERR, "invalid input args scc to be NULL!");
        return;
    }

#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    aipu_write32(scc, AIPU_WORK_STAT_END_REQ, 1);
#endif
}

void aipu_ctrl_io_req(io_region_t* io, aipu_io_req_t* io_req)
{
    if ((NULL == io) || (NULL == io_req))
    {
        LOG(LOG_ERR, "invalid input args io/io_req to be NULL!");
        return;
    }

    /* TBD: offset r/w permission should be checked */

    if (io_req->rw == AIPU_IO_READ)
    {
        io_req->value = aipu_read32(io, io_req->offset);
    }
    else if (io_req->rw == AIPU_IO_WRITE)
    {
        aipu_write32(io, io_req->offset, io_req->value);
    }
    io_req->errcode = AIPU_ERRCODE_NO_ERROR;
}
