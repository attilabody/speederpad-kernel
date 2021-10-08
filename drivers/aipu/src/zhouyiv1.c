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
 * @file zhouyiv1.c
 * Implementations of the zhouyiv1 AIPU hardware control and interrupt handle operations
 */

#include <linux/slab.h>
#include "uk_interface/aipu_signal.h"
#include "uk_interface/aipu_exception.h"
#include "uk_interface/aipu_errcode.h"
#include "aipu_job_manager.h"
#include "aipu_irq.h"
#include "zhouyiv1.h"
#include "aipu_ctrl.h"
#include "aipu_io.h"
#include "log.h"

/* interrupt handlers and irq probe func */
static int  aipu_upper_half(void *data);
static void aipu_bottom_half(void *data);
static void aipu_irq_probe_trigger(void *data);
static void aipu_irq_probe_ack(void *data);

zhouyiv1_aipu_t* zhouyi_create_aipu(int irqnum, u32 core_phys_base, u32 core_size,
    u32 scc_phys_base, u32 scc_size, u32 freq, struct device *dev)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    zhouyiv1_aipu_t* aipu = NULL;
    aipu_core_t* core0 = NULL;

    aipu = kmalloc(sizeof(zhouyiv1_aipu_t), GFP_KERNEL);
    if (NULL == aipu)
    {
        LOG(LOG_ERR, "kmalloc aipu_core failed!");
        goto finish;
    }

#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
    /* init scc */
    ret = aipu_init_ioregion(&aipu->scc, scc_phys_base, scc_size);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }
#endif

    /* init core0 */
    core0 = &aipu->core0;
    ret = aipu_core_init_irq(core0,
        irqnum,
        aipu_upper_half,
        aipu_bottom_half,
        aipu,
        aipu_irq_probe_trigger,
        aipu_irq_probe_ack,
        dev);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }
    ret = aipu_init_ioregion(&core0->io, core_phys_base, core_size);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    aipu->freq_in_MHz = freq;

    /* success */
    goto finish;

err_handle:
    zhouyi_destroy_aipu(aipu);

finish:
    return aipu;
}

void zhouyi_destroy_aipu(zhouyiv1_aipu_t *aipu)
{
    aipu_core_t* core0 = NULL;
    if (NULL != aipu)
    {
        core0 = &aipu->core0;
        aipu_deinit_ioregion(&core0->io);
#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
        aipu_deinit_ioregion(&aipu->scc);
#endif
        aipu_deinit_core_irq(core0);
        kfree(aipu);
        core0 = NULL;
        aipu = NULL;
    }
}

void zhouyi_ctrl_reset(zhouyiv1_aipu_t *aipu)
{
    if (NULL != aipu)
    {
        aipu_ctrl_reset(&aipu->scc);
    }
}

void zhouyi_ctrl_enable_interrupt(zhouyiv1_aipu_t* aipu)
{
    if (NULL != aipu)
    {
        aipu_ctrl_enable_interrupt(&aipu->core0.io);
    }
}

void zhouyi_ctrl_print_hw_info(zhouyiv1_aipu_t* aipu)
{
    if (NULL != aipu)
    {
        aipu_ctrl_get_hw_id_info(&aipu->core0.io);
    }
}

static int aipu_upper_half(void *data)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    zhouyiv1_aipu_t* aipu = (zhouyiv1_aipu_t*)data;
    aipu_core_t* core0 = &aipu->core0;

    if (NULL == core0)
    {
        LOG(LOG_ERR, "invalid input args data to be NULL!");
        ret = AIPU_ERRCODE_INTERNAL_NULLPTR;
        goto finish;
    }

    aipu_ctrl_disable_interrupt(&core0->io);
    ret = aipu_ctrl_read_status_reg(&core0->io);
    if (ret & ZHOUYIV1_IRQ_QEMPTY)
    {
        LOG(LOG_CLOSE, "QEmpty interrupt received.");
        aipu_ctrl_clear_intrrupt_flag(&core0->io, ZHOUYIV1_IRQ_QEMPTY);
    }
    else if (ret & ZHOUYIV1_IRQ_DONE)
    {
        LOG(LOG_INFO, "Done interrupt received.");
        aipu_ctrl_clear_intrrupt_flag(&core0->io, ZHOUYIV1_IRQ_DONE);
        aipu_job_manager_update_job_state_irq(aipu, AIPU_EXCEP_NO_EXCEPTION);
        aipu_irq_schedulework(core0->irq_obj);
    }
    else if (ret & ZHOUYIV1_IRQ_EXCEP)
    {
        LOG(LOG_INFO, "Exception interrupt received.");
        aipu_ctrl_clear_intrrupt_flag(&core0->io, ZHOUYIV1_IRQ_EXCEP);
        aipu_job_manager_update_job_state_irq(aipu, aipu_read32(&core0->io, AIPU_INTR_CAUSE_REG_OFFSET));
        aipu_irq_schedulework(core0->irq_obj);
    }
    aipu_ctrl_enable_interrupt(&core0->io);

    /* success */
    ret = AIPU_ERRCODE_NO_ERROR;

finish:
    return ret;
}

static void aipu_bottom_half(void *data)
{
    aipu_job_manager_update_job_queue_done_irq();
}

int aipu_core_is_support_profiling(void* core)
{
    return AIPU_PROFILING_SUPPORT;
}

static void aipu_irq_probe_trigger(void *data)
{
    /* TBD */
}

static void aipu_irq_probe_ack(void *data)
{
    /* TBD */
}