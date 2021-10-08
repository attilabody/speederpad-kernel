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
 * @file aipu_core.c
 * Implementation of AIPU core module
 */

#include <asm/io.h>
#include "uk_interface/aipu_errcode.h"
#include "aipu_core.h"
#include "log.h"

int aipu_init_ioregion(io_region_t *region, u32 phys_base, u32 size)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == region)
    {
        LOG(LOG_ERR, "invalid input region args to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    if (0 == size)
    {
        LOG(LOG_ERR, "invalid input size args to be 0!");
        ret = map_errcode(AIPU_ERRCODE_INVALID_ARGS);
        goto finish;
    }

    region->kern = ioremap_nocache(phys_base, size);
    if (NULL == region->kern)
    {
        LOG(LOG_ERR, "ioremap failed!");
        ret = map_errcode(AIPU_ERRCODE_INVALID_ARGS);
        goto finish;
    }

    region->phys = phys_base;
    region->size = size;
    LOG(LOG_CLOSE, "KMD IO region ioremap successfully: phys base 0x%lx, size 0x%lx.",
        (unsigned long)phys_base, (unsigned long)size);

finish:
    return ret;
}

void aipu_deinit_ioregion(io_region_t *region)
{
    if ((NULL != region) && (NULL != region->kern))
    {
        iounmap(region->kern);
        region->kern = NULL;
    }
}

int aipu_core_init_irq(aipu_core_t *core, u32 irqnum, aipu_irq_uhandler_t uhandler, aipu_irq_bhandler_t bhandler, void* aipu,
    aipu_irq_trigger_t trigger_func, aipu_irq_ack_t ack_func, struct device *dev)
{
    core->irq_obj = aipu_init_irq_object(irqnum, uhandler, bhandler, aipu, trigger_func, ack_func, dev, "aipu");
    if (NULL == core->irq_obj)
    {
        return map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
    }
    else
    {
        return AIPU_ERRCODE_NO_ERROR;
    }
}

void aipu_deinit_core_irq(aipu_core_t* core)
{
    if (NULL != core)
    {
        aipu_irq_flush_workqueue(core->irq_obj);
        aipu_irq_terminate(core->irq_obj);
    }
}