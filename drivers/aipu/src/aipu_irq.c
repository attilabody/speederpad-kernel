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
 * @file aipu_irq.c
 * Implementation of the interrupt request and handlers' abstraction
 */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include "uk_interface/aipu_errcode.h"
#include "aipu_irq.h"
#include "log.h"

/* interrupt handlers registered into kernel
 * real interrupt handlers for each HW module (registered in IRQ object) will be called
 * in these handler.
 */
static irqreturn_t aipu_irq_handler_upper_half(int irq, void *dev_id);
static void aipu_irq_handler_bottom_half(struct work_struct *work);
struct workqueue_struct* aipu_wq = NULL;

aipu_irq_object_t* aipu_init_irq_object(u32 irqnum, aipu_irq_uhandler_t uhandler, aipu_irq_bhandler_t bhandler,
    void *data, aipu_irq_trigger_t trigger_func, aipu_irq_ack_t ack_func, struct device *dev, char *description)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_irq_object_t* irq_obj = NULL;

    if ((NULL == data) || (NULL == dev) || (NULL == description))
    {
        LOG(LOG_ERR, "invalid input args data or dev or description to be NULL!");
        ret = AIPU_ERRCODE_INTERNAL_NULLPTR;
        goto finish;
    }

    irq_obj = kmalloc(sizeof(aipu_irq_object_t), GFP_KERNEL);
    if (NULL == irq_obj)
    {
        LOG(LOG_ERR, "create irq object failed!");
        goto finish;
    }

    if (NULL == aipu_wq)
    {
        /* workqueue should be executed in order strictly */
        aipu_wq = create_singlethread_workqueue("aipu");
        if (NULL == aipu_wq)
        {
            LOG(LOG_ERR, "create aipu workqueue failed!");
            goto err_handle;
        }
    }

    INIT_WORK(&irq_obj->work, aipu_irq_handler_bottom_half);

    /* Probe for IRQ if no valid irqnum provided */
    if (AIPU_PLATFORM_GET_IRQNUM_NONE == irqnum)
    {
        if ((NULL != trigger_func) && (NULL != ack_func))
        {
            /* do probe */
            /* TBD */
            if (AIPU_PLATFORM_GET_IRQNUM_NONE == irqnum)
            {
                LOG(LOG_ERR, "probe for irq failed!");
                goto err_handle;
            }
        }
        else
        {
            LOG(LOG_ERR, "probe for irq failed!");
            goto err_handle;
        }
    }

    irq_obj->irqnum = irqnum;
    irq_obj->uhandler = uhandler;
    irq_obj->bhandler = bhandler;
    irq_obj->data = data;
    dev->driver_data = irq_obj;
    irq_obj->dev = dev;

    ret = request_irq(irqnum, aipu_irq_handler_upper_half, IRQF_SHARED | IRQF_TRIGGER_RISING, description, dev);
    if (0 != ret)
    {
        LOG(LOG_ERR, "request IRQ (num %u) failed! (errno = %d)", irqnum, ret);
        goto err_handle;
    }

    /* success */
    goto finish;

err_handle:
    kfree(irq_obj);
    irq_obj = NULL;

finish:
    return irq_obj;
}

static irqreturn_t aipu_irq_handler_upper_half(int irq, void *dev_id)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_irq_object_t* irq_obj = NULL;

    if (NULL == dev_id)
    {
        LOG(LOG_INFO, "TRQ Handler: null pointer!");
        goto error;
    }
    else
    {
        irq_obj = (aipu_irq_object_t*)(((struct device*)dev_id)->driver_data);
        ret = irq_obj->uhandler(irq_obj->data);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            goto error;
        }
        else
        {
            /* success */
            goto finish;
        }
    }

error:
    return IRQ_NONE;

finish:
    return IRQ_HANDLED;
}

static void aipu_irq_handler_bottom_half(struct work_struct *work)
{
    aipu_irq_object_t* irq_obj = NULL;

    if (NULL != work)
    {
        irq_obj = container_of(work, aipu_irq_object_t, work);
        irq_obj->bhandler(irq_obj->data);
    }
}

void aipu_irq_schedulework(aipu_irq_object_t *irq_obj)
{
    if (NULL == irq_obj)
    {
        LOG(LOG_ERR, "invalid input args irq_obj to be NULL!");
    }
    else
    {
        queue_work(aipu_wq, &irq_obj->work);
    }
}

void aipu_irq_flush_workqueue(aipu_irq_object_t *irq_obj)
{
    /* only one workqueue currently */
    flush_workqueue(aipu_wq);
}

void aipu_irq_terminate(aipu_irq_object_t *irq_obj)
{
    if (NULL != irq_obj)
    {
        flush_workqueue(aipu_wq);
        destroy_workqueue(aipu_wq);
        aipu_wq = NULL;
        free_irq(irq_obj->irqnum, irq_obj->dev);
        kfree(irq_obj);
        flush_scheduled_work();
    }
}