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
 * @file aipu_irq.h
 * Header of the interrupt request and handlers' abstraction
 */

#ifndef _AIPU_IRQ_H_
#define _AIPU_IRQ_H_

#include <linux/device.h>
#include <linux/workqueue.h>

#define AIPU_PLATFORM_GET_IRQNUM_NONE -1

typedef int  (*aipu_irq_uhandler_t) (void *arg);
typedef void (*aipu_irq_bhandler_t) (void *arg);
typedef void (*aipu_irq_trigger_t) (void *arg);
typedef void (*aipu_irq_ack_t) (void *arg);

/**
 * struct aipu_irq_object - IRQ instance for each hw module in AIPU with interrupt function
 *
 * @irqnum: interrupt number used to request IRQ
 * @data: core data structure containing this irq object instance
 * @uhandler: real upper-half handler
 * @bhandler: real bottom-half handler
 * @work: work struct
 */
typedef struct aipu_irq_object {
    u32 irqnum;
    void *data;
    aipu_irq_uhandler_t uhandler;
    aipu_irq_bhandler_t bhandler;
    struct work_struct  work;
    struct device *dev;
} aipu_irq_object_t;

/**
 * @brief initialize an AIPU IRQ object for a HW module with interrupt function
 *
 * @param irqnum: interrupt number
 * @param uhandler: upper-half handler
 * @param bhandler: bottom-half handler
 * @param data: core instance containing the irq instance;
 * @param trigger_func: interrupt trigger function used in probing
 * @param ack_func: ack function
 * @param description: irq object description string
 *
 * @return irq_object pointer if successful; NULL if failed;
 */
aipu_irq_object_t *aipu_init_irq_object(u32 irqnum, aipu_irq_uhandler_t uhandler, aipu_irq_bhandler_t bhandler,
    void *data, aipu_irq_trigger_t trigger_func, aipu_irq_ack_t ack_func, struct device *dev, char *description);
/**
 * @brief workqueue schedule API
 *
 * @param irq_obj: interrupt object
 *
 * @return void
 */
void aipu_irq_schedulework(aipu_irq_object_t *irq_obj);
/**
 * @brief workqueue flush API
 *
 * @param irq_obj: interrupt object
 *
 * @return void
 */
void aipu_irq_flush_workqueue(aipu_irq_object_t *irq_obj);
/**
 * @brief workqueue terminate API
 *
 * @param irq_obj: interrupt object
 *
 * @return void
 */
void aipu_irq_terminate(aipu_irq_object_t *irq_obj);

#endif //_AIPU_IRQ_H_