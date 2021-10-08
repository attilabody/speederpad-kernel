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
 * @file aipu_core.h
 * Header of the core module, which is an abstraction of AIPU hardware core
 */

#ifndef _AIPU_CORE_H_
#define _AIPU_CORE_H_

#include <linux/device.h>
#include "aipu_irq.h"

/**
 * struct io_region - a general struct describe IO region
 *
 * @phys: physical address base of an IO region
 * @size: size of of an IO region in byte
 * @kern: kernel virtual address base remapped from phys
 */
typedef struct io_region {
    u32   phys;
    u32   size;
    void *kern;
} io_region_t;

/**
 * struct aipu_core - a general struct describe a hardware AIPU core
 *
 * @io: IO region of this AIPU core
 * @irq_obj: interrupt object of this core
 */
typedef struct aipu_core {
    io_region_t io;
    aipu_irq_object_t  *irq_obj;
} aipu_core_t;

/**
 * @brief initialize AIPU IO region
 *        No aipu or core param is passed because this is a general map func.
 *
 * @param region: region to be initialized;
 * @param phys_base: base address
 * @param size: region size
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_init_ioregion(io_region_t *region, u32 phys_base, u32 size);
/**
 * @brief de-initialize AIPU IO region
 *
 * @param region: region to be de-initialized;
 *
 * @return void
 */
void aipu_deinit_ioregion(io_region_t *region);
/**
 * @brief initialize AIPU IRQ object
 *
 * @param core: core to be de-initialized;
 * @param irqnum: interrupt number
 * @param uhandler: upper-half handler
 * @param bhandler: bottom-half handler
 * @param trigger_func: interrupt trigger function used in probing
 * @param ack_func: ack function
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_core_init_irq(aipu_core_t *core, u32 irqnum, aipu_irq_uhandler_t uhandler, aipu_irq_bhandler_t bhandler, void* aipu,
    aipu_irq_trigger_t trigger_func, aipu_irq_ack_t ack_func, struct device *dev);

void aipu_deinit_core_irq(aipu_core_t *core);
/**
 * @brief return AIPU core support profiling or not
 *
 * @param core: AIPU core
 *
 * @return true/false
 */
int aipu_core_is_support_profiling(void *core);

#endif //_AIPU_CORE_H_