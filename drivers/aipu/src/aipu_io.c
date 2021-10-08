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
 * @file aipu_io.c
 * Implementations of AIPU IO R/W API
 */

#include <asm/io.h>
#include "aipu_io.h"
#include "log.h"

u8 aipu_read8(io_region_t *region, __IO offset)
{
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return readb((void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        LOG(LOG_ERR, "KMD io error: read8 invalid operation or args!");
        return 0;
    }
}

u16 aipu_read16(io_region_t *region, __IO offset)
{
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return readw((void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        LOG(LOG_ERR, "KMD io error: read16 invalid operation or args!");
        return 0;
    }
}

u32 aipu_read32(io_region_t *region, __IO offset)
{
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return readl((void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        if (NULL != region)
        {
            LOG(LOG_ERR, "KMD io error: read32 invalid operation or args! (offset 0x%lx, region_max 0x%lx)",
                (unsigned long)offset, (unsigned long)region->size);
        }
        else
        {
            LOG(LOG_ERR, "KMD io error: read32 invalid args to be NULL!");
        }
        return 0;
    }
}

void aipu_write8(io_region_t *region, __IO offset, unsigned int data)
{
    data = data & 0xFF;
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return writeb((u8)data, (void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        LOG(LOG_ERR, "KMD io error: write8 invalid operation or args!");
    }
}

void aipu_write16(io_region_t *region, __IO offset, unsigned int data)
{
    data = data & 0xFFFF;
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return writew((u16)data, (void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        LOG(LOG_ERR, "KMD io error: write16 invalid operation or args!");
    }
}

void aipu_write32(io_region_t *region, __IO offset, unsigned int data)
{
    if ((NULL != region) && (NULL != region->kern) && (offset < region->size))
    {
        return writel((u32)data, (void __iomem*)((__IO)(region->kern) + offset));
    }
    else
    {
        if (NULL != region)
        {
            LOG(LOG_ERR, "KMD io error: write32 invalid operation or args! (offset 0x%lx, region_max 0x%lx)",
                (unsigned long)offset, (unsigned long)region->size);
        }
        else
        {
            LOG(LOG_ERR, "KMD io error: write32 invalid args to be NULL!");
        }
    }
}
