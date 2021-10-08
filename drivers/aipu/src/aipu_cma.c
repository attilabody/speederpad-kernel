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
 * @file aipu_cma.c
 * Implementations of AIPU Contiguous Memory Allocation (CMA) module
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include "uk_interface/aipu_errcode.h"
#include "aipu_cma.h"
#include "log.h"

static void init_cma_buffer(cma_buffer_t *buf, unsigned long phys_addr, void* kern_addr, unsigned long size)
{
    if (NULL != buf)
    {
        init_aipu_buffer(&buf->desc, phys_addr, kern_addr, size);
        buf->is_free = 1;
        INIT_LIST_HEAD(&buf->node);
    }
}

static cma_buffer_t* create_cma_buffer(unsigned long phys_addr, void *kern_addr, unsigned long size)
{
    cma_buffer_t* buf = NULL;

    buf = vmalloc(sizeof(cma_buffer_t));
    init_cma_buffer(buf, phys_addr, kern_addr, size);

    return buf;
}

static void destroy_cma_buffer(cma_buffer_t* buf)
{
    if (NULL != buf)
    {
        vfree(buf);
    }
}

int init_cma(aipu_cma_t *cma, struct device *dev, unsigned long size)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    cma_buffer_t* new_buf = NULL;

    if ((NULL == cma) || (NULL == dev) || (0 == size))
    {
        LOG(LOG_ERR, "invalid input args cma or dev to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    //ret = of_reserved_mem_device_init(dev);
    if (ret != 0)
    {
        LOG(LOG_ERR, "reserved mem device init failed!");
        goto finish;
    }

    ret = dma_set_mask(dev, DMA_BIT_MASK(32));
    if (ret != 0)
    {
        LOG(LOG_ERR, "DMA set mask failed!");
        goto finish;
    }

    ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
    if (ret != 0)
    {
        LOG(LOG_ERR, "DMA set coherent mask failed!");
        goto finish;
    }

    cma->cma_start_kern = dma_alloc_coherent(dev, size,
        (dma_addr_t*)&cma->cma_start_phys, GFP_KERNEL);
    if (cma->cma_start_kern == NULL)
    {
        LOG(LOG_INFO, "DMA alloc coherent failed: size = 0x%lx, try to alloc 0x%lx.", size, (unsigned long)(size/2));
        cma->cma_start_kern = dma_alloc_coherent(dev, size/2,
            (dma_addr_t*)&cma->cma_start_phys, GFP_KERNEL);
        if (cma->cma_start_kern == NULL)
        {
            LOG(LOG_ERR, "DMA alloc coherent failed: size = 0x%lx.", (unsigned long)(size / 2));
            ret = map_errcode(AIPU_ERRCODE_NO_MEMORY);
            goto finish;
        }
        else
        {
            cma->cma_size = size/2;
        }
    }
    else
    {
        cma->cma_size = size;
    }

    LOG(LOG_INFO, "KMD request reserved CMA successfully: base 0x%lx, size = 0x%lx.",
        (unsigned long)cma->cma_start_phys, (unsigned long)cma->cma_size);

    cma->buffer_head = create_cma_buffer(0, NULL, 0);
    new_buf = create_cma_buffer(cma->cma_start_phys, cma->cma_start_kern, cma->cma_size);
    if (new_buf == NULL)
    {
        goto err_handle;
    }
    else
    {
        list_add(&new_buf->node, &cma->buffer_head->node);
    }

    mutex_init(&cma->lock);

    /* success */
    cma->is_init = 1;
    goto finish;

err_handle:
    dma_free_coherent(dev, size, cma->cma_start_kern, cma->cma_start_phys);

finish:
    return ret;
}

static void verify_cma_buffer_list(cma_buffer_t *head)
{
#if ((defined BUILD_DEBUG_VERSION) && (BUILD_DEBUG_VERSION == 1))
    int iter = 0;
    struct list_head* cursor = NULL;
    cma_buffer_t* target_buf = NULL;

    if (NULL != head)
    {
        LOG(LOG_INFO, "=============Verify CMA Buffer List===========");
        list_for_each(cursor, &head->node)
        {
            target_buf = list_entry(cursor, cma_buffer_t, node);
            LOG(LOG_INFO, "buffer %d, [0x%lx, 0x%lx], is_free = %d.",
                iter++,
                (unsigned long)target_buf->desc.phys_addr,
                (unsigned long)(target_buf->desc.phys_addr + target_buf->desc.size - 1),
                target_buf->is_free);
        }
        LOG(LOG_INFO, "==============================================");
    }
#endif
}

int aipu_cma_alloc(aipu_cma_t *cma, buf_request_t *buf_req, aipu_buffer_t *buf)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    cma_buffer_t* target_buf = NULL;
    cma_buffer_t* alloc_buf = NULL;
    cma_buffer_t* back_remaining_buf = NULL;
    struct list_head* cursor = NULL;
    int start = 0;
    int end = 0;
    int align = 0;
    int align_size = 0;
    int found = 0;
    int origial_size = 0;

    if ((NULL == cma) || (NULL == buf_req) || (NULL == buf))
    {
        LOG(LOG_ERR, "invalid input args cma or buf to be NULL!");
        if (NULL != buf_req)
        {
            buf_req->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    align = buf_req->align_in_page * 4 * 1024;
    align_size = ALIGN(buf_req->size, PAGE_SIZE);

    /* LOCK */
    mutex_lock(&cma->lock);
    /* searching for the 1st matching buffer from the cma list which meets the following conditions:
       1. free, i.e. not allocated and under using by other users;
       2. addr alignment as requested;
       3. enough remaining size as requested;
    */
    list_for_each(cursor, &cma->buffer_head->node)
    {
        target_buf = list_entry(cursor, cma_buffer_t, node);

        if (target_buf->is_free != 0)
        {
            start = ALIGN(target_buf->desc.phys_addr, align);
            end = start + align_size;
            if (end <= (target_buf->desc.phys_addr + target_buf->desc.size))
            {
                found = 1;
                break;
            }
        }
    }

    if (0 == found)
    {
        buf_req->errcode = AIPU_ERRCODE_NO_MEMORY;
        ret = map_errcode(AIPU_ERRCODE_NO_MEMORY);
        LOG(LOG_ERR, "no matching buffer found to be allocated!");
        goto unlock;
    }

    /* found matching buffer: to do allocation */
    if ((target_buf->desc.phys_addr == start) &&
        ((target_buf->desc.phys_addr + target_buf->desc.size) == end))
    {
        /*
        alloc buffer:   |<---------allocated----------->|
        equals to
        target buffer:  |<----------original----------->|
        */
        alloc_buf = target_buf;
    }
    else
    {
        alloc_buf = create_cma_buffer(start,
            (void*)((unsigned long)target_buf->desc.kern_addr + (start - target_buf->desc.phys_addr)),
            align_size);
        if ((target_buf->desc.phys_addr == start) &&
            (end < (target_buf->desc.phys_addr + target_buf->desc.size)))
        {
            /*
            alloc buffer:   |<--allocated-->|<--remaining-->|
            from start point of
            target buffer:  |<----------original----------->|
            */
            target_buf->desc.phys_addr += alloc_buf->desc.size;
            target_buf->desc.kern_addr = (void*)((unsigned long)target_buf->desc.kern_addr + alloc_buf->desc.size);
            target_buf->desc.size -= alloc_buf->desc.size;
            list_add_tail(&alloc_buf->node, &target_buf->node);
        }
        else if ((target_buf->desc.phys_addr < start) &&
            (end == (target_buf->desc.phys_addr + target_buf->desc.size)))
        {
            /*
            alloc buffer:   |<--remaining-->|<--allocated-->|
            ends at end point of
            target buffer:  |<----------original----------->|
            */
            target_buf->desc.size -= alloc_buf->desc.size;
            list_add(&alloc_buf->node, &target_buf->node);
        }
        else
        {
            /*
            alloc buffer:   |<--remaining-->|<--allocated-->|<--remaining-->|
            insides of
            target buffer:  |<-------------------original------------------>|
            */
            origial_size = target_buf->desc.size;
            /* prev remaining */
            target_buf->desc.size = alloc_buf->desc.phys_addr - target_buf->desc.phys_addr;
            list_add(&alloc_buf->node, &target_buf->node);
            /* back remaining */
            back_remaining_buf = create_cma_buffer(
                alloc_buf->desc.phys_addr + alloc_buf->desc.size,
                (void*)((unsigned long)alloc_buf->desc.kern_addr + alloc_buf->desc.size),
                origial_size - target_buf->desc.size - alloc_buf->desc.size);
            list_add(&back_remaining_buf->node, &alloc_buf->node);
        }
    }

    alloc_buf->is_free = 0;
#if 0
    verify_cma_buffer_list(cma->buffer_head);
#endif
    init_aipu_buffer(buf,
        alloc_buf->desc.phys_addr,
        alloc_buf->desc.kern_addr,
        alloc_buf->desc.size);
unlock:
    mutex_unlock(&cma->lock);
    /* UNLOCK */

finish:
    return ret;
}

int aipu_cma_free(aipu_cma_t *cma, _aipu_const_ buf_desc_t *buf_desc)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    int found = 0;
    cma_buffer_t* target_buf = NULL;
    cma_buffer_t* prev = NULL;
    cma_buffer_t* next = NULL;
    struct list_head* cursor = NULL;

    if ((NULL == cma) || (NULL == buf_desc))
    {
        LOG(LOG_ERR, "invalid input args cma or buf to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* LOCK */
    mutex_lock(&cma->lock);
    /* searching for the matching buffer from the cma list which meets the following conditions:
       1. on using;
       2. addr and size as buf specified;
    */
    list_for_each(cursor, &cma->buffer_head->node)
    {
        target_buf = list_entry(cursor, cma_buffer_t, node);
        if (0 == target_buf->is_free)
        {
            if ((target_buf->desc.phys_addr == buf_desc->phys_addr) &&
                (target_buf->desc.size == buf_desc->align_size))
            {
                found = 1;
                break;
            }
        }
    }

    if (0 == found)
    {
        ret = map_errcode(AIPU_ERRCODE_NO_MEMORY);
        LOG(LOG_ERR, "no matching buffer found to be freed!");
        goto unlock;
    }

    target_buf->is_free = 1;
    prev = list_prev_entry(target_buf, node);
    next = list_next_entry(target_buf, node);
    /*
       merge prev buffer and target buffer if they are free

       buffer list: ... <=> |<--prev-->| <=> |<--target-->| <=> |<--next-->| <=> ...
                                free              free              free

       buffer list: ... <=> |<------------merged new buffer--------------->| <=> ...
                                                 free
    */
    if ((prev->desc.size != 0) && (1 == prev->is_free))
    {
        prev->desc.size += target_buf->desc.size;
        list_del(&target_buf->node);
        destroy_cma_buffer(target_buf);
        target_buf = prev;
    }

    if ((next->desc.size != 0) && (1 == next->is_free))
    {
        target_buf->desc.size += next->desc.size;
        list_del(&next->node);
        destroy_cma_buffer(next);
    }

#if 0
    verify_cma_buffer_list(cma->buffer_head);
#endif
unlock:
    mutex_unlock(&cma->lock);
    /* UNLOCK */

finish:
    return ret;
}

int deinit_cma(aipu_cma_t *cma, struct device *dev)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    cma_buffer_t* target_buf = NULL;
    struct list_head* cursor = NULL;
    struct list_head* cursor_next = NULL;

    if ((NULL == cma) || (NULL == dev))
    {
        LOG(LOG_ERR, "invalid input args cma or dev to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    if (0 == cma->is_init)
    {
        goto finish;
    }

    /* for debug */
    verify_cma_buffer_list(cma->buffer_head);

    dma_free_coherent(dev, cma->cma_size, cma->cma_start_kern, cma->cma_start_phys);
    //of_reserved_mem_device_release(dev);

    cursor = &cma->buffer_head->node;
    list_for_each_safe(cursor, cursor_next, &cma->buffer_head->node)
    {
        target_buf = list_entry(cursor, cma_buffer_t, node);
        list_del(cursor);
        destroy_cma_buffer(target_buf);
    }

    cma->cma_start_phys = 0;
    cma->cma_start_kern = NULL;
    cma->cma_size = 0;
    cma->is_init = 0;

finish:
    return ret;
}
