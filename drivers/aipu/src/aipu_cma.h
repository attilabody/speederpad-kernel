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
 * @file aipu_cma.h
 * CMA manager module header file
 */

#ifndef _AIPU_CMA_H_
#define _AIPU_CMA_H_

#include <linux/list.h>
#include <linux/mutex.h>
#include "uk_interface/aipu_buf_req.h"
#include "aipu_buffer.h"
#include "aipu_private.h"

/**
 * struct cma_buffer: CMA buffer node
 * @desc: buffer descriptor struct
 * @is_free: is free (not allocated to a session job) or not
 * @head: list head struct
 */
typedef struct cma_buffer {
    aipu_buffer_t desc;
    int is_free;
    struct list_head node;
} cma_buffer_t;

/**
 * struct aipu_cma: AIPU CMA module struct, maintaining all info related
 * @cma_start_phys: CMA region base address
 * @cma_start_kern: CMA base address mapped kernel virtual address
 * @cma_size: CMA region total size
 * @buffer: CMA buffer list, maintaining the allocation/free info.
 * @lock: mutex lock
 * @is_init: init flag
 */
typedef struct aipu_cma {
    unsigned long cma_start_phys;
    void *cma_start_kern;
    int   cma_size;
    cma_buffer_t *buffer_head;
    struct mutex lock;
    int is_init;
} aipu_cma_t;

/*
 * @brief initialize CMA module in MM
 *
 * @param cma: cma pointer
 * @param dev:  device pointer
 * @param size: total size
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int init_cma(aipu_cma_t *cma, struct device *dev, unsigned long size);
/*
 * @brief CMA allocation request API
 *
 * @param cma: cma pointer
 * @param buf_req:  buffer request struct from userland
 * @param buf: successfully allocated buffer descriptor
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_cma_alloc(aipu_cma_t *cma, buf_request_t *buf_req, aipu_buffer_t *buf);
/*
 * @brief CMA free request API
 *
 * @param cma: cma pointer
 * @param buf:  buffer free request struct from userland
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_cma_free(aipu_cma_t *cma, _aipu_const_ buf_desc_t *buf);
/*
 * @brief de-initialize CMA module in MM while kernel module unloading
 *
 * @param cma: cma pointer
 * @param dev:  device pointer
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int deinit_cma(aipu_cma_t *cma, struct device *dev);

#endif //_AIPU_CMA_H_