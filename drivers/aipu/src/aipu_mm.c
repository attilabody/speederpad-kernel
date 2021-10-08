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
 * @file aipu_mm.c
 * Memory Manager module implementation file
 */

#include "config/zhouyiv1_default_cfg.h"
#include "uk_interface/aipu_errcode.h"
#include "aipu_mm.h"
#include "log.h"

aipu_mm_t mm;

int aipu_init_mm(int type, struct device *dev, u32 phys_base, u32 size)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == dev)
    {
        LOG(LOG_ERR, "invalid input args dev to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    mm.dev = dev;
    mm.sbuf_base_addr = phys_base;
    mm.tot_size = size;

    if (KMD_SHARED_MEM_TYPE_CMA == type)
    {
        mm.use_cma = 1;
        ret = init_cma(&mm.cma, mm.dev, size);
        if (AIPU_ERRCODE_NO_ERROR == ret)
        {
            mm.sbuf_base_addr = mm.cma.cma_start_phys;
            mm.tot_size = mm.cma.cma_size;
        }
    }

finish:
    return ret;
}

void aipu_deinit_mm(void)
{
    if (1 == mm.use_cma)
    {
        deinit_cma(&mm.cma, mm.dev);
    }
}

int aipu_mm_alloc(buf_request_t *buf_req, aipu_buffer_t *buf)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if ((NULL == buf_req) || (NULL == buf))
    {
        LOG(LOG_ERR, "invalid input args buf_req or buf to be NULL!");
        buf_req->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    if ((buf_req->size >= mm.tot_size) || (buf_req->size == 0))
    {
        LOG(LOG_ERR, "invalid requested size!");
        buf_req->errcode = AIPU_ERRCODE_NO_MEMORY;
        ret = map_errcode(AIPU_ERRCODE_NO_MEMORY);
        goto finish;
    }

    if (buf_req->size == 0)
    {
        LOG(LOG_ERR, "invalid requested size to be 0!");
        buf_req->errcode = AIPU_ERRCODE_INVALID_ARGS;
        ret = map_errcode(AIPU_ERRCODE_INVALID_ARGS);
        goto finish;
    }

    if ((0 == buf_req->align_in_page) ||
        (buf_req->align_in_page > MAX_SUPPORT_ALIGN_PAGE))
    {
        LOG(LOG_ERR, "invalid alignment request not supported!");
        buf_req->errcode = AIPU_ERRCODE_NO_ALIGN_MEM;
        ret = map_errcode(AIPU_ERRCODE_NO_ALIGN_MEM);
        goto finish;
    }

    if (mm.use_cma)
    {
        ret = aipu_cma_alloc(&mm.cma, buf_req, buf);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            goto finish;
        }
    }
    else
    {
        /* other malloc mechanism */
        /* currently no supported */
        ret = AIPU_ERRCODE_INVALID_OPS;
    }

finish:
    return ret;
}

int aipu_mm_free(_aipu_const_ buf_desc_t *buf)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == buf)
    {
        LOG(LOG_ERR, "invalid input args buf to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    if (mm.use_cma)
    {
        ret = aipu_cma_free(&mm.cma, buf);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            goto finish;
        }
    }
    else
    {
        /* other free mechanism */
        /* currently no supported */
        ret = AIPU_ERRCODE_INVALID_OPS;
    }

finish:
    return ret;
}

unsigned long aipu_mm_get_sbuf_base(void)
{
    return mm.sbuf_base_addr;
}

int aipu_mm_free_session_buffers(aipu_session_t *session)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_buffer_t* buf = NULL;
    buf_desc_t desc;

    while (NULL != (buf = aipu_get_session_sbuf_head(session)))
    {
        desc.phys_addr = buf->phys_addr;
        desc.align_size = buf->size;
        ret = aipu_cma_free(&mm.cma, &desc);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            goto finish;
        }
        ret = aipu_session_detach_buf(session, &desc);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            goto finish;
        }
    }

finish:
    return ret;
}
