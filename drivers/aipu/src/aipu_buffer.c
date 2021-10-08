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
 * @file aipu_buffer.c
 * Implementations of AIPU memory buffer module
 */

#include "uk_interface/aipu_errcode.h"
#include "aipu_buffer.h"
#include "log.h"

int init_aipu_buffer(aipu_buffer_t *buf,
	unsigned long phys_addr, void *kern_addr, unsigned long size)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == buf)
    {
        LOG(LOG_ERR, "invalid input args buf to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    buf->phys_addr = phys_addr;
    buf->kern_addr = kern_addr;
    buf->size = size;

finish:
    return ret;
}

int init_buffer_desc(aipu_buffer_t *buf, _aipu_const_ aipu_buffer_t *desc)
{
	int ret = AIPU_ERRCODE_NO_ERROR;

	if (NULL == buf)
	{
		LOG(LOG_ERR, "invalid input args!");
		ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
		goto finish;
	}

	if (NULL != desc)
	{
		ret = init_aipu_buffer(buf, desc->phys_addr, desc->kern_addr, desc->size);
	}
	else
	{
		ret = init_aipu_buffer(buf, 0, NULL, 0);
	}

finish:
	return ret;
}

unsigned long get_phys_addr(_aipu_const_ aipu_buffer_t *buf)
{
    return (buf != NULL) ? buf->phys_addr : 0;
}

unsigned long get_buf_size(_aipu_const_ aipu_buffer_t *buf)
{
    return (buf != NULL) ? buf->size : 0;
}
