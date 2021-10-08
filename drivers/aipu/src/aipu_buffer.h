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
 * Header file of AIPU memory buffer module
 */

#ifndef _AIPU_BUFFER_H_
#define _AIPU_BUFFER_H_

#include <linux/list.h>
#include "aipu_private.h"

/**
 * struct aipu_buffer: buffer info descriptor struct
 * @phys_addr: physical address start point of this buffer
 * @kern_addr: kernel virtual address start point
 * @size: buffer size
 */
typedef struct aipu_buffer {
    __u32 phys_addr;
    void  *kern_addr;
    __u32 size;
} aipu_buffer_t;

/*
 * @brief initialize a buffer with values
 *
 * @param buf: buffer struct pointer
 * @param phys_addr: physical addr
 * @param kern_addr: kernel virtual addr
 * @param size: buffer size
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int init_aipu_buffer(aipu_buffer_t *buf,
    unsigned long phys_addr, void *kern_addr, unsigned long size);
/*
 * @brief initialize a buffer with desc
 *
 * @param buf: buffer struct pointer
 * @param desc: buffer descriptor as initial value
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int init_buffer_desc(aipu_buffer_t *buf, _aipu_const_ aipu_buffer_t *desc);
/*
 * @brief get phys addr of this buffer
 *
 * @param buf: buffer struct pointer
 *
 * @return addr if successful; 0 if failed.
 */
unsigned long  get_phys_addr(_aipu_const_ aipu_buffer_t *buf);
/*
 * @brief get sizer of this buffer
 *
 * @param buf: buffer struct pointer
 *
 * @return non-zero size if successful; 0 if failed.
 */
unsigned long  get_buf_size(_aipu_const_ aipu_buffer_t *buf);

#endif //_AIPU_BUFFER_H_
