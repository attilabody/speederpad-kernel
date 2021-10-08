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
 * @file aipu_mm.h
 * Memory Manager module header file
 */

#ifndef _AIPU_MM_H_
#define _AIPU_MM_H_

#include <linux/device.h>
#include "aipu_session.h"
#include "aipu_cma.h"
#include "aipu_buffer.h"
#include "aipu_private.h"

#define KMD_SHARED_MEM_TYPE_CMA   0

/**
 * struct aipu_memory_manager: manage memory buffers
 * @cma: contiguous memory allocation struct
 * @use_cma: flag inndicating using cma or not
 * @aipu_addr_start: AIPU cached addr space start
 * @aipu_addr_end: AIPU cached addr space end
 * @tot_size: AIPU cached addr space size
*/
typedef struct aipu_memory_manager{
    aipu_cma_t cma;
    int use_cma;
    int sbuf_base_addr;
    int tot_size;
    struct device *dev;
} aipu_mm_t;

/*
 * @brief initialize mm module
 *
 * @param type: specify memory type to be used in shared mem
 * @param dev: device struct pointer
 * @param phys_base: region base (if CMA type is not used)
 * @param size: region size (if CMA type is not used)
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_init_mm(int type, struct device *dev, u32 phys_base, u32 size);
/*
 * @brief de-initialize mm module while kernel module unloading
 *
 * @param void
 *
 * @return void
 */
void aipu_deinit_mm(void);
/*
 * @brief memory allocation API
 *
 * @param buf_req: buffer request from userland
 * @param buf: buffer descriptor returned
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_mm_alloc(buf_request_t *buf_req, aipu_buffer_t *buf);
/*
 * @brief memory free API
 *
 * @param buf: buffer descriptor
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_mm_free(_aipu_const_ buf_desc_t *buf);
/*
 * @brief get shared buffer start base address
 *
 * @param void
 *
 * @return base.
 */
unsigned long aipu_mm_get_sbuf_base(void);
/*
 * @brief free all allocation of a session
 *
 * @param session: session struct
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_mm_free_session_buffers(aipu_session_t *session);

#endif //_AIPU_MM_H_