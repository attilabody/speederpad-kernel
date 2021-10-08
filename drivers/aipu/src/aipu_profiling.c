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
 * @file aipu_profiling.c
 * Profiling related operations' implementation
 */

#include "uk_interface/aipu_profiling.h"
#include "uk_interface/aipu_errcode.h"
#include "log.h"

int init_profiling_data(profiling_data_t *pdata)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == pdata)
    {
        LOG(LOG_ERR, "invalid null profiling data args!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* currently do nothing */

finish:
    return ret;
}