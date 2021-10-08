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
 * @file aipu_job_desc.c
 * Implementation of the job descriptor operation APIs
 */

#include "uk_interface/aipu_job_desc.h"
#include "uk_interface/aipu_errcode.h"
#include "log.h"

int init_job_desc(user_job_desc_t *job_desc, _aipu_const_ user_job_desc_t *desc)
{
	int ret = AIPU_ERRCODE_NO_ERROR;

	if (NULL == job_desc)
	{
		LOG(LOG_ERR, "invalid null job desc args!");
		ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
		goto finish;
	}

	if (NULL != desc)
	{
		*job_desc = *desc;
	}
	else
	{
		job_desc->start_pc_addr = 0;
		job_desc->data_0_addr = 0;
		job_desc->data_1_addr = 0;
		job_desc->job_id = 0;
		job_desc->enable_prof = 0;
	}

finish:
	return ret;
}

int is_job_profiling_enable(user_job_desc_t *job_desc)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == job_desc)
    {
        LOG(LOG_ERR, "invalid null job desc args!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        return 0;
    }

    return job_desc->enable_prof;
}