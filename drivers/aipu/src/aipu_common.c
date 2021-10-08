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
 * @file aipu_common.c
 * Implementations of common module APIs
 */

#include "uk_interface/aipu_errcode.h"
#include "aipu_common.h"
#include "aipu_fops.h"
#include "log.h"

static struct miscdevice aipu_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "aipu",
};

static aipu_common_t com;

aipu_common_t* aipu_create_common(char *dev_name, struct device *p_dev, void *aipu, int host_to_aipu_map)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_common_t* common = &com;

    if ((NULL == dev_name) ||
        (NULL == p_dev) ||
        (NULL == aipu))
    {
        LOG(LOG_ERR, "invalid input args to be null!");
        goto err_handle;
    }

    common->open_num = 0;
    common->dev_name = dev_name;
    common->dev_num = 1; /* ? */
    common->aipu = aipu;
    common->p_dev = p_dev;
    common->aipu_misc = NULL;
    ret = aipu_fops_register(&common->aipu_fops);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    /* misc init */
    aipu_misc.fops = &common->aipu_fops;
    ret = misc_register(&aipu_misc);
    if (0 != ret)
    {
        LOG(LOG_ERR, "AIPU misc register failed!");
        goto err_handle;
    }

    common->aipu_misc = &aipu_misc;
    common->host_to_aipu_map = host_to_aipu_map;
    mutex_init(&common->lock);

    /* success */
    goto finish;

err_handle:
    aipu_destroy_common(common);
    common = NULL;

finish:
    return common;
}

int aipu_destroy_common(aipu_common_t *common)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if ((NULL != common) &&
        (0 == aipu_get_common_ref(common)))
    {
        if (NULL != common->aipu_misc)
        {
            misc_deregister(common->aipu_misc);
        }
    }

    return ret;
}

int aipu_inc_common_ref(aipu_common_t *common)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL != common)
    {
        mutex_lock(&common->lock);
        common->open_num++;
        mutex_unlock(&common->lock);
    }
    else
    {
        LOG(LOG_ERR, "invalid args aipu_dev pointer to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

    return ret;
}

int aipu_dec_common_ref(aipu_common_t *common)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL != common)
    {
        if (aipu_get_common_ref(common) != 0)
        {
            mutex_lock(&common->lock);
            common->open_num--;
            mutex_unlock(&common->lock);
        }
        else
        {
            LOG(LOG_ERR, "invalid operation: no ref to be decreased!");
            ret = map_errcode(AIPU_ERRCODE_INVALID_OPS);
        }
    }
    else
    {
        LOG(LOG_ERR, "invalid args common pointer to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

    return ret;
}

int aipu_get_common_ref(aipu_common_t *common)
{
    int ret = 0;

    if (NULL != common)
    {
        mutex_lock(&common->lock);
        ret = common->open_num;
        mutex_unlock(&common->lock);
    }
    else
    {
        LOG(LOG_ERR, "invalid args aipu_dev pointer to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

    return ret;
}

struct device* aipu_get_dev_ptr(aipu_common_t *common)
{
    /* protect device struct at caller part */
    return (common != NULL) ? (common->p_dev) : NULL;
}

void* aipu_get_ipu_core(aipu_common_t *common)
{
    return (NULL != common) ? (common->aipu) : NULL;
}