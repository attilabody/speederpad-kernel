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
 * @file aipu_common.h
 * header file of common module for basic os interface DS
 */

#ifndef _AIPU_COMMON_H_
#define _AIPU_COMMON_H_

#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

/**
 * struct aipu_common - a struct with common shared data for sessions to ref
 *        There should be exactly one common object instance in KMD.
 *
 * @open_num: total reference number
 * @dev_name: device name
 * @dev_num: device major/minor
 * @aipu: aipu core struct
 * @p_dev: device struct
 * @aipu_fops: aipu file ops API struct
 * @aipu_misc: miscdevice struct
 * @lock: mutex for this struct
 */
typedef struct aipu_common {
    int              open_num;
    char             *dev_name;
    dev_t            dev_num;
    void             *aipu;
    int              host_to_aipu_map;
    struct device    *p_dev;
    struct file_operations aipu_fops;
    struct miscdevice *aipu_misc;
	struct mutex     lock;
} aipu_common_t;

/**
 * @brief create common struct
 *
 * @param dev_name: device name
 * @param p_dev: device struct pointer
 * @param aipu: aipu core struct
 * @param host_to_aipu_map: addr space difference map between AIPU and CPU
 *
 * @return common struct pointer if successful; NULL if failed.
 */
aipu_common_t *aipu_create_common(char *dev_name, struct device *p_dev, void *aipu, int host_to_aipu_map);
/**
 * @brief destroy common struct
 *
 * @param common: common struct pointer
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_destroy_common(aipu_common_t *common);
/**
 * @brief increase reference for new opening ops in userland
 *
 * @param common: common struct pointer
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_inc_common_ref(aipu_common_t *common);
/**
 * @brief decrease reference for closing ops in userland
 *
 * @param common: common struct pointer
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
int aipu_dec_common_ref(aipu_common_t *common);
/**
 * @brief get current reference
 *
 * @param common: common struct pointer
 *
 * @return ref value if successful; errcode < 0 if failed.
 */
int aipu_get_common_ref(aipu_common_t *common);
/**
 * @brief get device struct pointer from common
 *
 * @param common: common struct pointer
 *
 * @return device pointer if successful; NULL if failed.
 */
struct device *aipu_get_dev_ptr(aipu_common_t *common);
/**
 * @brief get aipu core struct pointer from common
 *
 * @param common: common struct pointer
 *
 * @return ipu core pointer if successful; NULL if failed.
 */
void *aipu_get_ipu_core(aipu_common_t *common);

#endif //_AIPU_COMMON_H_

