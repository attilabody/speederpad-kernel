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
 * @file zhouyiv1_default_cfg.h
 * Config options file for Zhouyi-v1 AIPU
 */

#ifndef _ZHOUYIV1_DEFAULT_CFG_H_
#define _ZHOUYIV1_DEFAULT_CFG_H_

#define KMD_MEM_USE_CMA         1
#define AIPU_MAX_SCHED_JOB_NUM  1
#define MAX_SUPPORT_ALIGN_PAGE  1

#if ((defined BUILD_PLATFORM_JUNO) && (BUILD_PLATFORM_JUNO == 1))
#define KMD_USE_SCC_REGION      1
#define AIPU_PROFILING_SUPPORT  1
#define AIPU_RESET_ENABLED      0
#else
#define KMD_USE_SCC_REGION      0
#define AIPU_PROFILING_SUPPORT  0
#define AIPU_RESET_ENABLED      0
#endif

#endif //_ZHOUYIV1_DEFAULT_CFG_H_