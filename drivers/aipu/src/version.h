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
  * @file version.h
  * version herder file used to track KMD version for different projects & configs
  */

#ifndef _VERSION_H_
#define _VERSION_H_

#include "aipu_sunxi_platform.h"

#if ((defined BUILD_ZHOUYIV1_KMD_DEFAULT) && (BUILD_ZHOUYIV1_KMD_DEFAULT == 1))
#define KMD_VERSION  "1.1.18"
#define AIPU_VERSION "zhouyi v1"
#define AIPU_CONFIG  "default"
#endif //BUILD_ZHOUYIV1_KMD_DEFAULT

#if ((defined BUILD_DEBUG_VERSION) && (BUILD_DEBUG_VERSION == 1))
#define KMD_BUILD_DEBUG_FLAG "debug"
#else
#define KMD_BUILD_DEBUG_FLAG "release"
#endif //BUILD_DEBUG_VERSION

#endif //_VERSION_H_
