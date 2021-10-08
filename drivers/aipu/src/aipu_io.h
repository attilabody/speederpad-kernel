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
 * @file aipu_io.h
 * AIPU IO R/W API header file
 */

#ifndef _AIPU_IO_H_
#define _AIPU_IO_H_

#include <asm/types.h>
#include "aipu_core.h"

typedef volatile unsigned long __IO;

/*
 * @brief read AIPU register in byte (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @return register value
 */
u8 aipu_read8(io_region_t *region, __IO offset);
/*
 * @brief read AIPU register in half-word (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @return register value
 */
u16 aipu_read16(io_region_t *region, __IO offset);
/*
 * @brief read AIPU register in word (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @return register value
 */
u32 aipu_read32(io_region_t *region, __IO offset);
/*
 * @brief write AIPU register in byte (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @param data:   data to be writen
 * @return void
 */
void aipu_write8(io_region_t *region, __IO offset, unsigned int data);
/*
 * @brief write AIPU register in half-word (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @param data:   data to be writen
 * @return void
 */
void aipu_write16(io_region_t *region, __IO offset, unsigned int data);
/*
 * @brief write AIPU register in word (with memory barrier)
 *
 * @param region: IO region providing the base address
 * @param offset: AIPU register offset
 * @param data:   data to be writen
 * @return void
 */
void aipu_write32(io_region_t *region, __IO offset, unsigned int data);

#define AIPU_BIT(data, n) (((data)>>(n))&0x1)

#endif //_AIPU_IO_H_
