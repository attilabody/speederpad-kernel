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
 * @file aipu_ctrl.h
 * Control module header file
 */

#ifndef _AIPU_CTRL_H_
#define _AIPU_CTRL_H_

#include "uk_interface/aipu_job_desc.h"
#include "uk_interface/aipu_capability.h"
#include "uk_interface/aipu_profiling.h"
#include "uk_interface/aipu_io_req.h"
#include "aipu_core.h"
#include "aipu_private.h"

/*
 * @brief check if AIPU hardware is power off or not
 *
 * @param io: target io region of this operation
 *
 * @return true/false
 */
int  aipu_ctrl_ispwoff(io_region_t *io);
/*
 * @brief AIPU hardware power off API
 *
 * @param io: target io region of this operation
 *
 * @return void
 */
void aipu_ctrl_pwoff(io_region_t *io);
/*
 * @brief AIPU hardware power on API
 *
 * @param io: target io region of this operation
 *
 * @return void
 */
void aipu_ctrl_pwon(io_region_t *io);
/*
 * @brief AIPU hardware reset API
 *
 * @param io: target io region of this operation
 *
 * @return void
 */
void aipu_ctrl_reset(io_region_t *io);
/*
 * @brief AIPU hardware working status check API
 *
 * @param io: target io region of this operation
 *
 * @return AIPU_ERRCODE_NO_ERROR if hw status no error; others if error occurs;
 */
int  aipu_ctrl_status_check(io_region_t *io);
/*
 * @brief AIPU external interrupt (to host) enable API
 *
 * @param io: target io region of this operation
 *
 * @return void;
 */
void aipu_ctrl_enable_interrupt(io_region_t *io);
/*
 * @brief AIPU external interrupt (to host) disable API
 *
 * @param io: target io region of this operation
 *
 * @return void;
 */
void aipu_ctrl_disable_interrupt(io_region_t *io);
/*
 * @brief AIPU hardware working status check API
 *
 * @param io: target io region of this operation
 * @param udesc: job descriptor
* @param tid: thread ID
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_ctrl_trigger_run(io_region_t *io, _aipu_const_ user_job_desc_t *udesc, int tid);
/*
 * @brief AIPU hardware status register read API
 *
 * @param io: target io region of this operation
 *
 * @return reg val
 */
int aipu_ctrl_read_status_reg(io_region_t *io);
/*
 * @brief AIPU hardware interrupt clean API
 *
 * @param io: target io region of this operation
 * @param flag: specify which interrupt flag to be cleaned
 *
 * @return void
 */
void aipu_ctrl_clear_intrrupt_flag(io_region_t *io, unsigned int flag);
/*
 * @brief AIPU hardware info. read API
 *
 * @param io: target io region of this operation
 *
 * @return void
 */
void aipu_ctrl_get_hw_id_info(io_region_t *io);
/*
 * @brief AIPU hardware capability read API
 *
 * @param io: target io region of this operation
 * @param cap: cap struct to store the results
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_ctrl_query_cap(io_region_t *io, aipu_cap_t *cap);
/*
 * @brief AIPU hardware profiling data read API
 *
 * @param scc: target scc region of this operation
 * @param pdata: profiling data struct to store the results
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_ctrl_read_profiling_data(io_region_t *scc, profiling_data_t *pdata);

int aipu_ctrl_read_startpc_reg(io_region_t *io);

void aipu_ctrl_start_profiling(io_region_t *scc);

void aipu_ctrl_end_profiling(io_region_t *scc);

void aipu_ctrl_io_req(io_region_t *io, aipu_io_req_t *io_req);

bool aipu_ctrl_is_idle(io_region_t *io);

#endif //_AIPU_CTRL_H_