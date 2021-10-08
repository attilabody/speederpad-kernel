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
 * @file aipu_job_manager.h
 * Job manager module header file
 */

#ifndef _AIPU_JOB_MANAGER_H_
#define _AIPU_JOB_MANAGER_H_

#include <linux/list.h>
#include <linux/spinlock.h>
#include "uk_interface/aipu_job_desc.h"
#include "aipu_session.h"
#include "aipu_private.h"
#include "aipu_core.h"

/**
 * struct aipu_job - job element struct describing a job under scheduling in job manager
 *        Job status will be tracked as soon as interrupt or user evenets come in.
 *
 * @desc: job desctiptor from userland
 * @session: session pointer refernece of this job
 * @session_job: corresponding job object in session
 * @state: job state
 * @exception_flag: exception flag
 * @valid_flag: valid flag, indicating this job canceled by user or not
 * @node: list head struct
 */
typedef struct aipu_job {
    user_job_desc_t  desc;
    aipu_session_t*  session;
    session_job_t*   session_job;
    int state;
    int exception_flag;
    int valid_flag;
    struct list_head node;
} aipu_job_t;

/**
 * struct aipu_job_manager - job manager
 *        Maintain all jobs and update their status
 *
 * @scheduled_queue_head: scheduled job queue head
 * @pending_queue_head: pending job queue head
 * @sched_num: number of jobs have been scheduled
 * @max_sched_num: maximum allowed scheduled job number
 * @aipu: aipu core pointer reference
 * @lock: spinlock
 */
typedef struct aipu_job_manager {
    aipu_job_t* scheduled_queue_head;
    aipu_job_t* pending_queue_head;
    int sched_num;
    int max_sched_num;
    void* aipu;
    int reset_flag;
    spinlock_t lock;
} aipu_job_manager_t;

/**
 * @brief init job manager
 *
 * @param max_sched_num: maximum allowed scheduled job number;
 * @param aipu: aipu core pointer reference;
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_init_job_manager(int max_sched_num, void *aipu);
/**
 * @brief de-init job manager
 *
 * @param void
 *
 * @return void
 */
void aipu_deinit_job_manager(void);
/**
 * @brief schedule new job flushed from userland
 *
 * @param user_job: user_job struct;
 * @param kern_job: session job;
 * @param session: session pointer refernece of this job;
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_job_manager_schedule_new_job(user_job_t *user_job,
    _aipu_const_ session_job_t *kern_job, _aipu_const_ aipu_session_t *session);
/**
 * @brief update job state and indicating if exception happens
 *
 * @param aipu: aipu hw system description struct
 * @param exception_flag: exception flag
 *
 * @return void
 */
void aipu_job_manager_update_job_state_irq(void* aipu, int exception_flag);
/**
 * @brief done interrupt handler for job manager
 *
 * @param void
 *
 * @return void
 */
void aipu_job_manager_update_job_queue_done_irq(void);
/**
 * @brief cancel all jobs flushed by a user thread
 *
 * @param session: session serviced for that user thread
 *
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed;
 */
int aipu_job_manager_cancel_session_jobs(aipu_session_t *session);

#endif //_AIPU_JOB_MANAGER_H_