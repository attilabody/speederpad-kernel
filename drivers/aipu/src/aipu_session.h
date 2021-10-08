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
 * @file aipu_session.h
 * session module header file
 */

#ifndef _AIPU_SESSION_H_
#define _AIPU_SESSION_H_

#include <linux/list.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include "uk_interface/aipu_buf_req.h"
#include "uk_interface/aipu_job_desc.h"
#include "uk_interface/aipu_exception.h"
#include "uk_interface/aipu_profiling.h"
#include "aipu_common.h"
#include "aipu_buffer.h"
#include "aipu_private.h"
#include "aipu_core.h"

/**
 * struct session_buf: session private buffer list
 * @desc: buffer descriptor struct
 * @dev_offset: offset of this buffer in device file
 * @map_num: memory mmapped number
 * @head: list head struct
 */
typedef struct session_buf {
    aipu_buffer_t    desc;
    int              dev_offset;
    int              map_num;
    struct list_head head;
} session_buf_t;

/**
 * struct session_job: session private job list
 * @desc: job descriptor struct
 * @state: job state
 * @exception_type: type of exception if any
 * @pdata: profiling data struct
 * @head: list head struct
 */
typedef struct session_job {
    user_job_desc_t desc;
    int state;
    int exception_type;
    profiling_data_t pdata;
    struct list_head head;
} session_job_t;

/**
 * struct aipu_session: private data struct for every file open operation
 * @user_pid: ID of the user thread doing the open operation
 * @sbuf_base: shared buffer addr space start point
 * @sbuf_list: successfully allocated shared buffer of this session
 * @sbuf_lock: mutex lock for sbuf list
 * @job_list: job list of this session
 * @job_lock: spinlock for job list
 * @common: common struct shared among sessions
 */
typedef struct aipu_session {
    int            user_pid;
    int            sbuf_base;
    session_buf_t  sbuf_list;
    struct mutex   sbuf_lock;
    session_job_t  job_list;
    spinlock_t     job_lock;
    aipu_common_t  *common;
} aipu_session_t;

/*
 * @brief create unique session DS for an open request
 *
 * @param pid: user mode thread pid
 * @param sbuf_base: cma base address
 * @param common: common struct pointer
 *
 * @return NON-NULL if successful; NULL if failed.
 */
aipu_session_t *aipu_create_session(int pid, int sbuf_base, aipu_common_t *common);
/*
 * @brief destroy an existing session
 *
 * @param session: session pointer
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_destroy_session(aipu_session_t *session);
/*
 * @brief get pid of this session
 *
 * @param session: session pointer
 *
 * @return id if successful; 0 if failed.
 */
int aipu_get_session_pid(_aipu_const_ aipu_session_t *session);
/*
 * @brief get common struct pointer
 *
 * @param session: session pointer
 *
 * @return common if successful; NULL if failed.
 */
aipu_common_t *aipu_get_common_ptr(_aipu_const_ aipu_session_t *session);
/*
 * @brief add an allocated buffer of this session
 *
 * @param session: session pointer
 * @param buf_req: request buffer struct pointer
 * @param buf: buffer allocated
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_add_buf(aipu_session_t *session, buf_request_t *buf_req, _aipu_const_ aipu_buffer_t *buf);
/*
 * @brief remove an allocated buffer of this session
 *
 * @param session: session pointer
 * @param buf: buffer to be removed
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_detach_buf(aipu_session_t *session, _aipu_const_ buf_desc_t *buf);
/*
 * @brief mmap an allocated buffer of this session
 *
 * @param session: session pointer
 * @param vma: vm_area_struct
 * @param dev: device struct
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_mmap_buf(aipu_session_t *session, struct vm_area_struct *vma, struct device *dev);
/*
 * @brief get first valid buffer descriptor of this session
 *
 * @param session: session pointer
 *
 * @return buffer if successful; NULL if failed.
 */
aipu_buffer_t *aipu_get_session_sbuf_head(_aipu_const_ aipu_session_t *session);
/*
 * @brief add a job descriptor of this session
 *
 * @param session: session pointer
 * @param user_job: userspace job descriptor pointer
 *
 * @return non-NULL kernel job ptr if successful; NULL if failed.
 */
session_job_t *aipu_session_add_job(aipu_session_t *session, user_job_t *user_job);
/*
 * @brief delete all jobs of a session
 *
 * @param session: session pointer
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_delete_jobs(aipu_session_t *session);

void aipu_session_job_done(aipu_session_t *session, session_job_t *job);

void aipu_session_job_excep(aipu_session_t *session, session_job_t *job, int exception_flag);

void aipu_session_job_update_pdata(aipu_session_t *session, session_job_t *job, io_region_t *scc);

/*
 * @brief get the detailed exception info.
 *
 * @param session: session pointer
 * @param excep: exception pointer
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_query_err(aipu_session_t *session, aipu_exception_t *excep);
/*
 * @brief get profiling data of a finished job
 *
 * @param session: session pointer
 * @param pdata: profiling data struct pointer
 *
 * @return AIPU_KMD_ERR_OK if successful; others if failed.
 */
int aipu_session_query_pdata(aipu_session_t *session, req_pdata_t *pdata);

#endif //_AIPU_SESSION_H_