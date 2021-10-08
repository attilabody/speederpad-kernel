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
 * @file aipu_job_manager.c
 * Job manager module implementation file
 */

#include <linux/slab.h>
#include "uk_interface/aipu_job_desc.h"
#include "uk_interface/aipu_errcode.h"
#include "uk_interface/aipu_exception.h"
#include "aipu_job_manager.h"
#include "aipu_core.h"
#include "aipu_ctrl.h"
#include "log.h"
#include "zhouyiv1.h"

aipu_job_manager_t* job_manager = NULL;

static int init_aipu_job(aipu_job_t *aipu_job, _aipu_const_ user_job_desc_t *desc,
    _aipu_const_ session_job_t *kern_job, _aipu_const_ aipu_session_t *session)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (aipu_job == NULL)
    {
        LOG(LOG_ERR, "invalid input args buf_req or buf to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    init_job_desc(&aipu_job->desc, desc);
    aipu_job->session = (aipu_session_t*)session;
    aipu_job->session_job = (session_job_t*)kern_job;
    aipu_job->state = AIPU_JOB_STATE_IDLE;
    aipu_job->exception_flag = AIPU_EXCEP_NO_EXCEPTION;
    aipu_job->valid_flag = AIPU_JOB_FLAG_VALID;
    INIT_LIST_HEAD(&aipu_job->node);

finish:
    return ret;
}

static void destroy_aipu_job(aipu_job_t *job)
{
    if (job != NULL)
    {
        kfree(job);
    }
}

static aipu_job_t* create_aipu_job(user_job_desc_t *desc,
    _aipu_const_ session_job_t *kern_job, _aipu_const_ aipu_session_t *session)
{
    aipu_job_t* new_aipu_job = NULL;

    new_aipu_job = kmalloc(sizeof(aipu_job_t), GFP_KERNEL);
    if (init_aipu_job(new_aipu_job, desc, kern_job, session) != AIPU_ERRCODE_NO_ERROR)
    {
        destroy_aipu_job(new_aipu_job);
        new_aipu_job = NULL;
    }

    return new_aipu_job;
}

static void aipu_job_manager_trigger_job_sched(void *aipu, _aipu_const_ aipu_job_t *aipu_job)
{
    if (aipu_job != NULL)
    {
        aipu_ctrl_trigger_run(aipu, &aipu_job->desc, aipu_job->session->user_pid);
    }
}

int aipu_init_job_manager(int max_sched_num, void *aipu)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == aipu)
    {
        LOG(LOG_ERR, "invalid input args aipu to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto err_handle;
    }

    if (NULL != job_manager)
    {
        LOG(LOG_ERR, "invalid operation: re-init job manager!");
        ret = map_errcode(AIPU_ERRCODE_INVALID_OPS);
        goto err_handle;
    }

    job_manager = kmalloc(sizeof(aipu_job_manager_t), GFP_KERNEL);
    if (NULL == job_manager)
    {
        LOG(LOG_ERR, "create job manager obj failed!");
        ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
        goto err_handle;
    }

    job_manager->scheduled_queue_head = create_aipu_job(NULL, NULL, NULL);
    job_manager->pending_queue_head = create_aipu_job(NULL, NULL, NULL);
    if ((NULL == job_manager->pending_queue_head) ||
        (NULL == job_manager->scheduled_queue_head))
    {
        goto err_handle;
    }

    job_manager->reset_flag = 0;
    job_manager->sched_num = 0;
    job_manager->max_sched_num = max_sched_num;
    job_manager->aipu = aipu;

    spin_lock_init(&job_manager->lock);
    goto finish;

err_handle:
    aipu_deinit_job_manager();

finish:
    return ret;
}

static void delete_queue(aipu_job_t *head)
{
    aipu_job_t* cursor = head;
    aipu_job_t* next = NULL;

    if (cursor != NULL)
    {
        list_for_each_entry_safe(cursor, next, &cursor->node, node)
        {
            list_del(&cursor->node);
            destroy_aipu_job(cursor);
        }
    }
}

void aipu_deinit_job_manager(void)
{
    if (NULL != job_manager)
    {
        delete_queue(job_manager->scheduled_queue_head);
        delete_queue(job_manager->pending_queue_head);
        job_manager->sched_num = 0;
        kfree(job_manager);
        job_manager = NULL;
    }
}

static void aipu_schedule_pending_job_no_lock(void)
{
    aipu_job_t* curr = NULL;

    /* 1st pending job should be scheduled if any */
    if ((!list_empty(&job_manager->pending_queue_head->node)) &&
        ((job_manager->sched_num < job_manager->max_sched_num)))
    {
        /*
          detach head of pending queue and add it to the tail of scheduled job queue

                              |--->>------->>---|
                              |(real head)      |(tail)
          --------------------------------    ----------------------------------
          | j <=> j <=> j <=> j <=> head |    | [empty to fill] <=> j <=> head |
          --------------------------------    ----------------------------------
                  pending job queue                   scheduled job queue
        */
        curr = list_next_entry(job_manager->pending_queue_head, node);
        if (task_pid_nr(current) == 0)
        {
            LOG(LOG_DEBUG, "Trigger from pending queue...");
        }
        else
        {
            LOG(LOG_DEBUG, "Trigger directly...");
        }
        aipu_job_manager_trigger_job_sched(job_manager->aipu, curr);
        curr->state = AIPU_JOB_STATE_SCHED;
        list_move_tail(&curr->node, &job_manager->scheduled_queue_head->node);
        job_manager->sched_num++;
    }
    else
    {
        /**
         * do nothing because no pending job needs to be scheduled
         * or AIPU is not available to accept more jobs
         */
        LOG(LOG_DEBUG, "No pending jobs or job queue is full.");
    }
}

int aipu_job_manager_schedule_new_job(user_job_t *user_job,
    _aipu_const_ session_job_t *session_job, _aipu_const_ aipu_session_t *session)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_job_t* aipu_job = NULL;

    if ((user_job == NULL) ||
        (session_job == NULL) ||
        (session == NULL))
    {
        LOG(LOG_ERR, "invalid input args user_job or kern_job or session to be NULL!");
        if (user_job != NULL)
        {
            user_job->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    aipu_job = create_aipu_job(&user_job->desc, session_job, session);
    if (aipu_job == NULL)
    {
        user_job->errcode = AIPU_ERRCODE_CREATE_KOBJ_ERR;
        ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
        goto finish;
    }

    /* LOCK */
    spin_lock_irq(&job_manager->lock);

    /* pending the flushed job from userland and try to schedule it */
    aipu_job->state = AIPU_JOB_STATE_PENDING;
    list_add_tail(&aipu_job->node, &job_manager->pending_queue_head->node);
    aipu_schedule_pending_job_no_lock();

    spin_unlock_irq(&job_manager->lock);
    /* UNLOCK */

    /* success */
    user_job->errcode = AIPU_ERRCODE_NO_ERROR;

finish:
    return ret;
}

static void aipu_invalidate_canceled_jobs_no_lock(aipu_job_t *head, aipu_session_t *session)
{
    aipu_job_t* cursor = NULL;
    aipu_job_t* next = NULL;

    if ((NULL != head) && (NULL != session))
    {
        list_for_each_entry_safe(cursor, next, &head->node, node)
        {
            if (cursor->valid_flag == 0)
            {
                continue;
            }

            if (aipu_get_session_pid(cursor->session) == aipu_get_session_pid(session))
            {
                if (cursor->state == AIPU_JOB_STATE_SCHED)
                {
                    if (AIPU_RESET_ENABLED)
                    {
                        zhouyi_ctrl_reset(job_manager->aipu);
                        job_manager->sched_num--;
                        job_manager->reset_flag = 1;
                    }
                    LOG(LOG_DEBUG, "DBG: AIPU status 0x%x",
                        aipu_ctrl_read_status_reg(&((zhouyiv1_aipu_t*)job_manager->aipu)->core0.io));
                    LOG(LOG_DEBUG, "[Thread %u] cancel sched job 0x%x!",
                        aipu_get_session_pid(session), cursor->desc.job_id);
                }
                else if (cursor->state == AIPU_JOB_STATE_PENDING)
                {
                    LOG(LOG_DEBUG, "[Thread %u] cancel pending job 0x%x!",
                        aipu_get_session_pid(session), cursor->desc.job_id);
                }
                cursor->valid_flag = 0;
            }
        }
    }
}

static void aipu_delete_invalid_job_no_lock(aipu_job_t *head)
{
    aipu_job_t* cursor = NULL;
    aipu_job_t* next = NULL;

    if (NULL != head)
    {
        list_for_each_entry_safe(cursor, next, &head->node, node)
        {
            if (0 == cursor->valid_flag)
            {
                list_del(&cursor->node);
                destroy_aipu_job(cursor);
                cursor = NULL;
                /* sched_num has been decreased in invalidate func */
            }
        }
    }
}

int aipu_job_manager_cancel_session_jobs(aipu_session_t *session)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == session)
    {
        LOG(LOG_ERR, "invalid input args session to be NULL!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* LOCK */
    spin_lock_irq(&job_manager->lock);

    /**
     * mark cancelled jobs of this session as invalid and reset AIPU if they have been scheduled
     */
    aipu_invalidate_canceled_jobs_no_lock(job_manager->pending_queue_head, session);
    aipu_invalidate_canceled_jobs_no_lock(job_manager->scheduled_queue_head, session);

    /* delete invalid jobs data struct */
    aipu_delete_invalid_job_no_lock(job_manager->pending_queue_head);

    /* schedue pending jobs (if any) if AIPU has been reset */
    if (AIPU_RESET_ENABLED && job_manager->reset_flag)
    {
        aipu_schedule_pending_job_no_lock();
        job_manager->reset_flag = 0;
    }

    spin_unlock_irq(&job_manager->lock);
    /* UNLOCK */

    /* delete all session_job */
    aipu_session_delete_jobs(session);

finish:
    return ret;
}

static void aipu_job_manager_update_job_profiling_data(void *aipu, aipu_job_t *job)
{
    if ((job != NULL) &&
        (job->state == AIPU_JOB_STATE_END) &&
        (job->exception_flag == AIPU_EXCEP_NO_EXCEPTION) &&
        (job->session_job->desc.enable_prof != 0))
    {
        aipu_ctrl_end_profiling(&((zhouyiv1_aipu_t*)aipu)->scc);
        aipu_session_job_update_pdata(job->session, job->session_job, &((zhouyiv1_aipu_t*)aipu)->scc);
    }
}

void aipu_job_manager_update_job_state_irq(void* aipu, int exception_flag)
{
    aipu_job_t* curr = NULL;

    /* LOCK */
    spin_lock(&job_manager->lock);
    list_for_each_entry(curr, &job_manager->scheduled_queue_head->node, node)
    {
        if (curr->state == AIPU_JOB_STATE_SCHED)
        {
            curr->state = AIPU_JOB_STATE_END;
            curr->exception_flag = exception_flag;
            if (curr->valid_flag == AIPU_JOB_FLAG_VALID)
            {
                aipu_job_manager_update_job_profiling_data(aipu, curr);
                job_manager->sched_num--;
            }
            else if (!AIPU_RESET_ENABLED)
            {
                job_manager->sched_num--;
            }
            break;
        }
    }

    /* schedule a new pending job */
    aipu_schedule_pending_job_no_lock();
    spin_unlock(&job_manager->lock);
    /* UNLOCK */
}

void aipu_job_manager_update_job_queue_done_irq(void)
{
    aipu_job_t* curr = NULL;

    /* LOCK */
    spin_lock(&job_manager->lock);
    if (!list_empty(&job_manager->scheduled_queue_head->node))
    {
        curr = list_next_entry(job_manager->scheduled_queue_head, node);
        if (AIPU_JOB_STATE_END != curr->state)
        {
            LOG(LOG_ERR, "invalid state occurs in scheduled list! (job state = %u)", curr->state);
        }

        /*
           DO NOT call session API for invalid job because
           session struct probably not exist on this occasion
        */
        if (AIPU_JOB_FLAG_VALID == curr->valid_flag)
        {
            if (AIPU_EXCEP_NO_EXCEPTION == curr->exception_flag)
            {
                aipu_session_job_done(curr->session, curr->session_job);
            }
            else
            {
                aipu_session_job_excep(curr->session, curr->session_job, curr->exception_flag);
            }
        }
        else
        {
            LOG(LOG_DEBUG, "this done job has been cancelled by user.");
        }

        list_del(&curr->node);
        destroy_aipu_job(curr);
        curr = NULL;
        /* DO NOT minus sched_num here because upper half has done that */
    }
    spin_unlock(&job_manager->lock);
    /* UNLOCK */
}
