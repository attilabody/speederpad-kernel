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
 * @file aipu_session.c
 * Implementation of session module
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include "uk_interface/aipu_signal.h"
#include "uk_interface/aipu_errcode.h"
#include "aipu_session.h"
#include "aipu_ctrl.h"
#include "log.h"

static void init_session_buf(session_buf_t *buf,
	_aipu_const_ aipu_buffer_t *desc, int dev_offset)
{
    if (NULL != buf)
    {
        init_buffer_desc(&buf->desc, desc);
        buf->dev_offset = dev_offset;
        buf->map_num = 0;
        INIT_LIST_HEAD(&buf->head);
    }
}

static session_buf_t* create_session_buf(_aipu_const_ aipu_buffer_t *desc, int dev_offset)
{
    session_buf_t* sbuf = NULL;

    if (NULL == desc)
    {
        LOG(LOG_ERR, "descriptor is needed while creating new session buf!");
        goto finish;
    }

    sbuf = kmalloc(sizeof(session_buf_t), GFP_KERNEL);
    init_session_buf(sbuf, desc, dev_offset);

finish:
    return sbuf;
}

static int destroy_session_buffer(session_buf_t *buf)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL != buf)
    {
        kfree(buf);
    }
    else
    {
        LOG(LOG_ERR, "invalid null buf args or list not empty!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

    return ret;
}

static void init_session_job(session_job_t *job, _aipu_const_ user_job_desc_t *desc)
{
    if (NULL != job)
    {
        /* init job->desc to be all 0 if desc == NULL */
        init_job_desc(&job->desc, desc);
        job->state = 0;
        job->exception_type = AIPU_EXCEP_NO_EXCEPTION;
        init_profiling_data(&job->pdata);
        INIT_LIST_HEAD(&job->head);
    }
}

static session_job_t* create_session_job(_aipu_const_ user_job_desc_t *desc)
{
    session_job_t* new_job = NULL;

    if (NULL == desc)
    {
        LOG(LOG_ERR, "descriptor is needed while creating new session job!");
        goto finish;
    }

    new_job = vmalloc(sizeof(session_job_t));
    init_session_job(new_job, desc);

finish:
    return new_job;
}

static int destroy_session_job(session_job_t *job)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL != job)
    {
        vfree(job);
    }
    else
    {
        LOG(LOG_ERR, "invalid null job args or list not empty!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

    return ret;
}

#ifdef __inline__
static __inline__ int
#else
static int
#endif
is_session_all_jobs_end(_aipu_const_ aipu_session_t *session)
{
    return (session != NULL) ? list_empty(&session->job_list.head) : 1;
}

#ifdef __inline__
static __inline__ int
#else
static int
#endif
is_session_all_buffers_freed(_aipu_const_ aipu_session_t *session)
{
    return (session != NULL) ? list_empty(&session->sbuf_list.head) : 1;
}

static session_buf_t* find_buffer_bydesc_no_lock(_aipu_const_ aipu_session_t *session,
    _aipu_const_ buf_desc_t *buf_desc)
{
    session_buf_t* target_buf  = NULL;
    session_buf_t* session_buf = NULL;
    struct list_head* node = NULL;

    if ((NULL == session) || (NULL == buf_desc))
    {
        LOG(LOG_ERR, "invalid input session or buf_desc args to be null!");
        goto finish;
    }

    list_for_each(node, &session->sbuf_list.head)
    {
        session_buf = list_entry(node, session_buf_t, head);

        if ((NULL != session_buf) &&
            (get_phys_addr(&session_buf->desc) == buf_desc->phys_addr) &&
            (get_buf_size(&session_buf->desc) == buf_desc->align_size))
        {
            target_buf = session_buf;
            LOG(LOG_CLOSE, "found matching buffer to be deleted.");
            goto finish;
        }
    }

finish:
    return target_buf;
}

static session_buf_t* find_buffer_byoffset_no_lock(_aipu_const_ aipu_session_t* session,
    int offset, int len)
{
    session_buf_t* target_buf = NULL;
    session_buf_t* session_buf = NULL;
    struct list_head* node = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "invalid input session args to be null!");
        goto finish;
    }

    list_for_each(node, &session->sbuf_list.head)
    {
        session_buf = list_entry(node, session_buf_t, head);
        if ((NULL != session_buf) &&
            (session_buf->dev_offset == offset) &&
            (len <= session_buf->desc.size))
        {
            target_buf = session_buf;
            goto finish;
        }
    }

finish:
    return target_buf;
}

static session_job_t* find_job_byid_no_lock(_aipu_const_ aipu_session_t* session, int job_id)
{
    session_job_t* target_job = NULL;
    session_job_t* session_job = NULL;
    struct list_head* node = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "invalid input session args to be null!");
        goto finish;
    }

    list_for_each(node, &session->job_list.head)
    {
        session_job = list_entry(node, session_job_t, head);
        if ((NULL != session_job) &&
            (session_job->desc.job_id == job_id))
        {
            target_job = session_job;
            goto finish;
        }
    }

finish:
    return target_job;
}

static int get_dev_offset(_aipu_const_ aipu_session_t *session, int phys_addr)
{
    return (NULL != session) ? (phys_addr - session->sbuf_base) : 0;
}

/********************************************************************************
 *  The following APIs are called in thread context for session obj management  *
 *  and member query service                                                    *
 *  -- aipu_create_session                                                      *
 *  -- aipu_destroy_session                                                     *
 *  -- aipu_get_session_pid                                                     *
 *  -- aipu_get_common_ptr                                                      *
 ********************************************************************************/
aipu_session_t* aipu_create_session(int pid, int sbuf_base, aipu_common_t *common)
{
    aipu_session_t* session = NULL;

    if (NULL == common)
    {
        LOG(LOG_ERR, "invalid input session or common args to be null!");
        goto finish;
    }

    session = vmalloc(sizeof(aipu_session_t));
    if (NULL == session)
    {
        goto finish;
    }

    session->user_pid = pid;
    session->sbuf_base = sbuf_base;
    init_session_buf(&session->sbuf_list, NULL, 0);
    mutex_init(&session->sbuf_lock);
    init_session_job(&session->job_list, NULL);
    spin_lock_init(&session->job_lock);
    session->common = common;

finish:
    return session;
}

int aipu_destroy_session(aipu_session_t *session)
{
	int res = AIPU_ERRCODE_NO_ERROR;

    if ((NULL != session) &&
        is_session_all_jobs_end(session) &&
        is_session_all_buffers_freed(session))
    {
        vfree(session);
    }
    else
    {
        LOG(LOG_WARN, "invalid input session args to be null or invalid operation!");
        res = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }

	return res;
}

int aipu_get_session_pid(_aipu_const_ aipu_session_t *session)
{
    if (NULL != session)
    {
        return session->user_pid;
    }
    else
    {
        LOG(LOG_WARN, "invalid input session args to be null!");
        return map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
    }
}

aipu_common_t* aipu_get_common_ptr(_aipu_const_ aipu_session_t *session)
{
    if (NULL != session)
    {
        return session->common;
    }
    else
    {
        LOG(LOG_WARN, "invalid input session args to be null!");
        return NULL;
    }
}

/********************************************************************************
 *  The following APIs are called in thread context for servicing user space    *
 *  request in resource allocation/free and job scheduling via fops             *
 *  -- aipu_session_add_buf                                                     *
 *  -- aipu_session_detach_buf                                                  *
 *  -- aipu_get_session_sbuf_head                                               *
 *  -- aipu_session_mmap_buf                                                    *
 *  -- aipu_session_add_job                                                     *
 *  -- aipu_session_delete_jobs                                                 *
 ********************************************************************************/
int aipu_session_add_buf(aipu_session_t *session,
	buf_request_t *buf_req, _aipu_const_ aipu_buffer_t *buf)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    session_buf_t* new_sbuf = NULL;
    int dev_offset = 0;

    if ((NULL == session) ||
        (NULL == buf_req) ||
        (NULL == buf))
    {
        LOG(LOG_ERR, "invalid input session or buf_req or buf args to be null!");
        if (NULL != buf_req)
        {
            buf_req->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    dev_offset = get_dev_offset(session, get_phys_addr(buf));
    new_sbuf = create_session_buf(buf, dev_offset);
    if (NULL == new_sbuf)
    {
        LOG(LOG_ERR, "create session buf failed!");
        buf_req->errcode = AIPU_ERRCODE_CREATE_KOBJ_ERR;
        ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
    }
    else
    {
        mutex_lock(&session->sbuf_lock);
        list_add(&new_sbuf->head, &session->sbuf_list.head);

        /* success */
        /* copy buffer descriptor to userland */
        buf_req->desc.phys_addr = get_phys_addr(buf);
        buf_req->desc.dev_offset = dev_offset;
        buf_req->desc.align_size = get_buf_size(buf);
        buf_req->errcode = AIPU_ERRCODE_NO_ERROR;
        mutex_unlock(&session->sbuf_lock);
    }

finish:
    return ret;
}

int aipu_session_detach_buf(aipu_session_t *session, _aipu_const_ buf_desc_t *buf_desc)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    session_buf_t* target_buf = NULL;

    if ((NULL == session) ||
        (NULL == buf_desc))
    {
        LOG(LOG_ERR, "invalid input session or buf args to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* LOCK */
    mutex_lock(&session->sbuf_lock);
    target_buf = find_buffer_bydesc_no_lock(session, buf_desc);
    if (NULL == target_buf)
    {
        LOG(LOG_ERR, "no corresponding buffer found in this session!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
    }
    else
    {
        list_del(&target_buf->head);
        ret = destroy_session_buffer(target_buf);
        if (AIPU_ERRCODE_NO_ERROR != ret)
        {
            LOG(LOG_ERR, "destroy session failed!");
        }
        else
        {
            /* success */
            target_buf = NULL;
        }
    }
    mutex_unlock(&session->sbuf_lock);
    /* UNLOCK */

finish:
    return ret;
}

int aipu_session_mmap_buf(aipu_session_t *session, struct vm_area_struct *vma, struct device *dev)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    int offset = 0;
    int len = 0;
    unsigned long vm_pgoff = 0;
    session_buf_t* buf = NULL;

    if ((NULL == session) ||
        (NULL == vma))
    {
        LOG(LOG_ERR, "invalid input session or vma args to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    offset = vma->vm_pgoff * PAGE_SIZE;
    len = vma->vm_end - vma->vm_start;

    /* LOCK */
    mutex_lock(&session->sbuf_lock);
    /* to find an allocated buffer with matching dev offset and length */
    buf = find_buffer_byoffset_no_lock(session, offset, len);
    if (NULL == buf)
    {
        LOG(LOG_ERR, "invalid operation or args: no corresponding buffer found in this session!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
    }
    else
    {
        if (0 != buf->map_num)
        {
            LOG(LOG_ERR, "duplicated mmap operations on identical buffer!");
            ret = map_errcode(AIPU_ERRCODE_INVALID_OPS);
        }
        else
        {
            vm_pgoff = vma->vm_pgoff;
            vma->vm_pgoff = 0;
            vma->vm_flags |= VM_IO;
            vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
            ret = dma_mmap_coherent(dev, vma, buf->desc.kern_addr, (dma_addr_t)buf->desc.phys_addr, buf->desc.size);
            if (0 != ret)
            {
                LOG(LOG_ERR, "CMA mmap to userspace failed!");
            }
            else
            {
                /* success */
                buf->map_num++;
            }
            vma->vm_pgoff = vm_pgoff;
        }
    }
    mutex_unlock(&session->sbuf_lock);
    /* UNLOCK */

finish:
    return ret;
}

aipu_buffer_t* aipu_get_session_sbuf_head(_aipu_const_ aipu_session_t *session)
{
    session_buf_t* session_buf = NULL;
    aipu_buffer_t* buf_desc = NULL;
    struct list_head* node = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "invalid input session or buf_req or buf args to be null!");
        goto finish;
    }

    list_for_each(node, &session->sbuf_list.head)
    {
        if (!list_empty(node))
        {
            session_buf = list_entry(node, session_buf_t, head);
            buf_desc = &session_buf->desc;
            goto finish;
        }
    }

finish:
    return buf_desc;
}

session_job_t* aipu_session_add_job(aipu_session_t *session, user_job_t *user_job)
{
    session_job_t* kern_job = NULL;

    if ((NULL == session) ||
        (NULL == user_job))
    {
        LOG(LOG_ERR, "invalid input session or user_job args to be null!");
        if (NULL != user_job)
        {
            user_job->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        goto finish;
    }

    kern_job = create_session_job(&user_job->desc);
    if (NULL == kern_job)
    {
        LOG(LOG_ERR, "invalid input session or job args to be null!");
        user_job->errcode = AIPU_ERRCODE_CREATE_KOBJ_ERR;
    }
    else
    {
        /* THREAD LOCK */
        spin_lock_bh(&session->job_lock);
        list_add(&kern_job->head, &session->job_list.head);
        spin_unlock_bh(&session->job_lock);
        /* THREAD UNLOCK */

        /* success */
        user_job->errcode = AIPU_ERRCODE_NO_ERROR;
    }

finish:
    return kern_job;
}

int aipu_session_delete_jobs(aipu_session_t *session)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    session_job_t* cursor = NULL;
    session_job_t* next = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "invalid input session to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* THREAD LOCK */
    spin_lock_bh(&session->job_lock);
    list_for_each_entry_safe(cursor, next, &session->job_list.head, head)
    {
        list_del(&cursor->head);
        destroy_session_job(cursor);
        cursor = NULL;
    }
    spin_unlock_bh(&session->job_lock);
    /* THREAD UNLOCK */

finish:
    return ret;
}

/********************************************************************************
 *  The following APIs are called in interrupt context to update end job status *
 *  They will be called by IRQ handlers in job manager module                   *
 *  Note that param session and session_job passed by job manager is assumed    *
 *  to be valid and active (not cancelled by userland)                          *
 *  -- aipu_session_job_done                                                    *
 *  -- aipu_session_job_excep                                                   *
 *  -- aipu_session_job_update_pdata                                            *
 ********************************************************************************/
static struct task_struct* aipu_get_task_struct(int pid)
{
    struct task_struct* t;
    rcu_read_lock();
    t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
    rcu_read_unlock();
    return t;
}

void aipu_session_job_done(aipu_session_t *session, session_job_t *job)
{
    struct task_struct* t = NULL;
    struct siginfo info;

    if ((NULL == session) ||
        (NULL == job))
    {
        LOG(LOG_ERR, "invalid input session or job args to be null!");
        return;
    }

    /* IRQ LOCK */
    spin_lock(&session->job_lock);
    job->state = AIPU_JOB_STATE_END;
    job->exception_type = AIPU_EXCEP_NO_EXCEPTION;
    spin_unlock(&session->job_lock);
    /* IRQ UNLOCK */

    info.si_signo = AIPU_SIG_DONE;
    info.si_code = SI_KERNEL;
    t = aipu_get_task_struct(session->user_pid);
    if (NULL != t)
    {
        send_sig_info(AIPU_SIG_DONE, &info, t);
        LOG(LOG_CLOSE, "Done signal sent to UMD: job id 0x%x, sig 0x%x.", job->desc.job_id, info.si_signo);
    }
}

void aipu_session_job_excep(aipu_session_t *session, session_job_t *job, int exception_flag)
{
    struct task_struct* t = NULL;
    struct siginfo info;

    if ((NULL == session) ||
        (NULL == job))
    {
        LOG(LOG_ERR, "invalid input session or job args to be null!");
        return;
    }

    /* IRQ LOCK */
    spin_lock(&session->job_lock);
    job->state = AIPU_JOB_STATE_END;
    job->exception_type = exception_flag;
    spin_unlock(&session->job_lock);
    /* IRQ UNLOCK */

    info.si_signo = AIPU_SIG_EXCEPTION;
    info.si_code = SI_KERNEL;
    t = aipu_get_task_struct(session->user_pid);
    send_sig_info(AIPU_SIG_EXCEPTION, &info, t);
}

void aipu_session_job_update_pdata(aipu_session_t *session, session_job_t *job, io_region_t *scc)
{
    if ((NULL == session) ||
        (NULL == job) ||
        (NULL == scc))
    {
        LOG(LOG_ERR, "invalid input session or desc or scc args to be null!");
        return;
    }

    if (is_job_profiling_enable(&job->desc))
    {
        /* IRQ LOCK */
        spin_lock(&session->job_lock);
        aipu_ctrl_read_profiling_data(scc, &job->pdata);
        spin_unlock(&session->job_lock);
        /* IRQ UNLOCK */
    }
}

/********************************************************************************
 *  The following APIs are called in thread context for user query service      *
 *  after job end                                                               *
 *  -- aipu_session_query_err                                                   *
 *  -- aipu_session_query_pdata                                                 *                                                    *
 ********************************************************************************/
int aipu_session_query_err(aipu_session_t* session, aipu_exception_t* excep)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    session_job_t* job = NULL;

    if ((NULL == session) ||
        (NULL == excep))
    {
        LOG(LOG_ERR, "invalid input session or excep args to be null!");
        if (NULL != excep)
        {
            excep->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* THREAD LOCK */
    spin_lock_bh(&session->job_lock);
    job = find_job_byid_no_lock(session, excep->job_id);
    if (NULL == job)
    {
        LOG(LOG_ERR, "no job with matching id found!");
        excep->errcode = AIPU_ERRCODE_JOB_ID_NOT_FOUND;
        ret = map_errcode(AIPU_ERRCODE_JOB_ID_NOT_FOUND);
    }
    else
    {
        if (AIPU_JOB_STATE_END != job->state)
        {
            LOG(LOG_ERR, "job still running or idle!");
            excep->errcode = AIPU_ERRCODE_JOB_NOT_END;
            ret = map_errcode(AIPU_ERRCODE_JOB_NOT_END);
        }
        else
        {
            /* success */
            excep->type = job->exception_type;
            excep->errcode = AIPU_ERRCODE_NO_ERROR;
        }
    }
    spin_unlock_bh(&session->job_lock);
    /* THREAD UNLOCK */

finish:
    return ret;
}

int aipu_session_query_pdata(aipu_session_t* session, req_pdata_t* pdata)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    session_job_t* job = NULL;

    if ((NULL == session) ||
        (NULL == pdata))
    {
        LOG(LOG_ERR, "invalid input session or pdata args to be null!");
        if (NULL != pdata)
        {
            pdata->errcode = AIPU_ERRCODE_INTERNAL_NULLPTR;
        }
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    /* THREAD LOCK */
    spin_lock_bh(&session->job_lock);
    job = find_job_byid_no_lock(session, pdata->job_id);
    if (NULL == job)
    {
        LOG(LOG_ERR, "no job with matching id found!");
        pdata->errcode = AIPU_ERRCODE_JOB_ID_NOT_FOUND;
        ret = map_errcode(AIPU_ERRCODE_JOB_ID_NOT_FOUND);
    }
    else
    {
        if (AIPU_JOB_STATE_END != job->state)
        {
            LOG(LOG_ERR, "job still running or idle!");
            pdata->errcode = AIPU_ERRCODE_JOB_NOT_END;
            ret = map_errcode(AIPU_ERRCODE_JOB_NOT_END);
        }
        else if (!is_job_profiling_enable(&job->desc))
        {
            LOG(LOG_ERR, "job profiling is disabled!");
            pdata->errcode = AIPU_ERRCODE_JOB_PROF_DISABLED;
            ret = map_errcode(AIPU_ERRCODE_JOB_PROF_DISABLED);
        }
        else
        {
            /* success */
            pdata->data = job->pdata;
            pdata->errcode = AIPU_ERRCODE_NO_ERROR;
        }
    }
    spin_unlock_bh(&session->job_lock);
    /* THREAD UNLOCK */

finish:
    return ret;
}
