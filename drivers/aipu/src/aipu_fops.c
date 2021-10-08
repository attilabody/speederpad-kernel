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
 * @file aipu_fops.c
 * Implementations of KMD file operation API
 */

#include <linux/fs.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include "uk_interface/aipu_ioctl.h"
#include "uk_interface/aipu_capability.h"
#include "uk_interface/aipu_buf_req.h"
#include "uk_interface/aipu_job_desc.h"
#include "uk_interface/aipu_exception.h"
#include "uk_interface/aipu_profiling.h"
#include "uk_interface/aipu_io_req.h"
#include "uk_interface/aipu_errcode.h"
#include "aipu_common.h"
#include "aipu_mm.h"
#include "aipu_job_manager.h"
#include "aipu_session.h"
#include "aipu_buffer.h"
#include "aipu_ctrl.h"
#include "log.h"

static int aipu_open(struct inode *inode, struct file *filp)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_common_t* common = NULL;
    aipu_session_t* session  = NULL;
    int pid = task_pid_nr(current);
    void* aipu = NULL;

    common = container_of(filp->f_op, aipu_common_t, aipu_fops);
    aipu = aipu_get_ipu_core(common);

	ret = aipu_get_common_ref(common);
	if (0 == ret)
	{
		aipu_ctrl_pwon(aipu);
	}

    /* validate AIPU HW status */
    ret = aipu_ctrl_status_check(aipu);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    session = aipu_create_session(pid, aipu_mm_get_sbuf_base(), common);
    if (NULL == session)
    {
        ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
        goto err_handle;
    }
    else
    {
        filp->private_data = session;
        filp->f_pos = 0;
    }

    ret = aipu_inc_common_ref(common);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    /* success */
    goto finish;

err_handle:
    if (NULL != session)
    {
        aipu_destroy_session(session);
    }

finish:
    return ret;
}

static int aipu_release(struct inode *inode, struct file *filp)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_common_t* common = NULL;
    aipu_session_t* session = filp->private_data;
    void* aipu = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "session nonexist!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    common = aipu_get_common_ptr(session);
    if (NULL == common)
    {
        LOG(LOG_ERR, "common nonexist!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
        goto finish;
    }

    aipu = aipu_get_ipu_core(common);
    if (NULL == aipu)
    {
        LOG(LOG_ERR, "aipu nonexist!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
        goto finish;
    }

    /* jobs should be cleared prior to buffer free */
    ret = aipu_job_manager_cancel_session_jobs(session);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    ret = aipu_mm_free_session_buffers(session);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    ret = aipu_destroy_session(session);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

    ret = aipu_dec_common_ref(common);
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        goto err_handle;
    }

	ret = aipu_get_common_ref(common);
	if (0 == ret)
	{
		aipu_ctrl_pwoff(aipu);
	}

    /* success */
    goto finish;

err_handle:
    /* no handler */
finish:
    return ret;
}

static long aipu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    int cp_ret = AIPU_ERRCODE_NO_ERROR;
    aipu_session_t* session = filp->private_data;
    aipu_common_t* common = NULL;

    if (NULL == session)
    {
        LOG(LOG_ERR, "session nonexist!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    common = aipu_get_common_ptr(session);
    if (NULL == common)
    {
        LOG(LOG_ERR, "common nonexist!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
        goto finish;
    }

    switch (cmd)
    {
        case IPUIOC_CLEAR:
        {
            ret = aipu_job_manager_cancel_session_jobs(session);
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: clear jobs failed!");
            }
            else
            {
                ret = aipu_mm_free_session_buffers(session);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: free buffers failed!");
                }
            }
            break;
        }
        case IPUIOC_QUERYCAP:
        {
            aipu_cap_t cap;
            ret = copy_from_user(&cap, (aipu_cap_t __user*)arg, sizeof(aipu_cap_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: QUERYCAP copy from user failed!");
            }
            else
            {
                ret = aipu_ctrl_query_cap(aipu_get_ipu_core(common), &cap);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: QUERYCAP query failed!");
                }

                /* copy cap info/errcode to user for reference */
                cp_ret = copy_to_user((aipu_cap_t __user*)arg, &cap, sizeof(aipu_cap_t));
                if ((AIPU_ERRCODE_NO_ERROR == ret) && (AIPU_ERRCODE_NO_ERROR != cp_ret))
                {
                    ret = cp_ret;
                }
            }
            break;
        }
        case IPUIOC_REQBUF:
        {
            buf_request_t buf_req;
            aipu_buffer_t buf;
            ret = copy_from_user(&buf_req, (buf_request_t __user*)arg, sizeof(buf_request_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: REQBUF copy from user failed!");
            }
            else
            {
                ret = aipu_mm_alloc(&buf_req, &buf);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: request buf failed!");
                }
                else
                {
                    ret = aipu_session_add_buf(session, &buf_req, &buf);
                    if (AIPU_ERRCODE_NO_ERROR != ret)
                    {
                        LOG(LOG_ERR, "KMD ioctl: add buf failed!");
                    }
                }

                /* copy buf info/errcode to user for reference */
                cp_ret = copy_to_user((buf_request_t __user*)arg, &buf_req, sizeof(buf_request_t));
                if ((AIPU_ERRCODE_NO_ERROR == ret) && (AIPU_ERRCODE_NO_ERROR != cp_ret))
                {
                    ret = cp_ret;
                }
            }
            break;
        }
        case IPUIOC_RUNJOB:
        {
            user_job_t user_job;
            session_job_t* kern_job = NULL;
            ret = copy_from_user(&user_job, (user_job_t __user*)arg, sizeof(user_job_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: RUNJOB copy from user failed!");
            }
            else
            {
                kern_job = aipu_session_add_job(session, &user_job);
                if (NULL == kern_job)
                {
                    LOG(LOG_ERR, "KMD ioctl: RUNJOB add failed!");
                }
                else
                {
                    ret = aipu_job_manager_schedule_new_job(&user_job, kern_job, session);
                    if (AIPU_ERRCODE_NO_ERROR != ret)
                    {
                        LOG(LOG_ERR, "KMD ioctl: RUNJOB run failed!");
                    }
                }

                /* copy job errcode to user for reference */
                cp_ret = copy_to_user((user_job_t __user*)arg, &user_job, sizeof(user_job_t));
                if ((AIPU_ERRCODE_NO_ERROR == ret) && (AIPU_ERRCODE_NO_ERROR != cp_ret))
                {
                    ret = cp_ret;
                }
            }
            break;
        }
        case IPUIOC_QUERYERR:
        {
            aipu_exception_t err;
            ret = copy_from_user(&err, (aipu_exception_t __user*)arg, sizeof(aipu_exception_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: QUERYERR copy from user failed!");
            }
            else
            {
                ret = aipu_session_query_err(session, &err);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: QUERYERR query failed!");
                }

                /* copy exception info/errcode to user for reference */
                cp_ret = copy_to_user((aipu_exception_t __user*)arg, &err, sizeof(aipu_exception_t));
                if ((AIPU_ERRCODE_NO_ERROR == ret) && (AIPU_ERRCODE_NO_ERROR != cp_ret))
                {
                    ret = cp_ret;
                }
            }
            break;
        }
        case IPUIOC_FREEBUF:
        {
            buf_desc_t buf;
            ret = copy_from_user(&buf, (buf_desc_t __user*)arg, sizeof(buf_desc_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: FREEBUF copy from user failed!");
            }
            else
            {
                /* detach first to validate the free buf request */
                ret = aipu_session_detach_buf(session, &buf);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: detach session buffer failed!");
                }
                else
                {
                    /* do free operation */
                    ret = aipu_mm_free(&buf);
                    if (AIPU_ERRCODE_NO_ERROR != ret)
                    {
                        LOG(LOG_ERR, "KMD ioctl: free buf failed!");
                    }
                }
            }
            break;
        }
        case IPUIOC_QUERYPDATA:
        {
            req_pdata_t profiling_data;
            ret = copy_from_user(&profiling_data, (req_pdata_t __user*)arg, sizeof(req_pdata_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: QUERYPDATA copy from user failed!");
            }
            else
            {
                ret = aipu_session_query_pdata(session, &profiling_data);
                if (AIPU_ERRCODE_NO_ERROR != ret)
                {
                    LOG(LOG_ERR, "KMD ioctl: QUERYPDATA query failed!");
                }

                /* copy pdata info/errcode to user for reference */
                cp_ret = copy_to_user((req_pdata_t __user*)arg, &profiling_data, sizeof(req_pdata_t));
                if ((AIPU_ERRCODE_NO_ERROR == ret) && (AIPU_ERRCODE_NO_ERROR != cp_ret))
                {
                    ret = cp_ret;
                }
            }
            break;
        }
        case IPUIOC_REQSHMMAP:
        {
            ret = copy_to_user((unsigned int __user*)arg, &common->host_to_aipu_map, sizeof(int));
            break;
        }
        case IPUIOC_REQIO:
        {
            aipu_io_req_t io_req;
            ret = copy_from_user(&io_req, (aipu_io_req_t __user*)arg, sizeof(aipu_io_req_t));
            if (AIPU_ERRCODE_NO_ERROR != ret)
            {
                LOG(LOG_ERR, "KMD ioctl: REQIO copy from user failed!");
            }
            else
            {
                aipu_ctrl_io_req(aipu_get_ipu_core(common), &io_req);
                ret = copy_to_user((aipu_io_req_t __user*)arg, &io_req, sizeof(aipu_io_req_t));
            }
            break;
        }
        default:
            LOG(LOG_ERR, "no matching ioctl call (cmd = 0x%lx)!", (unsigned long)cmd);
            ret = -ENOTTY;
            break;
    }

finish:
    return ret;
}

static int aipu_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret = AIPU_ERRCODE_NO_ERROR;
    aipu_common_t* common = NULL;
    aipu_session_t* session = filp->private_data;
    if (NULL == session)
    {
        LOG(LOG_ERR, "session nonexist!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto finish;
    }

    common = aipu_get_common_ptr(session);
    if (NULL == common)
    {
        LOG(LOG_ERR, "common nonexist!");
        ret = map_errcode(AIPU_ERRCODE_ITEM_NOT_FOUND);
        goto finish;
    }

    ret = aipu_session_mmap_buf(session, vma, aipu_get_dev_ptr(common));
    if (AIPU_ERRCODE_NO_ERROR != ret)
    {
        LOG(LOG_ERR, "mmap to userspace failed!");
    }

finish:
    return ret;
}

int aipu_fops_register(struct file_operations *fops)
{
    int ret = AIPU_ERRCODE_NO_ERROR;

    if (NULL == fops)
    {
        LOG(LOG_ERR, "invalid input fops args to be null!");
        ret = map_errcode(AIPU_ERRCODE_INTERNAL_NULLPTR);
        goto err_handle;
    }

    fops->owner = THIS_MODULE;
    fops->open = aipu_open;
    fops->unlocked_ioctl = aipu_ioctl;
    fops->mmap = aipu_mmap;
    fops->release = aipu_release;

    /* success */
    goto finish;

err_handle:
    /* no handler */
finish:
    return ret;
}
