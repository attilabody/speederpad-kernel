/**********************************************************************************
 * This file is CONFIDENTIAL and any use by you is subject to the terms of the
 * agreement between you and Arm China or the terms of the agreement between you
 * and the party authorised by Arm China to disclose this file to you.
 * The confidential and proprietary information contained in this file
 * may only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm China.
 *
 *	    (C) Copyright 2020 Arm Technology (China) Co. Ltd.
 *	                All rights reserved.
 *
 * This entire notice must be reproduced on all copies of this file and copies of
 * this file may only be made by a person if such person is permitted to do so
 * under the terms of a subsisting license agreement from Arm China.
 *

 *********************************************************************************/

/**
 * @file platform_driver_init.c
 * Implementations of the AIPU platform driver probe/remove func
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include "uk_interface/aipu_errcode.h"
#include "aipu_mm.h"
#include "aipu_job_manager.h"
#include "aipu_common.h"
#include "aipu_ctrl.h"
#include "aipu_irq.h"
#include "log.h"
#include "version.h"
#include "zhouyiv1.h"

/**
 * @common: common struct pointer storing device related os struct, like misc_device, device, file_operations, etc.;
 * @aipu: aipu core struct storing io addr space, interrupt info., binding to different projects or config;
 */
static aipu_common_t *common = NULL;
static void *aipu = NULL;

struct clk *clk_pll_aipu;
struct clk *clk_aipu;
struct clk *clk_aipu_slv;

/**
 * @brief do real remove and resource release operation
 *	    This function will be called while probe failing or module unloading.
 *	    AIPU hardware should be reset or poweroff before calling this function to ensure OS stability.
 * @param p_dev: platform devide struct pointer
 * @return void
 */
static void platform_driver_do_aipu_remove(struct platform_device *p_dev)
{
	aipu_deinit_job_manager();
	aipu_deinit_mm();
	aipu_destroy_common(common);
	common = NULL;
	zhouyi_destroy_aipu((zhouyiv1_aipu_t *)aipu);
	aipu = NULL;
}

/**
 * @brief remove operation registered to platfom_driver struct
 *	    This function will be called while the module is unloading.
 * @param p_dev: platform devide struct pointer
 * @return AIPU_ERRCODE_NO_ERROR anyway
 */
static int aipu_remove(struct platform_device *p_dev)
{
	if (NULL != aipu) {
	    aipu_ctrl_pwoff(aipu);
	}

	if (!IS_ERR_OR_NULL(clk_aipu_slv))
	{
		clk_disable_unprepare(clk_aipu_slv);
		LOG(LOG_INFO, "clk_aipu_slv disable ok ");
	}

	if (!IS_ERR_OR_NULL(clk_aipu))
	{
		clk_disable_unprepare(clk_aipu);
		LOG(LOG_INFO, "clk_aipu disable ok");
	}

	clk_aipu     = NULL;
	clk_aipu_slv = NULL;
	clk_pll_aipu = NULL;

	platform_driver_do_aipu_remove(p_dev);

	LOG(LOG_INFO, "AIPU KMD unloaded: clear up done. Exit.");
	return AIPU_ERRCODE_NO_ERROR;
}

/**
 * @brief probe operation registered to platfom_driver struct
 *	    This function will be called while the module is loading.
 *	    Note that corresponding device tree should be loaded sucessfully before probing.
 *	    In this function, platform resource like irq, io, scc, cma will be obtained, which
 *	    are used to initialize the AIPU core, mm, and job manager pivately defined struct.
 * @param p_dev: platform devide struct pointer
 * @return AIPU_ERRCODE_NO_ERROR if successful; others if failed.
 */
static int aipu_probe(struct platform_device *p_dev)
{
	int ret = AIPU_ERRCODE_NO_ERROR;
	struct resource *res = NULL;
	struct device_node *dev_node = NULL;
	int irqnum = -1;
	u32 core_phys_base = 0;
	u32 core_size = 0;
	u32 scc_phys_base = 0;
	u32 scc_size = 0;
	int max_sched_num = AIPU_MAX_SCHED_JOB_NUM;
	int cma_reserve_size = 0;
	int freq = 0;
	int host_to_aipu_map = 0;

	LOG(LOG_INFO, "AIPU KMD Init Start...");
	LOG(LOG_INFO, "KMD version: %s %s", KMD_BUILD_DEBUG_FLAG, KMD_VERSION);
	LOG(LOG_INFO, "AIPU version: %s", AIPU_VERSION);
	LOG(LOG_INFO, "AIPU config: %s.", AIPU_CONFIG);

	//dev_node = of_find_node_by_name(NULL, "aipu");
	dev_node = p_dev->dev.of_node;
	if (NULL == dev_node) {
	    LOG(LOG_ERR, "find device node failed!");
	    goto probe_fail;
	}

	/* try to get irqnum */
	res = platform_get_resource(p_dev, IORESOURCE_IRQ, 0);
	if ((NULL == res) || (res->start <= 0)) {
	    irqnum = AIPU_PLATFORM_GET_IRQNUM_NONE;
	    LOG(LOG_WARN, "get platform irq resource failed and will try to probe irq.");
	} else {
	    /* obtain irqnum successfully */
	    irqnum = res->start;
	    LOG(LOG_INFO, "get platform resource name %s,irq  %d resource ok.", res->name, irqnum);
	}

	res = platform_get_resource(p_dev, IORESOURCE_MEM, 0);
	if (NULL == res) {
	    LOG(LOG_ERR, "get platform io region failed!");
	    ret = map_errcode(AIPU_ERRCODE_PLAT_GET_RES_FAIL);
	    goto probe_fail;
	} else
	    LOG(LOG_INFO, "platform_get_resource ok!");

	core_phys_base = res->start;
	core_size = res->end - res->start + 1;

#if ((defined KMD_USE_SCC_REGION) && (KMD_USE_SCC_REGION == 1))
	res = platform_get_resource(p_dev, IORESOURCE_MEM, 1);
	if (NULL == res) {
	    LOG(LOG_ERR, "get platform scc region failed!");
	    ret = map_errcode(AIPU_ERRCODE_PLAT_GET_RES_FAIL);
	    goto probe_fail;
	}
	scc_phys_base = res->start;
	scc_size = res->end - res->start + 1;
#endif

#if 0
	ret = of_property_read_u32(dev_node, "fpga-freq", &freq);//use for fpga test
	if (0 != ret)
	    LOG(LOG_WARN, "get freq property failed !");
	else
	    LOG(LOG_INFO, "get freq property ok,freq %d !", freq);
#endif

	aipu = zhouyi_create_aipu(irqnum, core_phys_base, core_size, scc_phys_base, scc_size,
	    freq, &p_dev->dev);

	if (NULL == aipu) {
	    LOG(LOG_ERR, "create AIPU core failed!");
	    ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
	    goto probe_fail;
	}

	LOG(LOG_INFO, "create AIPU core ok!");

#if 0
	ret = of_property_read_u32(dev_node, "host-aipu-offset", &host_to_aipu_map);
	if (0 != ret) {
	    LOG(LOG_ERR, "probe fail: get host-to-aipu offset property failed!");
	    goto probe_fail;
	}
#endif

	common = aipu_create_common("aipu", &p_dev->dev, aipu, host_to_aipu_map);
	if (NULL == common) {
	    ret = map_errcode(AIPU_ERRCODE_CREATE_KOBJ_ERR);
	    goto probe_fail;
	}

#if ((defined KMD_MEM_USE_CMA) && (KMD_MEM_USE_CMA == 1))
	ret = of_property_read_u32(dev_node, "cma-reserved-bytes", &cma_reserve_size);
	if (0 != ret) {
	    LOG(LOG_ERR, "get cma reserved size property failed!");
	    goto probe_fail;
	}

	ret = aipu_init_mm(KMD_SHARED_MEM_TYPE_CMA, &p_dev->dev, 0, cma_reserve_size);
	if (AIPU_ERRCODE_NO_ERROR != ret) {
	    LOG(LOG_ERR, "init memory manager failed!");
	    goto probe_fail;
	}
#endif

	//set clock
	clk_pll_aipu = of_clk_get(dev_node, 0);
	if (IS_ERR_OR_NULL(clk_pll_aipu)) {
		dev_err(&p_dev->dev, "clk_pll_aipu get failed\n");
		ret = PTR_ERR(clk_pll_aipu);
		goto probe_fail;
	}
	clk_aipu = of_clk_get(dev_node, 1);
	if (IS_ERR_OR_NULL(clk_aipu)) {
		dev_err(&p_dev->dev, "clk_aipu get failed\n");
		ret = PTR_ERR(clk_aipu);
		goto probe_fail;
	}
	clk_aipu_slv = of_clk_get(dev_node, 2);
	if (IS_ERR_OR_NULL(clk_aipu_slv)) {
		dev_err(&p_dev->dev, "clk_aipu_slv get failed\n");
		ret = PTR_ERR(clk_aipu_slv);
		goto probe_fail;
	}
	if (clk_set_parent(clk_aipu, clk_pll_aipu)) {
		dev_err(&p_dev->dev, "set clk_aipu parent fail\n");
		ret = -EBUSY;
		goto probe_fail;
	}
	if (clk_set_rate(clk_aipu, 600 * 1000000)) {
		dev_err(&p_dev->dev, "set clk_aipu rate fail\n");
		ret = -EBUSY;
		goto probe_fail;
	}
	if (clk_prepare_enable(clk_aipu_slv)) {
		dev_err(&p_dev->dev, "clk_aipu_slv enable failed\n");
		ret = -EBUSY;
		goto probe_fail;
	}
	if (clk_prepare_enable(clk_aipu)) {
		dev_err(&p_dev->dev, "clk_aipu enable failed\n");
		ret = -EBUSY;
		goto probe_fail;
	}

	LOG(LOG_INFO, "set aipu clock ok!");

	ret = aipu_init_job_manager(max_sched_num, aipu);
	if (AIPU_ERRCODE_NO_ERROR != ret) {
	    LOG(LOG_ERR, "probe fail: init job manager failed!");
	    goto probe_fail;
	}

	/* success */
	zhouyi_ctrl_reset(aipu);
	zhouyi_ctrl_enable_interrupt(aipu);
	zhouyi_ctrl_print_hw_info(aipu);
	LOG(LOG_INFO, "AIPU KMD probe done.");
	goto finish;

/* FIXME */
probe_fail:
	if(!IS_ERR_OR_NULL(clk_aipu_slv)) {
		clk_disable_unprepare(clk_aipu_slv);
	}
	if(!IS_ERR_OR_NULL(clk_aipu)) {
		clk_disable_unprepare(clk_aipu);
	}
	clk_aipu = NULL;
	clk_aipu_slv = NULL;
	clk_pll_aipu = NULL;

	platform_driver_do_aipu_remove(p_dev);

finish:
	return ret;
}

static int aipu_suspend(struct platform_device *p_dev,pm_message_t state)
{
	zhouyiv1_aipu_t *aipu_t = (zhouyiv1_aipu_t*)aipu;
	if (aipu_t && aipu_ctrl_is_idle(&aipu_t->core0.io)) {
		LOG(LOG_INFO,"aipu in idle status !");
	} else {
		LOG(LOG_INFO,"aipu in busy status !");
		return -1;
	}

	zhouyi_ctrl_print_hw_info(aipu);
	if (!IS_ERR_OR_NULL(clk_aipu_slv)) {
		clk_disable_unprepare(clk_aipu_slv);
		LOG(LOG_INFO, "disable clk_aipu_slv ok");
	}
	if (!IS_ERR_OR_NULL(clk_aipu)) {
		clk_disable_unprepare(clk_aipu);
		LOG(LOG_INFO, "disable clk_aipu ok");
	}
	LOG(LOG_INFO, "aipu_suspend ok");

	return 0;
}

static int aipu_resume(struct platform_device *p_dev)
{
	if(NULL == p_dev) {
		LOG(LOG_ERR,"aipu is null ,resume fail...!");
		return -1;
	}

	if (clk_set_parent(clk_aipu, clk_pll_aipu)) {
		dev_err(&p_dev->dev, "set clk_aipu parent fail\n");
	}
	if (clk_set_rate(clk_aipu, 600 * 1000000)) {
		dev_err(&p_dev->dev, "set clk_aipu rate fail\n");
	}
	if (clk_prepare_enable(clk_aipu_slv)) {
		dev_err(&p_dev->dev, "clk_aipu_slv enable failed\n");
	}
	if (clk_prepare_enable(clk_aipu)) {
		dev_err(&p_dev->dev, "clk_aipu enable failed\n");
	}

	zhouyi_ctrl_enable_interrupt(aipu);
	zhouyi_ctrl_print_hw_info(aipu);
	LOG(LOG_INFO, "aipu_resume ok.");

	return 0;
}

static const struct of_device_id aipu_dev_id[] = {
	{.compatible = "armchina,zhouyiv1aipu" },
	{ }
};

MODULE_DEVICE_TABLE(of, aipu_dev_id);

static struct platform_driver aipu_platform_driver = {
	.probe = aipu_probe,
	.remove = aipu_remove,
	.suspend = aipu_suspend,
	.resume  = aipu_resume,
	.driver = {
	    .name = "ArmChina_AIPU_Driver",
	    .owner = THIS_MODULE,
	    .of_match_table = aipu_dev_id,
	},
};

module_platform_driver(aipu_platform_driver);
MODULE_LICENSE("GPL");
