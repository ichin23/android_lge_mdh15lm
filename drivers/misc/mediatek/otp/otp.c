/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <mt-plat/mtk_otp.h>
#include <mt-plat/mtk_secure_api.h>
#include "otp_pmic_config.h"

static DEFINE_MUTEX(g_is_otp_blowing);

/**************************************************************************
 *FUNCTION DECLARATION
 **************************************************************************/
static u32 otp_reinit_efuse(void);

/**************************************************************************
 *EXTERN FUNCTION
 **************************************************************************/
u32 otp_blow_start(void)
{
	u32 ret_err = 0;
	u32 ret_vcorefs = 0;

	if (!mutex_trylock(&g_is_otp_blowing))
		return ERR_MUTEX_LOCKED_NEED_RETRY;

	/* Set High-VCORE */
	ret_vcorefs = otp_pmic_high_vcore_set();
	if (ret_vcorefs) {
		ret_err = ret_vcorefs;
		goto _fail;
	}

	/* Enable VEFUSE(fsource) */
	if (otp_pmic_fsource_set()) {
		ret_err = ERR_PMIC_CONFIG_FSOURCE;
		goto _fail;
	}

	return STATUS_DONE;

_fail:
	if (otp_pmic_is_fsource_enabled())
		otp_pmic_fsource_release();

	/* Only release the High-VCORE DVFS without
	 * even setting will not cause side-effect
	 */
	otp_pmic_high_vcore_release();

	/* Do not lock the mutex if any error */
	mutex_unlock(&g_is_otp_blowing);

	pr_err("[OTP] blow start failed : %d\n", ret_err);

	return ret_err;
}
EXPORT_SYMBOL(otp_blow_start);

u32 otp_is_blow_locked(void)
{
	u32 is_locked = 0;

	is_locked = mutex_is_locked(&g_is_otp_blowing);

	if (is_locked)
		return ERR_MUTEX_LOCKED_NEED_RETRY;

	return STATUS_DONE;
}
EXPORT_SYMBOL(otp_is_blow_locked);

u32 otp_blow_end(void)
{
	u32 ret_smc_err = 0;

	if (!otp_is_blow_locked())
		return ERR_SHOULD_CALL_BLOW_START_FIRST;

	/* Close VEFUSE(fsource) */
	if (otp_pmic_fsource_release()) {
		pr_err("[OTP] release fsource error\n");
		return ERR_PMIC_CONFIG_FSOURCE;
	}

	/* SMC to ATF (re-init EFUSE under High-Vcore and fsource closed) */
	ret_smc_err = otp_reinit_efuse();
	if (ret_smc_err) {
		pr_err("[OTP] reinit error\n");
		return ret_smc_err;
	}

	/* Release High-VCORE */
	if (otp_pmic_high_vcore_release()) {
		pr_err("[OTP] release VCORE error\n");
		return ERR_PMIC_RELEASE_VCORE_DVFS_HPM;
	}

	mutex_unlock(&g_is_otp_blowing);

	return STATUS_DONE;
}
EXPORT_SYMBOL(otp_blow_end);

u32 otp_ccci_handler(u32 ccci_cmd)
{
	u32 ret = 0;

	switch (ccci_cmd) {
	case CCCI_CMD_EFUSE_BLOW_START:
		ret = otp_blow_start();
		break;
	case CCCI_CMD_EFUSE_BLOW_END:
		ret = otp_blow_end();
		break;
	case CCCI_CMD_EFUSE_IS_BLOW_LOCKED:
		ret = otp_is_blow_locked();
		break;
	default:
		ret = ERR_UNSUPPORTED_CCCI_CMD;
		break;
	}

	return ret;
}
EXPORT_SYMBOL(otp_ccci_handler);

/**************************************************************************
 *STATIC FUNCTION : ATF wrapper
 **************************************************************************/
static u32 otp_reinit_efuse(void)
{
	int ret_smc = 0;
	struct arm_smccc_res res;

	/* SMC to re-init EFUSE */
	arm_smccc_smc(MTK_SIP_EFUSE_CONTROL, 0, 0, 0, 0, 0, 0, 0, &res);
	ret_smc = res.a0;
	if (ret_smc) {
		pr_err("[OTP] SMC call fail : %d\n", ret_smc);
		if (ret_smc == OTP_SMC_UNK)
			return ERR_EFUSE_RELOAD_SMC_NOT_FOUND;
		else
			return ERR_EFUSE_RELOAD_TIMEOUT;
	}

	return STATUS_DONE;
}

/**************************************************************************
 *STATIC FUNCTION : module init
 **************************************************************************/

static int __init otp_init(void)
{
	otp_pmic_high_vcore_init();

	pr_info("[OTP] init done\n");

	return 0;
}

module_init(otp_init);



MODULE_LICENSE("GPL");
