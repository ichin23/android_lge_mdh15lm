/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

#include <touch_hwif.h>
#include <touch_core.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>
#include <touch_hwif.h>

/*
 *  Include to Local Header File
 */
#include "touch_nt36xxx.h"
#include "touch_nt36xxx_mp_ctrlram.h"

#if NVT_TOUCH_MP

#define NVT_DEBUG 0

#define LOG_BUF_SIZE	(PAGE_SIZE * 2)
#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define MP_MODE_CC 0x41
#define ENG_MODE_ENABLE 0x61
#define ENG_MODE_DISABLE 0x62
#define FREQ_HOP_ENABLE 0x65
#define FREQ_HOP_DISABLE 0x66

#define SELF_TEST_FILE_PATH "/data/touch/touch_self_test.txt"
#define NORMAL_SELF_TEST_FILE_PATH "/sdcard/touch_self_test.txt"

#define MAX_LOG_FILE_SIZE 	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT 	4

#define NVT_ABS(x) (x >= 0 ? x : (-x))

#define nvt_mp_seq_printf(m, fmt, args...) do {	\
	seq_printf(m, fmt, ##args);	\
	if (!nvt_mp_test_result_printed)	\
		printk(fmt, ##args);	\
} while (0)

typedef enum {
	FW_MUTUAL = 0,
	FW_CC,
	FW_CC_I,
	FW_CC_Q,
	NOISE_DIFF_ABS,
	SHORT,
	SHORT_DIFF,
	SHORT_BASE,
	OPEN
} CHANNEL_TEST_ITEM;

static struct touch_core_data *gts;

static uint8_t *RecordResult_Short = NULL;
static uint8_t *RecordResult_Open = NULL;
static uint8_t *RecordResult_FWMutual = NULL;
static uint8_t *RecordResult_FW_CC = NULL;
static uint8_t *RecordResult_FW_DiffMax = NULL;
static uint8_t *RecordResult_FW_DiffMin = NULL;

static int32_t TestResult_Short = 0;
static int32_t TestResult_Open = 0;
static int32_t TestResult_FW_Rawdata = 0;
static int32_t TestResult_FWMutual = 0;
static int32_t TestResult_FW_CC = 0;
static int32_t TestResult_Noise = 0;
static int32_t TestResult_FW_DiffMax = 0;
static int32_t TestResult_FW_DiffMin = 0;
static int32_t TestResult_Channel_Status = 0;

static int32_t *RawData_Short = NULL;
static int32_t *RawData_Open = NULL;
static int32_t *RawData_Diff = NULL;
static int32_t *RawData_Diff_Min = NULL;
static int32_t *RawData_Diff_Max = NULL;
static int32_t *RawData_FWMutual = NULL;
static int32_t *RawData_FW_CC = NULL;
#if TOUCH_KEY_NUM > 0
static int32_t *xdata_btn_tmp = NULL;
#endif /* #if TOUCH_KEY_NUM > 0 */

static struct proc_dir_entry *NVT_proc_selftest_entry = NULL;
static int8_t nvt_mp_test_result_printed = 0;
static uint8_t fw_ver = 0;

extern void nvt_change_mode(uint8_t mode);
extern uint8_t nvt_get_fw_pipe(void);
extern void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr);
extern void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num);
int32_t nvt_mp_parse_dt(struct device_node *root, const char *node_compatible);


/*******************************************************
Description:
	Novatek touchscreen allocate buffer for mp selftest.

return:
	Executive outcomes. 0---succeed. -12---Out of memory
*******************************************************/
static int nvt_mp_buffer_init(void)
{
	size_t RecordResult_BufSize = IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE;
	size_t RawData_BufSize = (IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE) * sizeof(int32_t);

	RecordResult_Short = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_Short) {
		TOUCH_E("kzalloc for RecordResult_Short failed!\n");
		return -ENOMEM;
	}

	RecordResult_Open = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_Open) {
		TOUCH_E("kzalloc for RecordResult_Open failed!\n");
		return -ENOMEM;
	}

	RecordResult_FWMutual = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FWMutual) {
		TOUCH_E("kzalloc for RecordResult_FWMutual failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_CC = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_CC) {
		TOUCH_E("kzalloc for RecordResult_FW_CC failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_DiffMax = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_DiffMax) {
		TOUCH_E("kzalloc for RecordResult_FW_DiffMax failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_DiffMin = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_DiffMin) {
		TOUCH_E("kzalloc for RecordResult_FW_DiffMin failed!\n");
		return -ENOMEM;
	}

	RawData_Short = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Short) {
		TOUCH_E("kzalloc for RawData_Short failed!\n");
		return -ENOMEM;
	}

	RawData_Open = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Open) {
		TOUCH_E("kzalloc for RawData_Open failed!\n");
		return -ENOMEM;
	}

	RawData_Diff = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff) {
		TOUCH_E("kzalloc for RawData_Diff failed!\n");
		return -ENOMEM;
	}

	RawData_Diff_Min = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff_Min) {
		TOUCH_E("kzalloc for RawData_Diff_Min failed!\n");
		return -ENOMEM;
	}

	RawData_Diff_Max = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff_Max) {
		TOUCH_E("kzalloc for RawData_Diff_Max failed!\n");
		return -ENOMEM;
	}

	RawData_FWMutual = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FWMutual) {
		TOUCH_E("kzalloc for RawData_FWMutual failed!\n");
		return -ENOMEM;
	}

	RawData_FW_CC = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FW_CC) {
		TOUCH_E("kzalloc for RawData_FW_CC failed!\n");
		return -ENOMEM;
	}

#if TOUCH_KEY_NUM > 0
	xdata_btn_tmp = (int32_t *)kzalloc(IC_KEY_CFG_SIZE * sizeof(int32_t), GFP_KERNEL);
	if (!xdata_btn_tmp) {
		TOUCH_E("kzalloc for xdata_btn_tmp failed!\n");
		return -ENOMEM;
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	return 0;
}

static void nvt_print_data_log_in_one_line(int32_t *data, int32_t data_num)
{
	char *tmp_log = NULL;
	int32_t i = 0;

	tmp_log = (char *)kzalloc(data_num * 7 + 1, GFP_KERNEL);
	if (!tmp_log) {
		TOUCH_E("kzalloc for tmp_log failed!\n ");
		return;
	}

	for (i = 0; i < data_num; i++) {
		sprintf(tmp_log + i * 7, "%5d, ", data[i]);
	}
	tmp_log[data_num * 7] = '\0';
	printk("%s", tmp_log);
	if (tmp_log) {
		kfree(tmp_log);
		tmp_log = NULL;
	}

	return;
}

static void nvt_print_result_log_in_one_line(uint8_t *result, int32_t result_num)
{
	char *tmp_log = NULL;
	int32_t i = 0;

	tmp_log = (char *)kzalloc(result_num * 6 + 1, GFP_KERNEL);
	if (!tmp_log) {
		TOUCH_E("kzalloc for tmp_log failed!\n ");
		return;
	}

	for (i = 0; i < result_num; i++) {
		sprintf(tmp_log + i * 6, "0x%02X, ", result[i]);
	}
	tmp_log[result_num * 6] = '\0';
	printk("%s", tmp_log);
	if (tmp_log) {
		kfree(tmp_log);
		tmp_log = NULL;
	}

	return;
}

/*******************************************************
Description:
	Novatek touchscreen self-test criteria print function.

return:
	n.a.
*******************************************************/
static void nvt_print_lmt_array(int32_t *array, int32_t x_ch, int32_t y_ch)
{
	int32_t i = 0;
	int32_t j = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	for (j = 0; j < y_ch; j++) {
		for(i = 0; i < x_ch; i++) {
			printk("%5d, ", array[j * x_ch + i]);
		}
		printk("\n");
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		printk("%5d, ", array[y_ch * x_ch + k]);
	}
	printk("\n");
#endif /* #if TOUCH_KEY_NUM > 0 */
}

static void nvt_print_criteria(void)
{
	TOUCH_I("++\n");

	//---PS_Config_Lmt_Short_Rawdata---
	printk("PS_Config_Lmt_Short_Rawdata_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Short_Rawdata_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_Short_Rawdata_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Short_Rawdata_N, X_Channel, Y_Channel);

	//---PS_Config_Lmt_Open_Rawdata---
	printk("PS_Config_Lmt_Open_Rawdata_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Open_Rawdata_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_Open_Rawdata_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Open_Rawdata_N, X_Channel, Y_Channel);

	//---PS_Config_Lmt_FW_Rawdata---
	printk("PS_Config_Lmt_FW_Rawdata_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Rawdata_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_FW_Rawdata_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Rawdata_N, X_Channel, Y_Channel);

	//---PS_Config_Lmt_FW_CC---
	printk("PS_Config_Lmt_FW_CC_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_CC_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_FW_CC_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_CC_N, X_Channel, Y_Channel);

	//---PS_Config_Lmt_FW_Diff---
	printk("PS_Config_Lmt_FW_Diff_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Diff_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_FW_Diff_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Diff_N, X_Channel, Y_Channel);

	TOUCH_I("--\n");
}

static int32_t nvt_polling_cmd_clear(void)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		TOUCH_E("polling cmd clear failed, buf[1]=0x%02X\n", buf[1]);
		return -1;
	} else {
		return 0;
	}
}

static int32_t nvt_polling_hand_shake_status(void)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 70;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, 2);

		if ((buf[1] == 0xA0) || (buf[1] == 0xA1))
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		TOUCH_E("polling hand shake status failed, buf[1]=0x%02X\n", buf[1]);

		// Read back 5 bytes from offset EVENT_MAP_HOST_CMD for debug check
		nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, 6);
		TOUCH_E("Read back 5 bytes from offset EVENT_MAP_HOST_CMD: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);

		return -1;
	} else {
		return 0;
	}
}

static int8_t nvt_switch_FreqHopEnDis(uint8_t FreqHopEnDis)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	TOUCH_I("++\n");

	for (retry = 0; retry < 20; retry++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch FreqHopEnDis---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = FreqHopEnDis;
		CTP_I2C_WRITE(gts->dev, I2C_FW_Address, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		TOUCH_E("switch FreqHopEnDis 0x%02X failed, buf[1]=0x%02X\n", FreqHopEnDis, buf[1]);
		ret = -1;
	}

	TOUCH_I("--\n");

	return ret;
}

static int32_t nvt_read_baseline(int32_t *xdata)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("++\n");

	nvt_read_mdata(d->mmap->BASELINE_ADDR, d->mmap->BASELINE_BTN_ADDR);

	nvt_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("--\n");

	return 0;
}

static int32_t nvt_read_CC(int32_t *xdata)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("++\n");

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(d->mmap->DIFF_PIPE1_ADDR, d->mmap->DIFF_BTN_PIPE1_ADDR);
	else
		nvt_read_mdata(d->mmap->DIFF_PIPE0_ADDR, d->mmap->DIFF_BTN_PIPE0_ADDR);

	nvt_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("--\n");

	return 0;
}

static void nvt_enable_noise_collect(int32_t frame_num)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable noise collect---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x47;
	buf[2] = 0xAA;
	buf[3] = frame_num;
	buf[4] = 0x00;
	CTP_I2C_WRITE(gts->dev, I2C_FW_Address, buf, 5);
}

static int32_t nvt_read_fw_noise(int32_t *xdata)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
	int32_t frame_num = 0;
	uint32_t rawdata_diff_min_offset = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("++\n");

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		return -EAGAIN;
	}

	frame_num = PS_Config_Diff_Test_Frame / 10;
	if (frame_num <= 0)
		frame_num = 1;
	printk("%s: frame_num=%d\n", __func__, frame_num);
	nvt_enable_noise_collect(frame_num);
	// need wait PS_Config_Diff_Test_Frame * 8.3ms
	msleep(frame_num * 83);

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

	if (nvt_get_fw_info(gts->dev)) {
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(d->mmap->DIFF_PIPE0_ADDR, d->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(d->mmap->DIFF_PIPE1_ADDR, d->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			RawData_Diff_Max[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
			RawData_Diff_Min[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		RawData_Diff_Max[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
		RawData_Diff_Min[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

#if TOUCH_KEY_NUM > 0
	rawdata_diff_min_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2 + Key_Channel * 7 + 2;
#else
	rawdata_diff_min_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("--\n");

	return 0;
}

static void nvt_enable_open_test(void)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable open test---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x45;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	CTP_I2C_WRITE(gts->dev, I2C_FW_Address, buf, 5);
}

static void nvt_enable_short_test(void)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(gts->dev, I2C_FW_Address, d->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable short test---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x43;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	CTP_I2C_WRITE(gts->dev, I2C_FW_Address, buf, 5);
}

static int32_t nvt_read_fw_open(int32_t *xdata)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint32_t raw_pipe_addr = 0;
	uint8_t *rawdata_buf = NULL;
	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t buf[128] = {0};
#if TOUCH_KEY_NUM > 0
	uint32_t raw_btn_pipe_addr = 0;
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("++\n");

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		return -EAGAIN;
	}

	nvt_enable_open_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

#if TOUCH_KEY_NUM > 0
	rawdata_buf = (uint8_t *)kzalloc((IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE) * 2, GFP_KERNEL);
#else
	rawdata_buf = (uint8_t *)kzalloc(IC_X_CFG_SIZE * IC_Y_CFG_SIZE * 2, GFP_KERNEL);
#endif /* #if TOUCH_KEY_NUM > 0 */
	if (!rawdata_buf) {
		TOUCH_E("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (nvt_get_fw_pipe() == 0)
		raw_pipe_addr = d->mmap->RAW_PIPE0_ADDR;
	else
		raw_pipe_addr = d->mmap->RAW_PIPE1_ADDR;

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		//---change xdata index---
		nvt_set_page(gts->dev, I2C_FW_Address, raw_pipe_addr + y * IC_X_CFG_SIZE * 2);
		buf[0] = (uint8_t)((raw_pipe_addr + y * IC_X_CFG_SIZE * 2) & 0xFF);
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, IC_X_CFG_SIZE * 2 + 1);
		memcpy(rawdata_buf + y * IC_X_CFG_SIZE * 2, buf + 1, IC_X_CFG_SIZE * 2);
	}
#if TOUCH_KEY_NUM > 0
	if (nvt_get_fw_pipe() == 0)
		raw_btn_pipe_addr = d->mmap->RAW_BTN_PIPE0_ADDR;
	else
		raw_btn_pipe_addr = d->mmap->RAW_BTN_PIPE1_ADDR;

	//---change xdata index---
	nvt_set_page(I2C_FW_Address, raw_btn_pipe_addr);
	buf[0] = (uint8_t)(raw_btn_pipe_addr & 0xFF);
	CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, IC_KEY_CFG_SIZE * 2 + 1);
	memcpy(rawdata_buf + IC_Y_CFG_SIZE * IC_X_CFG_SIZE * 2, buf + 1, IC_KEY_CFG_SIZE * 2);
#endif /* #if TOUCH_KEY_NUM > 0 */

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		for (x = 0; x < IC_X_CFG_SIZE; x++) {
			if ((AIN_Y[y] != 0xFF) && (AIN_X[x] != 0xFF)) {
				xdata[AIN_Y[y] * X_Channel + AIN_X[x]] = (int16_t)((rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2] + 256 * rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2 + 1]));
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < IC_KEY_CFG_SIZE; k++) {
		if (AIN_KEY[k] != 0xFF)
			xdata[Y_Channel * X_Channel + AIN_KEY[k]] = (int16_t)(rawdata_buf[(IC_Y_CFG_SIZE * IC_X_CFG_SIZE + k) * 2] + 256 * rawdata_buf[(IC_Y_CFG_SIZE * IC_X_CFG_SIZE + k) * 2 + 1]);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

	TOUCH_I("--\n");

	return 0;
}

static int32_t nvt_read_fw_short(int32_t *xdata)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	uint32_t raw_pipe_addr = 0;
	uint8_t *rawdata_buf = NULL;
	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t buf[128] = {0};
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	uint32_t raw_btn_pipe_addr = 0;
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	TOUCH_I("++\n");

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		return -EAGAIN;
	}

	nvt_enable_short_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

#if TOUCH_KEY_NUM > 0
    rawdata_buf = (uint8_t *)kzalloc((X_Channel * Y_Channel + Key_Channel) * 2, GFP_KERNEL);
#else
    rawdata_buf = (uint8_t *)kzalloc(X_Channel * Y_Channel * 2, GFP_KERNEL);
#endif /* #if TOUCH_KEY_NUM > 0 */
	if (!rawdata_buf) {
		TOUCH_E("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (nvt_get_fw_pipe() == 0)
		raw_pipe_addr = d->mmap->RAW_PIPE0_ADDR;
	else
		raw_pipe_addr = d->mmap->RAW_PIPE1_ADDR;

	for (y = 0; y < Y_Channel; y++) {
		//---change xdata index---
		nvt_set_page(gts->dev, I2C_FW_Address, raw_pipe_addr + y * X_Channel * 2);
		buf[0] = (uint8_t)((raw_pipe_addr + y * X_Channel * 2) & 0xFF);
		CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, X_Channel * 2 + 1);
		memcpy(rawdata_buf + y * X_Channel * 2, buf + 1, X_Channel * 2);
	}
#if TOUCH_KEY_NUM > 0
	if (nvt_get_fw_pipe() == 0)
		raw_btn_pipe_addr = d->mmap->RAW_BTN_PIPE0_ADDR;
	else
		raw_btn_pipe_addr = d->mmap->RAW_BTN_PIPE1_ADDR;

    //---change xdata index---
	nvt_set_page(gts->dev, I2C_FW_Address, raw_btn_pipe_addr);
	buf[0] = (uint8_t)(raw_btn_pipe_addr & 0xFF);
	CTP_I2C_READ(gts->dev, I2C_FW_Address, buf, Key_Channel * 2 + 1);
	memcpy(rawdata_buf + Y_Channel * X_Channel * 2, buf + 1, Key_Channel * 2);
#endif /* #if TOUCH_KEY_NUM > 0 */

	for (y = 0; y < Y_Channel; y++) {
		for (x = 0; x < X_Channel; x++) {
			iArrayIndex = y * X_Channel + x;
			xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

	TOUCH_I("--\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen raw data test for each single point function.

return:
	Executive outcomes. 0---passed. negative---failed.
*******************************************************/
static int32_t RawDataTest_SinglePoint_Sub(int32_t rawdata[], uint8_t RecordResult[], uint8_t x_ch, uint8_t y_ch, int32_t Rawdata_Limit_Postive[], int32_t Rawdata_Limit_Negative[])
{
	int32_t i = 0;
	int32_t j = 0;
#if TOUCH_KEY_NUM > 0
    int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */
	int32_t iArrayIndex = 0;
	bool isPass = true;

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;

			RecordResult[iArrayIndex] = 0x00; // default value for PASS

			if(rawdata[iArrayIndex] > Rawdata_Limit_Postive[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x01;

			if(rawdata[iArrayIndex] < Rawdata_Limit_Negative[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x02;
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = y_ch * x_ch + k;

		RecordResult[iArrayIndex] = 0x00; // default value for PASS

		if(rawdata[iArrayIndex] > Rawdata_Limit_Postive[iArrayIndex])
			RecordResult[iArrayIndex] |= 0x01;

		if(rawdata[iArrayIndex] < Rawdata_Limit_Negative[iArrayIndex])
			RecordResult[iArrayIndex] |= 0x02;
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	//---Check RecordResult---
	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			if (RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = y_ch * x_ch + k;
		if (RecordResult[iArrayIndex] != 0) {
			isPass = false;
			break;
		}
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	if (isPass == false) {
		return -1; // FAIL
	} else {
		return 0; // PASS
	}
}

/*******************************************************
Description:
	Novatek touchscreen print self-test result function.

return:
	n.a.
*******************************************************/
void print_selftest_result(struct seq_file *m, int32_t TestResult, uint8_t RecordResult[], int32_t rawdata[], uint8_t x_len, uint8_t y_len)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	switch (TestResult) {
		case 0:
			nvt_mp_seq_printf(m, " PASS!\n");
			break;

		case 1:
			nvt_mp_seq_printf(m, " ERROR! Read Data FAIL!\n");
			break;

		case -1:
			nvt_mp_seq_printf(m, " FAIL!\n");
			nvt_mp_seq_printf(m, "RecordResult:\n");
			for (i = 0; i < y_len; i++) {
				for (j = 0; j < x_len; j++) {
					iArrayIndex = i * x_len + j;
					seq_printf(m, "0x%02X, ", RecordResult[iArrayIndex]);
				}
				if (!nvt_mp_test_result_printed)
					nvt_print_result_log_in_one_line(RecordResult + i * x_len, x_len);
				nvt_mp_seq_printf(m, "\n");
			}
#if TOUCH_KEY_NUM > 0
			for (k = 0; k < Key_Channel; k++) {
				iArrayIndex = y_len * x_len + k;
				seq_printf(m, "0x%02X, ", RecordResult[iArrayIndex]);
			}
			if (!nvt_mp_test_result_printed)
				nvt_print_result_log_in_one_line(RecordResult + y_len * x_len, Key_Channel);
			nvt_mp_seq_printf(m, "\n");
#endif /* #if TOUCH_KEY_NUM > 0 */
			nvt_mp_seq_printf(m, "ReadData:\n");
			for (i = 0; i < y_len; i++) {
				for (j = 0; j < x_len; j++) {
					iArrayIndex = i * x_len + j;
					seq_printf(m, "%5d, ", rawdata[iArrayIndex]);
				}
				if (!nvt_mp_test_result_printed)
					nvt_print_data_log_in_one_line(rawdata + i * x_len, x_len);
				nvt_mp_seq_printf(m, "\n");
			}
#if TOUCH_KEY_NUM > 0
			for (k = 0; k < Key_Channel; k++) {
				iArrayIndex = y_len * x_len + k;
				seq_printf(m, "%5d, ", rawdata[iArrayIndex]);
			}
			if (!nvt_mp_test_result_printed)
				nvt_print_data_log_in_one_line(rawdata + y_len * x_len, Key_Channel);
			nvt_mp_seq_printf(m, "\n");
#endif /* #if TOUCH_KEY_NUM > 0 */
			break;
	}
	nvt_mp_seq_printf(m, "\n");
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show_selftest(struct seq_file *m, void *v)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);

	TOUCH_I("++\n");

	nvt_mp_seq_printf(m, "FW Version: %d\n\n", d->fw_ver);

	nvt_mp_seq_printf(m, "Short Test");
	if ((TestResult_Short == 0) || (TestResult_Short == 1)) {
		print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
	} else { // TestResult_Short is -1
		print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
	}

	nvt_mp_seq_printf(m, "Open Test");
	print_selftest_result(m, TestResult_Open, RecordResult_Open, RawData_Open, X_Channel, Y_Channel);

	nvt_mp_seq_printf(m, "FW Rawdata Test");
	if ((TestResult_FW_Rawdata == 0) || (TestResult_FW_Rawdata == 1)) {
		 print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
	} else { // TestResult_FW_Rawdata is -1
		nvt_mp_seq_printf(m, " FAIL!\n");
		if (TestResult_FWMutual == -1) {
			nvt_mp_seq_printf(m, "FW Mutual");
			print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
		}
		if (TestResult_FW_CC == -1) {
			nvt_mp_seq_printf(m, "FW CC");
			print_selftest_result(m, TestResult_FW_CC, RecordResult_FW_CC, RawData_FW_CC, X_Channel, Y_Channel);
		}
	}

	nvt_mp_seq_printf(m, "Noise Test");
	if ((TestResult_Noise == 0) || (TestResult_Noise == 1)) {
		print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
	} else { // TestResult_Noise is -1
		nvt_mp_seq_printf(m, " FAIL!\n");

		if (TestResult_FW_DiffMax == -1) {
			nvt_mp_seq_printf(m, "FW Diff Max");
			print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
		}
		if (TestResult_FW_DiffMin == -1) {
			nvt_mp_seq_printf(m, "FW Diff Min");
			print_selftest_result(m, TestResult_FW_DiffMin, RecordResult_FW_DiffMin, RawData_Diff_Min, X_Channel, Y_Channel);
		}
	}

	nvt_mp_test_result_printed = 1;

	TOUCH_I("--\n");

    return 0;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_selftest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show_selftest
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_selftest open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_selftest_open(struct inode *inode, struct file *file)
{
	struct nt36xxx_data *d = to_nt36xxx_data(gts->dev);
	struct device_node *np = gts->dev->of_node;
	unsigned char mpcriteria[32] = {0};	//novatek-mp-criteria-default
	TestResult_Short = 0;
	TestResult_Open = 0;
	TestResult_FW_Rawdata = 0;
	TestResult_FWMutual = 0;
	TestResult_FW_CC = 0;
	TestResult_Noise = 0;
	TestResult_FW_DiffMax = 0;
	TestResult_FW_DiffMin = 0;

	TOUCH_I("++\n");

	if (mutex_lock_interruptible(&d->lock)) {
		return -ERESTARTSYS;
	}

	if (nvt_get_fw_info(gts->dev)) {
		mutex_unlock(&d->lock);
		TOUCH_E("get fw info failed!\n");
		return -EAGAIN;
	}

	fw_ver = d->fw_ver;

	/* Parsing criteria from dts */
	if(of_property_read_bool(np, "novatek,mp-support-dt")) {
		/*
		 * Parsing Criteria by Novatek PID
		 * The string rule is "novatek-mp-criteria-<nvt_pid>"
		 * nvt_pid is 2 bytes (show hex).
		 *
		 * Ex. nvt_pid = 500A
		 *     mpcriteria = "novatek-mp-criteria-500A"
		 */
		snprintf(mpcriteria, PAGE_SIZE, "novatek-mp-criteria-%04X", d->nvt_pid);

		if (nvt_mp_parse_dt(np, mpcriteria)) {
			mutex_unlock(&d->lock);
			TOUCH_E("mp parse device tree failed!\n");
			return -EINVAL;
		}
	} else {
		TOUCH_I("Not found novatek,mp-support-dt, use default setting\n");
		//---Print Test Criteria---
		nvt_print_criteria();
	}

	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		mutex_unlock(&d->lock);
		TOUCH_E("switch frequency hopping disable failed!\n");
		return -EAGAIN;
	}

	if (nvt_check_fw_reset_state(gts->dev, RESET_STATE_NORMAL_RUN)) {
		mutex_unlock(&d->lock);
		TOUCH_E("check fw reset state failed!\n");
		return -EAGAIN;
	}

	msleep(100);

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		mutex_unlock(&d->lock);
		TOUCH_E("clear fw status failed!\n");
		return -EAGAIN;
	}

	nvt_change_mode(MP_MODE_CC);

	if (nvt_check_fw_status(gts->dev)) {
		mutex_unlock(&d->lock);
		TOUCH_E("check fw status failed!\n");
		return -EAGAIN;
	}

	//---FW Rawdata Test---
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}

	//---FW CC Test---
	if (nvt_read_CC(RawData_FW_CC) != 0) {
		TestResult_FW_CC = 1;
	} else {
		TestResult_FW_CC = RawDataTest_SinglePoint_Sub(RawData_FW_CC, RecordResult_FW_CC, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_CC_P, PS_Config_Lmt_FW_CC_N);
	}

	if ((TestResult_FWMutual == 1) || (TestResult_FW_CC == 1)) {
		TestResult_FW_Rawdata = 1;
	} else {
		if ((TestResult_FWMutual == -1) || (TestResult_FW_CC == -1))
			TestResult_FW_Rawdata = -1;
		else
			TestResult_FW_Rawdata = 0;
	}

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Noise Test---
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	// 1: ERROR
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff_Max, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);

		TestResult_FW_DiffMin = RawDataTest_SinglePoint_Sub(RawData_Diff_Min, RecordResult_FW_DiffMin, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);

		if ((TestResult_FW_DiffMax == -1) || (TestResult_FW_DiffMin == -1))
			TestResult_Noise = -1;
		else
			TestResult_Noise = 0;
	}

	//--Short Test---
	if (nvt_read_fw_short(RawData_Short) != 0) {
		TestResult_Short = 1; // 1:ERROR
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		TestResult_Short = RawDataTest_SinglePoint_Sub(RawData_Short, RecordResult_Short, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Rawdata_P, PS_Config_Lmt_Short_Rawdata_N);
	}

	//---Open Test---
	if (nvt_read_fw_open(RawData_Open) != 0) {
		TestResult_Open = 1;    // 1:ERROR
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		TestResult_Open = RawDataTest_SinglePoint_Sub(RawData_Open, RecordResult_Open, X_Channel, Y_Channel,
											PS_Config_Lmt_Open_Rawdata_P, PS_Config_Lmt_Open_Rawdata_N);
	}

	//---Reset IC---
	nvt_bootloader_reset(gts->dev);

	mutex_unlock(&d->lock);

	TOUCH_I("--\n");

	nvt_mp_test_result_printed = 0;

	return seq_open(file, &nvt_selftest_seq_ops);
}

static const struct file_operations nvt_selftest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_selftest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#ifdef CONFIG_OF
/*******************************************************
Description:
	Novatek touchscreen parse AIN setting for array type.

return:
	n.a.
*******************************************************/
int32_t nvt_mp_parse_ain(struct device_node *np, const char *name, uint8_t *array, int32_t size)
{
	struct property *data;
	int32_t len, ret;
	int32_t tmp[40];
	int32_t i;

	data = of_find_property(np, name, &len);
	len /= sizeof(u32);
	if ((!data) || (!len) || (len != size)) {
		TOUCH_E("error find %s. len=%d\n", name, len);
		return -1;
	} else {
		TOUCH_I("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, tmp, len);
		if (ret) {
			TOUCH_E("error reading %s. ret=%d\n", name, ret);
			return -1;
		}

		for (i = 0; i < len; i++)
			array[i] = tmp[i];

#if NVT_DEBUG
		printk("[NVT-ts] %s = ", name);
		nvt_print_result_log_in_one_line(array, len);
		printk("\n");
#endif
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for u32 type.

return:
	n.a.
*******************************************************/
int32_t nvt_mp_parse_u32(struct device_node *np, const char *name, int32_t *para)
{
	int32_t ret;

	ret = of_property_read_u32(np, name, para);
	if (ret) {
		TOUCH_E("error reading %s. ret=%d\n", name, ret);
		return -1;
	} else {
#if NVT_DEBUG
		TOUCH_I("%s=%d\n", name, *para);
#endif
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for array type.

return:
	n.a.
*******************************************************/
int32_t nvt_mp_parse_array(struct device_node *np, const char *name, int32_t *array,
		int32_t size)
{
	struct property *data;
	int32_t len, ret;
#if NVT_DEBUG
	int32_t j = 0;
#endif

	data = of_find_property(np, name, &len);
	len /= sizeof(u32);
	if ((!data) || (!len) || (len < size)) {
		TOUCH_E("error find %s. len=%d\n", name, len);
		return -1;
	} else {
		TOUCH_I("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, array, len);
		if (ret) {
			TOUCH_E("error reading %s. ret=%d\n", name, ret);
			return -1;
		}

#if NVT_DEBUG
		TOUCH_I("%s =\n", name);
		for (j = 0; j < Y_Channel; j++) {
			nvt_print_data_log_in_one_line(array + j * X_Channel, X_Channel);
			printk("\n");
		}
#if TOUCH_KEY_NUM > 0
		nvt_print_data_log_in_one_line(array + Y_Channel * X_Channel, Key_Channel);
		printk("\n");
#endif
#endif
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree mp function.

return:
	n.a.
*******************************************************/
int32_t nvt_mp_parse_dt(struct device_node *root, const char *node_compatible)
{
	struct device_node *np = root;
	struct device_node *child = NULL;

	TOUCH_I("Parse mp criteria for node %s\n", node_compatible);

	/* find each MP sub-nodes */
	for_each_child_of_node(root, child) {
		/* find the specified node */
		if (of_device_is_compatible(child, node_compatible)) {
			TOUCH_I("found child node %s\n", node_compatible);
			np = child;
			break;
		}
	}
	if (child == NULL) {
		TOUCH_E("Not found compatible node %s!\n", node_compatible);
		return -1;
	}

	/* MP Config*/
	if (nvt_mp_parse_u32(np, "IC_X_CFG_SIZE", &IC_X_CFG_SIZE))
		return -1;

	if (nvt_mp_parse_u32(np, "IC_Y_CFG_SIZE", &IC_Y_CFG_SIZE))
		return -1;

#if TOUCH_KEY_NUM > 0
	if (nvt_mp_parse_u32(np, "IC_KEY_CFG_SIZE", &IC_KEY_CFG_SIZE))
		return -1;
#endif

	if (nvt_mp_parse_u32(np, "X_Channel", &X_Channel))
		return -1;

	if (nvt_mp_parse_u32(np, "Y_Channel", &Y_Channel))
		return -1;

	if (nvt_mp_parse_ain(np, "AIN_X", AIN_X, IC_X_CFG_SIZE))
		return -1;

	if (nvt_mp_parse_ain(np, "AIN_Y", AIN_Y, IC_Y_CFG_SIZE))
		return -1;

#if TOUCH_KEY_NUM > 0
	if (nvt_mp_parse_ain(np, "AIN_KEY", AIN_KEY, IC_KEY_CFG_SIZE))
		return -1;
#endif

	/* MP Criteria */
	if (nvt_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_P", PS_Config_Lmt_Short_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_N", PS_Config_Lmt_Short_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_P", PS_Config_Lmt_Open_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_N", PS_Config_Lmt_Open_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_P", PS_Config_Lmt_FW_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_N", PS_Config_Lmt_FW_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_CC_P", PS_Config_Lmt_FW_CC_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_CC_N", PS_Config_Lmt_FW_CC_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_P", PS_Config_Lmt_FW_Diff_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_N", PS_Config_Lmt_FW_Diff_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (nvt_mp_parse_u32(np, "PS_Config_Diff_Test_Frame", &PS_Config_Diff_Test_Frame))
		return -1;

	TOUCH_I("Parse mp criteria done!\n");

	return 0;
}
#endif /* #ifdef CONFIG_OF */

/*******************************************************
Description:
	Novatek touchscreen MP function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_mp_proc_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	gts = ts;

	NVT_proc_selftest_entry = proc_create("nvt_selftest", 0444, NULL, &nvt_selftest_fops);
	if (NVT_proc_selftest_entry == NULL) {
		TOUCH_E("create /proc/nvt_selftest Failed!\n");
		return -1;
	} else {
		if(nvt_mp_buffer_init()) {
			TOUCH_E("Allocate mp memory failed\n");
			return -1;
		}
		else {
			TOUCH_I("create /proc/nvt_selftest Succeeded!\n");
		}
		return 0;
	}
}

void nt36xxx_write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	TOUCH_TRACE();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu\n",
			my_date.tm_mon + 1,my_date.tm_mday,
			my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	TOUCH_I("write open %s, fd : %d\n", (fd >= 0)? "success":"fail",fd);
	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);

}

static void nt36xxx_check_log_file_size(struct device *dev, char *fname)
{
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;

	TOUCH_TRACE();

	set_fs(KERNEL_DS);
	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto log_file_size_check_error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto log_file_size_check_error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto log_file_size_check_error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto log_file_size_check_error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

log_file_size_check_error:
	set_fs(old_fs);
	return;
}

static void nt36xxx_print_selftest_result(struct device *dev, CHANNEL_TEST_ITEM item, int32_t TestResult, uint8_t RecordResult[], int32_t rawdata[], uint8_t x_len, uint8_t y_len, int32_t lpwg)
{
	struct nt36xxx_data *d = to_nt36xxx_data(dev);
	char *write_buf = NULL;
	int ret = 0;
	int i = 0;
	int j = 0;

	TOUCH_TRACE();

	write_buf = kmalloc(sizeof(char) * LOG_BUF_SIZE, GFP_KERNEL);
	if(write_buf) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
	}
	else {
		TOUCH_E("kamlloc write_buf fail!\n");
		return;
	}

	switch(item) {
		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RAWDATA]\n");
		break;

		case FW_CC:
			TOUCH_I("[FW_CC_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_RAWDATA]\n");
		break;

		case FW_CC_I:
			TOUCH_I("[FW_CC_I_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_I_RAWDATA]\n");
		break;

		case FW_CC_Q:
			TOUCH_I("[FW_CC_Q_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_Q_RAWDATA]\n");
		break;

		case NOISE_DIFF_ABS:
			TOUCH_I("[NOISE_DIFF_ABS_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[NOISE_DIFF_ABS_RAWDATA]\n");
		break;

		case SHORT:
			TOUCH_I("[SHORT_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RAWDATA]\n");
		break;

		case SHORT_DIFF:
			TOUCH_I("[SHORT_DIFF_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_DIFF_RAWDATA]\n");
		break;

		case SHORT_BASE:
			TOUCH_I("[SHORT_BASE_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_BASE_RAWDATA]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RAWDATA]\n");
		break;

		default:

		break;
	}

	for (i = 0; i < y_len; i++) {
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[%2d]", i);
		for (j = 0; j < x_len; j++) {
			printk("%7d", rawdata[i * x_len + j]);
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "%7d", rawdata[i * x_len + j]);
		}
		printk("\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	switch(item) {
		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RESULT]\n");
		break;

		case FW_CC:
			TOUCH_I("[FW_CC_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_RESULT]\n");
		break;

		case FW_CC_I:
			TOUCH_I("[FW_CC_I_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_I_RESULT]\n");
		break;

		case FW_CC_Q:
			TOUCH_I("[FW_CC_Q_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_CC_Q_RESULT]\n");
		break;

		case NOISE_DIFF_ABS:
			TOUCH_I("[NOISE_DIFF_ABS_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[NOISE_DIFF_ABS_RESULT]\n");
		break;

		case SHORT:
			TOUCH_I("[SHORT_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RESULT]\n");
		break;

		case SHORT_DIFF:
			TOUCH_I("[SHORT_DIFF_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_DIFF_RESULT]\n");
		break;

		case SHORT_BASE:
			TOUCH_I("[SHORT_BASE_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_BASE_RESULT]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RESULT]\n");
		break;

		default:

		break;
	}

	switch(TestResult) {
		case 0:
			TOUCH_I("PASS!\n\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[PASS]\n\n");
		break;

		case 1:
			TOUCH_E("ERROR! Read Data FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
		break;

		case -1:
			TOUCH_E ("FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FAIL]\n");
		break;
	}

	//---Write txt File---
	if(d->boot_mode == NORMAL_BOOT) {
		nt36xxx_write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
		nt36xxx_check_log_file_size(dev, NORMAL_SELF_TEST_FILE_PATH);
	}
	else {
		nt36xxx_write_file(SELF_TEST_FILE_PATH, write_buf, 1);
		nt36xxx_check_log_file_size(dev, SELF_TEST_FILE_PATH);
	}

	if (TestResult == -1) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
		ret = 0;
		TOUCH_I("RecordResult:\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[RECORD_RESULT]\n");
		for (i = 0; i < y_len; i++) {
			for(j = 0; j < x_len; j++) {
				printk("0x%02X, ", RecordResult[i * x_len + j]);
				ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", RecordResult[i * x_len + j]);
			}
			printk("\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		}
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		printk("\n");

		//---Write txt File---
		if(d->boot_mode == NORMAL_BOOT) {
			nt36xxx_write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
			nt36xxx_check_log_file_size(dev, NORMAL_SELF_TEST_FILE_PATH);
		}
		else {
			nt36xxx_write_file(SELF_TEST_FILE_PATH, write_buf, 1);
			nt36xxx_check_log_file_size(dev, SELF_TEST_FILE_PATH);
		}
	}

	if(write_buf) {
		kfree(write_buf);
	}
}

int32_t nt36xxx_selftest(struct device *dev, char* buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;	
	struct device_node *np = NULL;
	unsigned char mpcriteria[32] = {0};	//novatek-mp-criteria-default
	int32_t ret = 0;
	int i = 0;
	int j = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);
	np = gts->dev->of_node;

	TestResult_Short = 0;
	TestResult_Open = 0;
	TestResult_FW_Rawdata = 0;
	TestResult_FWMutual = 0;
	TestResult_FW_CC = 0;
	TestResult_Noise = 0;
	TestResult_FW_DiffMax = 0;
	TestResult_FW_DiffMin = 0;
	TestResult_Channel_Status = 0;

	if (nvt_get_fw_info(gts->dev)) {
		mutex_unlock(&d->lock);
		TOUCH_E("get fw info failed!\n");
		return -EAGAIN;
	}

	fw_ver = d->fw_ver;

	/* Parsing criteria from dts */
	if(of_property_read_bool(np, "novatek,mp-support-dt")) {
		/*
		 * Parsing Criteria by Novatek PID
		 * The string rule is "novatek-mp-criteria-<nvt_pid>"
		 * nvt_pid is 2 bytes (show hex).
		 *
		 * Ex. nvt_pid = 500A
		 *     mpcriteria = "novatek-mp-criteria-500A"
		 */
		snprintf(mpcriteria, PAGE_SIZE, "novatek-mp-criteria-%04X", d->nvt_pid);

		if (nvt_mp_parse_dt(np, mpcriteria)) {
			mutex_unlock(&d->lock);
			TOUCH_E("mp parse device tree failed!\n");
			return -EINVAL;
		}
	} else {
		TOUCH_I("Not found novatek,mp-support-dt, use default setting\n");
		//---Print Test Criteria---
		nvt_print_criteria();
	}

	//---Disable FW Frequency Hopping---
	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		TOUCH_E("switch frequency hopping disable failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "switch frequency hopping disable failed!\n");
		goto nt36xxx_selftest_out;
	}

	if (nvt_check_fw_reset_state(gts->dev, RESET_STATE_NORMAL_RUN)) {
		TOUCH_E("check fw reset state failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "check fw reset state failed!\n");
		goto nt36xxx_selftest_out;
	}

	msleep(100);

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		TOUCH_E("clear fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "clear fw status failed!\n");
		goto nt36xxx_selftest_out;
	}

	nvt_change_mode(MP_MODE_CC);

	if (nvt_check_fw_status(gts->dev)) {
		TOUCH_E("check fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "check fw status failed!\n");
		goto nt36xxx_selftest_out;
	}

	if (nvt_get_fw_info(gts->dev)) {
		TOUCH_E("get fw info failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "get fw info failed!\n");
		goto nt36xxx_selftest_out;
	}

	//---FW Rawdata Test---
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}
	nt36xxx_print_selftest_result(dev, FW_MUTUAL, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel, 0);

	//---FW CC Test---
	if (nvt_read_CC(RawData_FW_CC) != 0) {
		TestResult_FW_CC = 1;
	} else {
		TestResult_FW_CC = RawDataTest_SinglePoint_Sub(RawData_FW_CC, RecordResult_FW_CC, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_P, PS_Config_Lmt_FW_CC_N);
		nt36xxx_print_selftest_result(dev, FW_CC, TestResult_FW_CC, RecordResult_FW_CC, RawData_FW_CC, X_Channel, Y_Channel, 0);
	}

	if ((TestResult_FWMutual == 1) || (TestResult_FW_CC == 1)) {
		TestResult_FW_Rawdata = 1;
	} else {
		if ((TestResult_FWMutual == -1) || (TestResult_FW_CC == -1))
			TestResult_FW_Rawdata = -1;
		else
			TestResult_FW_Rawdata = 0;
	}

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data Test : ");
	switch(TestResult_FW_Rawdata) {
		case 0:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "PASS.\n");
			break;
	
		case 1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
			break;
	
		case -1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "FAIL!\n");
			break;
	}

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Noise Test---
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	// 1: ERROR
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		for (i = 0; i < Y_Channel; i++)
		{
			for (j = 0; j < X_Channel; j++)
			{
				if (NVT_ABS(RawData_Diff_Max[i*X_Channel+j]) > NVT_ABS(RawData_Diff_Min[i*X_Channel+j]))
					RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Max[i*X_Channel+j]);
				else
					RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Min[i*X_Channel+j]);
			}
		}

		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);
		nt36xxx_print_selftest_result(dev, NOISE_DIFF_ABS, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff, X_Channel, Y_Channel, 0);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "P2P Jitter Test : ");
		switch(TestResult_FW_DiffMax) {
			case 0:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "PASS.\n");
				break;

			case 1:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
				break;

			case -1:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "FAIL!\n");
				break;
		}

		if (TestResult_FW_DiffMax == -1)
			TestResult_Noise = -1;
		else
			TestResult_Noise = 0;
	}

	//--Short Test---
	if (nvt_read_fw_short(RawData_Short) != 0) {
		TestResult_Short = 1; // 1:ERROR
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		TestResult_Short = RawDataTest_SinglePoint_Sub(RawData_Short, RecordResult_Short, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Rawdata_P, PS_Config_Lmt_Short_Rawdata_N);
		nt36xxx_print_selftest_result(dev, SHORT, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel, 0);
	}

	//---Open Test---
	if (nvt_read_fw_open(RawData_Open) != 0) {
		TestResult_Open = 1;    // 1:ERROR
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		TestResult_Open = RawDataTest_SinglePoint_Sub(RawData_Open, RecordResult_Open, X_Channel, Y_Channel,
											PS_Config_Lmt_Open_Rawdata_P, PS_Config_Lmt_Open_Rawdata_N);
		nt36xxx_print_selftest_result(dev, OPEN, TestResult_Open, RecordResult_Open, RawData_Open, X_Channel, Y_Channel, 0);
	}

	if ((TestResult_Short == 1) || (TestResult_Open == 1)) {
		TestResult_Channel_Status= 1;
	} else {
		if ((TestResult_Short == -1) || (TestResult_Open == -1))
			TestResult_Channel_Status = -1;
		else
			TestResult_Channel_Status = 0;
	}

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Channel Status : ");
	switch(TestResult_Channel_Status) {
		case 0:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "PASS.\n");
			break;

			case 1:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
				break;

		case -1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "FAIL!\n");
			break;
	}

nt36xxx_selftest_out:
	//---Reset IC---
	nvt_bootloader_reset(dev);

	return ret;
}

int32_t nt36xxx_lpwg_selftest(struct device *dev, char* buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	struct device_node *np = NULL;
	unsigned char mpcriteria[32] = {0};	//novatek-mp-criteria-default
	int32_t ret = 0;
	int i = 0;
	int j = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);
	np = gts->dev->of_node;

	TestResult_FW_Rawdata = 0;
	TestResult_FWMutual = 0;
	TestResult_Noise = 0;
	TestResult_FW_DiffMax = 0;
	TestResult_FW_DiffMin = 0;

	if (nvt_get_fw_info(gts->dev)) {
		mutex_unlock(&d->lock);
		TOUCH_E("get fw info failed!\n");
		return -EAGAIN;
	}

	fw_ver = d->fw_ver;

	/* Parsing criteria from dts */
	if(of_property_read_bool(np, "novatek,mp-support-dt")) {
		/*
		 * Parsing Criteria by Novatek PID
		 * The string rule is "novatek-mp-criteria-<nvt_pid>"
		 * nvt_pid is 2 bytes (show hex).
		 *
		 * Ex. nvt_pid = 500A
		 *     mpcriteria = "novatek-mp-criteria-500A"
		 */
		snprintf(mpcriteria, PAGE_SIZE, "novatek-mp-criteria-%04X", d->nvt_pid);

		if (nvt_mp_parse_dt(np, mpcriteria)) {
			mutex_unlock(&d->lock);
			TOUCH_E("mp parse device tree failed!\n");
			return -EINVAL;
		}
	} else {
		TOUCH_I("Not found novatek,mp-support-dt, use default setting\n");
		//---Print Test Criteria---
		nvt_print_criteria();
	}

	//---Disable FW Frequency Hopping---
	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		TOUCH_E("switch frequency hopping disable failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! switch frequency hopping disable failed!\n");
		goto nt36xxx_lpwg_selftest_out;
	}

	//---Wait FW disable  Freq. Hopping---
	if (nvt_check_fw_reset_state(gts->dev, RESET_STATE_NORMAL_RUN)) {
		TOUCH_E("check fw reset state failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw reset state failed!\n");
		goto nt36xxx_lpwg_selftest_out;
	}

	msleep(100);

	//---Enter Test Mode---
	if (nvt_clear_fw_status(gts->dev)) {
		TOUCH_E("clear fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! clear fw status failed!\n");
		goto nt36xxx_lpwg_selftest_out;
	}

	//---Enter TEST Mode---
	nvt_change_mode(TEST_MODE_2);
	if (nvt_check_fw_status(gts->dev)) {
		TOUCH_E("check fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto nt36xxx_lpwg_selftest_out;
	}

	//---Get FW Info.---
	if (nvt_get_fw_info(gts->dev)) {
		TOUCH_E("get fw info failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! get fw info failed!\n");
		goto nt36xxx_lpwg_selftest_out;
	}

	//---FW Rawdata Test---
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}
	nt36xxx_print_selftest_result(dev, FW_MUTUAL, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel, 1);
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG Raw Data Test : ");
	switch(TestResult_FWMutual) {
		case 0:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "PASS.\n");
			break;

		case 1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
			break;

		case -1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "FAIL!\n");
			break;
	}

	if ((TestResult_FWMutual == 1) || (TestResult_FW_CC == 1)) {
		TestResult_FW_Rawdata = 1;
	} else {
		if ((TestResult_FWMutual == -1) || (TestResult_FW_CC == -1))
			TestResult_FW_Rawdata = -1;
		else
			TestResult_FW_Rawdata = 0;
	}

	//---Leave Test Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Noise Test---
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	// 1: ERROR
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		for (i = 0; i < Y_Channel; i++)
		{
			for (j = 0; j < X_Channel; j++)
			{
				if (NVT_ABS(RawData_Diff_Max[i*X_Channel+j]) > NVT_ABS(RawData_Diff_Min[i*X_Channel+j]))
					RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Max[i*X_Channel+j]);
				else
					RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Min[i*X_Channel+j]);
			}
		}

		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);
		nt36xxx_print_selftest_result(dev, NOISE_DIFF_ABS, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff, X_Channel, Y_Channel, 1);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "P2P Jitter Test : ");
		switch(TestResult_FW_DiffMax) {
			case 0:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "PASS.\n");
				break;

			case 1:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
				break;

			case -1:
				ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "FAIL!\n");
				break;
		}

		if (TestResult_FW_DiffMax == -1)
			TestResult_Noise = -1;
		else
			TestResult_Noise = 0;
	}

nt36xxx_lpwg_selftest_out:
	return ret;
}


static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int32_t ret = 0;
	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	//---Check the LCD state---
	if (ts->lpwg.screen == 0){
		TOUCH_E("LCD is OFF, Please turn on.\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD is OFF, Please turn on.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	ret = nt36xxx_selftest(dev, buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int ret = 0;
	int32_t i = 0;
	int32_t j = 0;
	int32_t* xdata = NULL;
	uint8_t x_num = 0;
	uint8_t y_num = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	mutex_lock(&ts->lock);

	xdata = (int32_t *)kmalloc(sizeof(int32_t) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! xdata Alloc Fail!\n");
		goto show_delta_out;
	}

	//---Clear FW handshake byte---
	if (nvt_clear_fw_status(gts->dev)) {
		TOUCH_E("clear fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! clear fw status failed!\n");
		goto show_delta_out;
	}

	//---Enable Eng. Mode---
	nvt_change_mode(ENG_MODE_ENABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_delta_out;
	}

	//---Enter TEST Mode---
	nvt_change_mode(TEST_MODE_1);
	if (nvt_check_fw_status(gts->dev)) {
		TOUCH_E("check fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! check fw status failed!\n");
		goto show_delta_out;
	}

	//---Get FW Info.---
	if (nvt_get_fw_info(gts->dev)) {
		TOUCH_E("get fw info failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! get fw info failed!\n");
		goto show_delta_out;
	}

	//---Get Delta Data---
	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(d->mmap->DIFF_PIPE0_ADDR, d->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(d->mmap->DIFF_PIPE1_ADDR, d->mmap->DIFF_BTN_PIPE1_ADDR);

	//---Leave TEST Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Disable Eng. Mode---
	nvt_change_mode(ENG_MODE_DISABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_delta_out;
	}

	if (xdata) {
		memset(xdata, 0, 2048 * sizeof(int32_t));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}

	for(i = 0; i < y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

show_delta_out:
	if(xdata)
		kfree(xdata);
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int ret = 0;
	int32_t i = 0;
	int32_t j = 0;
	int32_t* xdata = NULL;
	uint8_t x_num = 0;
	uint8_t y_num = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	mutex_lock(&ts->lock);

	xdata = (int32_t *)kmalloc(sizeof(int32_t) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! xdata Alloc Fail!\n");
		goto show_rawdata_out;
	}

	//---Clear FW handshake byte---
	if (nvt_clear_fw_status(gts->dev)) {
		TOUCH_E("clear fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! clear fw status failed!\n");
		goto show_rawdata_out;
	}

	//---Enable Eng. Mode---
	nvt_change_mode(ENG_MODE_ENABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_rawdata_out;
	}

	//---Enter TEST Mode---
	nvt_change_mode(TEST_MODE_1);
	if (nvt_check_fw_status(gts->dev)) {
		TOUCH_E("check fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! check fw status failed!\n");
		goto show_rawdata_out;
	}

	//---Get FW Info.---
	if (nvt_get_fw_info(gts->dev)) {
		TOUCH_E("get fw info failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! get fw info failed!\n");
		goto show_rawdata_out;
	}

	//---Get Rawdata---
	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(d->mmap->RAW_PIPE0_ADDR, d->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(d->mmap->RAW_PIPE1_ADDR, d->mmap->RAW_BTN_PIPE1_ADDR);

	//---Leave TEST Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Disable Eng. Mode---
	nvt_change_mode(ENG_MODE_DISABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_rawdata_out;
	}

	if (xdata) {
		memset(xdata, 0, 2048 * sizeof(int32_t));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}

	for(i = 0; i < y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

show_rawdata_out:
	if(xdata)
		kfree(xdata);
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_baseline(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int ret = 0;
	int32_t i = 0;
	int32_t j = 0;
	int32_t* xdata = NULL;
	uint8_t x_num = 0;
	uint8_t y_num = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	mutex_lock(&ts->lock);

	xdata = (int32_t *)kmalloc(sizeof(int32_t) * 2048, GFP_KERNEL);
	if(xdata == NULL) {
		TOUCH_E("xdata Alloc Fail\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! xdata Alloc Fail!\n");
		goto show_baseline_out;
	}

	//---Clear FW handshake byte---
	if (nvt_clear_fw_status(gts->dev)) {
		TOUCH_E("clear fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! clear fw status failed!\n");
		goto show_baseline_out;
	}

	//---Enable Eng. Mode---
	nvt_change_mode(ENG_MODE_ENABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_baseline_out;
	}

	//---Enter TEST Mode---
	nvt_change_mode(TEST_MODE_1);
	if (nvt_check_fw_status(gts->dev)) {
		TOUCH_E("check fw status failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! check fw status failed!\n");
		goto show_baseline_out;
	}

	//---Get FW Info.---
	if (nvt_get_fw_info(gts->dev)) {
		TOUCH_E("get fw info failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROE! get fw info failed!\n");
		goto show_baseline_out;
	}

	//---Get Baseline---
	nvt_read_mdata(d->mmap->BASELINE_ADDR, d->mmap->BASELINE_BTN_ADDR);

	//---Leave TEST Mode---
	nvt_change_mode(NORMAL_MODE);

	//---Disable Eng. Mode---
	nvt_change_mode(ENG_MODE_DISABLE);
	if (nvt_polling_cmd_clear()) {
		TOUCH_E("polling cmd clear failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw status failed!\n");
		goto show_baseline_out;
	}

	if (xdata) {
		memset(xdata, 0, 2048 * sizeof(int32_t));
		nvt_get_mdata(xdata, &x_num, &y_num);
	}

	for(i = 0; i < y_num; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < x_num; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (short)xdata[i*x_num+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

show_baseline_out:
	if(xdata)
		kfree(xdata);
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int32_t ret = 0;
	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	//---Check the LCD state---
	if (ts->lpwg.screen == 1){
		TOUCH_E("LCD is ON, Please turn off.\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD is OFF, Please turn on.\n");
		return ret;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
	ret = nt36xxx_lpwg_selftest(dev, buf);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	return ret;
}

static ssize_t show_jitter(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct nt36xxx_data *d = NULL;
	int ret = 0;
	int i = 0;
	int j = 0;

	TOUCH_TRACE();

	gts = ts;
	d = to_nt36xxx_data(gts->dev);

	TOUCH_I("Noise Test Start\n");
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	//---Disable FW Frequency Hopping---
	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		TOUCH_E("switch frequency hopping disable failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! switch frequency hopping disable failed!\n");
		goto show_jitter_out;
	}

	//---Wait FW disable  Freq. Hopping---
	if (nvt_check_fw_reset_state(gts->dev, RESET_STATE_NORMAL_RUN)) {
		TOUCH_E("check fw reset state failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! check fw reset state failed!\n");
		goto show_jitter_out;
	}

	msleep(100);

	//---Do FW Noise Test and Get Noise Data---
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TOUCH_E("get noise data failed!\n");
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "ERROR! get noise data failed!\n");
		goto show_jitter_out;
	}

	for (i = 0; i < Y_Channel; i++)
	{
		for (j = 0; j < X_Channel; j++)
		{
			if (NVT_ABS(RawData_Diff_Max[i*X_Channel+j]) > NVT_ABS(RawData_Diff_Min[i*X_Channel+j]))
				RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Max[i*X_Channel+j]);
			else
				RawData_Diff[i*X_Channel+j] = NVT_ABS(RawData_Diff_Min[i*X_Channel+j]);
		}
	}

	for (i = 0; i < Y_Channel; i++)
	{
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "[%2d] ", i);
		for (j = 0; j < X_Channel; j++)
		{
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "%5d", (int8_t)RawData_Diff[i*X_Channel+j]);
		}
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "\n");

	//---Reset IC---
	//nvt_bootloader_reset(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	TOUCH_I("Noise Test End\n");

show_jitter_out:
	mutex_unlock(&ts->lock);
	return ret;
}




static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(baseline, show_baseline, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
static TOUCH_ATTR(jitter, show_jitter, NULL);


static struct attribute *prd_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_baseline.attr,
	&touch_attr_lpwg_sd.attr,
	&touch_attr_jitter.attr,
	NULL,
};

static const struct attribute_group prd_attribute_group = {
	.attrs = prd_attribute_list,
};

int32_t nt36xxx_prd_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &prd_attribute_group);

	if (ret < 0) {
		TOUCH_E("failed to create sysfs\n");
		return ret;
	}

	return ret;
}
#endif /* #if NVT_TOUCH_MP */
