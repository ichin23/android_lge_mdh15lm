/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_ft8006p.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* Normal Mode SET SPEC */
#define RAW_DATA_MAX		10000
#define RAW_DATA_MIN		2000
#define RAW_DATA_MARGIN		0
#define IB_MAX			55
#define IB_MIN			1
#define NOISE_MAX		140
#define NOISE_MIN		0
/* LPWG Mode SET SPEC */
#define LPWG_RAW_DATA_MAX	10000
#define LPWG_RAW_DATA_MIN	2000
#define LPWG_IB_MAX		55
#define LPWG_IB_MIN		1
#define LPWG_NOISE_MAX		140
#define LPWG_NOISE_MIN		0
#define LPWG_JITTER_MAX		140
#define LPWG_JITTER_MIN		0

#define FTS_WORK_MODE		0x00
#define FTS_FACTORY_MODE	0x40

#define FTS_MODE_CHANGE_LOOP	20

#define TEST_PACKET_LENGTH	342

/* Number of channel */
#define MAX_ROW			32
#define MAX_COL			18

#define LOG_BUF_SIZE		4095 /*4096 - 1 */
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)

#define DELTA_ATTR_SIZE		(8 * 1024) - 500
#define RAWDATA_ATTR_SIZE	(10 * 1024) - 1100
#define LGE_ATTR_DELTA		"delta_ext"
#define LGE_ATTR_RAWDATA	"rawdata_ext"

enum {
	RAW_DATA_TEST = 0,
	IB_DATA_TEST,
	NOISE_TEST,
	DELTA_SHOW,
	LPWG_RAW_DATA_TEST,
	LPWG_IB_DATA_TEST,
	LPWG_NOISE_TEST,
	JITTER_TEST,
};

enum {
	TEST_FAIL = 0,
	TEST_PASS,
};

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};
extern void ft8006p_dummy_read(struct device *dev);
extern void touch_msleep(unsigned int msecs);
int ft8006p_prd_register_sysfs(struct device *dev);
#endif
