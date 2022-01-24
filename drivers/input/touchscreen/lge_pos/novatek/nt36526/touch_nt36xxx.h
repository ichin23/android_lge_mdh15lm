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
#ifndef  LGE_DEVICE_NT36XXX_H
#define LGE_TOUCH_NT36XXX_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "touch_nt36xxx_mem_map.h"

//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

//---Touch info.---
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 1

enum {
	DEEP_SLEEP_MODE = 0,
	LPWG_MODE,
	FW_NORMAL_MODE,
	FW_UPGRADE_MODE,
};

struct nt36xxx_data {
	uint8_t fw_ver;
	uint8_t fw_ver_bar;
	uint8_t fw_sub_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint16_t nvt_pid;
	uint32_t x_dimention;
	uint32_t y_dimention;
	uint8_t mode_state;
	uint8_t boot_mode;
	u8 charger;
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN	// normal run
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
	EVENT_MAP_LPWG_PARM						= 0xAC,
	EVENT_MAP_LPWG_RESULT					= 0xC9,
	EVENT_MAP_LPWG_CONTROL_FLAG				= 0xF3,
	EVENT_MAP_LPWG_WKG_DATA_RW				= 0xF4,
	EVENT_MAP_LPWG_WKG_DATA_READY			= 0xF5,
	EVENT_MAP_LPWG_FAIL_REASON				= 0xF6,
} I2C_EVENT_MAP;

//---extern functions---
extern int32_t CTP_I2C_READ(struct device *dev, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct device *dev, uint16_t address, uint8_t *buf, uint16_t len);
extern void nvt_bootloader_reset(struct device *dev);
extern void nvt_sw_reset_idle(struct device *dev);
extern int32_t nvt_check_fw_reset_state(struct device *dev, RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_get_fw_info(struct device *dev);
extern int32_t nvt_clear_fw_status(struct device *dev);
extern int32_t nvt_check_fw_status(struct device *dev);
extern int32_t nvt_set_page(struct device *dev, uint16_t i2c_addr, uint32_t addr);
extern void nvt_stop_crc_reboot(struct device *dev);

static inline struct nt36xxx_data *to_nt36xxx_data(struct device *dev)
{
	return (struct nt36xxx_data *)touch_get_device(to_touch_core(dev));
}

#endif /* _LINUX_NVT_TOUCH_H */
