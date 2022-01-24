/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "kd_imgsensor.h"

#include "regulator/regulator.h"
#include "gpio/gpio.h"
/*#include "mt6306/mt6306.h"*/
#include "mclk/mclk.h"



#include "imgsensor_cfg_table.h"

enum IMGSENSOR_RETURN
	(*hw_open[IMGSENSOR_HW_ID_MAX_NUM])(struct IMGSENSOR_HW_DEVICE **) = {
	imgsensor_hw_regulator_open,
	imgsensor_hw_gpio_open,
	/*imgsensor_hw_mt6306_open,*/
	imgsensor_hw_mclk_open
};

struct IMGSENSOR_HW_CFG imgsensor_custom_config[] = {
	{
		IMGSENSOR_SENSOR_IDX_MAIN,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AFVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB,
		IMGSENSOR_I2C_DEV_1,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_MIPI_SWITCH_EN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL},
#endif
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN2,
		IMGSENSOR_I2C_DEV_2,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
#ifdef MIPI_SWITCH
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_MIPI_SWITCH_EN},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL},
#endif
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_SUB2,
		IMGSENSOR_I2C_DEV_0,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			//{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},
	{
		IMGSENSOR_SENSOR_IDX_MAIN3,
		IMGSENSOR_I2C_DEV_3,
		{
			{IMGSENSOR_HW_ID_MCLK, IMGSENSOR_HW_PIN_MCLK},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_AVDD},
			{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DOVDD},
			//{IMGSENSOR_HW_ID_REGULATOR, IMGSENSOR_HW_PIN_DVDD},
			{IMGSENSOR_HW_ID_GPIO, IMGSENSOR_HW_PIN_RST},
			{IMGSENSOR_HW_ID_NONE, IMGSENSOR_HW_PIN_NONE},
		},
	},

	{IMGSENSOR_SENSOR_IDX_NONE}
};

struct IMGSENSOR_HW_POWER_SEQ platform_power_sequence[] = {
#ifdef MIPI_SWITCH
	{
		IMGSENSOR_SENSOR_IDX_NAME_SUB,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0
			},
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		}
	},
	{
		IMGSENSOR_SENSOR_IDX_NAME_MAIN2,
		{
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_EN,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0
			},
			{
				IMGSENSOR_HW_PIN_MIPI_SWITCH_SEL,
				IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH,
				0,
				IMGSENSOR_HW_PIN_STATE_LEVEL_0,
				0
			},
		}
	},
#endif

	{NULL}
};

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {
#if defined(HI1336_MIPI_RAW)
        {
                SENSOR_DRVNAME_HI1336_MIPI_RAW,
                {
                        {RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 1},
                        {DVDD, Vol_1200, 1},
			{AFVDD, Vol_2800, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 5},
                },
        },
#endif
#if defined(HI846_MIPI_RAW)
        {
                SENSOR_DRVNAME_HI846_MIPI_RAW,
                {
                        {RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
                        {AVDD, Vol_2800, 1},
                        {DVDD, Vol_1200, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 5},
                },
        },
#endif
#if defined(HI556_MIPI_RAW)
        {
                SENSOR_DRVNAME_HI556_MIPI_RAW,
                {
                        {RST, Vol_Low, 0},
                        {AVDD, Vol_2800, 1},
                        {DOVDD, Vol_1800, 1},
                        {DVDD, Vol_1200, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 5},
                },
        },
#endif
#if defined(GC2375HMACRO_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC2375HMACRO_MIPI_RAW,
		{
			{RST, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			//{DVDD, Vol_1800, 0},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 10},
		},
	},
#endif
#if defined(GC2375H_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC2375H_MIPI_RAW,
		{
			{RST, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			//{DVDD, Vol_1800, 0},
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 10},
		},
	},
#endif
#if defined(OV13B10_MIPI_RAW)
        {
                SENSOR_DRVNAME_OV13B10_MIPI_RAW,
                {
                        {DOVDD, Vol_1800, 0},
                        {AVDD, Vol_2800, 0},
                        {DVDD, Vol_1200, 0},
						{AFVDD, Vol_2800, 1},
                        {SensorMCLK, Vol_High, 1},
                        {RST, Vol_High, 1}
                },
        },
#endif
#if defined(S5K4H7DH15_MIPI_RAW)
        {
                SENSOR_DRVNAME_S5K4H7DH15_MIPI_RAW,
                {
			{RST, Vol_Low, 1},
			{DVDD, Vol_1200, 2},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 1},
			{SensorMCLK, Vol_High, 10},
		},
        },
#endif
#if defined(GC5035DH15_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC5035DH15_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 1},
		},
	},
#endif
#if defined(GC02M1DH15MACRO_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M1DH15MACRO_MIPI_RAW,
		{
			//{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 1},
			//{DVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 1},
			//{PDN, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(GC02M1DH15JS_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M1DH15JS_MIPI_RAW,
		{
			//{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 1},
			//{DVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 1},
			//{PDN, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
	/* add new sensor before this line */
	{NULL,},
};

