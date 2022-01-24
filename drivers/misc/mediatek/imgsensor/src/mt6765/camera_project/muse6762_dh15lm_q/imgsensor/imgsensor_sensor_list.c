/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "kd_imgsensor.h"
#include "imgsensor_sensor_list.h"

/* Add Sensor Init function here
 * Note:
 * 1. Add by the resolution from ""large to small"", due to large sensor
 *    will be possible to be main sensor.
 *    This can avoid I2C error during searching sensor.
 * 2. This should be the same as
 *     mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
 */
struct IMGSENSOR_INIT_FUNC_LIST kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(HI1336_MIPI_RAW)
        {HI1336_SENSOR_ID,
        SENSOR_DRVNAME_HI1336_MIPI_RAW,
        HI1336_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
        {HI846_SENSOR_ID,
        SENSOR_DRVNAME_HI846_MIPI_RAW,
        HI846_MIPI_RAW_SensorInit},
#endif
#if defined(HI556_MIPI_RAW)
        {HI556_SENSOR_ID,
        SENSOR_DRVNAME_HI556_MIPI_RAW,
        HI556_MIPI_RAW_SensorInit},
#endif
#if defined(GC2375HMACRO_MIPI_RAW)
        {GC2375HMACRO_SENSOR_ID,
        SENSOR_DRVNAME_GC2375HMACRO_MIPI_RAW,
        GC2375HMACRO_MIPI_RAW_SensorInit},
#endif
#if defined(GC2375H_MIPI_RAW)
        {GC2375H_SENSOR_ID,
        SENSOR_DRVNAME_GC2375H_MIPI_RAW,
        GC2375H_MIPI_RAW_SensorInit},
#endif
#if defined(OV13B10_MIPI_RAW)
        {OV13B10_SENSOR_ID,
        SENSOR_DRVNAME_OV13B10_MIPI_RAW,
        OV13B10_MIPI_RAW_SensorInit},
#endif
#if defined(S5K4H7DH15_MIPI_RAW)
        {S5K4H7DH15_SENSOR_ID,
        SENSOR_DRVNAME_S5K4H7DH15_MIPI_RAW,
        S5K4H7DH15_MIPI_RAW_SensorInit},
#endif
#if defined(GC5035DH15_MIPI_RAW)
        {GC5035DH15_SENSOR_ID,
        SENSOR_DRVNAME_GC5035DH15_MIPI_RAW,
        GC5035DH15_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1DH15MACRO_MIPI_RAW)
        {GC02M1DH15MACRO_SENSOR_ID,
        SENSOR_DRVNAME_GC02M1DH15MACRO_MIPI_RAW,
        GC02M1DH15MACRO_MIPI_RAW_SensorInit},
#endif
#if defined(GC02M1DH15JS_MIPI_RAW)
        {GC02M1DH15JS_SENSOR_ID,
        SENSOR_DRVNAME_GC02M1DH15JS_MIPI_RAW,
        GC02M1DH15JS_MIPI_RAW_SensorInit},
#endif
	/*  ADD sensor driver before this line */
	{0, {0}, NULL}, /* end of list */
};
/* e_add new sensor driver here */

