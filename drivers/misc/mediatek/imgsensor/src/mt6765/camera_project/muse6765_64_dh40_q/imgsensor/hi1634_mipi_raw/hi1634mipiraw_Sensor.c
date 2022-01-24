/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "hi1634mipiraw_Sensor.h"

#define PFX "hi1634_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

//PDAF
#define ENABLE_PDAF 0
#define per_frame 1
//#define e2prom 0
//extern bool read_hi1634_eeprom( kal_uint16 addr, BYTE *data, kal_uint32 size); 
//extern bool read_eeprom( kal_uint16 addr, BYTE * data, kal_uint32 size);

 

#define MULTI_WRITE 1
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HI1634_SENSOR_ID,
	
	.checksum_value = 0x2fbf863,       // Auto Test Mode ����..
	.pre = {
		.pclk = 640000000,				//record different mode's pclk
		.linelength =  5680, 			//record different mode's linelength
		//.framelength = 1865, 			//record different mode's framelength
		.framelength = 3755, 			//record different mode's framelength
		.startx = 0,				    //record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2320, 		//record different mode's width of grabwindow
		.grabwindow_height = 1744,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		//.max_framerate = 600,
		.max_framerate = 300,
		.mipi_pixel_rate = 339200000//848*4/10
	},
	.cap = {
		.pclk = 640000000,
		.linelength = 5680, 	
		.framelength = 3755,//3766,
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 678400000//1696*4/10
	},
	// need to setting
    .cap1 = {                            
		.pclk = 310000000,
		.linelength = 5680,
		.framelength = 7381, //3609,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4656,
		.grabwindow_height = 3496,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 640000000//800*4/10

    },
	.normal_video = {
		.pclk = 640000000,
		.linelength = 5680, 	
		.framelength = 3755,//3766,
		.startx = 0,	
		.starty = 0,
		.grabwindow_width = 4640,
		.grabwindow_height = 3488,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 678400000//1696*4/10
	},					   
	.hs_video = {
		.pclk = 640000000,				//record different mode's pclk
		.linelength =  5800, 			//record different mode's linelength
		.framelength = 1149, 			//record different mode's framelength
		.startx = 0,				    //record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,
		.mipi_pixel_rate = 226132000//565.33Mbps x 4Lane/10
	},
    .slim_video = {
		.pclk = 640000000,
		.linelength = 5800,
		.framelength = 2298,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,	/* record different mode's width of grabwindow */
		.grabwindow_height = 1080,	/* record different mode's height of grabwindow */
    	.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
    	.mipi_pixel_rate = 339200000//848*4/10
    },

	.margin = 4,
	.min_shutter = 4, 
	.max_frame_length = 0xFFFF,
#if per_frame
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
#else
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
#endif
	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 2, 
	.pre_delay_frame = 2, 
	.video_delay_frame = 2, 
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_8MA, //ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
//	.mipi_sensor_type = MIPI_OPHY_CSI2,//MIPI_OPHY_NCSI2,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,//MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	//.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr, //SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr, //SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr, //SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr, //SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gr 
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x42, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x42,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 4672, 3520, 0,    12, 4672, 3496,  2336, 1748,    8,  2, 2320, 1744, 0, 0, 2320, 1744}, 	// preview (2320 x 1744)//DIFF
	{ 4672, 3520, 0,    14, 4672, 3492,  4672, 3492,  16,  2, 4640, 3488, 0, 0, 4640, 3488}, 	// capture (4640 x 3488)
	{ 4672, 3520, 0,    14, 4672, 3492,  4672, 3492,  16,  2, 4640, 3488, 0, 0, 4640, 3488}, 	// normal_video (4640 x 2608)
	{ 4672, 3520, 8,  674, 4656, 2172,  1552,   724, 136,  2, 1280,   720, 0, 0, 1280,  720}, 	// hs video (1280 x 720)//DIFF
	{ 4672, 3520, 0,  676, 4672, 2168,  2336, 1084, 208,  2, 1920, 1080, 0, 0, 1920, 1080}, 	// slim video (1920 x 1080)
};


#if ENABLE_PDAF

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{
	/* Preview mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x0918, 0x06D4,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x0000, 0x0000,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Capture mode setting */ 
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x1230, 0x0DB0,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  //0x01, 0x30, 0x0168, 0x0360,   // VC2 PDAF //type2
	  0x01, 0x30, 0x0168, 0x0360,   // VC2 PDAF //type2
	  //0x01, 0x30, 0x0168, 0x0090,   // VC2 PDAF
	  //0x01, 0x2B, 0x0168, 0x0360,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??
	/* Video mode setting */
	 {0x02, //VC_Num
	  0x0a, //VC_PixelNum	
	  0x00, //ModeSelect	/* 0:auto 1:direct */
	  0x00, //EXPO_Ratio	/* 1/1, 1/2, 1/4, 1/8 */
	  0x00, //0DValue		/* 0D Value */
	  0x00, //RG_STATSMODE	/* STATS divistion mode 0:16x16  1:8x8	2:4x4  3:1x1 */
	  0x00, 0x2B, 0x1070, 0x0C30,	// VC0 Maybe image data?
	  0x00, 0x00, 0x0000, 0x0000,	// VC1 MVHDR
	  0x01, 0x30, 0x0140, 0x0300,   // VC2 PDAF
	  0x00, 0x00, 0x0000, 0x0000},	// VC3 ??

};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	.i4OffsetX	= 24,
	.i4OffsetY	= 24,
	.i4PitchX	= 32,
	.i4PitchY	= 32,
	.i4PairNum	= 8,
	.i4SubBlkW	= 16,
	.i4SubBlkH	= 8,
	.i4BlockNumX = 144,
	.i4BlockNumY = 108,
	.iMirrorFlip = 0,
	.i4PosR =	{
						{28,25}, {44,25}, {36,37}, {52,37},
						{28,41}, {44,41}, {36,53}, {52,53},
				},
	.i4PosL =	{
						{28,29}, {44,29}, {36,33}, {52,33},
						{28,45}, {44,45}, {36,49}, {52,49},
				}

};
#endif

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020

static kal_uint16 hi1634_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		}

		if ((I2C_BUFFER_LEN - tosend) < 4 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				4, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x020e, imgsensor.frame_length & 0xFFFF); 
	write_cmos_sensor(0x0206, imgsensor.line_length/8);

}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0716) << 8) | read_cmos_sensor(0x0717));
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;

	spin_unlock(&imgsensor_drv_lock);
	
	LOG_INF("[jitae] framerate = %d\n",framerate);
	LOG_INF("[jitae] frame_length = %d, line_length = %d\n",imgsensor.frame_length,imgsensor.line_length);

	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint32 realtime_fps = 0;

	kal_uint32 capture_shutter = 0;
	capture_shutter = shutter;
	
	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	LOG_INF("shutter = %d, imgsensor.frame_length = %d, imgsensor.min_frame_length = %d, imgsensor.line_length= %d\n",
		shutter, imgsensor.frame_length, imgsensor.min_frame_length,imgsensor.line_length);


	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) :
		shutter;
	if (imgsensor.autoflicker_en) {
		LOG_INF("pclk = %d, imgsensor.frame_length = %d, imgsensor.min_frame_length = %d, imgsensor.line_length= %d\n",
			imgsensor.pclk, imgsensor.frame_length, imgsensor.min_frame_length,imgsensor.line_length);
		
		realtime_fps = imgsensor.pclk /
			(imgsensor.line_length * imgsensor.frame_length) * 10;
		
		LOG_INF("realtime_fps = %d\n",realtime_fps);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor(0x020e, imgsensor.frame_length);

	} else{
			write_cmos_sensor(0x020e, imgsensor.frame_length);
	}

#if 0 
	//shutter for Q2B Bayer(Capture)  by odin
	if( imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE)
	{
		capture_shutter = capture_shutter*4;
		write_cmos_sensor_8(0x020D, (capture_shutter & 0xFF0000) >> 16 );
		write_cmos_sensor(0x020A, capture_shutter);

	}
	else // for 4Sum by odin
	{
		write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16 );
		write_cmos_sensor(0x020A, shutter);
	}
#endif	
#if 1 
		write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16 );
		write_cmos_sensor(0x020A, shutter);
#endif

	LOG_INF("SKhynix frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);

}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	LOG_INF("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 gain2reg(kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;


    return (kal_uint16)reg_gain;

}


static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 16 * BASEGAIN)
            gain = 16 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("SKhynix gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	reg_gain = reg_gain & 0x00FF;
    write_cmos_sensor_8(0x0213,reg_gain);
	return gain;

}

#if 0
static void ihdr_write_shutter_gain(kal_uint16 le,
				kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;
		// Extend frame length first
		write_cmos_sensor(0x0006, imgsensor.frame_length);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0000, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0000, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0000, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0000, 0x0300);

		break;
	default:
		LOG_INF("Error image_mirror setting");
		break;
	}

}
#endif
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

#if MULTI_WRITE
kal_uint16 addr_data_pair_init_hi1634[] = {
	0x0790, 0x0100,
	0x2000, 0x1001,
	0x2002, 0x0000,
	0x2006, 0x40B2,
	0x2008, 0xB032,
	0x200A, 0x8430,
	0x200C, 0x40B2,
	0x200E, 0xB07C,
	0x2010, 0x8476,
	0x2012, 0x40B2,
	0x2014, 0xB0EE,
	0x2016, 0x847A,
	0x2018, 0x40B2,
	0x201A, 0xB122,
	0x201C, 0x8434,
	0x201E, 0x40B2,
	0x2020, 0xB168,
	0x2022, 0x8488,
	0x2024, 0x40B2,
	0x2026, 0xB39A,
	0x2028, 0x871E,
	0x202A, 0x40B2,
	0x202C, 0xB314,
	0x202E, 0x86C8,
	0x2030, 0x4130,
	0x2032, 0x120B,
	0x2034, 0x120A,
	0x2036, 0x403B,
	0x2038, 0x0261,
	0x203A, 0x4B6A,
	0x203C, 0xC3EB,
	0x203E, 0x0000,
	0x2040, 0x1292,
	0x2042, 0xD000,
	0x2044, 0x4ACB,
	0x2046, 0x0000,
	0x2048, 0xB3EB,
	0x204A, 0x0000,
	0x204C, 0x2411,
	0x204E, 0x421F,
	0x2050, 0x85DA,
	0x2052, 0xD21F,
	0x2054, 0x85D8,
	0x2056, 0x930F,
	0x2058, 0x2404,
	0x205A, 0x40F2,
	0x205C, 0xFF80,
	0x205E, 0x0619,
	0x2060, 0x3C07,
	0x2062, 0x90F2,
	0x2064, 0x0011,
	0x2066, 0x0619,
	0x2068, 0x2803,
	0x206A, 0x50F2,
	0x206C, 0xFFF0,
	0x206E, 0x0619,
	0x2070, 0x40B2,
	0x2072, 0xB31A,
	0x2074, 0x86D0,
	0x2076, 0x413A,
	0x2078, 0x413B,
	0x207A, 0x4130,
	0x207C, 0x120B,
	0x207E, 0x120A,
	0x2080, 0x8231,
	0x2082, 0x430A,
	0x2084, 0x93C2,
	0x2086, 0x0C0A,
	0x2088, 0x2404,
	0x208A, 0xB3D2,
	0x208C, 0x0B05,
	0x208E, 0x2401,
	0x2090, 0x431A,
	0x2092, 0x403B,
	0x2094, 0x8438,
	0x2096, 0x422D,
	0x2098, 0x403E,
	0x209A, 0x192A,
	0x209C, 0x403F,
	0x209E, 0x86EC,
	0x20A0, 0x12AB,
	0x20A2, 0x422D,
	0x20A4, 0x403E,
	0x20A6, 0x86EC,
	0x20A8, 0x410F,
	0x20AA, 0x12AB,
	0x20AC, 0x930A,
	0x20AE, 0x2003,
	0x20B0, 0xD3D2,
	0x20B2, 0x1921,
	0x20B4, 0x3C09,
	0x20B6, 0x403D,
	0x20B8, 0x0200,
	0x20BA, 0x422E,
	0x20BC, 0x403F,
	0x20BE, 0x86EC,
	0x20C0, 0x1292,
	0x20C2, 0x8448,
	0x20C4, 0xC3D2,
	0x20C6, 0x1921,
	0x20C8, 0x1292,
	0x20CA, 0xD046,
	0x20CC, 0x403B,
	0x20CE, 0x8438,
	0x20D0, 0x422D,
	0x20D2, 0x410E,
	0x20D4, 0x403F,
	0x20D6, 0x86EC,
	0x20D8, 0x12AB,
	0x20DA, 0x422D,
	0x20DC, 0x403E,
	0x20DE, 0x86EC,
	0x20E0, 0x403F,
	0x20E2, 0x192A,
	0x20E4, 0x12AB,
	0x20E6, 0x5231,
	0x20E8, 0x413A,
	0x20EA, 0x413B,
	0x20EC, 0x4130,
	0x20EE, 0x4382,
	0x20F0, 0x052C,
	0x20F2, 0x4F0D,
	0x20F4, 0x930D,
	0x20F6, 0x3402,
	0x20F8, 0xE33D,
	0x20FA, 0x531D,
	0x20FC, 0xF03D,
	0x20FE, 0x07F0,
	0x2100, 0x4D0E,
	0x2102, 0xC312,
	0x2104, 0x100E,
	0x2106, 0x110E,
	0x2108, 0x110E,
	0x210A, 0x110E,
	0x210C, 0x930F,
	0x210E, 0x3803,
	0x2110, 0x4EC2,
	0x2112, 0x052C,
	0x2114, 0x3C04,
	0x2116, 0x4EC2,
	0x2118, 0x052D,
	0x211A, 0xE33D,
	0x211C, 0x531D,
	0x211E, 0x4D0F,
	0x2120, 0x4130,
	0x2122, 0x120B,
	0x2124, 0x425F,
	0x2126, 0x0205,
	0x2128, 0xC312,
	0x212A, 0x104F,
	0x212C, 0x114F,
	0x212E, 0x114F,
	0x2130, 0x114F,
	0x2132, 0x114F,
	0x2134, 0x114F,
	0x2136, 0x4F0B,
	0x2138, 0xF31B,
	0x213A, 0x5B0B,
	0x213C, 0x5B0B,
	0x213E, 0x5B0B,
	0x2140, 0x503B,
	0x2142, 0xD1CC,
	0x2144, 0x1292,
	0x2146, 0xD004,
	0x2148, 0x93C2,
	0x214A, 0x86BF,
	0x214C, 0x240B,
	0x214E, 0xB2E2,
	0x2150, 0x0400,
	0x2152, 0x2008,
	0x2154, 0x425F,
	0x2156, 0x86BB,
	0x2158, 0xD36F,
	0x215A, 0xF37F,
	0x215C, 0x5F0F,
	0x215E, 0x5F0B,
	0x2160, 0x4BA2,
	0x2162, 0x0402,
	0x2164, 0x413B,
	0x2166, 0x4130,
	0x2168, 0x8231,
	0x216A, 0xD3D2,
	0x216C, 0x7A12,
	0x216E, 0xC3D2,
	0x2170, 0x0F00,
	0x2172, 0x422D,
	0x2174, 0x403E,
	0x2176, 0x06D6,
	0x2178, 0x410F,
	0x217A, 0x1292,
	0x217C, 0x8438,
	0x217E, 0x93C2,
	0x2180, 0x86C1,
	0x2182, 0x2430,
	0x2184, 0x0B00,
	0x2186, 0x7304,
	0x2188, 0x0000,
	0x218A, 0x0800,
	0x218C, 0x7A10,
	0x218E, 0x421F,
	0x2190, 0x7100,
	0x2192, 0xF03F,
	0x2194, 0x0003,
	0x2196, 0x931F,
	0x2198, 0x241E,
	0x219A, 0x931F,
	0x219C, 0x2815,
	0x219E, 0x932F,
	0x21A0, 0x240D,
	0x21A2, 0x903F,
	0x21A4, 0x0003,
	0x21A6, 0x2404,
	0x21A8, 0x9382,
	0x21AA, 0x7112,
	0x21AC, 0x27EB,
	0x21AE, 0x3C1C,
	0x21B0, 0x41A2,
	0x21B2, 0x06D6,
	0x21B4, 0x4192,
	0x21B6, 0x0002,
	0x21B8, 0x06D8,
	0x21BA, 0x3FF6,
	0x21BC, 0x4192,
	0x21BE, 0x0002,
	0x21C0, 0x06DA,
	0x21C2, 0x41A2,
	0x21C4, 0x06DC,
	0x21C6, 0x3FF0,
	0x21C8, 0x4192,
	0x21CA, 0x0004,
	0x21CC, 0x06DA,
	0x21CE, 0x4192,
	0x21D0, 0x0006,
	0x21D2, 0x06DC,
	0x21D4, 0x3FE9,
	0x21D6, 0x4192,
	0x21D8, 0x0006,
	0x21DA, 0x06D6,
	0x21DC, 0x4192,
	0x21DE, 0x0004,
	0x21E0, 0x06D8,
	0x21E2, 0x3FE2,
	0x21E4, 0x1292,
	0x21E6, 0xD058,
	0x21E8, 0x5231,
	0x21EA, 0x4130,
	0x21EC, 0x7400,
	0x21EE, 0x8058,
	0x21F0, 0x1807,
	0x21F2, 0x00E0,
	0x21F4, 0x7002,
	0x21F6, 0x17C7,
	0x21F8, 0x7000,
	0x21FA, 0x1305,
	0x21FC, 0x0006,
	0x21FE, 0x001F,
	0x2200, 0x0055,
	0x2202, 0x00DB,
	0x2204, 0x0012,
	0x2206, 0x1754,
	0x2208, 0x206F,
	0x220A, 0x009E,
	0x220C, 0x00DD,
	0x220E, 0x5023,
	0x2210, 0x00DE,
	0x2212, 0x005B,
	0x2214, 0x0119,
	0x2216, 0x0390,
	0x2218, 0x00D1,
	0x221A, 0x0055,
	0x221C, 0x0040,
	0x221E, 0x0553,
	0x2220, 0x0456,
	0x2222, 0x5041,
	0x2224, 0x700D,
	0x2226, 0x2F99,
	0x2228, 0x2318,
	0x222A, 0x005C,
	0x222C, 0x7000,
	0x222E, 0x1586,
	0x2230, 0x0001,
	0x2232, 0x2032,
	0x2234, 0x0012,
	0x2236, 0x0008,
	0x2238, 0x0343,
	0x223A, 0x0148,
	0x223C, 0x2123,
	0x223E, 0x0046,
	0x2240, 0x05DD,
	0x2242, 0x00DE,
	0x2244, 0x00DD,
	0x2246, 0x00DC,
	0x2248, 0x00DE,
	0x224A, 0x07D6,
	0x224C, 0x5061,
	0x224E, 0x704F,
	0x2250, 0x2F99,
	0x2252, 0x005C,
	0x2254, 0x5080,
	0x2256, 0x4D90,
	0x2258, 0x50A1,
	0x225A, 0x2122,
	0x225C, 0x7800,
	0x225E, 0xC08C,
	0x2260, 0x0001,
	0x2262, 0x9038,
	0x2264, 0x59F7,
	0x2266, 0x903B,
	0x2268, 0x121C,
	0x226A, 0x9034,
	0x226C, 0x1218,
	0x226E, 0x8C34,
	0x2270, 0x0180,
	0x2272, 0x8DC0,
	0x2274, 0x01C0,
	0x2276, 0x7400,
	0x2278, 0x8058,
	0x227A, 0x1807,
	0x227C, 0x00E0,
	0x227E, 0x00DF,
	0x2280, 0x0047,
	0x2282, 0x7000,
	0x2284, 0x17C5,
	0x2286, 0x0046,
	0x2288, 0x0095,
	0x228A, 0x7000,
	0x228C, 0x148C,
	0x228E, 0x005B,
	0x2290, 0x0014,
	0x2292, 0x001D,
	0x2294, 0x216F,
	0x2296, 0x005E,
	0x2298, 0x00DD,
	0x229A, 0x2244,
	0x229C, 0x001C,
	0x229E, 0x00DE,
	0x22A0, 0x005B,
	0x22A2, 0x0519,
	0x22A4, 0x0150,
	0x22A6, 0x0091,
	0x22A8, 0x00D5,
	0x22AA, 0x0040,
	0x22AC, 0x0393,
	0x22AE, 0x0356,
	0x22B0, 0x5021,
	0x22B2, 0x700D,
	0x22B4, 0x2F99,
	0x22B6, 0x2318,
	0x22B8, 0x005C,
	0x22BA, 0x0006,
	0x22BC, 0x0016,
	0x22BE, 0x425A,
	0x22C0, 0x0012,
	0x22C2, 0x0008,
	0x22C4, 0x0403,
	0x22C6, 0x01C8,
	0x22C8, 0x2123,
	0x22CA, 0x0046,
	0x22CC, 0x095D,
	0x22CE, 0x00DE,
	0x22D0, 0x00DD,
	0x22D2, 0x00DC,
	0x22D4, 0x00DE,
	0x22D6, 0x04D6,
	0x22D8, 0x5041,
	0x22DA, 0x704F,
	0x22DC, 0x2F99,
	0x22DE, 0x7000,
	0x22E0, 0x1702,
	0x22E2, 0x202C,
	0x22E4, 0x0016,
	0x22E6, 0x5060,
	0x22E8, 0x2122,
	0x22EA, 0x7800,
	0x22EC, 0xC08C,
	0x22EE, 0x0001,
	0x22F0, 0x903B,
	0x22F2, 0x121C,
	0x22F4, 0x9034,
	0x22F6, 0x1218,
	0x22F8, 0x8DC0,
	0x22FA, 0x01C0,
	0x22FC, 0x0000,
	0x22FE, 0xB1EC,
	0x2300, 0x0000,
	0x2302, 0xB1EC,
	0x2304, 0xB25E,
	0x2306, 0x0002,
	0x2308, 0x0000,
	0x230A, 0xB276,
	0x230C, 0x0000,
	0x230E, 0xB276,
	0x2310, 0xB2EC,
	0x2312, 0x0002,
	0x2314, 0xB2FC,
	0x2316, 0xB308,
	0x2318, 0xFCE0,
	0x231A, 0x0040,
	0x231C, 0x0040,
	0x231E, 0x0040,
	0x2320, 0x0040,
	0x2322, 0x0040,
	0x2324, 0x0042,
	0x2326, 0x005A,
	0x2328, 0x005B,
	0x232A, 0x005C,
	0x232C, 0x005E,
	0x232E, 0x0060,
	0x2330, 0x0064,
	0x2332, 0x0066,
	0x2334, 0x006B,
	0x2336, 0x006F,
	0x2338, 0x0073,
	0x233A, 0x0077,
	0x233C, 0x007B,
	0x233E, 0x0081,
	0x2340, 0x0085,
	0x2342, 0x0089,
	0x2344, 0x008D,
	0x2346, 0x0091,
	0x2348, 0x0095,
	0x234A, 0x0099,
	0x234C, 0x009D,
	0x234E, 0x00A7,
	0x2350, 0x00AA,
	0x2352, 0x00B2,
	0x2354, 0x00B6,
	0x2356, 0x00B9,
	0x2358, 0x00B9,
	0x235A, 0x0040,
	0x235C, 0x0040,
	0x235E, 0x0040,
	0x2360, 0x0040,
	0x2362, 0x0040,
	0x2364, 0x0040,
	0x2366, 0x0054,
	0x2368, 0x0056,
	0x236A, 0x005A,
	0x236C, 0x005D,
	0x236E, 0x0062,
	0x2370, 0x0066,
	0x2372, 0x006B,
	0x2374, 0x0074,
	0x2376, 0x007B,
	0x2378, 0x007E,
	0x237A, 0x0085,
	0x237C, 0x008C,
	0x237E, 0x0095,
	0x2380, 0x009B,
	0x2382, 0x00A2,
	0x2384, 0x00A8,
	0x2386, 0x00B0,
	0x2388, 0x00B4,
	0x238A, 0x00BE,
	0x238C, 0x00C9,
	0x238E, 0x00D0,
	0x2390, 0x00D8,
	0x2392, 0x00DD,
	0x2394, 0x00EA,
	0x2396, 0x00ED,
	0x2398, 0x00F2,
	0x239A, 0x03FE,
	0x239C, 0x0000,
	0x239E, 0x0000,
	0x23A0, 0x0000,
	0x23A2, 0x0000,
	0x23A4, 0x03FE,
	0x23A6, 0x0000,
	0x23A8, 0x0000,
	0x23AA, 0x0000,
	0x23AC, 0x0000,
	0x23AE, 0x03FE,
	0x23B0, 0x0000,
	0x23B2, 0x0000,
	0x23B4, 0x0000,
	0x23B6, 0x0000,
	0x23B8, 0x03FE,
	0x23BA, 0x0000,
	0x23BC, 0x0000,
	0x23BE, 0x0000,
	0x23C0, 0x0000,
	0x0262, 0x0600,
	0x026A, 0xFFFF,
	0x026C, 0x00FF,
	0x026E, 0x0000,
	0x0360, 0x0E8E,
	0x0400, 0x0A10,
	0x040C, 0x01EB,
	0x0600, 0x1112,
	0x0602, 0x3112,
	0x0604, 0x8008,
	0x0644, 0x07FE,
	0x0676, 0x07FF,
	0x0678, 0x0002,
	0x06A8, 0x0240,
	0x06AA, 0x00CA,
	0x06AC, 0x0041,
	0x06AE, 0x03FC,
	0x06B4, 0x3FFF,
	0x06E2, 0xFF00,
	0x052A, 0x0000,
	0x052C, 0x0000,
	0x0F06, 0x0002,
	0x1102, 0x0008,
	0x1106, 0x0124,
	0x11C2, 0x0400,
	0x0902, 0x0003,
	0x0904, 0x0003,
	0x0912, 0x0303,
	0x0914, 0x0300,
	0x0A04, 0xB4C5,
	0x0A06, 0xC400,
	0x0A08, 0xA881,
	0x0A0E, 0xFEC0,
	0x0A12, 0x0000,
	0x0A18, 0x0010,
	0x0A1E, 0x0013,
	0x0A20, 0x0015,
	0x0C00, 0x0021,
	0x0C16, 0x0002,
	0x0708, 0x6F80,
	0x070C, 0x0000,
	0x0780, 0x010E,
	0x1202, 0x1E00,
	0x1204, 0xD700,
	0x1210, 0x8028,
	0x1216, 0xA0A0,
	0x1218, 0x00A0,
	0x121A, 0x0000,
	0x121C, 0x4128,
	0x121E, 0x0000,
	0x1220, 0x0000,
	0x1222, 0x28FA,
	0x100C, 0xB000,
	0x105C, 0x0F0B,
	0x1960, 0x03FE,
	0x196A, 0x03FE,
	0x1974, 0x03FE,
	0x197E, 0x03FE,
	0x1986, 0x0000,
	0x19C6, 0x0000,
	0x1A06, 0x0000,
	0x1A46, 0x0000,
	0x1958, 0x0041,
	0x195A, 0x008F,
	0x195C, 0x00C4,
	0x195E, 0x0288,
	0x1962, 0x0041,
	0x1964, 0x008F,
	0x1966, 0x00C4,
	0x1968, 0x0288,
	0x196C, 0x0041,
	0x196E, 0x008F,
	0x1970, 0x00C4,
	0x1972, 0x0288,
	0x1976, 0x0041,
	0x1978, 0x008F,
	0x197A, 0x00C4,
	0x197C, 0x0288,
	0x1980, 0x0045,
	0x1982, 0x002B,
	0x1984, 0x2015,
	0x1988, 0x0076,
	0x198A, 0x0000,
	0x198C, 0x082C,
	0x198E, 0x0000,
	0x1990, 0x2A17,
	0x1992, 0x0000,
	0x1994, 0x0000,
	0x1996, 0x0000,
	0x19C0, 0x0045,
	0x19C2, 0x002B,
	0x19C4, 0x2015,
	0x19C8, 0x0076,
	0x19CA, 0x0000,
	0x19CC, 0x082C,
	0x19CE, 0x0000,
	0x19D0, 0x2A17,
	0x19D2, 0x0000,
	0x19D4, 0x0000,
	0x19D6, 0x0000,
	0x1A00, 0x0045,
	0x1A02, 0x002B,
	0x1A04, 0x2015,
	0x1A08, 0x0076,
	0x1A0A, 0x0000,
	0x1A0C, 0x082C,
	0x1A0E, 0x0000,
	0x1A10, 0x2A17,
	0x1A12, 0x0000,
	0x1A14, 0x0000,
	0x1A16, 0x0000,
	0x1A40, 0x0045,
	0x1A42, 0x002B,
	0x1A44, 0x2015,
	0x1A48, 0x0076,
	0x1A4A, 0x0000,
	0x1A4C, 0x082C,
	0x1A4E, 0x0000,
	0x1A50, 0x2A17,
	0x1A52, 0x0000,
	0x1A54, 0x0000,
	0x1A56, 0x0000,
	0x19BC, 0x2000,
	0x19FC, 0x2000,
	0x1A3C, 0x2000,
	0x1A7C, 0x2000,
	0x361C, 0x0000,
	0x027E, 0x0100,
};
#endif

static void sensor_init(void)
{
#if MULTI_WRITE
	hi1634_table_write_cmos_sensor(
		addr_data_pair_init_hi1634,
		sizeof(addr_data_pair_init_hi1634) /
		sizeof(kal_uint16));
#else 
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_hi1634[] = {
	0x0B00, 0x0000,
	0x0204, 0x0200,
	0x0206, 0x02C6,
	0x020A, 0x0EA7,
	0x020E, 0x0EAB,
	0x0224, 0x0034,
	0x022A, 0x0015,
	0x022C, 0x0E25,
	0x022E, 0x0DD9,
	0x0234, 0x3311,
	0x0236, 0x3311,
	0x0238, 0x3311,
	0x023A, 0x2222,
	0x0268, 0x0108,
	0x0404, 0x0008,
	0x0406, 0x1244,
	0x0440, 0x011D,
	0x0D28, 0x0008,
	0x0D2A, 0x1247,
	0x0524, 0x5858,
	0x0526, 0x5858,
	0x0F00, 0x0400,
	0x0F04, 0x0008,
	0x0B04, 0x00FC,
	0x0B12, 0x0910,
	0x0B14, 0x06D0,
	0x0B20, 0x0200,
	0x1100, 0x1100,
	0x1108, 0x0002,
	0x1116, 0x0000,
	0x1118, 0x0004,
	0x0A0A, 0x8388,
	0x0A10, 0xB440,
	0x0C14, 0x0010,
	0x0C18, 0x1220,
	0x0C1A, 0x0700,
	0x0736, 0x0050,
	0x0738, 0x0002,
	0x073C, 0x0700,
	0x0746, 0x00D4,
	0x0748, 0x0002,
	0x074A, 0x0900,
	0x074C, 0x0100,
	0x074E, 0x0100,
	0x1200, 0x0946,
	0x1000, 0x0300,
	0x1002, 0xC311,
	0x1004, 0x2BB0,
	0x1010, 0x06CB,
	0x1012, 0x0092,
	0x1014, 0x0020,
	0x1016, 0x0020,
	0x101A, 0x0020,
	0x1020, 0xC107,
	0x1022, 0x071D,
	0x1024, 0x0307,
	0x1026, 0x080B,
	0x1028, 0x1209,
	0x102A, 0x0C0A,
	0x102C, 0x1500,
	0x1038, 0x0000,
	0x103E, 0x0101,
	0x1042, 0x0008,
	0x1044, 0x0120,
	0x1046, 0x01B0,
	0x1048, 0x0090,
	0x1066, 0x06EE,
	0x1600, 0x0400,
	0x1608, 0x0020,
	0x160A, 0x1200,
	0x160C, 0x001A,
	0x160E, 0x0D80,
};
#endif

static void preview_setting(void)
{
#if MULTI_WRITE
	hi1634_table_write_cmos_sensor(
		addr_data_pair_preview_hi1634,
		sizeof(addr_data_pair_preview_hi1634) /
		sizeof(kal_uint16));
#else

#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_30fps_hi1634[] = {
	0x0B00, 0x0000,
	0x0204, 0x0000,
	0x0206, 0x02C6,	//line_length_pck   
	0x020A, 0x0EA7,	//coarse_integ_time 
	0x020E, 0x0EAB,	//frame_length_lines
	0x0224, 0x0036,
	0x022A, 0x0017,
	0x022C, 0x0E1B,
	0x022E, 0x0DD9,
	0x0234, 0x1111,
	0x0236, 0x1111,
	0x0238, 0x1111,
	0x023A, 0x1111,
	0x0268, 0x0108,
	0x0404, 0x0008,
	0x0406, 0x1244,
	0x0440, 0x011D,
	0x0D28, 0x0008,
	0x0D2A, 0x1247,
	0x0524, 0x5858,
	0x0526, 0x5858,
	0x0F00, 0x0000,
	0x0F04, 0x0010,
	0x0B04, 0x00DC,
	0x0B12, 0x1220,	//x_output_size
	0x0B14, 0x0DA0,	//y_output_size
	0x0B20, 0x0100,
	0x1100, 0x1100,
	0x1108, 0x0002,
	0x1116, 0x0000,
	0x1118, 0x0006,
	0x0A0A, 0x8388,
	0x0A10, 0xB040,
	0x0C14, 0x0010,
	0x0C18, 0x1220,
	0x0C1A, 0x0E00,
	0x0736, 0x0050,
	0x0738, 0x0002,
	0x073C, 0x0700,
	0x0746, 0x00D4,
	0x0748, 0x0002,
	0x074A, 0x0900,
	0x074C, 0x0000,
	0x074E, 0x0100,
	0x1200, 0x0946,
	0x1000, 0x0300,
	0x1002, 0xC311,
	0x1004, 0x2BB0,
	0x1010, 0x0DBB,
	0x1012, 0x013E,
	0x1014, 0x0020,
	0x1016, 0x0020,
	0x101A, 0x0020,
	0x1020, 0xC10D,
	0x1022, 0x0D38,
	0x1024, 0x050D,
	0x1026, 0x1012,
	0x1028, 0x1C10,
	0x102A, 0x170A,
	0x102C, 0x2800,
	0x1038, 0x0000,
	0x103E, 0x0001,
	0x1042, 0x0008,
	0x1044, 0x0120,
	0x1046, 0x01B0,
	0x1048, 0x0090,
	0x1066, 0x0E01,
	0x1600, 0x0000,
	0x1608, 0x0020,
	0x160A, 0x1200,
	0x160C, 0x001A,
	0x160E, 0x0D80,
};
#if 1
kal_uint16 addr_data_pair_capture_15fps_hi1634[] = {
	
	
	};
#endif
#endif

static void capture_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
	if (currefps == 300) {

		hi1634_table_write_cmos_sensor(
			addr_data_pair_capture_30fps_hi1634,
			sizeof(addr_data_pair_capture_30fps_hi1634) /
			sizeof(kal_uint16));
	}
#if 1	
	else {
		hi1634_table_write_cmos_sensor(
			addr_data_pair_capture_15fps_hi1634,
			sizeof(addr_data_pair_capture_15fps_hi1634) /
			sizeof(kal_uint16));	
	}
#endif
#else
  if( currefps == 300) {



  } else	{
	  // PIP

	}
#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_video_hi1634[] = {
	0x0B00, 0x0000,
	0x0204, 0x0000,
	0x0206, 0x02C6,	//line_length_pck   
	0x020A, 0x0EA7,	//coarse_integ_time 
	0x020E, 0x0EAB,	//frame_length_lines
	0x0224, 0x0036,
	0x022A, 0x0017,
	0x022C, 0x0E1B,
	0x022E, 0x0DD9,
	0x0234, 0x1111,
	0x0236, 0x1111,
	0x0238, 0x1111,
	0x023A, 0x1111,
	0x0268, 0x0108,
	0x0404, 0x0008,
	0x0406, 0x1244,
	0x0440, 0x011D,
	0x0D28, 0x0008,
	0x0D2A, 0x1247,
	0x0524, 0x5858,
	0x0526, 0x5858,
	0x0F00, 0x0000,
	0x0F04, 0x0010,
	0x0B04, 0x00DC,
	0x0B12, 0x1220,	//x_output_size
	0x0B14, 0x0DA0,	//y_output_size
	0x0B20, 0x0100,
	0x1100, 0x1100,
	0x1108, 0x0002,
	0x1116, 0x0000,
	0x1118, 0x0006,
	0x0A0A, 0x8388,
	0x0A10, 0xB040,
	0x0C14, 0x0010,
	0x0C18, 0x1220,
	0x0C1A, 0x0E00,
	0x0736, 0x0050,
	0x0738, 0x0002,
	0x073C, 0x0700,
	0x0746, 0x00D4,
	0x0748, 0x0002,
	0x074A, 0x0900,
	0x074C, 0x0000,
	0x074E, 0x0100,
	0x1200, 0x0946,
	0x1000, 0x0300,
	0x1002, 0xC311,
	0x1004, 0x2BB0,
	0x1010, 0x0DBB,
	0x1012, 0x013E,
	0x1014, 0x0020,
	0x1016, 0x0020,
	0x101A, 0x0020,
	0x1020, 0xC10D,
	0x1022, 0x0D38,
	0x1024, 0x050D,
	0x1026, 0x1012,
	0x1028, 0x1C10,
	0x102A, 0x170A,
	0x102C, 0x2800,
	0x1038, 0x0000,
	0x103E, 0x0001,
	0x1042, 0x0008,
	0x1044, 0x0120,
	0x1046, 0x01B0,
	0x1048, 0x0090,
	0x1066, 0x0E01,
	0x1600, 0x0000,
	0x1608, 0x0020,
	0x160A, 0x1200,
	0x160C, 0x001A,
	0x160E, 0x0D80,
};
#endif

static void video_setting(void)
{
#if MULTI_WRITE
	hi1634_table_write_cmos_sensor(
		addr_data_pair_video_hi1634,
		sizeof(addr_data_pair_video_hi1634) /
		sizeof(kal_uint16));
#else

#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_hi1634[] = {
	0x0B00, 0x0000,
	0x0204, 0x0001,
	0x0206, 0x02D5,
	0x020A, 0x0478,
	0x020E, 0x047C,
	0x0224, 0x02CA,
	0x022A, 0x0017,
	0x022C, 0x0E31,
	0x022E, 0x0B43,
	0x0234, 0x1111,
	0x0236, 0x3333,
	0x0238, 0x3333,
	0x023A, 0x1133,
	0x0268, 0x0141,
	0x0404, 0x0190,
	0x0406, 0x10BC,
	0x0440, 0x0070,
	0x0D28, 0x0190,
	0x0D2A, 0x10BF,
	0x0524, 0x6060,
	0x0526, 0x6060,
	0x0F00, 0x0800,
	0x0F04, 0x0008,
	0x0B04, 0x00FC,
	0x0B12, 0x0500,
	0x0B14, 0x02D0,
	0x0B20, 0x0300,
	0x1100, 0x1100,
	0x1108, 0x0000,
	0x1116, 0x0218,
	0x1118, 0x0452,
	0x0A0A, 0x838A,
	0x0A10, 0xAC70,
	0x0C14, 0x0018,
	0x0C18, 0x0F00,
	0x0C1A, 0x0300,
	0x0736, 0x0064,
	0x0738, 0x0002,
	0x073C, 0x0700,
	0x0746, 0x00D4,
	0x0748, 0x0002,
	0x074A, 0x0900,
	0x074C, 0x0200,
	0x074E, 0x0100,
	0x1200, 0x0946,
	0x1000, 0x0300,
	0x1002, 0xC311,
	0x1004, 0x2BB0,
	0x1010, 0x039A,
	0x1012, 0x003A,
	0x1014, 0x0020,
	0x1016, 0x0020,
	0x101A, 0x0020,
	0x1020, 0xC105,
	0x1022, 0x0514,
	0x1024, 0x0305,
	0x1026, 0x0608,
	0x1028, 0x0E07,
	0x102A, 0x0805,
	0x102C, 0x0F00,
	0x1038, 0x0000,
	0x103E, 0x0201,
	0x1042, 0x0008,
	0x1044, 0x0120,
	0x1046, 0x01B0,
	0x1048, 0x0090,
	0x1066, 0x03B2,
	0x1600, 0x0000,
	0x1608, 0x0020,
	0x160A, 0x1200,
	0x160C, 0x001A,
	0x160E, 0x0D80,
};
#endif

static void hs_video_setting(void)
{

#if MULTI_WRITE
	hi1634_table_write_cmos_sensor(
		addr_data_pair_hs_video_hi1634,
		sizeof(addr_data_pair_hs_video_hi1634) /
		sizeof(kal_uint16));
#else
// setting ver5.0 None PD 640x480 120fps

#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_hi1634[] = {
	0x0B00, 0x0000,
	0x0204, 0x0201,
	0x0206, 0x02D5,
	0x020A, 0x11F1,
	0x020E, 0x11F5,
	0x0224, 0x02CC,
	0x022A, 0x0015,
	0x022C, 0x0E25,
	0x022E, 0x0B41,
	0x0234, 0x3311,
	0x0236, 0x3311,
	0x0238, 0x3311,
	0x023A, 0x2222,
	0x0268, 0x0141,
	0x0404, 0x0198,
	0x0406, 0x10B4,
	0x0440, 0x00E6,
	0x0D28, 0x0198,
	0x0D2A, 0x10B7,
	0x0524, 0x6060,
	0x0526, 0x6060,
	0x0F00, 0x0400,
	0x0F04, 0x0008,
	0x0B04, 0x00FC,
	0x0B12, 0x0780,
	0x0B14, 0x0438,
	0x0B20, 0x0200,
	0x1100, 0x1100,
	0x1108, 0x0000,
	0x1116, 0x021A,
	0x1118, 0x0454,
	0x0A0A, 0x838A,
	0x0A10, 0xB070,
	0x0C14, 0x0010,
	0x0C18, 0x0F00,
	0x0C1A, 0x0480,
	0x0736, 0x0064,
	0x0738, 0x0002,
	0x073C, 0x0700,
	0x0746, 0x00D4,
	0x0748, 0x0002,
	0x074A, 0x0900,
	0x074C, 0x0100,
	0x074E, 0x0100,
	0x1200, 0x0946,
	0x1000, 0x0300,
	0x1002, 0xC311,
	0x1004, 0x2BB0,
	0x1010, 0x0573,
	0x1012, 0x0063,
	0x1014, 0x0020,
	0x1016, 0x0020,
	0x101A, 0x0020,
	0x1020, 0xC107,
	0x1022, 0x071D,
	0x1024, 0x0307,
	0x1026, 0x080B,
	0x1028, 0x1209,
	0x102A, 0x0C0A,
	0x102C, 0x1500,
	0x1038, 0x0000,
	0x103E, 0x0101,
	0x1042, 0x0008,
	0x1044, 0x0120,
	0x1046, 0x01B0,
	0x1048, 0x0090,
	0x1066, 0x0596,
	0x1600, 0x0400,
	0x1608, 0x0020,
	0x160A, 0x1200,
	0x160C, 0x001A,
	0x160E, 0x0D80,
};
#endif


static void slim_video_setting(void)
{

#if MULTI_WRITE
	hi1634_table_write_cmos_sensor(
		addr_data_pair_slim_video_hi1634,
		sizeof(addr_data_pair_slim_video_hi1634) /
		sizeof(kal_uint16));
#else
	


#endif
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
			LOG_INF("i2c write id : 0x%x, sensor id: 0x%x\n",
			imgsensor.i2c_write_id, *sensor_id);
			return ERROR_NONE;
			}

			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		LOG_INF("Read id fail,sensor id: 0x%x\n", *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	LOG_INF("[open]: PLATFORM:MT6737,MIPI 24LANE\n");
	LOG_INF("preview 1296*972@30fps,360Mbps/lane;"
		"capture 2592*1944@30fps,880Mbps/lane\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		LOG_INF("open sensor id fail: 0x%x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();
	LOG_INF("SK hynix Hi-1634 Sensor Init\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */
static kal_uint32 close(void)
{
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
	 //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;

}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	video_setting();

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_INFO_STRUCT *sensor_info,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType =
	imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =	imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent =
		imgsensor_info.isp_driving_current;
/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame =
		imgsensor_info.ae_shut_delay_frame;
/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine =
		imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum =
		imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber =
		imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;    // 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

#if ENABLE_PDAF
	//sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;
	sensor_info->PDAF_Support = 1;
	//sensor_info->PDAF_Support = 3;
#endif

//	sensor_info->SensorMIPIDeskew = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
	break;
	default:
	    sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	}

	return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		LOG_INF("[odin]video preview\n");
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("[odin]default mode\n");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable,
			UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n",
				scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
	    frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
				imgsensor_info.cap1.max_framerate) {
		frame_length = imgsensor_info.cap1.pclk / framerate * 10 /
				imgsensor_info.cap1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length >
			imgsensor_info.cap1.framelength) ?
			(frame_length - imgsensor_info.cap1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate)
			LOG_INF("fps %d fps not support,use cap: %d fps!\n",
			framerate, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk /
				framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length >
				imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    frame_length = imgsensor_info.hs_video.pclk /
			framerate * 10 / imgsensor_info.hs_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.hs_video.framelength) ? (frame_length -
			imgsensor_info.hs_video.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    frame_length = imgsensor_info.slim_video.pclk /
			framerate * 10 / imgsensor_info.slim_video.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.slim_video.framelength) ? (frame_length -
			imgsensor_info.slim_video.framelength) : 0;
	    imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	default:  //coding with  preview scenario by default
	    frame_length = imgsensor_info.pre.pclk / framerate * 10 /
						imgsensor_info.pre.linelength;
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.dummy_line = (frame_length >
			imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
	    imgsensor.frame_length = imgsensor_info.pre.framelength +
				imgsensor.dummy_line;
	    imgsensor.min_frame_length = imgsensor.frame_length;
	    spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	    LOG_INF("error scenario_id = %d, we use preview scenario\n",
				scenario_id);
	break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
				enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    *framerate = imgsensor_info.pre.max_framerate;
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    *framerate = imgsensor_info.normal_video.max_framerate;
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    *framerate = imgsensor_info.cap.max_framerate;
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
	    *framerate = imgsensor_info.hs_video.max_framerate;
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
	    *framerate = imgsensor_info.slim_video.max_framerate;
	break;
	default:
	break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("set_test_pattern_mode enable: %d", enable);
	if (enable) {
//		write_cmos_sensor(0x1200, 0x08FF);
		write_cmos_sensor(0x0b04, 0x00D9);
		write_cmos_sensor(0x0C0A, 0x0204);

	} else {
//		write_cmos_sensor(0x1200, 0x09FF);
		write_cmos_sensor(0x0b04, 0x00DC);
		write_cmos_sensor(0x0C0A, 0x0000);

	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0b00, 0x0100); // stream on
	else
		write_cmos_sensor(0x0b00, 0x0000); // stream off

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(
			MSDK_SENSOR_FEATURE_ENUM feature_id,
			UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;

#if ENABLE_PDAF
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif

	unsigned long long *feature_data =
		(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	    set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
	    set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
	    write_cmos_sensor(sensor_reg_data->RegAddr,
						sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
	    sensor_reg_data->RegData =
				read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	    set_video_mode(*feature_data);
	break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    get_imgsensor_id(feature_return_para_32);
	break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
	    set_auto_flicker_mode((BOOL)*feature_data_16,
			*(feature_data_16+1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	    set_max_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data+1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	    get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
	break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
	    set_test_pattern_mode((BOOL)*feature_data);
	break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	    *feature_return_para_32 = imgsensor_info.checksum_value;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				(UINT32)*feature_data);

	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		break;
		}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
	    LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
	#if 0
	    ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
	#endif
	break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 20;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;

#if ENABLE_PDAF

		case SENSOR_FEATURE_GET_VC_INFO:
				LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
				pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
				switch (*feature_data_32) 
				{
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO CAPTURE_JPEG\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO VIDEO PREVIEW\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
					case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					default:
						LOG_INF("SENSOR_FEATURE_GET_VC_INFO DEFAULT_PREVIEW\n");
						memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
						break;
				}
				break;

		case SENSOR_FEATURE_GET_PDAF_DATA:
/*		LOG_INF("odin GET_PDAF_DATA EEPROM\n");
		// read from e2prom
#if e2prom
		read_eeprom((kal_uint16)(*feature_data), 
				(char *)(uintptr_t)(*(feature_data+1)), 
				(kal_uint32)(*(feature_data+2)) );
#else
		// read from file

	        LOG_INF("READ PDCAL DATA\n");
		read_hi1634_eeprom((kal_uint16)(*feature_data), 
				(char *)(uintptr_t)(*(feature_data+1)), 
				(kal_uint32)(*(feature_data+2)) );

#endif*/
		break;
		case SENSOR_FEATURE_GET_PDAF_INFO:
			PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));  
			switch( *feature_data) 
			{
		 		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(struct SET_PD_BLOCK_INFO_T));
					break;

		 		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 		//case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		 		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		 		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	 	 		default:
					break;
			}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		//PDAF capacity enable or not, 2p8 only full size support PDAF
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // type2 - VC enable
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		break;

	case SENSOR_FEATURE_SET_PDAF:
			 	imgsensor.pdaf_mode = *feature_data_16;
	        	LOG_INF("[odin] pdaf mode : %d \n", imgsensor.pdaf_mode);
				break;
	
#endif

#if 1 
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
#endif		
	default:
	break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI1634_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	/*	HI1634_MIPI_RAW_SensorInit	*/
