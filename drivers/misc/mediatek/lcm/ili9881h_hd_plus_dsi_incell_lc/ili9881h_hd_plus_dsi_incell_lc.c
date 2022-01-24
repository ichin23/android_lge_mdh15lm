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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#endif

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

#include "ddp_hal.h"
#include "ddp_path.h"
#include "lcd_bias.h"
#include "disp_recovery.h"
#include "mt6370_pmu_dsv.h"
#include <linux/lcd_power_mode.h>
#include <soc/mediatek/lge/board_lge.h>
#include <linux/lge_panel_notify.h>
#define LCM_ID (0x80)

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;
static bool flag_is_panel_deep_sleep = false;
static bool flag_deep_sleep_ctrl_available = false;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE								0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT										(1560)

#define LCM_PHYSICAL_WIDTH									(69120)
#define LCM_PHYSICAL_HEIGHT									(149760)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static struct LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 150, {} }
};

static struct LCM_setting_table init_setting_vdo[] = {
//GIP timing
{0xFF,03,{0x98,0x81,0x01}},
{0x00,01,{0x55}},
{0x01,01,{0x2E}},
{0x02,01,{0x00}},//10
{0x03,01,{0x00}},

{0x04,01,{0x03}},
{0x05,01,{0x25}},
{0x06,01,{0x00}},
{0x07,01,{0x00}},

{0x08,01,{0x84}},
{0x09,01,{0x85}},//DUMMT CK
{0x0A,01,{0xF5}},
{0x0B,01,{0x00}},//clk keep 10 off 00  10
{0x0C,01,{0x06}},//01
{0x0D,01,{0x06}},//01
{0x0E,01,{0x00}},//10
{0x0F,01,{0x00}},//10

{0x10,01,{0x00}},
{0x11,01,{0x00}},
{0x12,01,{0x00}},

{0x14,01,{0x87}},
{0x15,01,{0x87}},
{0x16,01,{0x84}},
{0x17,01,{0x85}},
{0x18,01,{0x75}},
{0x19,01,{0x00}},
{0x1A,01,{0x06}},
{0x1B,01,{0x06}},
{0x1C,01,{0x00}},
{0x1D,01,{0x00}},
{0x1E,01,{0x00}},
{0x1F,01,{0x00}},
{0x20,01,{0x00}},
{0x22,01,{0x87}},

{0x23,01,{0x87}},
{0x2A,01,{0x8B}},
{0x2B,01,{0x4E}},

//FW
{0x31,01,{0x2A}},//STV_C
{0x32,01,{0x2A}},
{0x33,01,{0x0C}},//FW
{0x34,01,{0x0C}},//BW
{0x35,01,{0x23}},//VGL
{0x36,01,{0x23}},
{0x37,01,{0x2A}},
{0x38,01,{0x2A}},
{0x39,01,{0x10}},
{0x3A,01,{0x07}},//CLK8
{0x3B,01,{0x18}},//CLK6
{0x3C,01,{0x12}},
{0x3D,01,{0x1A}},
{0x3E,01,{0x14}},//CLK4
{0x3F,01,{0x1C}},//CLK2
{0x40,01,{0x16}},//STV_A
{0x41,01,{0x1E}},
{0x42,01,{0x08}},
{0x43,01,{0x08}},
{0x44,01,{0x2A}},
{0x45,01,{0x2A}},
{0x46,01,{0x2A}},

{0x47,01,{0x2A}},//STV_C
{0x48,01,{0x2A}},
{0x49,01,{0x0D}},//FW
{0x4A,01,{0x0D}},//BW
{0x4B,01,{0x23}},//VGL
{0x4C,01,{0x23}},
{0x4D,01,{0x2A}},
{0x4E,01,{0x2A}},
{0x4F,01,{0x11}},
{0x50,01,{0x07}},//CLK7
{0x51,01,{0x19}},//CLK5
{0x52,01,{0x13}},
{0x53,01,{0x1B}},
{0x54,01,{0x15}},//CLK3
{0x55,01,{0x1D}},//CLK1
{0x56,01,{0x17}},//STV_A
{0x57,01,{0x1F}},
{0x58,01,{0x09}},
{0x59,01,{0x09}},
{0x5A,01,{0x2A}},
{0x5B,01,{0x2A}},
{0x5C,01,{0x2A}},

//BW
{0x61,01,{0x2A}},
{0x62,01,{0x2A}},
{0x63,01,{0x09}},
{0x64,01,{0x09}},
{0x65,01,{0x2A}},
{0x66,01,{0x2A}},
{0x67,01,{0x23}},
{0x68,01,{0x23}},
{0x69,01,{0x17}},
{0x6A,01,{0x07}},
{0x6B,01,{0x1F}},
{0x6C,01,{0x15}},
{0x6D,01,{0x1D}},
{0x6E,01,{0x13}},
{0x6F,01,{0x1B}},
{0x70,01,{0x11}},
{0x71,01,{0x19}},
{0x72,01,{0x0D}},
{0x73,01,{0x0D}},
{0x74,01,{0x2A}},
{0x75,01,{0x2A}},
{0x76,01,{0x2A}},

{0x77,01,{0x2A}},
{0x78,01,{0x2A}},
{0x79,01,{0x08}},
{0x7A,01,{0x08}},
{0x7B,01,{0x2A}},
{0x7C,01,{0x2A}},
{0x7D,01,{0x23}},
{0x7E,01,{0x23}},
{0x7F,01,{0x16}},
{0x80,01,{0x07}},
{0x81,01,{0x1E}},
{0x82,01,{0x14}},
{0x83,01,{0x1C}},
{0x84,01,{0x12}},
{0x85,01,{0x1A}},
{0x86,01,{0x10}},
{0x87,01,{0x18}},
{0x88,01,{0x0C}},
{0x89,01,{0x0C}},
{0x8A,01,{0x2A}},
{0x8B,01,{0x2A}},
{0x8C,01,{0x2A}},

{0xB9,01,{0x10}},
{0xC3,01,{0x00}},
{0xC4,01,{0x80}},

{0xD3,01,{0x20}},
{0xDD,01,{0x20}},

{0xD1,01,{0x23}},
{0xD5,01,{0x05}},
{0xD6,01,{0x91}},
{0xD7,01,{0x01}},
{0xD8,01,{0x15}},
{0xD9,01,{0x55}},
{0xDA,01,{0x65}},

{0xE2,01,{0x55}},
{0xE6,01,{0x45}},

{0xFF,03,{0x98,0x81,0x02}},
{0x4B,01,{0x5A}},
{0x4D,01,{0x4E}},
{0x4E,01,{0x00}},
{0x1A,01,{0x48}},
{0x06,01,{0x90}},

// GVDDP GVDDN VCOM VGH VGHO VGL VGLO setup
{0xFF,03,{0x98,0x81,0x05}},
{0x03,01,{0x00}},//VCOM
{0x04,01,{0xB0}},//VCOM
{0x58,01,{0x62}},//VGL x2
{0x63,01,{0x88}},//GVDDN   -5.14V
{0x64,01,{0x88}},//GVDDP    5.14V
{0x68,01,{0x79}},//VGHO   15V
{0x69,01,{0x7F}},//VGH    16V
{0x6A,01,{0x79}},//VGLO  79  -10V    -12V
{0x6B,01,{0x6B}},//VGL   6B -11V   -13V

// Resolution 720RGB*1560
{0xFF,03,{0x98,0x81,0x06}},
{0x2E,01,{0x01}},//NL enable
{0xC0,01,{0x0B}},
{0xC1,01,{0x03}},
//frame target 60Hz
{0x11,01,{0x03}},
{0x13,01,{0x45}},
{0x14,01,{0x41}},
{0x15,01,{0xF1}},
{0x16,01,{0x40}},
{0x17,01,{0xFF}},
{0x18,01,{0x00}},

{0xC2,01,{0x04}},
{0x27,01,{0xFF}},
{0x28,01,{0x20}},
{0x48,01,{0x0F}},
{0x4D,01,{0x80}},
{0x4E,01,{0x40}},
{0x7F,01,{0x78}},
{0xD6,01,{0x85}},//FTE=TSVD1,0x FTE1=TSHD

{0xFF,03,{0x98,0x81,0x06}},
{0x12,01,{0x00}},
{0x94,01,{0x01}},

/* enable ESD */
{0xFF,03,{0x98,0x81,0x06}},
{0xC7,01,{0x05}}, //1 bit ESD check(0Ah)

{0xFF,03,{0x98,0x81,0x08}},	  //0809
{0xE0,27,{0x40,0x1A,0x96,0xCB,0x09,0x55,0x3C,0x61,0x8E,0xB0,0xA9,0xE5,0x0F,0x35,0x59,0xAA,0x7F,0xAE,0xCC,0xF4,0xFF,0x16,0x41,0x73,0x97,0x03,0xE6}},
{0xE1,27,{0x40,0x6D,0x96,0xCB,0x09,0x55,0x3C,0x61,0x8E,0xB0,0xA9,0xE5,0x0F,0x35,0x59,0xAA,0x7F,0xAE,0xCC,0xF4,0xFF,0x16,0x41,0x73,0x97,0x03,0x93}},


{0xFF,03,{0x98,0x81,0x0E}},
{0x00,01,{0xA0}},//LV mode
{0x01,01,{0x26}},//LV mode
{0x13,01,{0x10}}, // THSD time updata 0703 0F

{0xFF,03,{0x98,0x81,0x04}},
{0x00,01,{0x07}},//Hue enable
{0x02,01,{0x45}},//CE & Hue 24 axis enable

{0x0A,01,{0x00}},//0 Red
{0x0B,01,{0x00}},//1
{0x0C,01,{0x00}},//2
{0x0D,01,{0x21}},//3
{0x0E,01,{0x22}},//4 Yellow
{0x0F,01,{0x21}},//5
{0x10,01,{0x00}},//6
{0x11,01,{0x00}},//7
{0x12,01,{0x00}},//8
{0x13,01,{0x00}},//9
{0x14,01,{0x00}},//10
{0x15,01,{0x00}},//11
{0x16,01,{0x00}},//12
{0x17,01,{0x00}},//13
{0x18,01,{0x00}},//14
{0x19,01,{0x00}},//15
{0x1A,01,{0x00}},//16
{0x1B,01,{0x00}},//17
{0x1C,01,{0x00}},//18
{0x1D,01,{0x00}},//19
{0x1E,01,{0x00}},//20
{0x1F,01,{0x00}},//21
{0x20,01,{0x00}},//22
{0x21,01,{0x00}},//23

{0x3C,01,{0x00}},//0 Red
{0x3D,01,{0x00}},//1
{0x3E,01,{0x00}},//2
{0x3F,01,{0x11}},//3
{0x40,01,{0x12}},//4 Yellow
{0x41,01,{0x11}},//5
{0x42,01,{0x00}},//6
{0x43,01,{0x00}},//7
{0x44,01,{0x00}},//8
{0x45,01,{0x00}},//9
{0x46,01,{0x00}},//10
{0x47,01,{0x00}},//11
{0x48,01,{0x00}},//12
{0x49,01,{0x00}},//13
{0x4A,01,{0x00}},//14
{0x4B,01,{0x00}},//15
{0x4C,01,{0x00}},//16
{0x4D,01,{0x00}},//17
{0x4E,01,{0x00}},//18
{0x4F,01,{0x00}},//19
{0x50,01,{0x00}},//20
{0x51,01,{0x00}},//21
{0x52,01,{0x00}},//22
{0x53,01,{0x00}},//23

{0xFF,03,{0x98,0x81,0x00}},//Page0

{0x11,00,{}},
{REGFLAG_DELAY,120,{}},
{0x29,00,{}},
{REGFLAG_DELAY,100,{}},
{0x35,01,{0x00}},//TE enable

};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	//params->physical_width_um = LCM_PHYSICAL_WIDTH;
	//params->physical_height_um = LCM_PHYSICAL_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	//lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	//params->dsi.switch_mode = CMD_MODE;
	//lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
	//params->dsi.mode   = SYNC_PULSE_VDO_MODE;	//SYNC_EVENT_VDO_MODE
#endif
	//LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 240;
	//params->dsi.vertical_frontporch_for_low_power = 540;/*disable dynamic frame rate*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
        params->dsi.horizontal_backporch = 28;
        params->dsi.horizontal_frontporch = 28;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_range = 4;
	params->dsi.ssc_disable = 1;
	/*params->dsi.ssc_disable = 1;*/
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 87;	/* this value must be in MTK suggested table */
#else
        params->dsi.PLL_CLOCK = 285;    /* this value must be in MTK suggested table */
#endif
	//params->dsi.PLL_CK_CMD = 220;
	//params->dsi.PLL_CK_VDO = 255;
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	params->lcm_seq_suspend = LCM_MIPI_VIDEO_FRAME_DONE_SUSPEND;
	params->lcm_seq_shutdown = LCM_SHUTDOWN_AFTER_DSI_OFF;
	params->lcm_seq_resume = LCM_MIPI_READY_VIDEO_FRAME_RESUME;
	params->lcm_seq_power_on = NOT_USE_RESUME;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 32;
#endif
//	params->use_gpioID = 1;
//	params->pioID_value = 8;
}
static void lcd_reset_pin(unsigned int mode)
{
    lcd_bias_set_gpio_ctrl(LCM_RST, mode);

    LCM_PRINT("[LCD] LCD Reset %s \n",(mode)? "High":"Low");
}
static void lcm_reset_ctrl(unsigned int enable, unsigned int delay)
{
    if(enable) {
        lcd_reset_pin(enable);
    }
    else
	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_LOW);

    if(delay)
        MDELAY(delay);

    if(enable)
	lge_panel_notifier_call_chain(LGE_PANEL_EVENT_RESET, 0, LGE_PANEL_RESET_HIGH);
    else
        lcd_reset_pin(enable);

    LCM_PRINT("[LCD] %s\n",__func__);
}


#ifdef BUILD_LK
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
   mt_set_gpio_mode(GPIO, GPIO_MODE_00);
   mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}
#endif
static void lcm_init(void)
{
	lcm_reset_ctrl(0, 5);
	MDELAY(5);
	lcm_reset_ctrl(1, 5);
	MDELAY(55);
	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_suspend(void)
{
	//pr_debug("[LCM]lcm_suspend\n");

	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	flag_deep_sleep_ctrl_available = true;
    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume(void)
{
	//pr_debug("[LCM]lcm_resume\n");
	flag_deep_sleep_ctrl_available = false;

	if(flag_is_panel_deep_sleep) {
		flag_is_panel_deep_sleep = false;
		LCM_PRINT("[LCD] %s : deep sleep mode state. call lcm_init(). \n", __func__);
	}
	lcm_init();

	LCM_PRINT("[LCD] %s\n", __func__);
}

static void lcm_enter_deep_sleep(void)
{
	if(flag_deep_sleep_ctrl_available) {
		flag_is_panel_deep_sleep = true;
		LCM_PRINT("[LCD] %s\n", __func__);
	}
}

static void lcm_exit_deep_sleep(void)
{
	if(flag_deep_sleep_ctrl_available) {
		lcd_reset_pin(0);
		MDELAY(2);
		lcd_reset_pin(1);
		MDELAY(10);

		flag_is_panel_deep_sleep = false;
		LCM_PRINT("[LCD] %s\n", __func__);
	}
}

static void lcm_set_deep_sleep(unsigned int mode)
{
	switch (mode){
	case DEEP_SLEEP_ENTER:
		lcm_enter_deep_sleep();
		break;
	case DEEP_SLEEP_EXIT:
		lcm_exit_deep_sleep();
		break;
	default :
		break;
	}

	LCM_PRINT("[LCD] %s : %d \n", __func__,  mode);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	if (buffer[0] != 0x9c) {
		LCM_LOGI("[LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_LOGI("[LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043700;	/* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;	/* mode control addr */
		lcm_switch_mode_cmd.val[0] = 0x13;	/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[1] = 0x10;	/* disable video mode secondly */
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0] = 0x03;	/* disable GRAM and enable video mode */
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}

#if (LCM_DSI_CMD_MODE)

/* partial update restrictions:
 * 1. roi width must be 1080 (full lcm width)
 * 2. vertical start (y) must be multiple of 16
 * 3. vertical height (h) must be multiple of 16
 */
static void lcm_validate_roi(int *x, int *y, int *width, int *height)
{
	unsigned int y1 = *y;
	unsigned int y2 = *height + y1 - 1;
	unsigned int x1, w, h;

	x1 = 0;
	w = FRAME_WIDTH;

	y1 = round_down(y1, 16);
	h = y2 - y1 + 1;

	/* in some cases, roi maybe empty. In this case we need to use minimu roi */
	if (h < 16)
		h = 16;

	h = round_up(h, 16);

	/* check height again */
	if (y1 >= FRAME_HEIGHT || y1 + h > FRAME_HEIGHT) {
		/* assign full screen roi */
		LCM_LOGD("%s calc error,assign full roi:y=%d,h=%d\n", __func__, *y, *height);
		y1 = 0;
		h = FRAME_HEIGHT;
	}

	/*LCM_LOGD("lcm_validate_roi (%d,%d,%d,%d) to (%d,%d,%d,%d)\n",*/
	/*	*x, *y, *width, *height, x1, y1, w, h);*/

	*x = x1;
	*width = w;
	*y = y1;
	*height = h;
}
#endif
/*bug 350122 - add white point reading function in lk , houbenzhong.wt, 20180411, begin*/
struct boe_panel_white_point{
	unsigned short int white_x;
	unsigned short int white_y;
};

//#define WHITE_POINT_BASE_X 167
//#define WHITE_POINT_BASE_Y 192
#if (LCM_DSI_CMD_MODE)
struct LCM_DRIVER ili9881h_hd_plus_dsi_incell_lc_lcm_drv = {
	.name = "ili9881h_hd_plus_dsi_incell_lc",
#else

struct LCM_DRIVER ili9881h_hd_plus_dsi_incell_lc_lcm_drv = {
	.name = "MANTIX-ILI9881H",
#endif
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.esd_check = lcm_esd_check,
	.ata_check = lcm_ata_check,
	.switch_mode = lcm_switch_mode,
#if (LCM_DSI_CMD_MODE)
	.validate_roi = lcm_validate_roi,
#endif
	.set_deep_sleep = lcm_set_deep_sleep,

};
