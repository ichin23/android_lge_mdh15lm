/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "upmu_common.h"
#include <linux/string.h>
#endif

#include "lcm_drv.h"
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#if defined(BUILD_LK)
#include <boot_mode.h>
#else
#include <linux/types.h>
#include <upmu_hw.h>
#endif

#include "ddp_hal.h"
#include "ddp_path.h"
#include "disp_recovery.h"
#include "disp_dts_gpio.h"
#include "primary_display.h"
#include <linux/lcd_power_mode.h>
#include <linux/lge_panel_notify.h>
#include <linux/input/lge_touch_notify_oos.h>
#include <soc/mediatek/lge/board_lge.h>
#ifdef CONFIG_MT6370_PMU_DSV
#include "mt6370_pmu_dsv.h"
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

/* pixel */
#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2160)
#define LCM_DENSITY             (320)

/* physical dimension */
#define PHYSICAL_WIDTH          (70)
#define PHYSICAL_HEIGHT         (140)

#define LCM_ID                  (0xb9)
#define LCM_DSI_CMD_MODE        0

#define REGFLAG_DELAY           0xAB
#define REGFLAG_END_OF_TABLE    0xAA   /* END OF REGISTERS MARKER */

#define ENABLE  1
#define DISABLE 0
#define DELAY	0

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static struct LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define UDELAY(n)               (lcm_util.udelay(n))
#define MDELAY(n)               (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V3(para_tbl, size, force_update)\
lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)\
lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)\
lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)\
lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)\
lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)\
lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)\
lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern void dw8768l_ctrl(unsigned int enable);

/* --------------------------------------------------------------------------- */
/* GPIO Set */
/* --------------------------------------------------------------------------- */

//dir out : mode 00 dir in : mode 01

#if 0
#define GPIO_LCD_LDO_EN             (GPIO177 | 0x80000000)
#define GPIO_LCD_LDO_EN_MODE        GPIO_MODE_00

#define GPIO_LCD_RESET_N            (GPIO158 | 0x80000000)
#define GPIO_LCD_RESET_N_MODE       GPIO_MODE_00

#define GPIO_TOUCH_RESET_N          (GPIO10 | 0x80000000)
#define GPIO_TOUCH_RESET_N_MODE     GPIO_MODE_00
#endif

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
  unsigned char cmd;
  unsigned char count;
  unsigned char para_list[64];
};

/* Display Initial Set : V0.91 ( 2018-02-08 )*/
static struct LCM_setting_table_V3 lcm_initial_command[] = {
  {0x39, 0x2B, 4,   {0x00, 0x00, 0x08, 0x6F}},
  {0x15, 0x36, 1,   {0x00}},
  {0x39, 0x44, 2,   {0x05, 0xDC}},
  {0x15, 0x55, 1,   {0x80}},
  {0x15, 0xB0, 1,   {0xAC}},
  {0x39, 0xB1, 5,   {0x36, 0x00, 0x80, 0x14, 0x85}},
  {0x39, 0xB2, 3,   {0x77, 0x04, 0x4C}},
  {0x39, 0xB3, 8,   {0x02, 0x04, 0x0A, 0x00, 0x5C, 0x00, 0x02, 0x12}},
  {0x39, 0xB4, 15,  {0x03, 0x00, 0x78, 0x05, 0x05, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
  {0x39, 0xB5, 18,  {0x02, 0x0D, 0x03, 0x01, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x05, 0x10, 0x10, 0x10,
                     0x10,0x00}},
  {0x39, 0xB6, 7,   {0x00, 0x28, 0x14, 0x5B, 0x04, 0xB6, 0x08}},
  {0x39, 0xB7, 6,   {0x08, 0x50, 0x1B, 0x02, 0x10, 0x8C}},
  {0x39, 0xB8, 31,  {0x07, 0x3C, 0x90, 0x44, 0xA6, 0x00, 0x05, 0x00, 0x00, 0x10, 0x04, 0x04, 0x01, 0x40, 0x01, 0x45,
                     0x1C, 0xC2, 0x21, 0x00, 0x00, 0x10, 0x01, 0x01, 0x01, 0x64, 0x00, 0x58, 0x00, 0x00, 0x00}},
  {0x39, 0xB9, 5,   {0x32, 0x32, 0x2A, 0x37, 0x03}},
  {0x39, 0xC3, 6,   {0x05, 0x06, 0x06, 0x50, 0x66, 0x1B}},
  {0x39, 0xC4, 4,   {0xA2, 0xA4, 0xA4, 0x00}},
  {0x39, 0xC5, 5,   {0x94, 0x44, 0x6C, 0x29, 0x29}},
  {0x39, 0xCA, 6,   {0x05, 0x0D, 0x00, 0x34, 0x0A, 0xFF}},
  {0x39, 0xCB, 4,   {0x3F, 0x0A, 0x80, 0xA0}},
  {0x39, 0xCC, 6,   {0x73, 0x90, 0x55, 0x3D, 0x3D, 0x11}},
  {0x39, 0xCD, 7,   {0x00, 0x40, 0x50, 0x90, 0x00, 0xF3, 0xA0}},
  {0x39, 0xCE, 6,   {0x48, 0x48, 0x24, 0x24, 0x00, 0xAB}},
  {0x39, 0xD0, 126, {0x18, 0x18, 0x27, 0x27, 0x31, 0x31, 0x3C, 0x3C, 0x4A, 0x4A, 0x55, 0x55, 0x73, 0x73, 0x8F, 0x8F,
                     0xA1, 0xA1, 0xB0, 0xB0, 0x7F, 0x7F, 0xB4, 0xB4, 0xA4, 0xA4, 0x91, 0x91, 0x76, 0x76, 0x5B, 0x5B,
                     0x52, 0x52, 0x49, 0x49, 0x40, 0x40, 0x39, 0x39, 0x30, 0x2E, 0x18, 0x18, 0x27, 0x27, 0x31, 0x31,
                     0x3C, 0x3C, 0x4A, 0x4A, 0x56, 0x56, 0x74, 0x74, 0x90, 0x90, 0xA3, 0xA3, 0xB1, 0xB1, 0x80, 0x80,
                     0xB2, 0xB2, 0xA2, 0xA2, 0x8D, 0x8D, 0x71, 0x71, 0x52, 0x52, 0x45, 0x45, 0x3B, 0x3B, 0x2C, 0x2C,
                     0x22, 0x22, 0x1B, 0x19, 0x18, 0x18, 0x1C, 0x1C, 0x25, 0x25, 0x35, 0x35, 0x45, 0x45, 0x54, 0x54,
                     0x71, 0x71, 0x90, 0x90, 0xA3, 0xA3, 0xB1, 0xB1, 0x80, 0x80, 0xB1, 0xB1, 0xA1, 0xA1, 0x8B, 0x8B,
                     0x6E, 0x6E, 0x4B, 0x4B, 0x40, 0x40, 0x36, 0x36, 0x24, 0x24, 0x17, 0x17, 0x02, 0x00}},
  {0x39, 0xD1, 126, {0x18, 0x18, 0x27, 0x27, 0x31, 0x31, 0x3C, 0x3C, 0x4A, 0x4A, 0x55, 0x55, 0x73, 0x73, 0x8F, 0x8F,
                     0xA1, 0xA1, 0xB0, 0xB0, 0x7F, 0x7F, 0xB4, 0xB4, 0xA4, 0xA4, 0x91, 0x91, 0x76, 0x76, 0x5B, 0x5B,
                     0x52, 0x52, 0x49, 0x49, 0x40, 0x40, 0x39, 0x39, 0x30, 0x2E, 0x18, 0x18, 0x27, 0x27, 0x31, 0x31,
                     0x3C, 0x3C, 0x4A, 0x4A, 0x56, 0x56, 0x74, 0x74, 0x90, 0x90, 0xA3, 0xA3, 0xB1, 0xB1, 0x80, 0x80,
                     0xB2, 0xB2, 0xA2, 0xA2, 0x8D, 0x8D, 0x71, 0x71, 0x52, 0x52, 0x45, 0x45, 0x3B, 0x3B, 0x2C, 0x2C,
                     0x22, 0x22, 0x1B, 0x19, 0x18, 0x18, 0x1C, 0x1C, 0x25, 0x25, 0x35, 0x35, 0x45, 0x45, 0x54, 0x54,
                     0x71, 0x71, 0x90, 0x90, 0xA3, 0xA3, 0xB1, 0xB1, 0x80, 0x80, 0xB1, 0xB1, 0xA1, 0xA1, 0x8B, 0x8B,
                     0x6E, 0x6E, 0x4B, 0x4B, 0x40, 0x40, 0x36, 0x36, 0x24, 0x24, 0x17, 0x17, 0x02, 0x00}},
  {0x39, 0xE5, 12,  {0x24, 0x08, 0x06, 0x04, 0x02, 0x0C, 0x0B, 0x0A, 0x21, 0x0F, 0x02, 0x00}},
  {0x39, 0xE6, 12,  {0x24, 0x09, 0x07, 0x05, 0x03, 0x0C, 0x0B, 0x0A, 0x0E, 0x0F, 0x03, 0x01}},
  {0x39, 0xE7, 12,  {0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E}},
  {0x39, 0xE8, 12,  {0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E}},
  {0x39, 0xED, 5,   {0x80, 0x00, 0x01, 0x56, 0x08}},
  {0x15, 0xF0, 1,   {0x82}},
  {0x39, 0xF2, 4,   {0x01, 0x00, 0xC0, 0x00}},
  {0x39, 0xF3, 6,   {0x00, 0x40, 0x80, 0xC0, 0x00, 0x01}},
};

/* Sleep Out Set */
static struct LCM_setting_table_V3 lcm_initial_sleep_out[] = {
  {0x05, 0x11, 1, {0x00}},
};

/* Display On Set */
static struct LCM_setting_table_V3 lcm_initial_disp_on[] = {
  {0x05, 0x29, 1, {0x00}},
};

/* TE Signal on Set */
static struct LCM_setting_table_V3 lcm_initial_te_on[] = {
  {0x15, 0x35, 1, {0x00}},
};

/* Sleep In Set */
static struct LCM_setting_table_V3 lcm_initial_sleep_in[] = {
  {0x05, 0x10, 1, {0x00}},
};

/* Display Off Set */
static struct LCM_setting_table_V3 lcm_initial_display_off[] = {
  {0x05, 0x28, 1, {0x00}},
};

static void init_lcm_registers(void)
{
  dsi_set_cmdq_V3(lcm_initial_command, sizeof(lcm_initial_command) / sizeof(struct LCM_setting_table_V3), 1);
}

static void init_lcm_registers_sleep_in(void)
{
  dsi_set_cmdq_V3(lcm_initial_display_off, sizeof(lcm_initial_display_off) / sizeof(struct LCM_setting_table_V3), 1);

  dsi_set_cmdq_V3(lcm_initial_sleep_in, sizeof(lcm_initial_sleep_in) / sizeof(struct LCM_setting_table_V3), 1);
  MDELAY(140);

  LCM_PRINT("[LCD] %s\n",__func__);
}

static void init_lcm_registers_sleep_out(void)
{
  dsi_set_cmdq_V3(lcm_initial_sleep_out, sizeof(lcm_initial_sleep_out) / sizeof(struct LCM_setting_table_V3), 1);
  MDELAY(100);

  dsi_set_cmdq_V3(lcm_initial_te_on, sizeof(lcm_initial_te_on) / sizeof(struct LCM_setting_table_V3), 1);

  dsi_set_cmdq_V3(lcm_initial_disp_on, sizeof(lcm_initial_disp_on) / sizeof(struct LCM_setting_table_V3), 1);
  MDELAY(5);

  LCM_PRINT("[LCD] %s\n",__func__);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
  memcpy((void *)&lcm_util, (void *)util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
  memset(params, 0, sizeof(struct LCM_PARAMS));

  params->type = LCM_TYPE_DSI;

  params->width = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  /* physical size */
  params->physical_width = PHYSICAL_WIDTH;
  params->physical_height = PHYSICAL_HEIGHT;

  params->dsi.mode = CMD_MODE;
  /* enable tearing-free */
  //params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
  //params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

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

  params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

  /* non-continuous clock */
  params->dsi.cont_clock = 0;

  /* porch value is not necessary in command mode. */
  if(params->dsi.mode != CMD_MODE) {
    params->dsi.vertical_sync_active = 1;
    params->dsi.vertical_backporch = 68;
    params->dsi.vertical_frontporch = 68;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 26;
    params->dsi.horizontal_frontporch = 16;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
  }

  params->dsi.PLL_CLOCK = 510;

#if defined(CONFIG_LGE_DISPLAY_ESD_RECOVERY)
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 0;

  params->dsi.lcm_esd_check_table[0].cmd  = 0x0A;
  params->dsi.lcm_esd_check_table[0].count  = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

  set_disp_esd_check_lcm(true);
#endif
  lcm_dsi_mode = params->dsi.mode;

  LCM_PRINT("[LCD] %s\n",__func__);
}

/* 1.8V VDDI Always ON */
#if 0
static void ldo_1v8io_ctrl(unsigned int enable)
{
    pmic_set_register_value(PMIC_LDO_VCN18_EN, enable);

    LCM_PRINT("[LCD] %s : %s\n",__func__,(enable)? "ON":"OFF");
}
#endif

static void lcm_dsv_ctrl(unsigned int mode)
{
  dw8768l_ctrl(mode);

  LCM_PRINT("[LCD] %s : %s\n",__func__,(mode)? "ON":"OFF");
}

static void lcd_reset_pin(unsigned int mode)
{
  disp_set_gpio_ctrl(LCM_RST, mode);

  LCM_PRINT("[LCD] LCD Reset %s \n",(mode)? "High":"Low");
}

static void touch_reset_pin(unsigned int mode)
{
  disp_set_gpio_ctrl(TOUCH_RST, mode);

  LCM_PRINT("[LCD] Touch Reset %s \n",(mode)? "High":"Low");
}

static void lcm_reset_ctrl(unsigned int mode, unsigned int delay)
{
  if(mode)
      lcd_reset_pin(mode);
  else
      touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_START, NULL);

  if(delay)
      MDELAY(delay);

  if(mode)
      touch_notifier_call_chain(LCD_EVENT_TOUCH_RESET_END, NULL);
  else
      lcd_reset_pin(mode);

  LCM_PRINT("[LCD] %s\n",__func__);
}

#if 0
static void lcm_mipi_lane_ctrl(unsigned int enable)
{
  if(enable) {
      ddp_path_top_clock_on();
      ddp_dsi_power_on(DISP_MODULE_DSI0, NULL);
  } else {
      ddp_dsi_power_off(DISP_MODULE_DSI0, NULL);
      ddp_path_top_clock_off();
  }

  LCM_PRINT("[LCD] %s : %s \n",__func__,(enable)? "UP":"DOWN");
}
#endif

static void lcm_suspend_mfts(void)
{
  LCM_PRINT("[LCD] %s : do nothing \n",__func__);
}

static void lcm_suspend(void)
{
  init_lcm_registers_sleep_in();

  if(primary_get_shutdown_status()) {
      lcd_reset_pin(DISABLE);
      touch_reset_pin(DISABLE);

      MDELAY(2);
  }

  LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume_mfts(void)
{
  lcm_reset_ctrl(DISABLE,DELAY);

  LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume(void)
{
  lcm_reset_ctrl(DISABLE,DELAY);
  MDELAY(5);

  lcm_reset_ctrl(ENABLE,DELAY);
  MDELAY(11);

  init_lcm_registers();
  init_lcm_registers_sleep_out();

  LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_shutdown(void)
{
  MDELAY(2);

  lcm_dsv_ctrl(DISABLE);
  MDELAY(5);

  LCM_PRINT("[LCD] %s\n",__func__);
}

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */

struct LCM_DRIVER sw49107_lgd_fhdplus_dsi_cmd_cv7a_lcm_drv = {
    .name = "LGD-SW49107",
    //.lcm_drv_name = "sw49107_lgd_fhdplus_dsi_cmd_cv7a",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .suspend_mfts = lcm_suspend_mfts,
    .resume_mfts = lcm_resume_mfts,
    .shutdown = lcm_shutdown,
    //.reset_ctrl = lcm_reset_ctrl,
};
