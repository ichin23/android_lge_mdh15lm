#ifdef BUILD_LK
#include <sys/types.h>
#else
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include "disp_dts_gpio.h"
#include "disp_recovery.h"
#include "sm5109.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
    #define LCM_PRINT printf
#else
    #define LCM_PRINT printk
#endif

#ifndef FALSE
  #define FALSE   0
#endif

#ifndef TRUE
  #define TRUE    1
#endif

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
#define FRAME_WIDTH               (720)
#define FRAME_HEIGHT              (1560)
#define LCM_DENSITY               (320)

#define PHYSICAL_WIDTH            (65)
#define PHYSICAL_HEIGHT           (130)

#define LCM_MODULE_NAME    "JD9365D_HLT"

#ifdef BUILD_LK
#define LCM_ID                    (0x9365)
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
static LCM_UTIL_FUNCS lcm_util;
#else
static struct LCM_UTIL_FUNCS lcm_util;
#endif

#define SET_RESET_PIN(v)                                          lcm_util.set_reset_pin((v))
#define UDELAY(n)                                                 lcm_util.udelay(n)
#define MDELAY(n)                                                 lcm_util.mdelay(n)
#define dsi_set_cmdq(pdata, queue_size, force_update)             lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)          lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq_V3(para_tbl,size,force_update)               lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)   lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)                     lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifdef CONFIG_HQ_SET_LCD_BIAS
#define SET_LCD_BIAS_EN(en, seq, value)                           lcm_util.set_lcd_bias_en(en, seq, value)
#endif

extern struct pinctrl *lcd_bias_pctrl; 
extern struct pinctrl_state *lcd_rst_pins0, *lcd_rst_pins1;
extern void disp_set_gpio_ctrl(unsigned int ctrl_pin, unsigned int en);
extern int sm5109_set_vspn_value(unsigned int value);

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned int cmd;

    for (i = 0; i < count; i++) {
        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY:
            if (table[i].count <= 10)
                UDELAY(1000 * table[i].count);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_UDELAY:
            UDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V22(cmdq, cmd,
                table[i].count,
                table[i].para_list,
                force_update);
        }
    }
}

static struct LCM_setting_table lcm_suspend_setting[] = {
    {REGFLAG_DELAY, 5, {} },
    {0x28, 0, {} },
    {REGFLAG_DELAY, 50, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting_vdo[] = {
    {0xE0,0x01,{0x00}},

    {0xE1,0x01,{0x93}},
    {0xE2,0x01,{0x65}},
    {0xE3,0x01,{0xF8}},
    {0x80,0x01,{0x03}},     //DSI 4 lane

    {0xE0,0x01,{0x01}},

    {0x00,0x01,{0x00}},
    {0x01,0x01,{0x35}},
    {0x03,0x01,{0x00}},
    {0x04,0x01,{0x35}},

    {0x17,0x01,{0x00}},
    {0x18,0x01,{0xF7}},
    {0x19,0x01,{0x00}},
    {0x1A,0x01,{0x00}},
    {0x1B,0x01,{0xF7}},
    {0x1C,0x01,{0x00}},

    {0x24,0x01,{0xFE}},
    {0x25,0x01,{0x20}},

    {0x37,0x01,{0x09}},
    {0x38,0x01,{0x04}},
    {0x39,0x01,{0x08}},
    {0x3A,0x01,{0x12}},
    {0x3C,0x01,{0x5C}},
    {0x3D,0x01,{0xFF}},
    {0x3E,0x01,{0xFF}},
    {0x3F,0x01,{0x64}},

    {0x40,0x01,{0x04}},
    {0x41,0x01,{0xC3}},
    {0x42,0x01,{0x6B}},
    {0x43,0x01,{0x12}},
    {0x44,0x01,{0x0F}},
    {0x45,0x01,{0x28}},
    {0x4B,0x01,{0x04}},

    {0x55,0x01,{0x0F}},
    {0x56,0x01,{0x01}},
    {0x57,0x01,{0x65}},
    {0x58,0x01,{0x0A}},
    {0x59,0x01,{0x0A}},
    {0x5A,0x01,{0x29}},
    {0x5B,0x01,{0x10}},

    {0x5D,0x01,{0x6F}},
    {0x5E,0x01,{0x4F}},
    {0x5F,0x01,{0x3C}},
    {0x60,0x01,{0x2F}},
    {0x61,0x01,{0x2A}},
    {0x62,0x01,{0x1B}},
    {0x63,0x01,{0x22}},
    {0x64,0x01,{0x0F}},
    {0x65,0x01,{0x2E}},
    {0x66,0x01,{0x31}},
    {0x67,0x01,{0x35}},
    {0x68,0x01,{0x56}},
    {0x69,0x01,{0x46}},
    {0x6A,0x01,{0x4F}},
    {0x6B,0x01,{0x42}},
    {0x6C,0x01,{0x40}},
    {0x6D,0x01,{0x34}},
    {0x6E,0x01,{0x23}},
    {0x6F,0x01,{0x09}},
    {0x70,0x01,{0x6F}},
    {0x71,0x01,{0x4F}},
    {0x72,0x01,{0x3C}},
    {0x73,0x01,{0x2F}},
    {0x74,0x01,{0x2A}},
    {0x75,0x01,{0x1B}},
    {0x76,0x01,{0x22}},
    {0x77,0x01,{0x0F}},
    {0x78,0x01,{0x2E}},
    {0x79,0x01,{0x31}},
    {0x7A,0x01,{0x35}},
    {0x7B,0x01,{0x56}},
    {0x7C,0x01,{0x46}},
    {0x7D,0x01,{0x4F}},
    {0x7E,0x01,{0x42}},
    {0x7F,0x01,{0x40}},
    {0x80,0x01,{0x34}},
    {0x81,0x01,{0x23}},
    {0x82,0x01,{0x09}},

    {0xE0,0x01,{0x02}},

    {0x00,0x01,{0x5E}},
    {0x01,0x01,{0x5F}},
    {0x02,0x01,{0x57}},
    {0x03,0x01,{0x58}},
    {0x04,0x01,{0x44}},
    {0x05,0x01,{0x46}},
    {0x06,0x01,{0x48}},
    {0x07,0x01,{0x4A}},
    {0x08,0x01,{0x40}},
    {0x09,0x01,{0x1D}},
    {0x0A,0x01,{0x1D}},
    {0x0B,0x01,{0x1D}},
    {0x0C,0x01,{0x1D}},
    {0x0D,0x01,{0x1D}},
    {0x0E,0x01,{0x1D}},
    {0x0F,0x01,{0x50}},
    {0x10,0x01,{0x5F}},
    {0x11,0x01,{0x5F}},
    {0x12,0x01,{0x5F}},
    {0x13,0x01,{0x5F}},
    {0x14,0x01,{0x5F}},
    {0x15,0x01,{0x5F}},

    {0x16,0x01,{0x5E}},
    {0x17,0x01,{0x5F}},
    {0x18,0x01,{0x57}},
    {0x19,0x01,{0x58}},
    {0x1A,0x01,{0x45}},
    {0x1B,0x01,{0x47}},
    {0x1C,0x01,{0x49}},
    {0x1D,0x01,{0x4B}},
    {0x1E,0x01,{0x41}},
    {0x1F,0x01,{0x1D}},
    {0x20,0x01,{0x1D}},
    {0x21,0x01,{0x1D}},
    {0x22,0x01,{0x1D}},
    {0x23,0x01,{0x1D}},
    {0x24,0x01,{0x1D}},
    {0x25,0x01,{0x51}},
    {0x26,0x01,{0x5F}},
    {0x27,0x01,{0x5F}},
    {0x28,0x01,{0x5F}},
    {0x29,0x01,{0x5F}},
    {0x2A,0x01,{0x5F}},
    {0x2B,0x01,{0x5F}},

    {0x2C,0x01,{0x1F}},
    {0x2D,0x01,{0x1E}},
    {0x2E,0x01,{0x17}},
    {0x2F,0x01,{0x18}},
    {0x30,0x01,{0x0B}},
    {0x31,0x01,{0x09}},
    {0x32,0x01,{0x07}},                                                                                          
    {0x33,0x01,{0x05}},
    {0x34,0x01,{0x11}},
    {0x35,0x01,{0x1F}},
    {0x36,0x01,{0x1F}},
    {0x37,0x01,{0x1F}},
    {0x38,0x01,{0x1F}},
    {0x39,0x01,{0x1F}},
    {0x3A,0x01,{0x1F}},
    {0x3B,0x01,{0x01}},
    {0x3C,0x01,{0x1F}},
    {0x3D,0x01,{0x1F}},
    {0x3E,0x01,{0x1F}},
    {0x3F,0x01,{0x1F}},
    {0x40,0x01,{0x1F}},
    {0x41,0x01,{0x1F}},

    {0x42,0x01,{0x1F}},
    {0x43,0x01,{0x1E}},
    {0x44,0x01,{0x17}},
    {0x45,0x01,{0x18}},
    {0x46,0x01,{0x0A}},
    {0x47,0x01,{0x08}},
    {0x48,0x01,{0x06}},
    {0x49,0x01,{0x04}},
    {0x4A,0x01,{0x10}},
    {0x4B,0x01,{0x1F}},
    {0x4C,0x01,{0x1F}},
    {0x4D,0x01,{0x1F}},
    {0x4E,0x01,{0x1F}},
    {0x4F,0x01,{0x1F}},
    {0x50,0x01,{0x1F}},
    {0x51,0x01,{0x00}},
    {0x52,0x01,{0x1F}},
    {0x53,0x01,{0x1F}},
    {0x54,0x01,{0x1F}},
    {0x55,0x01,{0x1F}},
    {0x56,0x01,{0x1F}},
    {0x57,0x01,{0x1F}},

    {0x58,0x01,{0x40}},
    {0x59,0x01,{0x00}},
    {0x5A,0x01,{0x00}},
    {0x5B,0x01,{0x10}},
    {0x5C,0x01,{0x0B}},
    {0x5D,0x01,{0x30}},
    {0x5E,0x01,{0x01}},
    {0x5F,0x01,{0x02}},
    {0x60,0x01,{0x30}},
    {0x61,0x01,{0x03}},
    {0x62,0x01,{0x04}},
    {0x63,0x01,{0x10}},
    {0x64,0x01,{0x52}},
    {0x65,0x01,{0x56}},
    {0x66,0x01,{0x27}},
    {0x67,0x01,{0x73}},
    {0x68,0x01,{0x0D}},
    {0x69,0x01,{0x16}},
    {0x6A,0x01,{0x52}},
    {0x6B,0x01,{0x00}},
    {0x6C,0x01,{0x00}},
    {0x6D,0x01,{0x08}},
    {0x6E,0x01,{0x00}},
    {0x6F,0x01,{0x08}},
    {0x70,0x01,{0x00}},
    {0x71,0x01,{0x00}},
    {0x72,0x01,{0x06}},
    {0x73,0x01,{0x7B}},
    {0x74,0x01,{0x00}},
    {0x75,0x01,{0xBC}},
    {0x76,0x01,{0x00}},
    {0x77,0x01,{0x0E}},
    {0x78,0x01,{0x3C}},
    {0x79,0x01,{0x00}},
    {0x7A,0x01,{0x00}},
    {0x7B,0x01,{0x00}},
    {0x7C,0x01,{0x00}},
    {0x7D,0x01,{0x03}},
    {0x7E,0x01,{0x7B}},

    {0xE0,0x01,{0x04}},
    {0x04,0x01,{0x01}},
    {0x09,0x01,{0x10}},
    {0x0E,0x01,{0x4A}},
    {0x36,0x01,{0x69}},

    {0xE0,0x01,{0x00}},

    {0x11,0x01,{0x00}},  	// SLPOUT
    {REGFLAG_DELAY,120,{}},
    {0x55,0x01,{0x00}},
    {REGFLAG_DELAY,5,{}},
    {0xE0,0x01,{0x03}},
    {0x02,0x01,{0x03}},
    {0x04,0x01,{0x31}},
    {0x12,0x01,{0x1C}},
    {0x13,0x01,{0x1D}},
    {0x14,0x01,{0x1E}},
    {0x15,0x01,{0x1F}},
    {0x16,0x01,{0x1F}},
    {0x17,0x01,{0x1D}},
    {0x18,0x01,{0x1B}},
    {0x19,0x01,{0x19}},
    {0x1A,0x01,{0x1B}},
    {0x1B,0x01,{0x1D}},
    {0x1C,0x01,{0x1F}},
    {0x1D,0x01,{0x1F}},
    {0x1E,0x01,{0x1F}},
    {0x1F,0x01,{0x1F}},
    {0x20,0x01,{0x1A}},
    {0x21,0x01,{0x1C}},
    {0x22,0x01,{0x1F}},
    {0x23,0x01,{0x1F}},
    {0x24,0x01,{0x1F}},
    {0x25,0x01,{0x17}},
    {0x26,0x01,{0x1E}},
    {0x27,0x01,{0x1C}},
    {0x28,0x01,{0x1E}},
    {0x29,0x01,{0x1D}},

    {0x2B,0x01,{0x01}},
    {0x2C,0x01,{0x01}},

    {0x30,0x01,{0x40}},
    {0x31,0x01,{0x00}},
    {0x32,0x01,{0x00}},
    {0x33,0x01,{0x00}},
    {0x34,0x01,{0x00}},
    {0x35,0x01,{0x00}},
    {0x36,0x01,{0x00}},
    {0x37,0x01,{0x00}},
    {0x38,0x01,{0x00}},
    {0x39,0x01,{0xFF}},
    {0x3A,0x01,{0xFF}},
    {0x3B,0x01,{0xFF}},
    {0x3C,0x01,{0xFE}},
    {0x3D,0x01,{0xEF}},
    {0x3E,0x01,{0xEF}},
    {0x3F,0x01,{0xEE}},
    {0x40,0x01,{0xFF}},
    {0x41,0x01,{0xFF}},
    {0x42,0x01,{0xFF}},
    {0x43,0x01,{0xFF}},
    {0x44,0x01,{0xF0}},
    {0x45,0x01,{0x00}},
    {0x46,0x01,{0x00}},
    {0x47,0x01,{0x00}},
    {0x48,0x01,{0x00}},
    {0x49,0x01,{0x00}},
    {0x4A,0x01,{0x11}},
    {0x4B,0x01,{0x11}},
    {0x4C,0x01,{0x11}},
    {0x4D,0x01,{0x11}},
    {0x4E,0x01,{0x21}},
    {0x4F,0x01,{0x11}},
    {0x50,0x01,{0x11}},
    {0x51,0x01,{0x00}},
    {0x52,0x01,{0x00}},
    {0x53,0x01,{0x00}},

    {0x54,0x01,{0x3F}},
    {0x55,0x01,{0xD3}},
    {0x56,0x01,{0x33}},
    {0x57,0x01,{0x33}},
    {0x58,0x01,{0x21}},
    {0x59,0x01,{0x00}},
    {0x5A,0x01,{0x11}},
    {0x5B,0x01,{0x00}},
    {0x5C,0x01,{0x00}},
    {0x5D,0x01,{0x0F}},
    {0x5E,0x01,{0xFF}},
    {0x5F,0x01,{0xEE}},
    {0x60,0x01,{0xFE}},
    {0x61,0x01,{0xEE}},
    {0x62,0x01,{0xDD}},
    {0x63,0x01,{0xDC}},
    {0x64,0x01,{0xDC}},
    {0x65,0x01,{0xCC}},
    {0x66,0x01,{0xDD}},
    {0x67,0x01,{0xDC}},
    {0x68,0x01,{0xCC}},
    {0x69,0x01,{0xBB}},
    {0x6A,0x01,{0xDD}},
    {0x6B,0x01,{0xDC}},
    {0x6C,0x01,{0xCB}},
    {0x6D,0x01,{0xAA}},
    {0x6E,0x01,{0xDD}},
    {0x6F,0x01,{0xCC}},
    {0x70,0x01,{0xDD}},
    {0x71,0x01,{0xDD}},
    {0x72,0x01,{0xEF}},
    {0x73,0x01,{0x00}},
    {0x74,0x01,{0x11}},
    {0x75,0x01,{0x33}},
    {0x76,0x01,{0x44}},
    {0x77,0x01,{0x05}},

    {0xE0,0x01,{0x00}},
    {0x29,0x01,{0x00}},  	// DSPON
    {REGFLAG_DELAY,5,{}},
    
    {0x35,0x01,{0x00}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
#else
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}
#endif

#ifdef BUILD_LK
static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));
#else
static void lcm_get_params(struct LCM_PARAMS *params)
{
    memset(params, 0, sizeof(struct LCM_PARAMS));
#endif

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.mode                       = SYNC_PULSE_VDO_MODE;   //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    params->dsi.data_format.format         = LCM_DSI_FORMAT_RGB888;
    params->dsi.PS                         = LCM_PACKED_PS_24BIT_RGB888;
    params->type                           = LCM_TYPE_DSI;
    params->width                          = FRAME_WIDTH;
    params->height                         = FRAME_HEIGHT;

    // DSI
    params->dsi.LANE_NUM                   = LCM_FOUR_LANE;
    params->dsi.vertical_sync_active                       = 2;
    params->dsi.vertical_backporch                         = 16;
    params->dsi.vertical_frontporch                        = 23;
    params->dsi.vertical_active_line       = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active                     = 30;
    params->dsi.horizontal_backporch                       = 60;
    params->dsi.horizontal_frontporch                      = 60;
    params->dsi.horizontal_active_pixel    = FRAME_WIDTH;
    params->dsi.PLL_CLOCK                                  = 262;

    params->physical_width                 = PHYSICAL_WIDTH;
    params->physical_height                = PHYSICAL_HEIGHT;
#ifndef BUILD_LK
    params->density                        = LCM_DENSITY;
#endif

    params->lcm_seq_suspend = LCM_MIPI_VIDEO_FRAME_DONE_SUSPEND;
    params->lcm_seq_shutdown = LCM_SHUTDOWN_AFTER_DSI_OFF;
    params->lcm_seq_resume = LCM_MIPI_READY_VIDEO_FRAME_RESUME;
    params->lcm_seq_power_on = NOT_USE_RESUME;

    params->esd_powerctrl_support = false;

    // Non-continuous clock
    params->dsi.noncont_clock = TRUE;
    //params->dsi.noncont_clock_period = 2; // Unit : frames
    params->dsi.clk_lp_per_line_enable              = 1;

    params->dsi.ssc_disable = 1;//default enable SSC
    //params->dsi.ssc_range = 5;

    params->dsi.esd_check_enable                    = 1;
    params->dsi.customization_esd_check_enable      = 1;

    params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
}
static void lcm_dsv_ctrl(unsigned int enable, unsigned int delay,unsigned int value)
{
    if(enable==1){
        disp_set_gpio_ctrl(DSV_VPOS_EN, 1);
        sm5109_set_vspn_value(value);

        if(delay)
            MDELAY(delay);

        disp_set_gpio_ctrl(DSV_VNEG_EN, 1);
    } else {
        disp_set_gpio_ctrl(DSV_VNEG_EN, 0);

        if(delay)
            MDELAY(delay);

        disp_set_gpio_ctrl(DSV_VPOS_EN, 0);
    }

    LCM_PRINT("[LCD] %s : %s\n",__func__,(enable)? "ON":"OFF");
}

static void lcd_reset_pin(unsigned int mode)
{
    disp_set_gpio_ctrl(LCM_RST, mode);

    LCM_PRINT("[LCD] LCD Reset %s \n",(mode)? "High":"Low");
}

static void lcm_reset(unsigned int ms1, unsigned int ms2, unsigned int ms3)
{
    if (ms1) {
        //pinctrl_select_state(lcd_bias_pctrl, lcd_rst_pins1);
        //SET_RESET_PIN(1);
	lcd_reset_pin(1);
        MDELAY(ms1);
    }

    if (ms2) {
        //pinctrl_select_state(lcd_bias_pctrl, lcd_rst_pins0);
        //SET_RESET_PIN(0);
	lcd_reset_pin(0);
        MDELAY(ms2);
    }

    if (ms3) {
        //pinctrl_select_state(lcd_bias_pctrl, lcd_rst_pins1);
        //SET_RESET_PIN(1);
	lcd_reset_pin(1);
        MDELAY(ms3);
    }
    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_init(void)
{
#ifdef CONFIG_HQ_SET_LCD_BIAS
    SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5800);  //open lcd bias
    MDELAY(5);
#endif
    lcm_dsv_ctrl(TRUE, 5,5500);
    MDELAY(5);

    lcm_reset(5, 10, 120);

    push_table(NULL, init_setting_vdo, ARRAY_SIZE(init_setting_vdo), 1);

    LCM_PRINT("[LCD] %s\n",__func__);
}



static void lcm_suspend(void)
{

    push_table(NULL, lcm_suspend_setting, ARRAY_SIZE(lcm_suspend_setting), 1);

    lcd_reset_pin(0);
    MDELAY(10);
    lcm_dsv_ctrl(0, 1,5500);
    MDELAY(5);
    LCM_PRINT("[LCD] %s\n",__func__);
}

static void lcm_resume(void)
{

    lcm_init();

    LCM_PRINT("[LCD] %s\n",__func__);
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[4] = {0};
    unsigned int array[16] = {0};

#ifdef CONFIG_HQ_SET_LCD_BIAS
     SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5800);  //open lcd bias
     MDELAY(15);
#endif
     lcm_dsv_ctrl(TRUE, 5,5500);
     MDELAY(5);

    lcm_reset(10, 10, 120);

    array[0] = 0x00023700;  /* read id return two byte,version and id */
    dsi_set_cmdq(array, 1, 1);
    MDELAY(5);

    read_reg_v2(0x04, buffer, 2);
    id = (buffer[0]<<8) | buffer[1] ;

    LCM_PRINT("[%s]:%s, id:0x%x\n", LCM_MODULE_NAME, __func__, id);

    return (LCM_ID == id) ? TRUE : FALSE;
}
#endif

#ifdef BUILD_LK
LCM_DRIVER jd9365d_hdplus_dsi_vdo_hlt_lcm_drv =
#else
struct LCM_DRIVER jd9365d_hdplus_dsi_vdo_hlt_lcm_drv =
#endif
{
    .name           = "HLT-JD9365D",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#ifdef BUILD_LK
    .compare_id     = lcm_compare_id,
#endif
};


