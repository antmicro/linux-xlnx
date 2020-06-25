#ifndef _AXI_DISPCTRL_ENCODER_H_
#define _AXI_DISPCTRL_ENCODER_H_

struct drm_encoder *axi_dispctrl_encoder_create(struct drm_device *dev);

#define IN_FREQ 100000000

#define BIT_DISPLAY_START 0
#define BIT_DISPLAY_INVERT_PIX_CLOCK 1

#define OFST_DISPLAY_CTRL 0x0
#define OFST_DISPLAY_STATUS 0x4
#define OFST_DISPLAY_VIDEO_START 0x8
#define OFST_DISPLAY_CLK_L 0x1C
#define OFST_DISPLAY_FB_L 0x20
#define OFST_DISPLAY_FB_H_CLK_H 0x24
#define OFST_DISPLAY_DIV 0x28
#define OFST_DISPLAY_LOCK_L 0x2C
#define OFST_DISPLAY_FLTR_LOCK_H 0x30
#define OFST_DISPLAY_DEBUG 0x4c

struct clk_mode {
	unsigned int freq;
	unsigned int fbmult;
	unsigned int clkdiv;
	unsigned int maindiv;
};

struct clk_config {
	unsigned int clk0L;
	unsigned int clkFBL;
	unsigned int clkFBH_clk0H;
	unsigned int divclk;
	unsigned int lockL;
	unsigned int fltr_lockH;
};


#define CLK_BIT_WEDGE 13
#define CLK_BIT_NOCOUNT 12
#define ERR_CLKDIVIDER (1 << CLK_BIT_WEDGE | 1 << CLK_BIT_NOCOUNT)
#define ERR_CLKCOUNTCALC 0xFFFFFFFF

extern uint8_t samsung_edid[];
extern const uint64_t lock_lookup[];
extern const uint32_t filter_lookup_low[];

#endif
