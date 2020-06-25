/*
 * AXI display controller DRM driver
 * based on Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2020 Antmicro Ltd <www.antmicro.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_edid.h>

#include "axi_disp_drv.h"
#include "axi_disp_encoder.h"
#include "axi_disp_ctrl.h"

uint8_t samsung_edid[] =\
{0x00 ,0xff ,0xff ,0xff ,0xff ,0xff ,0xff ,0x00 ,0x4c ,0x2d ,0x20 ,0x0a ,0x31 ,0x38 ,0x32 ,0x30\
,0x30 ,0x17 ,0x01 ,0x03 ,0x80 ,0x30 ,0x1b ,0x78 ,0x2a ,0x90 ,0xc1 ,0xa2 ,0x59 ,0x55 ,0x9c ,0x27\
,0x0e ,0x50 ,0x54 ,0xbf ,0xef ,0x80 ,0x71 ,0x4f ,0x81 ,0xc0 ,0x81 ,0x00 ,0x81 ,0x80 ,0x95 ,0x00\
,0xa9 ,0xc0 ,0xb3 ,0x00 ,0x01 ,0x01 ,0x02 ,0x3a ,0x80 ,0x18 ,0x71 ,0x38 ,0x2d ,0x40 ,0x58 ,0x2c\
,0x45 ,0x00 ,0xdd ,0x0c ,0x11 ,0x00 ,0x00 ,0x1e ,0x01 ,0x1d ,0x00 ,0x72 ,0x51 ,0xd0 ,0x1e ,0x20\
,0x6e ,0x28 ,0x55 ,0x00 ,0xdd ,0x0c ,0x11 ,0x00 ,0x00 ,0x1e ,0x00 ,0x00 ,0x00 ,0xfd ,0x00 ,0x32\
,0x4b ,0x1e ,0x51 ,0x11 ,0x00 ,0x0a ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x00 ,0x00 ,0x00 ,0xfc\
,0x00 ,0x53 ,0x32 ,0x32 ,0x43 ,0x33 ,0x30 ,0x30 ,0x0a ,0x20 ,0x20 ,0x20 ,0x20 ,0x20 ,0x01 ,0xb1\
,0x02 ,0x03 ,0x11 ,0xb1 ,0x46 ,0x90 ,0x04 ,0x1f ,0x13 ,0x12 ,0x03 ,0x65 ,0x03 ,0x0c ,0x00 ,0x10\
,0x00 ,0x01 ,0x1d ,0x00 ,0xbc ,0x52 ,0xd0 ,0x1e ,0x20 ,0xb8 ,0x28 ,0x55 ,0x40 ,0xdd ,0x0c ,0x11\
,0x00 ,0x00 ,0x1e ,0x8c ,0x0a ,0xd0 ,0x90 ,0x20 ,0x40 ,0x31 ,0x20 ,0x0c ,0x40 ,0x55 ,0x00 ,0xdd\
,0x0c ,0x11 ,0x00 ,0x00 ,0x18 ,0x8c ,0x0a ,0xd0 ,0x8a ,0x20 ,0xe0 ,0x2d ,0x10 ,0x10 ,0x3e ,0x96\
,0x00 ,0xdd ,0x0c ,0x11 ,0x00 ,0x00 ,0x18 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00\
,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00\
,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00\
,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x50 };

static unsigned int axi_dispctrl_clk_divider(unsigned int divide);
static unsigned int axi_dispctrl_clk_count_calc(unsigned int divide);
static void axi_dispctrl_encoder_find_clock_parms(unsigned int freq,
						  struct clk_mode *best_pick);

struct axi_dispctrl_encoder {
	struct drm_encoder_slave encoder;
	struct drm_connector connector;
};

static inline struct
axi_dispctrl_encoder *to_axi_dispctrl_encoder(struct drm_encoder *enc)
{
	return container_of(enc, struct axi_dispctrl_encoder, encoder.base);
}

static inline struct
drm_encoder *connector_to_encoder(struct drm_connector *connector)
{
	struct axi_dispctrl_encoder *enc;

	enc  = container_of(connector, struct axi_dispctrl_encoder, connector);
	return &enc->encoder.base;
}


static int axi_dispctrl_connector_init(struct drm_device *dev,
				       struct drm_connector *connector,
				       struct drm_encoder *encoder);

static void axi_dispctrl_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct axi_dispctrl_private *private = encoder->dev->dev_private;
	uint32_t reg;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/* start the display */
		axi_dispctrl_enable(private);
		break;
	default:
		/* stop the display */
		axi_dispctrl_disable(private);
		break;
	}
}

static bool
axi_dispctrl_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	return true;
}

unsigned int axi_dispctrl_clk_count_calc(unsigned int divide)
{
	unsigned int output = 0;
	unsigned int divCalc = 0;

	divCalc = axi_dispctrl_clk_divider(divide);
	if (divCalc == ERR_CLKDIVIDER)
		output = ERR_CLKCOUNTCALC;
	else
		output = (0xFFF & divCalc) | ((divCalc << 10) & 0x00C00000);
	return output;
}


static unsigned int axi_dispctrl_clk_divider(unsigned int divide)
{
	unsigned int output = 0;
	unsigned int highTime = 0;
	unsigned int lowTime = 0;

	if ((divide < 1) || (divide > 128))
		return ERR_CLKDIVIDER;

	if (divide == 1)
		return 0x1041;

	highTime = divide / 2;
	if (divide & 0x1) {
		lowTime = highTime + 1;
		output = 1 << CLK_BIT_WEDGE;
	} else {
		lowTime = highTime;
	}

	output |= 0x03F & lowTime;
	output |= 0xFC0 & (highTime << 6);
	return output;
}

static unsigned int axi_dispctrl_clk_find_reg(struct clk_config *reg_values,
					      struct clk_mode *clk_params)
{
	if ((clk_params->fbmult < 2) || clk_params->fbmult > 64)
		return 0;

	reg_values->clk0L = axi_dispctrl_clk_count_calc(clk_params->clkdiv);
	if (reg_values->clk0L == ERR_CLKCOUNTCALC)
		return 0;

	reg_values->clkFBL = axi_dispctrl_clk_count_calc(clk_params->fbmult);
	if (reg_values->clkFBL == ERR_CLKCOUNTCALC)
		return 0;

	reg_values->clkFBH_clk0H = 0;

	reg_values->divclk = axi_dispctrl_clk_divider(clk_params->maindiv);
	if (reg_values->divclk == ERR_CLKDIVIDER)
		return 0;

	reg_values->lockL =
	(unsigned int)((lock_lookup[clk_params->fbmult - 1]) & 0xFFFFFFFF);

	reg_values->fltr_lockH =
	(unsigned int)((lock_lookup[clk_params->fbmult - 1]>>32) & 0x000000FF);
	reg_values->fltr_lockH |=
	((filter_lookup_low[clk_params->fbmult - 1] << 16) & 0x03FF0000);

	return 1;
}

static void axi_dispctrl_encoder_find_clock_parms(unsigned int freq,
						  struct clk_mode *best_pick)
{
	unsigned int div;
	unsigned int currFreq = 0;
	unsigned int minFb, maxFb;
	unsigned int currMult;
	unsigned int clkDiv = 1;
	int currErr = 0x7fffffff;
	int err;

	for (div = 1; div < 10; div++) {
		minFb = div * 6;
		maxFb = div * 12;
		if (maxFb > 64)
			maxFb = 64;
		currMult = minFb;
		while (currMult <= maxFb) {
			for (clkDiv = 1; clkDiv < 128; clkDiv++) {
				currFreq = (IN_FREQ * currMult);
				currFreq /= (div * clkDiv);
				err = freq - currFreq;
				/* get abs val */
				if (err < 0)
					err = -err;
				if (err < currErr) {
					currErr = err;
					best_pick->clkdiv = clkDiv;
					best_pick->fbmult = currMult;
					best_pick->maindiv = div;
					best_pick->freq = currFreq;
				}
				if (currFreq == freq)
					goto ret;
			}
			currMult++;
		}
	}
ret:
	return;
}


static void
axi_dispctrl_encoder_mode_set(struct drm_encoder *encoder,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode)
{
	struct axi_dispctrl_private *private = encoder->dev->dev_private;
	axi_dispctrl_init(private);
}

static void axi_dispctrl_encoder_commit(struct drm_encoder *encoder)
{
	axi_dispctrl_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void axi_dispctrl_encoder_prepare(struct drm_encoder *encoder)
{
	axi_dispctrl_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static struct
drm_crtc *axi_dispctrl_encoder_get_crtc(struct drm_encoder *encoder)
{
	return encoder->crtc;
}

static struct drm_encoder_helper_funcs axi_dispctrl_encoder_helper_funcs = {
	.dpms		= axi_dispctrl_encoder_dpms,
	.mode_fixup	= axi_dispctrl_encoder_mode_fixup,
	.mode_set	= axi_dispctrl_encoder_mode_set,
	.prepare	= axi_dispctrl_encoder_prepare,
	.commit		= axi_dispctrl_encoder_commit,
	.get_crtc	= axi_dispctrl_encoder_get_crtc,
};

static void axi_dispctrl_encoder_destroy(struct drm_encoder *encoder)
{
	struct axi_dispctrl_encoder *axi_dispctrl_encoder;

	axi_dispctrl_encoder = to_axi_dispctrl_encoder(encoder);
	drm_encoder_cleanup(encoder);
	encoder->dev->mode_config.num_encoder--;
	kfree(axi_dispctrl_encoder);
}

static struct drm_encoder_funcs axi_dispctrl_encoder_funcs = {
	.destroy = axi_dispctrl_encoder_destroy,
};

struct drm_encoder *axi_dispctrl_encoder_create(struct drm_device *dev)
{
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct axi_dispctrl_encoder *axi_dispctrl_encoder;

	axi_dispctrl_encoder = kzalloc(sizeof(*axi_dispctrl_encoder),
				       GFP_KERNEL);
	if (!axi_dispctrl_encoder) {
		return NULL;
	}

	encoder = &axi_dispctrl_encoder->encoder.base;
	encoder->possible_crtcs = 1;

	drm_encoder_init(dev, encoder, &axi_dispctrl_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);
	drm_encoder_helper_add(encoder, &axi_dispctrl_encoder_helper_funcs);

	connector = &axi_dispctrl_encoder->connector;
	axi_dispctrl_connector_init(dev, connector, encoder);

	return encoder;
}

static int axi_dispctrl_connector_get_modes(struct drm_connector *connector)
{
	struct axi_dispctrl_private *private = connector->dev->dev_private;
	struct drm_display_mode *mode;
	struct drm_encoder *encoder = connector_to_encoder(connector);
	int count = 0;
	struct edid  *edid;
	/* If we are in lcd mode use fixed modes */
	if (private->lcd_mode) {
		mode = drm_mode_duplicate(connector->dev,
					  private->lcd_fixed_mode);
		if (!mode)
			return 0;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}
	/* Get edid otherwise */
	else {
		edid = (struct edid *) samsung_edid;
		if (!edid) {
			dev_err(encoder->dev->dev, "Failed to read edid\n");
			return 0;
		}
		drm_mode_connector_update_edid_property(connector, edid);
		count = drm_add_edid_modes(connector, edid);
	}
	return count;
}

static int axi_dispctrl_connector_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;


	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static struct
drm_encoder *axi_dispctrl_best_encoder(struct drm_connector *connector)
{
	return connector_to_encoder(connector);
}

static struct
drm_connector_helper_funcs axi_dispctrl_connector_helper_funcs = {
	.get_modes	= axi_dispctrl_connector_get_modes,
	.mode_valid	= axi_dispctrl_connector_mode_valid,
	.best_encoder	= axi_dispctrl_best_encoder,
};

static enum
drm_connector_status axi_dispctrl_connector_detect(struct drm_connector *con,
						   bool force)
{
	enum drm_connector_status status = connector_status_connected;

	/*XXX: add connection detect
	Always connected?*/
	return status;
}

static void axi_dispctrl_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs axi_dispctrl_connector_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.detect		= axi_dispctrl_connector_detect,
	.destroy	= axi_dispctrl_connector_destroy,
};

static int axi_dispctrl_connector_init(struct drm_device *dev,
		struct drm_connector *connector, struct drm_encoder *encoder)
{
	int type;
	int err;
	struct axi_dispctrl_private *private = dev->dev_private;

	if (private->lcd_mode)
		type = DRM_MODE_CONNECTOR_VGA;
	else
		type = DRM_MODE_CONNECTOR_HDMIA;
	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	err = drm_connector_init(dev, connector,
			   &axi_dispctrl_connector_funcs, type);
	if (err) {
		goto err_connector;
	}

	drm_connector_helper_add(connector,
				 &axi_dispctrl_connector_helper_funcs);

	err = drm_connector_register(connector);
	if (err) {
		goto err_connector;
	}

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		dev_err(dev->dev,
			"failed to attach a connector to a encoder\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	drm_connector_unregister(connector);
err_connector:
	drm_connector_cleanup(connector);
	return err;
}
