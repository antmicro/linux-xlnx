#ifndef _ANT_DISPCTRL_DRV_H_
#define _ANT_DISPCTRL_DRV_H_

#include <drm/drm.h>
#include <drm/drm_fb_cma_helper.h>
#include <linux/of.h>
#include <linux/clk.h>

struct axi_dispctrl_private {
	struct drm_device *drm_dev;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc *crtc;
	void __iomem *base;
	void __iomem *drp_base;
	struct dma_chan *dma;
	bool lcd_mode;
	bool invert_pix_clk;
	struct drm_display_mode *lcd_fixed_mode;
	struct i2c_client *encoder_slave;
	struct i2c_adapter *i2c_adapter;

	struct fast_vdma_priv *dma_priv;
};

extern struct of_device_id displays_of_match[];
#endif
