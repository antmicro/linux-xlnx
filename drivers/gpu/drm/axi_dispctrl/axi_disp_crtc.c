/*
 * AXI display controller DRM driver
 *
 * Copyright 2020 Antmicro Ltd <www.antmicro.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>

#include "axi_disp_crtc.h"
#include "axi_disp_drv.h"
#include "axi_disp_encoder.h"
#include "fast-vdma.h"


struct axi_dispctrl_crtc {
	struct drm_crtc drm_crtc;
	struct dma_chan *dma;
	struct fast_vdma_priv *dma_priv;
	struct xilinx_vdma_config dma_config;
	struct transfer_config conf;
	int mode;
};

static inline struct
axi_dispctrl_crtc *to_axi_dispctrl_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_dispctrl_crtc, drm_crtc);
}

void fast_vdma_setup_chan(void* base, u32 src, u32 dst, u32 width, u32 height, u32 stride);

static int axi_dispctrl_crtc_update(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct dma_async_tx_descriptor *desc;
	struct drm_gem_cma_object *obj;
	size_t offset;
	int ret;
	struct axi_dispctrl_private *priv;
	struct fast_vdma_priv *dma_priv;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);

	if (!mode || !fb)
		return -EINVAL;

	priv = axi_dispctrl_crtc->drm_crtc.dev->dev_private;
	dma_priv = priv->dma_priv;

	if (axi_dispctrl_crtc->mode == DRM_MODE_DPMS_OFF)
		dma_priv->ops->stop(priv->dma_priv);

	if (axi_dispctrl_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;

		offset = 0;

		/* configure DMA */
		dma_priv->cfg.width = mode->hdisplay / 2;
		dma_priv->cfg.height = mode->vdisplay;

		dma_priv->cfg.stride = 0;
		dma_priv->cfg.src_buf = obj->paddr + offset;
		dma_priv->cfg.dst_buf = 0;

		fast_vdma_setup_chan(dma_priv->dev.base_mm2s, dma_priv->cfg.src_buf, 0, dma_priv->cfg.width, dma_priv->cfg.height, dma_priv->cfg.stride);

		dma_priv->ops->start(priv->dma_priv);

		ret = desc ? 0 : -ENOMEM;
	}
	return ret;
}

static void axi_dispctrl_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	if (axi_dispctrl_crtc->mode != mode) {
		axi_dispctrl_crtc->mode = mode;
		axi_dispctrl_crtc_update(crtc);
	}
}

static void axi_dispctrl_crtc_prepare(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct fast_vdma_priv *dma_priv;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	dma_priv = axi_dispctrl_crtc->dma_priv;

	dma_priv->ops->stop(dma_priv);

}

static void axi_dispctrl_crtc_commit(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc = to_axi_dispctrl_crtc(crtc);
	axi_dispctrl_crtc->mode = DRM_MODE_DPMS_ON;
	axi_dispctrl_crtc_update(crtc);
}

static bool axi_dispctrl_crtc_mode_fixup(struct drm_crtc *crtc,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int axi_dispctrl_crtc_mode_set(struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode,
				      int x, int y,
				      struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int axi_dispctrl_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					   struct drm_framebuffer *old_fb)
{
	return axi_dispctrl_crtc_update(crtc);
}

static void axi_dispctrl_atomic_enable(struct drm_crtc *crtc,
				       struct drm_crtc_state *old_state)
{

}

static void axi_dispctrl_atomic_disable(struct drm_crtc *crtc,
				       struct drm_crtc_state *old_state)
{
}

static struct drm_crtc_helper_funcs axi_dispctrl_crtc_helper_funcs = {
	.dpms		= axi_dispctrl_crtc_dpms,
	.prepare	= axi_dispctrl_crtc_prepare,
	.commit		= axi_dispctrl_crtc_commit,
	.mode_fixup	= axi_dispctrl_crtc_mode_fixup,
	.mode_set	= axi_dispctrl_crtc_mode_set,
	.mode_set_base	= axi_dispctrl_crtc_mode_set_base,
};

static void axi_dispctrl_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;

	axi_dispctrl_crtc  = to_axi_dispctrl_crtc(crtc);
	drm_crtc_cleanup(crtc);
	kfree(axi_dispctrl_crtc);
}

static struct drm_crtc_funcs axi_dispctrl_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_dispctrl_crtc_destroy,
};

struct drm_crtc *axi_dispctrl_crtc_create(struct drm_device *dev)
{
	struct axi_dispctrl_private *p = dev->dev_private;
	struct axi_dispctrl_crtc *axi_dispctrl_crtc;
	struct drm_crtc *crtc;

	axi_dispctrl_crtc = kzalloc(sizeof(*axi_dispctrl_crtc), GFP_KERNEL);
	if (!axi_dispctrl_crtc) {
		return ERR_PTR(-ENOMEM);
	}

	crtc = &axi_dispctrl_crtc->drm_crtc;

	axi_dispctrl_crtc->dma_priv = p->dma_priv;

	drm_crtc_init(dev, crtc, &axi_dispctrl_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_dispctrl_crtc_helper_funcs);

	return crtc;
}
