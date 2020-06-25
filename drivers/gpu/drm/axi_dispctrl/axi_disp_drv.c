/*
 * AXI display controller DRM driver
 * based on Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2020 Antmicro Ltd <www.antmicro.com>
 *
 * Licensed under the GPL-2.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "axi_disp_drv.h"
#include "axi_disp_encoder.h"
#include "axi_disp_crtc.h"
#include "fast-vdma.h"

#define DRIVER_NAME "axi_dispctrl_drm"
#define DRIVER_DESC "AXI DISPCTRL DRM"
#define DRIVER_DATE "20200820"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

#define READ_PROPERTY(child, name, field) \
	do { \
		err = of_property_read_u32(child, name, field); \
		if (err) \
			return err; \
	} while (0) \


static void axi_dispctrl_output_poll_changed(struct drm_device *dev)
{
	struct axi_dispctrl_private *private = dev->dev_private;

	drm_fbdev_cma_hotplug_event(private->fbdev);
}

static struct drm_mode_config_funcs axi_dispctrl_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = axi_dispctrl_output_poll_changed,
};

static void axi_dispctrl_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 1920;
	dev->mode_config.max_height = 1080;
	dev->mode_config.funcs = &axi_dispctrl_mode_config_funcs;
}


static void axi_dispctrl_lastclose(struct drm_device *dev)
{
	struct axi_dispctrl_private *private = dev_get_drvdata(dev->dev);
	drm_fbdev_cma_restore_mode(private->fbdev);
}

DEFINE_DRM_GEM_CMA_FOPS(axi_dispctrl_driver_fops);

static struct drm_driver axi_dispctrl_driver = {
	.driver_features        = DRIVER_MODESET | DRIVER_GEM,
	.lastclose              = axi_dispctrl_lastclose,
	.gem_free_object        = drm_gem_cma_free_object,
	.gem_vm_ops             = &drm_gem_cma_vm_ops,
	.dumb_create            = drm_gem_cma_dumb_create,
	.dumb_map_offset        = drm_gem_dumb_map_offset,
	.dumb_destroy           = drm_gem_dumb_destroy,
	.fops                   = &axi_dispctrl_driver_fops,
	.name                   = DRIVER_NAME,
	.desc                   = DRIVER_DESC,
	.date                   = DRIVER_DATE,
	.major                  = DRIVER_MAJOR,
	.minor                  = DRIVER_MINOR,
};

static const struct of_device_id axi_dispctrl_encoder_of_match[] = {
	{
		.compatible = "hdmi-fastvdma",
	},
	{},
};
MODULE_DEVICE_TABLE(of, axi_dispctrl_encoder_of_match);

static int lcd_set_properties(struct axi_dispctrl_private *private,
		struct device_node *child)
{
	int err;

	READ_PROPERTY(child, "lcd,clock",
		      (u32 *)&(private->lcd_fixed_mode->clock));
	READ_PROPERTY(child, "lcd,hdisplay",
		      (u32 *)&(private->lcd_fixed_mode->hdisplay));
	READ_PROPERTY(child, "lcd,hsync_start",
		      (u32 *)&(private->lcd_fixed_mode->hsync_start));
	READ_PROPERTY(child, "lcd,hsync_end",
		      (u32 *)&(private->lcd_fixed_mode->hsync_end));
	READ_PROPERTY(child, "lcd,htotal",
		      (u32 *)&(private->lcd_fixed_mode->htotal));
	READ_PROPERTY(child, "lcd,vdisplay",
		      (u32 *)&(private->lcd_fixed_mode->vdisplay));
	READ_PROPERTY(child, "lcd,vsync_start",
		      (u32 *)&(private->lcd_fixed_mode->vsync_start));
	READ_PROPERTY(child, "lcd,vsync_end",
		      (u32 *)&(private->lcd_fixed_mode->vsync_end));
	READ_PROPERTY(child, "lcd,vtotal",
		      (u32 *)&(private->lcd_fixed_mode->vtotal));
	READ_PROPERTY(child, "lcd,vrefresh",
		      (u32 *)&(private->lcd_fixed_mode->vrefresh));
	READ_PROPERTY(child, "lcd,flags",
		      (u32 *)&(private->lcd_fixed_mode->flags));
	return 0;
}

static irqreturn_t fast_vdma_irq_handler(int irq, void *data)
{
	struct fast_vdma_priv *priv = (struct fast_vdma_priv*)data;

	priv->dev.dma_done_timestamp = ktime_get();

	priv->ops->handler(priv);

	complete(&priv->dev.dma_done);

	return IRQ_HANDLED;
}

static int axi_dispctrl_platform_probe(struct platform_device *pdev)
{
	struct axi_dispctrl_private *private;
	struct device_node *child, *np = pdev->dev.of_node;
	const struct of_device_id *id;
	struct resource *res;
	int err;
	int ret;
	struct device_node *adapter_node;
	struct i2c_adapter *adapter;
	struct drm_encoder *encoder;
	struct drm_device *drm;

	drm = drm_dev_alloc(&axi_dispctrl_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	private = devm_kzalloc(&pdev->dev, sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	drm_mode_config_init(drm);

	/* this requires a 'reg = <..>' in the dts, else its a null pointer*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	private->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(private->base)) {
		printk("DRM error ioremap_res\n");
		return PTR_ERR(private->base);
	}

    /* litex drp base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	private->drp_base = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(private->drp_base)){
		printk("DRM error ioremap_res\n");
		return PTR_ERR(private->base);
	}


	private->lcd_mode = of_device_is_compatible(np, "ant,axi-dispctrl-lcd");
	dev_info(&pdev->dev, "We are%sin lcd_mode\n",
		 private->lcd_mode?" ":" not ");

	if (private->lcd_mode) {
		for_each_child_of_node(np, child) {
			id = of_match_node(displays_of_match, child);
			/* check for the predefined LCD's */
			if (id) {
				private->lcd_fixed_mode =
				devm_kzalloc(&pdev->dev,
					     sizeof(*private->lcd_fixed_mode),
					     GFP_KERNEL);

				if (!private->lcd_fixed_mode)
					return -ENOMEM;

				memcpy((void *)private->lcd_fixed_mode,
				       (void *)id->data,
				       sizeof(*private->lcd_fixed_mode));
				/* if any predefined wasn't found check
				   if the custom setting are provided */
			} else if (of_device_is_compatible(child, "custom")) {
				private->lcd_fixed_mode =
				devm_kzalloc(&pdev->dev,
					     sizeof(*private->lcd_fixed_mode),
					     GFP_KERNEL);
				if (!private->lcd_fixed_mode)
					return -ENOMEM;
				/* get custom LCD properties */
				err = lcd_set_properties(private, child);
				if (err)
					return err;
			}
			/* We can handle only one LCD display */
			if (private->lcd_fixed_mode) {
				private->invert_pix_clk =
				of_property_read_bool(child,
						      "lcd,invert-pxl-clock");
				dev_info(&pdev->dev, "invert clock is %sset\n",
					 private->invert_pix_clk?"":"not ");
				break;
			}
		}
		/* check if any display was found */
		if (!private->lcd_fixed_mode) {
			dev_info(&pdev->dev, "No LCD display provided,");
			dev_info(&pdev->dev, " or provided unknown model\n");
			return -EINVAL;
		}
	}

	/*find the i2c_adapter*/
	adapter_node = of_parse_phandle(np, "ddc-adapter", 0);
	if (adapter_node) {
		adapter = of_find_i2c_adapter_by_node(adapter_node);
		if (adapter == NULL) {
			printk("DRM no i2c adapter found!\n");
			dev_err(&pdev->dev, "failed to parse ddc-i2c-bus\n");
			return -EPROBE_DEFER;
		}
		private->i2c_adapter = adapter;
	}

	// DMA
	struct fast_vdma_priv *dma;

	private->dma_priv = kzalloc(sizeof(*private->dma_priv), GFP_KERNEL);
	if(!private->dma_priv){
		printk("failed to allocate priv buffer");
		return -ENOMEM;
	}
	dma = private->dma_priv;
	dma->ops = &dma_ops;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dma->dev.base_mm2s = devm_ioremap_resource(&pdev->dev, res);
	if(!dma->dev.base_mm2s){
		dev_err(&pdev->dev, "failed to remap memory");
		return -EINVAL;
	}

	dma->dev.irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if(dma->dev.irq <= 0){
		dev_err(&pdev->dev, "failed to parse IRQ");
		return -EINVAL;
	}

	ret = request_irq(dma->dev.irq, fast_vdma_irq_handler, 0, "fastvdma", dma);
	if(ret){
		dev_err(&pdev->dev, "failed to request IRQ");
		return ret;
	}

	init_completion(&dma->dev.dma_done);

	dma->dev.src_virt = dma_alloc_coherent(&pdev->dev, FAST_VDMA_MEM_SIZE, &dma->dev.src_buf, GFP_KERNEL);
	if(!dma->dev.src_virt)
		return -ENOMEM;

	dma->dev.dst_virt = dma_alloc_coherent(&pdev->dev, FAST_VDMA_MEM_SIZE, &dma->dev.dst_buf, GFP_KERNEL);
	if(!dma->dev.dst_virt)
		return -ENOMEM;

	drm->dev_private = private;
	private->drm_dev = drm;

	if (private->drm_dev == NULL)
	{
		printk("drm_dev is NULL\n");
		return -EPROBE_DEFER;
	}

	axi_dispctrl_mode_config_init(drm);

	// CRTC
	private->crtc = axi_dispctrl_crtc_create(drm);
	if (IS_ERR(private->crtc)) {
		printk("%s: ERROR private->crtc", __func__);
		ret = PTR_ERR(private->crtc);
		goto err;
	}

	// Encoder + Connector
	encoder = axi_dispctrl_encoder_create(drm);
	if (IS_ERR(encoder)) {
		ret = PTR_ERR(encoder);
		goto err;
	}

	// FB
	private->fbdev = drm_fbdev_cma_init(drm, 32, drm->mode_config.num_connector);
	if (IS_ERR(private->fbdev)) {
		printk("%s: failed to initialize drm fbdev\n", __func__);
		dev_err(drm->dev, "failed to initialize drm fbdev\n");
		ret = PTR_ERR(private->fbdev);
		goto err;
	}


	drm_kms_helper_poll_init(drm);
	platform_set_drvdata(pdev, private);

	drm_dev_register(drm, 0);

	return 0;

err:
	drm_mode_config_cleanup(drm);
	return ret;
}

static int __init litehdmi_init(void)
{
	return 0;
}

static int axi_dispctrl_platform_remove(struct platform_device *pdev)
{
	struct axi_dispctrl_private *private = platform_get_drvdata(pdev);

	drm_dev_unregister(private->drm_dev);
	dma_release_channel(private->dma);

	return 0;
}

static struct platform_driver axi_dispctrl_encoder_driver = {
	.driver = {
		.name = "axi-dispctrl",
		.owner = THIS_MODULE,
		.of_match_table = axi_dispctrl_encoder_of_match,
	},
	.probe = axi_dispctrl_platform_probe,
	.remove = axi_dispctrl_platform_remove,
};
module_platform_driver(axi_dispctrl_encoder_driver);


module_init(litehdmi_init);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Karol Gugala <kgugala@antmicro.com>");
MODULE_DESCRIPTION("");

