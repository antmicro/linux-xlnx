#ifndef _ANT_DISPCTRL_CRTC_H_
#define _ANT_DISPCTRL_CRTC_H_

struct drm_device;
struct drm_crtc;
struct drm_crtc* axi_dispctrl_crtc_create(struct drm_device *dev);

#endif
