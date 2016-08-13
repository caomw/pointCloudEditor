#ifndef __PCEPreviewTextureToTargetGL_h__
#define __PCEPreviewTextureToTargetGL_h__

#include "PCEPreviewTextureToTarget.h"

class PCEPreviewTextureToTargetGL : public PCEPreviewTextureToTarget
{
public:
	PCEPreviewTextureToTargetGL();
	virtual ~PCEPreviewTextureToTargetGL();

protected:
	virtual bool renderTextureToTarget(MHWRender::MTexture* texture, MHWRender::MRenderTarget *target);
};

#endif
