#ifndef __PCEPreviewTextureToTarget_h__
#define __PCEPreviewTextureToTarget_h__

namespace MHWRender
{
	class MRenderer;
	class MTexture;
	class MRenderTarget;
	//class MTextureDescription;
}

class PCEPreviewTextureToTarget
{
public:
	static PCEPreviewTextureToTarget* create(MHWRender::MRenderer* renderer);

protected:
	PCEPreviewTextureToTarget();
	
public:
	virtual ~PCEPreviewTextureToTarget();
	
	virtual bool renderTextureToTarget(MHWRender::MTexture* texture, MHWRender::MRenderTarget *target) = 0;
};

#endif 
