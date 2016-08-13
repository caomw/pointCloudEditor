#include "PCEPreviewTextureToTarget.h"

#include <maya/MViewport2Renderer.h>
#include <maya/MGlobal.h>

#include "PCEPreviewTextureToTargetGL.h"

#ifdef _WIN32
#include "PCEPreviewTextureToTargetDX.h"
#endif

PCEPreviewTextureToTarget::PCEPreviewTextureToTarget()
{
}

PCEPreviewTextureToTarget::~PCEPreviewTextureToTarget()
{
}

/*static*/
PCEPreviewTextureToTarget* PCEPreviewTextureToTarget::create(MHWRender::MRenderer* renderer)
{
	PCEPreviewTextureToTarget* helper = NULL;

	if(renderer->drawAPIIsOpenGL())
	{
		helper = new PCEPreviewTextureToTargetGL();
	}
#ifdef _WIN32
	else
	{
		ID3D11Device* dxDevice = (ID3D11Device*) renderer->GPUDeviceHandle();
		helper = new PCEPreviewTextureToTargetDX(dxDevice);
	}
#endif

	return helper;
}
