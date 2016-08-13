#include "PCEGrabberPreView.h"
#include "PCEGrabberRenderOverride.h"

PCEGrabberPreView::PCEGrabberPreView()
	: fPreviewRenderer(NULL)
{
	setMultipleDrawEnable(true);

	//M3dView view;
	//getAsM3dView( view );

	//registerOverride();
	//MStatus status = view.setRenderOverrideName( PCEGrabberRenderOverride::kPCEGrabberPreviewerName );
	//if (status != MStatus::kSuccess)
	//{
	//	//MGlobal::displayError( rStereoOverrideSetFailed );
	//}
}

PCEGrabberPreView::~PCEGrabberPreView()
{
	if (fPreviewRenderer)
	{
		MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer(false /*createViewport*/);
		if (renderer && fPreviewRenderer)
		{
			renderer->deregisterOverride(fPreviewRenderer);
			delete fPreviewRenderer;
			fPreviewRenderer = NULL;
		}
	}
}

void * PCEGrabberPreView::creator()
{
	return new PCEGrabberPreView();
}

MString PCEGrabberPreView::viewType() const
//
//	Description:
//		Returns a string with the type of the view.
//
{
	return MString(kPCEGrabberPreviewerTypeName);
}

bool PCEGrabberPreView::okForMultipleDraw(const MDagPath &dagPath)
{
	setViewSelected(false);

	return true;
}

unsigned PCEGrabberPreView::multipleDrawPassCount()
//
// Description:
//  Tells the draw pass engine how many passes we want to draw.  For
//  stereoscopic, this value should at least two (one for each buffer).
//
{
	

	M3dView view;
	getAsM3dView( view );

	registerOverride();
	MStatus status = view.setRenderOverrideName( PCEGrabberRenderOverride::kPCEGrabberPreviewerName );
	if (status != MStatus::kSuccess)
	{
		//MGlobal::displayError( rStereoOverrideSetFailed );
	}
	return 1;
}

void PCEGrabberPreView::registerOverride()
{
	if (!fPreviewRenderer)
	{
		MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer(false /*createViewport*/);
		if (renderer)
		{
			fPreviewRenderer = new PCEGrabberRenderOverride( PCEGrabberRenderOverride::kPCEGrabberPreviewerName );
			if (fPreviewRenderer)
			{
				MStatus status = renderer->registerOverride( fPreviewRenderer );
				if (status != MStatus::kSuccess)
				{
					//MGlobal::displayError( rStereoOverrideRegisterFailed  );
					delete fPreviewRenderer;
					fPreviewRenderer = NULL;
				}
			}
		}
	}
}