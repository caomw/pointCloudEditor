#ifndef _PCEGrabberPreView
#define _PCEGrabberPreView

#include <maya/MPx3dModelView.h>
#include <maya/MString.h>

#define kPCEGrabberPreviewerTypeName "PCEGrabberPreviewer"

//Forwards
class PCEGrabberRenderOverride;

class PCEGrabberPreView: public MPx3dModelView
{
public:
	PCEGrabberPreView();
	virtual ~PCEGrabberPreView();

	virtual MString 	viewType() const;
	virtual unsigned    multipleDrawPassCount();
	virtual bool		okForMultipleDraw(const MDagPath &dagPath);

	static void*		creator();
private:
	void				registerOverride();
private:
	PCEGrabberRenderOverride*	fPreviewRenderer;
};

#endif