#ifndef _PCEGrabberPreViewCmd
#define _PCEGrabberPreViewCmd

#include <maya/MPxModelEditorCommand.h>

class MPx3dModelView;

class PCEGrabberPreViewCmd: public MPxModelEditorCommand
{
public:
	PCEGrabberPreViewCmd();
	virtual				~PCEGrabberPreViewCmd();

	static void*		creator();

	virtual MStatus		doEditFlags();

	virtual MStatus 	appendSyntax();

	virtual MPx3dModelView*	userView();

	static const MString kGrabberPreViewCmd;
protected:

};

#endif