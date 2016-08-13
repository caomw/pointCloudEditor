#include "PCEGrabberPreViewCmd.h"
#include "PCEGrabberPreView.h"

#include <maya/MGlobal.h>

const MString PCEGrabberPreViewCmd::kGrabberPreViewCmd = "PCEGrabberPreView";

PCEGrabberPreViewCmd::PCEGrabberPreViewCmd()
	: 	MPxModelEditorCommand()
//
//	Description:
//		Class constructor.
//
{
}

PCEGrabberPreViewCmd::~PCEGrabberPreViewCmd()
//
//	Description:
//		Class destructor.
//
{
}

void* PCEGrabberPreViewCmd::creator()
//
//	Description:
//		Create the command.
//
{
	return new PCEGrabberPreViewCmd();
}

MPx3dModelView *PCEGrabberPreViewCmd::userView()
//
//	Description:
//		Create the MPx3dModelPanel used by this command.
//
{
	return new PCEGrabberPreView();
}

MStatus PCEGrabberPreViewCmd::appendSyntax()
//
//	Description:
//		Add syntax to the command. All of the parent syntax is added
//		before this call is made.
//
{
	MStatus ReturnStatus;

	MSyntax theSyntax = syntax(&ReturnStatus);
	if (MS::kSuccess != ReturnStatus) {
		MGlobal::displayError("Could not get the parent's syntax");
		return ReturnStatus;
	}

	/*theSyntax.addFlag(kTestMultiPackInitFlag, 
		kTestMultiPackInitFlagLong);

	theSyntax.addFlag(kTestMultiPackResultsFlag, 
		kTestMultiPackResultsFlagLong);

	theSyntax.addFlag(kTestMultiPackClearFlag, 
		kTestMultiPackClearFlagLong);*/

	return ReturnStatus;
}

MStatus PCEGrabberPreViewCmd::doEditFlags()
//
//	Description:
//		Handle edits for flags added by this class.
//		If the flag is unknown, return MS::kSuccess and the parent class
//		will attempt to process the flag. Returning MS::kUnknownParameter
//		will cause the parent class to process the flag.
//
{
	MPx3dModelView *user3dModelView = modelView();
	if (NULL == user3dModelView) {
		MGlobal::displayError("NULL == user3dModelView!");
		return MS::kFailure;
	}

	//	This is now safe to do, since the above test passed.
	//
	PCEGrabberPreView *dView = (PCEGrabberPreView *)user3dModelView;

	/*MArgParser argData = parser();
	if (argData.isFlagSet(kTestMultiPackInitFlag)) {
		return initTests(*dView);
	} else if (argData.isFlagSet(kTestMultiPackResultsFlag)) {
		return testResults(*dView);
	} else if (argData.isFlagSet(kTestMultiPackClearFlag)) {
		return clearResults(*dView);
	}*/

	return MS::kUnknownParameter;
}
