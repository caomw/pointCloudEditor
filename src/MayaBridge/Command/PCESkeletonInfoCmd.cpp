#include "PCESkeletonInfoCmd.h"
#include "PCEPointCloudGrabber.h"

#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>

#include <pcl/io/pcd_io.h>

#define DEFAULT_FILE_NAME		"..\\..\\PCETest";

const MString PCESkeletonInfoCmd::kPCDFileIOCmd = "pointCloudPcdFile";

static const char *isModeFlag = "-m", *isModeLongFlag = "-mode";
static const char *frameFlag = "-f", *isFrameLongFlag = "-frame";
static const char *pathNameFlag = "-p", *pathNameLongFlag = "-pathName";

using namespace pcl;

/*static*/
void* PCESkeletonInfoCmd::creator()
{
	return new PCESkeletonInfoCmd;
}

MStatus PCESkeletonInfoCmd::doIt(const MArgList& args)
{
	MStatus stat = parseArgs(args);
	if(stat !=MS::kSuccess){
		MGlobal::displayError( "Parse arguments error" );
		return stat;
	}

	MSelectionList selectionList;
	MGlobal::getActiveSelectionList(selectionList);

	MObject 	dependNode;		// Selected dependency node
	if ( MS::kSuccess != selectionList.getDependNode(0, dependNode) ) {
		MGlobal::displayError( "Error getting the dependency node" );
		return stat;
	}
	MFnDependencyNode dgNode( dependNode, &stat );
	if(stat !=MS::kSuccess){
		MGlobal::displayError( "Error creating MFnDependencyNode" );
		return stat;
	}

	if(dgNode.typeId() != PCEPointCloudGrabber::id){
		MGlobal::displayError( "Error getting the PCEPointCloudGrabber id" );
		return MS::kFailure;
	}
	PCEPointCloudGrabber* grabber = dynamic_cast<PCEPointCloudGrabber*>(dgNode.userNode());
	if(!grabber)
	{
		MGlobal::displayError( "Error getting the grabber in cmd" );
		return MS::kFailure;
	}
	// 
	//
	switch (_mode)
	{
	case 1:
		{
			if(0 == _frame)
			{
				for (int i = 0; i < grabber->cachedPointCloudSize(); ++i)
				{
					PCEPointCloud* cloud = grabber->getCachedPointCloud(i);
					if(cloud)
					{
						PCDWriter writer;
						std::stringstream ss;
						ss << _path_name.asChar() << i << ".pcd";
						//writer.writeASCII (ss.str (), *cloud);
					}
				}
			}
			else if(_frame < grabber->cachedPointCloudSize())
			{
				PCEPointCloud* cloud = grabber->getCachedPointCloud(_frame);
				if(cloud)
				{
					PCDWriter writer;
					std::stringstream ss;
					ss << _path_name.asChar() << _frame << ".pcd";
					//writer.writeASCII (ss.str (), *cloud);
				}
			}
		}
		break;
	case 2:
		{

		}
		break;
	default:
		{

		}
	}

	return MS::kSuccess;
}

MStatus PCESkeletonInfoCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_mode = 3;
	_frame = 1;
	_path_name = DEFAULT_FILE_NAME;

	// Parse the arguments
	MArgDatabase argData(syntax(), args, &status);
	if (status != MS::kSuccess) {
		MGlobal::displayError( "Invalid arguments" );
		return status;
	}

	if (argData.isFlagSet(isModeFlag)) {
		argData.getFlagArgument(isModeFlag, 0, _mode);
	}
	if (argData.isFlagSet(frameFlag)) {
		argData.getFlagArgument(frameFlag, 0, _frame);
	}
	if (argData.isFlagSet(pathNameFlag)) {
		argData.getFlagArgument(pathNameFlag, 0, _path_name);
	}

	//status = argData.getObjects(fSelectionList);

	return status;
}

/*static*/
MSyntax PCESkeletonInfoCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( isModeFlag, isModeLongFlag, MSyntax::kLong );
	syntax.addFlag( frameFlag, isFrameLongFlag, MSyntax::kLong );

	return syntax;
}