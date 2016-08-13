#include "PCESkeletonJointInfoCmd.h"
#include "PointCloudShape/PCEPointCloud.h"
#include "PointCloudShape/PCEPointCloudShape.h"
#include "Skeleton/PCESkeletonPosition.h"

#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>

const MString PCESkeletonJointInfoCmd::kSkeletionJointInfoCmd = "pointCloudSkeletionInfo";

static const char *bodyPartFlag = "-b", *bodyPartLongFlag = "-bodyPart";

/*static*/
void* PCESkeletonJointInfoCmd::creator()
{
	return new PCESkeletonJointInfoCmd;
}

MStatus PCESkeletonJointInfoCmd::doIt(const MArgList& args)
{
	MStatus stat = parseArgs(args);
	if(stat !=MS::kSuccess){
		MGlobal::displayError( "Parse arguments error" );
		return stat;
	}

	MSelectionList selectionList;
	MGlobal::getActiveSelectionList(selectionList);
	
	MDagPath dagPath;
	if ( MS::kSuccess !=  selectionList.getDagPath(0, dagPath) ){
		cerr << "Error getting the dag path" << endl;
		return MS::kFailure;
	}
	dagPath.extendToShape();
	MFnDagNode dagNode( dagPath.node() );
	if(dagNode.typeId() != PCEPointCloudShape::id) {
		MGlobal::displayError( "Error getting the PCEPointCloudShape in cmd." );
		return MS::kFailure;
	}

	PCEPointCloudShape* shape = dynamic_cast<PCEPointCloudShape*>(dagNode.userNode());
	if(!shape)
	{
		MGlobal::displayError( "Error getting the shape in cmd" );
		return MS::kFailure;
	}
	PCEPointCloud* cloud = shape->meshGeom();
	if(!cloud)
	{
		MGlobal::displayError( "Error getting the point cloud data in cmd" );
		return MS::kFailure;
	}
	
	// Return expanded selection list as an array of strings
	//
	MDoubleArray returnArray;
	//const PCESkeletonData& skData = cloud->skeletonJointPositions;
	PCESkeletonPosition* skData = new PCESkeletonPosition();
	if(_BodyPart < SKELETON_POSITION_COUNT)
	{
		float result_x = skData->getJointPositionX((PCEJointType)_BodyPart);
		float result_y = skData->getJointPositionY((PCEJointType)_BodyPart);
		float result_z = skData->getJointPositionZ((PCEJointType)_BodyPart);
		if (!pcl_isfinite (result_x) || 
			!pcl_isfinite (result_y) || 
			!pcl_isfinite (result_z))
			return MS::kFailure;

		returnArray.append(result_x);
		returnArray.append(result_y);
		returnArray.append(result_z);
		//returnArray.append(skData->_joints[_BodyPart].confidence);
		MPxCommand::setResult( returnArray );
	}
	else
	{
		MGlobal::displayError( "Error getting the specified skeleton data in cmd" );
		return MS::kFailure;
	}
	
	return MS::kSuccess;
}

MStatus PCESkeletonJointInfoCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_BodyPart = 0;

	// Parse the arguments
	MArgDatabase argData(syntax(), args, &status);
	if (status != MS::kSuccess) {
		MGlobal::displayError( "Invalid arguments" );
		return status;
	}

	if (argData.isFlagSet(bodyPartFlag)) {
		argData.getFlagArgument(bodyPartFlag, 0, _BodyPart);
	}

	return status;
}

/*static*/
MSyntax PCESkeletonJointInfoCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( bodyPartFlag, bodyPartLongFlag, MSyntax::kLong );

	return syntax;
}