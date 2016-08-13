///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudIndicesCmd.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudIndicesCmd.h"
#include "Shape/PCEPointCloud.h"
#include "../PointCloudShape/PCEPointCloudShape.h"

#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MPxCommand.h>

static const char *clearFlag = "-c", *clearLongFlag = "-clear";
static const char *clearAllFlag = "-a", *clearAllLongFlag = "-clearAll";

const MString PCEPointCloudIndicesCmd::kPointCloudIndicesCmd = "pointCloudIndices";

PCEPointCloudIndicesCmd::PCEPointCloudIndicesCmd()
	: _clearIndex(-1)
	, _clearAll(false)
{
}

PCEPointCloudIndicesCmd::~PCEPointCloudIndicesCmd()
{
}

/*static*/
void* PCEPointCloudIndicesCmd::creator()
{
	return new PCEPointCloudIndicesCmd;
}

/*virtual*/
MStatus PCEPointCloudIndicesCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_clearIndex			= -1;
	_clearAll		= false;

	// Parse the arguments
	MArgDatabase argData(syntax(), args, &status);
	if (status != MS::kSuccess) {
		MGlobal::displayError( "Invalid arguments" );
		return status;
	}

	if (argData.isFlagSet(clearFlag)) {
		argData.getFlagArgument(clearFlag, 0, _clearIndex);
	}
	if (argData.isFlagSet(clearAllFlag)) {
		argData.getFlagArgument(clearAllFlag, 0, _clearAll);
	}
	
	return status;
}

/*virtual*/
void PCEPointCloudIndicesCmd::compute(PCEPointCloud* cloud,  PCEIndicesPtr indices)
{
	if(_clearAll)
	{
		cloud->indicesClusters.clear();
	}
	else if(_clearIndex > 0)
	{
		IndicesClusterT::iterator Iter = cloud->indicesClusters.begin() + _clearIndex;
		cloud->indicesClusters.erase(Iter);
		for( ; Iter != cloud->indicesClusters.end(); Iter++ )
		{
			cloud->indicesClusters.insert(Iter, *(Iter + 1));
		}
		cloud->indicesClusters.pop_back();
	}
	if(indices)
	{
		pcl::PointIndices ptIndices;
		ptIndices.indices = *indices;
		cloud->indicesClusters.push_back(ptIndices);
	}

	int size = cloud->indicesClusters.size();
	MPxCommand::setResult( size );
}

/*virtual*/
bool PCEPointCloudIndicesCmd::needRecoverData()
{
	return true;
}

void PCEPointCloudIndicesCmd::recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices)
{
	cloud->indicesClusters.pop_back();
}

/*static*/
MSyntax PCEPointCloudIndicesCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( clearFlag, clearLongFlag, MSyntax::kLong );
	syntax.addFlag( clearAllFlag, clearAllLongFlag, MSyntax::kBoolean );

	return syntax;
}
