///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudSegmentCmd.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudSegmentCmd.h"
#include "Shape/PCEPointCloud.h"
#include "../PointCloudShape/PCEPointCloudShape.h"

#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

#include <pcl/console/time.h>
#include <pcl/segmentation/extract_clusters.h>

#define DEFAULT_MIN			10;
#define DEFAULT_MAX			25000;
#define DEFAULT_TORELANCE	0.02;

static const char *methodFlag = "-m", *methodLongFlag = "-method";
static const char *minFlag = "-i", *minLongFlag = "-cmin";
static const char *maxFlag = "-a", *maxLongFlag = "-cmax";
static const char *toleranceFlag = "-t", *toleranceLongFlag = "-tolerance";

const MString PCEPointCloudSegmentCmd::kPointCloudSegmentCmd = "pointCloudSegment";

using namespace pcl;
using namespace pcl::console;

PCEPointCloudSegmentCmd::PCEPointCloudSegmentCmd()
	: _methodValue(0)
	, _min(50)
	, _max(25000)
	, _tolerance(0.02)
{
}

PCEPointCloudSegmentCmd::~PCEPointCloudSegmentCmd()
{
}

/*static*/
void* PCEPointCloudSegmentCmd::creator()
{
	return new PCEPointCloudSegmentCmd;
}

/*virtual*/
MStatus PCEPointCloudSegmentCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_methodValue	= 0;
	_min	= DEFAULT_MIN;
	_max	= DEFAULT_MAX;
	_tolerance		= DEFAULT_TORELANCE;

	// Parse the arguments
	MArgDatabase argData(syntax(), args, &status);
	if (status != MS::kSuccess) {
		MGlobal::displayError( "Invalid arguments" );
		return status;
	}

	if (argData.isFlagSet(methodFlag)) {
		argData.getFlagArgument(methodFlag, 0, _methodValue);
	}
	if (argData.isFlagSet(minFlag)) {
		argData.getFlagArgument(minFlag, 0, _min);
	}
	if (argData.isFlagSet(maxFlag)) {
		argData.getFlagArgument(maxFlag, 0, _max);
	}
	if (argData.isFlagSet(toleranceFlag)) {
		argData.getFlagArgument(toleranceFlag, 0, _tolerance);
	}

	return status;
}

/*virtual*/
void PCEPointCloudSegmentCmd::compute(PCEPointCloud* cloud,  PCEIndicesPtr indices)
{
	TicToc tt;
	tt.tic ();

	if (1 == _methodValue)
	{
		//PCL_TODO
	}
	else
	{
		PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
		copyPointCloud(cloud->pntCloud, *xyz);
		
		// Creating the KdTree object for the search method of the extraction
		search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
		tree->setInputCloud (xyz);

		EuclideanClusterExtraction<PointXYZ> ec;
		ec.setClusterTolerance (_tolerance);
		ec.setMinClusterSize (_min);
		ec.setMaxClusterSize (_max);
		ec.setSearchMethod (tree);
		ec.setInputCloud (xyz);
		if(indices && !indices->empty())
			ec.setIndices(indices);
		ec.extract (cloud->indicesClusters);
	}

	MGlobal::displayInfo( MString("Segment in ") + tt.toc () + " ms for " + cloud->indicesClusters.size () + " clusters." );
}

/*virtual*/
bool PCEPointCloudSegmentCmd::needRecoverData()
{
	return true;
}

void PCEPointCloudSegmentCmd::recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices)
{
	cloud->indicesClusters.clear();
}

/*static*/
MSyntax PCEPointCloudSegmentCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( methodFlag, methodLongFlag, MSyntax::kLong );
	syntax.addFlag( minFlag, minLongFlag, MSyntax::kLong );
	syntax.addFlag( maxFlag, maxLongFlag, MSyntax::kLong );
	syntax.addFlag( toleranceFlag, toleranceLongFlag, MSyntax::kDouble );

	return syntax;
}
