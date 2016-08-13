///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudNormalCmd.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudNormalCmd.h"
#include "Shape/PCEPointCloud.h"
#include "../PointCloudShape/PCEPointCloudShape.h"

#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#define DEFAULT_K		0;
#define DEFAULT_RADIUS	0.03;

static const char *methodFlag = "-m", *methodLongFlag = "-method";
static const char *knearestFlag = "-k", *knearestLongFlag = "-knearest";
static const char *radiusFlag = "-r", *radiusLongFlag = "-radius";

const MString PCEPointCloudNormalCmd::kPointCloudNormalCmd = "pointCloudNormal";

using namespace pcl;
using namespace pcl::console;

PCEPointCloudNormalCmd::PCEPointCloudNormalCmd()
	: _methodValue(0)
	, _kNearest(0)
	, _searchRadius(0.03)
{
}

PCEPointCloudNormalCmd::~PCEPointCloudNormalCmd()
{
}

/*static*/
void* PCEPointCloudNormalCmd::creator()
{
	return new PCEPointCloudNormalCmd;
}

/*virtual*/
bool PCEPointCloudNormalCmd::needRecoverData()
{
	return false;
}

/*virtual*/
MStatus PCEPointCloudNormalCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_methodValue	= 0;
	_searchRadius	= DEFAULT_RADIUS;
	_kNearest		= DEFAULT_K;

	// Parse the arguments
	MArgDatabase argData(syntax(), args, &status);
	if (status != MS::kSuccess) {
		MGlobal::displayError( "Invalid arguments" );
		return status;
	}

	if (argData.isFlagSet(methodFlag)) {
		argData.getFlagArgument(methodFlag, 0, _methodValue);
	}
	if (argData.isFlagSet(knearestFlag)) {
		argData.getFlagArgument(knearestFlag, 0, _kNearest);
		if(_kNearest < 0)
			_kNearest = DEFAULT_K;
	}
	if (argData.isFlagSet(radiusFlag)) {
		argData.getFlagArgument(radiusFlag, 0, _searchRadius);
	}

	return status;
}

/*virtual*/
void PCEPointCloudNormalCmd::compute(PCEPointCloud* cloud,  PCEIndicesPtr indices)
{
	TicToc tt;
	tt.tic ();

	PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
	copyPointCloud(cloud->pntCloud, *xyz);
	PointCloud<Normal> normals;
	
	// Try our luck with organized integral image based normal estimation
	if (xyz->isOrganized () && (1 == _methodValue))
	{
		IntegralImageNormalEstimation<PointXYZ, Normal> ne;
		ne.setInputCloud (xyz);
		ne.setNormalEstimationMethod (IntegralImageNormalEstimation<PointXYZ, Normal>::COVARIANCE_MATRIX);
		ne.setNormalSmoothingSize (float (_searchRadius));
		ne.setDepthDependentSmoothing (true);
		if(indices && !indices->empty())
			ne.setIndices(indices);
		ne.compute (normals);
	}
	else
	{
		NormalEstimationOMP<PointXYZ, Normal> ne;
		ne.setInputCloud (xyz);
		ne.setSearchMethod (search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
		ne.setNumberOfThreads(10);
		ne.setKSearch (_kNearest);
		ne.setRadiusSearch (_searchRadius);
		if(indices && !indices->empty())
			ne.setIndices(indices);
		ne.compute (normals);
	}

	for (int i = 0; i < normals.size(); ++i)
	{
		int index = i;
		if(indices && !indices->empty())
			index = indices->at(i);
		if(index >= cloud->pntCloud.size())
			break;
		PCEPointT& pnt = cloud->pntCloud[index];
		pnt.normal_x = normals[i].normal_x;
		pnt.normal_y = normals[i].normal_y;
		pnt.normal_z = normals[i].normal_z;
		pnt.curvature = normals[i].curvature;
	}
	
	MGlobal::displayInfo( MString("Computed normals in ") + tt.toc () + " ms for " + normals.width * normals.height + " points." );
}

/*static*/
MSyntax PCEPointCloudNormalCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( methodFlag, methodLongFlag, MSyntax::kLong );
	syntax.addFlag( knearestFlag, knearestLongFlag, MSyntax::kLong );
	syntax.addFlag( radiusFlag, radiusLongFlag, MSyntax::kDouble );

	return syntax;
}
