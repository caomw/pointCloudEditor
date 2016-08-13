///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudKeyPointsCmd.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudKeyPointsCmd.h"
#include "Shape/PCEPointCloud.h"
#include "../PointCloudShape/PCEPointCloudShape.h"

#include <maya/MGlobal.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

#include <boost/make_shared.hpp>
#include <pcl/console/time.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>

#define DEFAULT_MIN			-std::numeric_limits<double>::max ();
#define DEFAULT_MAX			std::numeric_limits<double>::max ();
#define DEFAULT_RADIUS		0.03;

static const char *methodFlag = "-m", *methodLongFlag = "-method";
static const char *minFlag = "-i", *minLongFlag = "-min";
static const char *maxFlag = "-a", *maxLongFlag = "-max";
static const char *radiusFlag = "-r", *radiusLongFlag = "-radius";

const MString PCEPointCloudKeyPointsCmd::kPointCloudKeyPointsCmd = "pointCloudKeyPoints";

using namespace pcl;
using namespace pcl::console;

PCEPointCloudKeyPointsCmd::PCEPointCloudKeyPointsCmd()
	: _methodValue(0)
	, _min(0.0)
	, _max(0.0)
	, _searchRadius(0.03)
{
}

PCEPointCloudKeyPointsCmd::~PCEPointCloudKeyPointsCmd()
{
}

/*static*/
void* PCEPointCloudKeyPointsCmd::creator()
{
	return new PCEPointCloudKeyPointsCmd;
}

/*virtual*/
MStatus PCEPointCloudKeyPointsCmd::parseArgs( const MArgList& args)
{
	MStatus status = MS::kSuccess;

	_methodValue	= 0;
	_min			= DEFAULT_MIN;
	_max			= DEFAULT_MAX;
	_searchRadius	= DEFAULT_RADIUS;

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
	if (argData.isFlagSet(radiusFlag)) {
		argData.getFlagArgument(radiusFlag, 0, _searchRadius);
	}

	return status;
}

/*virtual*/
void PCEPointCloudKeyPointsCmd::compute(PCEPointCloud* cloud,  PCEIndicesPtr indices)
{
	TicToc tt;
	tt.tic ();

	unsigned int outputSize = 0;
	if (1 == _methodValue)
	{
		PointCloud<PointXYZRGB>::Ptr cloudTmp (new PointCloud<PointXYZRGB>);
		copyPointCloud(cloud->pntCloud, *cloudTmp);

		HarrisKeypoint3D <PointXYZRGB, PointXYZI> detector;
		PointCloud<PointXYZI>::Ptr keypoints (new PointCloud<PointXYZI>);
		detector.setNonMaxSupression (true);
		detector.setInputCloud (cloudTmp);
		detector.setThreshold (1e-6);
		detector.compute (*keypoints);
		outputSize = (unsigned int)keypoints->size ();
		pcl::PointIndicesConstPtr keypoints_indices( detector.getKeypointsIndices() );
		cloud->indicesClusters.push_back(*keypoints_indices);
		//PCL_TODO
		// Inverse the indices
		//PointIndices outIndices;
		//ExtractIndices<PointT> extract(false);
		//extract.setInputCloud (boost::make_shared< const PointCloud<PointT> > (cloud->pntCloud));
		//extract.setIndices (boost::make_shared<const PointIndices> (removalIndices));
		//extract.filter (outIndices.indices);
		//// Set the filtered indices into indicesClusters
		//cloud->indicesClusters.push_back(outIndices);

	}
	else
	{
		UniformSampling<PCEPointT> us;
		us.setInputCloud (boost::make_shared< const PointCloudT > (cloud->pntCloud));
		us.setRadiusSearch (_searchRadius);

		PointCloud<int> keypoints;
		us.compute (keypoints);
		outputSize = keypoints.width * keypoints.height;

		PointIndices keypoints_indices;
		copy(keypoints.points.begin(), keypoints.points.end(), back_inserter(keypoints_indices.indices));
		cloud->indicesClusters.push_back(keypoints_indices);
	}
	
	MGlobal::displayInfo( MString("Down sample: ") + tt.toc () + " ms for " + outputSize + " points.");
}

/*virtual*/
bool PCEPointCloudKeyPointsCmd::needRecoverData()
{
	return true;
}

void PCEPointCloudKeyPointsCmd::recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices)
{
	cloud->indicesClusters.pop_back();
}

/*static*/
MSyntax PCEPointCloudKeyPointsCmd::newSyntax()
{
	MSyntax syntax;

	syntax.addFlag( methodFlag, methodLongFlag, MSyntax::kLong );
	syntax.addFlag( minFlag, minLongFlag, MSyntax::kDouble );
	syntax.addFlag( maxFlag, maxLongFlag, MSyntax::kDouble );
	syntax.addFlag( radiusFlag, radiusLongFlag, MSyntax::kDouble );

	return syntax;
}
