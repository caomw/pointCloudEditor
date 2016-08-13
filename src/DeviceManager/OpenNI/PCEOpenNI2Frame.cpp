// File: PCEOpenNI2Frame.cpp
//
// Author: Tingzhu Zhou
//

#include "PCEOpenNI2Frame.h"

#include <pcl/io/openni2/openni.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;

PCEOpenNI2Frame::PCEOpenNI2Frame (BufferPointCloudT::ConstPtr pPointCloud)
	: mp_PointCloud(pPointCloud)
	, mp_SkeletonData(NULL)
	, mp_IndicesCluster(NULL)
{}

PCEOpenNI2Frame::~PCEOpenNI2Frame ()
{
	/*if(mp_PointCloud)
		delete mp_PointCloud;*/
	if(mp_SkeletonData)
		delete mp_SkeletonData;
	if(mp_IndicesCluster)
		delete mp_IndicesCluster;
}

void PCEOpenNI2Frame::write(const char* pathName, int timeStamp) const
{
	PCDWriter writer;
	std::stringstream ss;
	ss << pathName << timeStamp << ".pcd";
	writer.writeASCII (ss.str (), *mp_PointCloud);
}