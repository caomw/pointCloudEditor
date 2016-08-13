#ifndef _PCEOpenNI2Frame
#define _PCEOpenNI2Frame
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEOpenNI2Frame.h
//
//
// Author: Tingzhu Zhou
//
#include "../PCEDeviceFrame.h"
#include "Shape/PCEPointCloud.h"
#include "Shape/PCESkeletonPosition.h"
#include "../PCEPointCloudBuffer.h"

typedef pcl::PointCloud<BufferPointT> BufferPointCloudT;

class PCEOpenNI2Frame : public PCEDeviceFrame
{
public:
	PCEOpenNI2Frame (BufferPointCloudT::ConstPtr pPointCloud);
	virtual ~PCEOpenNI2Frame ();

	virtual void				write(const char* pathName, int timeStamp) const;

private:
	BufferPointCloudT::ConstPtr	mp_PointCloud;
	PCESkeletonPosition*		mp_SkeletonData;
	IndicesClusterT*			mp_IndicesCluster;

private:
	//PCEDeviceFrame (const PCEDeviceFrame&); // Disabled copy constructor
	//PCEDeviceFrame& operator = (const PCEDeviceFrame&); // Disabled assignment operator
};

#endif
