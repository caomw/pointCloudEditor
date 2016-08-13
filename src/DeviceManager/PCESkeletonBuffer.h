#ifndef _PCESkeletonBuffer
#define _PCESkeletonBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletonBuffer.h
//
// Dependency Graph Node:  PCESkeletonBuffer
//
// Author: Tingzhu Zhou
//
#include <NiTE.h>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Shape/PCESkeletonPosition.h"

typedef pcl::PointXYZRGBA BufferPointT;

#define PCE_MAX_USERS 10

class PCESkeletonBuffer
{
public:
	PCESkeletonBuffer () {}
	~PCESkeletonBuffer () {}
		
	bool	initialize();
	void	shutDown();
	bool	grabAndUpdate(const pcl::PointCloud<BufferPointT>::ConstPtr& cloud);
	bool	getSkeletonData(int skeletonId, PCESkeletonPosition* pSkeletonData);
	bool	getUserLabels();
	bool	isValid() const;
	void	updateSkeletonSmoothFactor(float smoothFactor);

private:
	void	grabSkeletonCenter(const nite::UserData& userData, const pcl::PointCloud<BufferPointT>::ConstPtr& cloud);
	void	grabJoint(const nite::UserData& userData, const pcl::PointCloud<BufferPointT>::ConstPtr& cloud, nite::JointType niteType, PCEJointType pceType);

	nite::UserTracker			fUserTracker_;
	boost::mutex				bmutex_;
	PCESkeletonPosition			fSkeletonData;
	nite::SkeletonState			fGrabState[PCE_MAX_USERS];
	nite::UserMap				fUserLabels;
};

#endif