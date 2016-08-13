#ifndef _PCEOpenNIGrabber
#define _PCEOpenNIGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceGrabber.h
//
// Dependency Graph Node:  PCEDeviceGrabber
//
// Author: Tingzhu Zhou
//
#include "../PCEDeviceGrabber.h"
#include "../PCEPointCloudBuffer.h"
#include "../PCESkeletonBuffer.h"

#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/openni2_grabber.h>

typedef pcl::PointXYZRGBA BufferPointT;

// PCEOpenNIGrabber thread class
class PCEOpenNI2Grabber : public PCEDeviceGrabber
{
public:
	PCEOpenNI2Grabber(PCEPointCloudGrabberBuffer &buf, PCESkeletonBuffer &skeletonBuf);
	virtual ~PCEOpenNI2Grabber();

	virtual bool	setupGrabber();
	virtual void	startThread ();
	virtual void	stop();
	virtual void	updateSkeletonOn(bool skeletonOn);
private:
	void			grabAndSend ();
	void			grabberCallBack (const pcl::PointCloud<BufferPointT>::ConstPtr& cloud);

	pcl::io::OpenNI2Grabber*			fGrabber;
	boost::signals2::connection			cloud_connection;
	boost::shared_ptr<boost::thread>	fThread;
	bool								threadOn;
	bool								fSkeletonOn;

	PCEPointCloudGrabberBuffer&			fBuf;
	PCESkeletonBuffer &					fSkeletonBuf;
};

#endif
