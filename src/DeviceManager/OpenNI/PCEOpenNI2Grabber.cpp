// File: PCEOpenNIGrabber.cpp
//
// Dependency Graph Node: PCEOpenNIGrabber
//
// Author: Tingzhu Zhou
//

#include "PCEOpenNI2Grabber.h"

#include <pcl/io/openni2/openni.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;

PCEOpenNI2Grabber::PCEOpenNI2Grabber(PCEPointCloudGrabberBuffer &buf, PCESkeletonBuffer &skeletonBuf)
	: fSkeletonOn(false)
	, fGrabber(NULL)
	, fBuf(buf)
	, fSkeletonBuf(skeletonBuf)
{}

PCEOpenNI2Grabber::~PCEOpenNI2Grabber()
{
	stop ();
}

void PCEOpenNI2Grabber::updateSkeletonOn(bool skeletonOn)
{
	fSkeletonOn = skeletonOn;
}

void PCEOpenNI2Grabber::grabberCallBack (const PointCloud<BufferPointT>::ConstPtr& cloud)
{
	fBuf.pushBack (cloud);
	fSkeletonBuf.grabAndUpdate(cloud);
}

void PCEOpenNI2Grabber::grabAndSend ()
{
	fGrabber->start ();

	while (true)
	{
		if (!threadOn)
			break;
		boost::this_thread::sleep (boost::posix_time::seconds (1));
	}
	fGrabber->stop ();
	cloud_connection.disconnect ();
}

bool PCEOpenNI2Grabber::setupGrabber()
{
	threadOn = false;
	std::string device_id ("");
	OpenNI2Grabber::Mode depth_mode = OpenNI2Grabber::OpenNI_Default_Mode;
	OpenNI2Grabber::Mode image_mode = OpenNI2Grabber::OpenNI_Default_Mode;

	if(!fGrabber)
		fGrabber = new OpenNI2Grabber (device_id, depth_mode, image_mode);
	if(!fGrabber)
		return false;

	boost::function<void (const PointCloud<BufferPointT>::ConstPtr&) > cloud_cb = boost::bind (&PCEOpenNI2Grabber::grabberCallBack, this, _1);
	cloud_connection = fGrabber->registerCallback (cloud_cb);

	return true;
}

void PCEOpenNI2Grabber::startThread ()
{
	if(!threadOn)
	{
		fThread.reset (new boost::thread (boost::bind (&PCEOpenNI2Grabber::grabAndSend, this)));
		threadOn = true;
	}
}

void PCEOpenNI2Grabber::stop ()
{
	threadOn = false;
	if(fThread)
	{
		//fThread->interrupt();
		fThread->join();
	}
}