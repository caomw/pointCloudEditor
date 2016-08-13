// File: PCEKinectManager.cpp
//
// Dependency Graph Node: PCEKinectManager
//
// Author: Tingzhu Zhou
//

#include "PCEOpenNI2Manager.h"
#include "PCEOpenNI2Grabber.h"
#include "PCEOpenNI2Frame.h"

#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;

PCEOpenNI2Manager::PCEOpenNI2Manager()
	: fGrabber(NULL)
{}

PCEOpenNI2Manager::~PCEOpenNI2Manager()
{
	shutdownDevice ();
}

bool PCEOpenNI2Manager::initializeDevice (int deviceFlag)
{
	if(fGrabber)
	{
		fGrabber->updateSkeletonOn((bool)(deviceFlag&EDevice_Skeleton_On));
		//fSkeletonBuf.initialize();
		return true;
	}

	boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
	if(deviceManager->getNumOfConnectedDevices () <= 0)
	{
		return false;
	}

	if(!fGrabber)
	{
		fGrabberBuf.setCapacity (GRABBER_BUFFER_SIZE);
		fGrabber = new PCEOpenNI2Grabber(fGrabberBuf, fSkeletonBuf);
		if(!fGrabber->setupGrabber())
		{
			return false;
		}
		if(deviceFlag & EDevice_Skeleton_On)
		{
			fSkeletonBuf.initialize();
		}
	}
	return true;
}

void	PCEOpenNI2Manager::updateBuffer(PCEDeviceCacheImp* pBuffer)
{

}

void PCEOpenNI2Manager::startDevice ()
{
	if(fGrabber)
		fGrabber->startThread();
}

void PCEOpenNI2Manager::pauseDevice ()
{
	/*if(fGrabber)
		fGrabber->pauseThread();*/
}

void PCEOpenNI2Manager::shutdownDevice()
{
	if(fGrabber)
	{
		fGrabber->stop();
		delete fGrabber;
		fGrabber = NULL;
	}

	fGrabberBuf.clear();
	fSkeletonBuf.shutDown();
}

bool PCEOpenNI2Manager::isDeviceOn() const
{
	return (NULL != fGrabber);
}

//PCEDeviceFrame*	PCEOpenNI2Manager::getFrame (short recordMode)
//{
//	const BufferPointCloudT::ConstPtr& cloud = (recordMode == 1) ?
//		fGrabberBuf.getBackBeforeClear(fGrabberMutex) :
//		fGrabberBuf.getFront(fGrabberMutex);
//	//fSkeletonBuf.recieveSkeletonData(geomPtr);
//
//	PCEDeviceFrame* pFrame = new PCEOpenNI2Frame(cloud);
//	return pFrame;
//}

bool PCEOpenNI2Manager::readPointCloudData(int frameId, PCEPointCloud* pPointCloud)
{
	BufferPointCloudT::ConstPtr cloud = fGrabberBuf.getFront();
	if(cloud)
		copyPointCloud( *cloud, pPointCloud->pntCloud );

	// Compute point UV
	PointCloudT&	cachedCloud = pPointCloud->pntCloud;
	if(cachedCloud.isOrganized())
	{
		int width = cachedCloud.width;
		int height = cachedCloud.height;
		for (int i = 0; i < cachedCloud.size(); ++i)
		{
			cachedCloud.at(i).u = float(i % width) / width;
			cachedCloud.at(i).v = float(i / width) / height;
		}
	}
	return true;
}

bool PCEOpenNI2Manager::readSkeletonData(int skeletonId, PCESkeletonPosition* pSkeleton)
{
	return fSkeletonBuf.getSkeletonData(skeletonId, pSkeleton);
}

void PCEOpenNI2Manager::updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius)
{
	fSkeletonBuf.updateSkeletonSmoothFactor(smoothing);
}
