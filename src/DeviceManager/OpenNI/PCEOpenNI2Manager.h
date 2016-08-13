#ifndef _PCEOpenNI2Manager
#define _PCEOpenNI2Manager
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEOpenNI2Manager.h
//
// Dependency Graph Node:  PCEOpenNI2Manager
//
// Author: Tingzhu Zhou
//
#include "../PCEDeviceManager.h"
#include "../PCEDeviceGrabber.h"
#include "../PCESkeletonBuffer.h"
#include "../PCEPointCloudBuffer.h"

//Forwards
class PCEPointCloud;
class PCESkeletonPosition;

// PCEOpenNI2Manager thread class
class PCEOpenNI2Manager : public PCEDeviceManager
{
public:
	PCEOpenNI2Manager();
	virtual ~PCEOpenNI2Manager();

	virtual bool			initializeDevice(int deviceFlag) override;
	virtual void			updateBuffer(PCEDeviceCacheImp* pBuffer) override;
	virtual bool			isDeviceOn() const override;
	virtual void			startDevice () override;
	virtual void			pauseDevice () override;
	virtual void			shutdownDevice () override;

	bool			readPointCloudData(int frameId, PCEPointCloud* pPointCloud);
	bool			readSkeletonData(int skeletonId, PCESkeletonPosition* pSkeleton);

	virtual void			updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius);

private:
	PCEDeviceGrabber*			fGrabber;
	PCEPointCloudGrabberBuffer	fGrabberBuf;
	PCESkeletonBuffer			fSkeletonBuf;
};

#endif
