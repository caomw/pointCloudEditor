#ifndef _PCEDeviceGrabber
#define _PCEDeviceGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceGrabber.h
//
// Dependency Graph Node:  PCEDeviceGrabber
//
// Author: Tingzhu Zhou
//


//Forwards
class PCEDataCache;
// PCEDeviceGrabber thread class
class PCEDeviceGrabber
{
public:
	PCEDeviceGrabber(){};
	virtual ~PCEDeviceGrabber(){};

	virtual bool	setupGrabber() = 0;
	virtual void	startThread () = 0;
	virtual void	stop() = 0;
	virtual void	updateSkeletonOn(bool skeletonOn) = 0;
};

#endif
