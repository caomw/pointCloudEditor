#ifndef _PCEDeviceManager
#define _PCEDeviceManager
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceManager.h
//
// Dependency Graph Node:  PCEDeviceManager
//
// Author: Tingzhu Zhou
//
#include "PCEMacros.h"

//Forwards
class PCEDeviceCacheImp;

// PCEDeviceManager thread class
class PCEDeviceManager
{
public:
	enum DeviceManagementFlag
	{
		EDevice_None = 0,
		EDevice_Depth_On = (0x1 << 0),
		EDevice_Color_On = (0x1 << 1),
		EDevice_PlayIndex_On = (0x1 << 2),
		EDevice_Skeleton_On = (0x1 << 3),
		EDevice_Face_On = (0x1 << 4),
		EDevice_Fusion_On = (0x1 << 5),

		Expand_LAST_BIT = (0x1 << 6),

	};

	PCEDeviceManager(){};
	virtual ~PCEDeviceManager(){};

	virtual bool			initializeDevice(int deviceFlag) = 0;
	virtual void			updateBuffer(PCEDeviceCacheImp* pBuffer) = 0;
	virtual bool			isDeviceOn() const = 0;
	virtual void			startDevice () = 0;
	virtual void			pauseDevice () = 0;
	virtual void			shutdownDevice () = 0;

	virtual bool			updateNearMode(bool bNearMode) { return false; }
	virtual void			updateElevationAngle(long tiltAngle) {};
	virtual void			updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius) {};
};

#endif
