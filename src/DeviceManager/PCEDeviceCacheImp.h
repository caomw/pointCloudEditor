#ifndef _PCEDeviceCacheImp
#define _PCEDeviceCacheImp
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceCacheImp.h
//
// Author: Tingzhu Zhou
//
#include "PCEMacros.h"

#include <opencv2\core\core.hpp>

// Forwards
class PCECompoundFrame;
class PCEFaceTrackingResult;
class PCEFusionMesh;

class PCEDeviceCacheImp
{
public:
	PCEDeviceCacheImp () {}
	virtual ~PCEDeviceCacheImp () {}
	
	virtual void					clear() = 0;

	virtual PCECompoundFrame*		acquireCompoundFrame() = 0;
	virtual PCECompoundFrame*		getLatestCompoundFrame() = 0;
	virtual PCECompoundFrame*		getCompoundFrame(int frameId) = 0;

	virtual bool					setFaceShape(PCEFaceTrackingResult* pFaceShape) = 0;
	virtual bool					readFaceShape(int frameId, PCEFaceTrackingResult* pFaceShape) = 0;
	virtual int						getFaceTrackingFrame() const { return -1; }
	virtual int						getFaceTrackingFrameToProcess() { return -1; }

	virtual bool					setFusionImage(int frameId, unsigned int width, unsigned int height, BYTE* pBuffer) = 0;
	virtual bool					readFusionImage(int frameId, IplImage* pImage) = 0;
	virtual int						getFusionFrame() const { return -1; }
	virtual int						getFusionFrameToProcess() { return -1; }

	virtual bool					setCameraPose(int frameId, const Matrix4& mat) = 0;
	virtual bool					readCameraPose(int frameId, Matrix4* mat) = 0;

private:
	PCEDeviceCacheImp (const PCEDeviceCacheImp&); // Disabled copy constructor
	PCEDeviceCacheImp& operator = (const PCEDeviceCacheImp&); // Disabled assignment operator
};

#endif