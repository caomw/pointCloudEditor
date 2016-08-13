#ifndef _PCEDeviceBuffer
#define _PCEDeviceBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceBuffer.h
//
// Author: Tingzhu Zhou
//
#include "PCEDeviceCacheImp.h"

#include "PCECompoundFrame.h"
#include "Shape/PCEFaceTrackingResult.h"

class PCEDeviceBuffer : public PCEDeviceCacheImp
{
public:
	PCEDeviceBuffer();
	virtual ~PCEDeviceBuffer();

	virtual void				clear() override;

	virtual PCECompoundFrame*	acquireCompoundFrame() override;
	virtual PCECompoundFrame*	getLatestCompoundFrame() override;
	virtual PCECompoundFrame*	getCompoundFrame(int frameId) override;

	// Face
	virtual bool				setFaceShape(PCEFaceTrackingResult* pFaceShape) override;
	virtual bool				readFaceShape(int frameId, PCEFaceTrackingResult* pFaceShape) override;

	// Fusion
	virtual bool				setFusionImage(int frameId, unsigned int width, unsigned int height, BYTE* pBuffer) override;
	virtual bool				readFusionImage(int frameId, IplImage* pImage) override;

	// Camera
	virtual bool				setCameraPose(int frameId, const Matrix4& mat) override;
	virtual bool				readCameraPose(int frameId, Matrix4* mat) override;

private:
	PCECompoundFrame			m_frame;

	//Face
	boost::mutex				m_bFaceMutex;
	PCEFaceTrackingResult*		m_pFaceShape;

	// Fusion
	boost::mutex				m_bFusionImageMutex;
	IplImage*					m_pFusionImage;

	boost::mutex				m_bCameraMutex;
	Matrix4						m_cameraPose;
};

#endif