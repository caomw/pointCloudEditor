#ifndef _PCEKinectFaceTracker
#define _PCEKinectFaceTracker
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectFaceTracker.h
//
// Author: Tingzhu Zhou
//

#include <boost/thread/thread.hpp>

#include "PCEMacros.h"
#include "../PCEDeviceCacheImp.h"

#include <FaceTrackLib.h>
#include <NuiApi.h>

// Forwards
class PCECompoundFrame;

// PCEKinectThread thread class
class PCEKinectFaceTracker
{
public:
	PCEKinectFaceTracker();
	~PCEKinectFaceTracker();

	bool								setupTracker(NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution);
	void								updateBuffer(PCEDeviceCacheImp* pBuffer);
	
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	bool								isThreadOn() const { return m_threadOn; }

private:
	void								runThread ();
	bool								readFrame(PCECompoundFrame* pFrame);
	void								getFaceShape(float zoomFactor, POINT* viewOffSet);

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
	bool								m_threadPause;

	IFTFaceTracker*						m_pFaceTracker;
	IFTResult*							m_pFTResult;
	IFTImage*							m_colorImage;
	IFTImage*							m_depthImage;
	FT_CAMERA_CONFIG					m_videoConfig;

	bool								m_LastTrackSucceeded;
	Vector3								m_hint3D[2];
	
	PCEDeviceCacheImp*					m_pCache;
};

#endif
