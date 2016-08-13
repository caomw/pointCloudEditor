#ifndef _PCEKinectGrabber
#define _PCEKinectGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectGrabber.h
//
// Author: Tingzhu Zhou
//
#include <windows.h>
#include <boost/thread/thread.hpp>

#include "../PCESkeletonFrame.h"
#include "../PCEDeviceCacheImp.h"
#include "PCEKinectImageFrame.h"

// Forwards

// PCEKinectThread thread class
class PCEKinectGrabber
{
public:
	PCEKinectGrabber(INuiSensor* m_pNuiSensor);
	~PCEKinectGrabber();

	bool								setupDevice(DWORD dwFlags, NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION	colorResolution);
	void								updateBuffer(PCEDeviceCacheImp* pBuffer) { m_pCache = pBuffer; }
	
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	bool								isThreadOn() const { return m_threadOn; }

	void								setNearMode(bool bNearMode);
	bool								getNearMode() const;
	void								setSkeletonSeatedMode(bool bSeatedMode);
	void								updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius);
	void								updateDepthRange(UINT minDepth, UINT maxDepth);
	void								updateNearDepthRange(UINT minDepth, UINT maxDepth);

private:
	void								updateNearMode();
	void								updateSkeletonMode();
	void								grabThread ();
	bool								grabDepthFrame ();
	bool								grabColorFrame ();
	bool								grabSkeletonFrame ();
	bool								frameToCache(bool bDepthGrabbed, bool bColorGrabbed, bool bSkeletonGrabbed);

private:
	PCEKinectGrabber (const PCEKinectGrabber&); // Disabled copy constructor
	PCEKinectGrabber& operator = (const PCEKinectGrabber&); // Disabled assignment operator

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;

	PCEDeviceCacheImp*					m_pCache;

	INuiSensor*							m_pNuiSensor;
	INuiCoordinateMapper*				m_pMapper;

	NUI_IMAGE_RESOLUTION				m_depthResolution;
	PCEKinectDepthImageFrame*			m_pDepthFrame;
	UINT								m_depthFrameCount;
	LONGLONG							m_lastDepthTime;

	NUI_IMAGE_RESOLUTION				m_colorResolution;
	PCEKinectColorImageFrame*			m_pColorFrame;
	UINT								m_colorFrameCount;
	LONGLONG							m_lastColorTime;

	PCESkeletonFrame*					m_pSkeletonFrame;
	
	HANDLE								m_hNextDepthFrameEvent;
	HANDLE								m_pDepthStreamHandle;
	HANDLE								m_hNextColorFrameEvent;
	HANDLE								m_pColorStreamHandle;
	HANDLE								m_hNextSkeletonFrameEvent;

	BOOL								m_bNearMode;
	BOOL								m_bSkeletonSeatedMode;
	UINT								m_minDepth;
	UINT								m_maxDepth;
	UINT								m_minNearDepth;
	UINT								m_maxNearDepth;
	NUI_TRANSFORM_SMOOTH_PARAMETERS		m_skeletonSmoothParams;
};

#endif
