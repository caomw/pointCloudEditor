#ifndef _PCEKinectAudioGrabber
#define _PCEKinectAudioGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectAudioGrabber.h
//
// Author: Tingzhu Zhou
//
#include <windows.h>
#include <NuiApi.h>
#include <boost/thread/thread.hpp>

#include "../PCEDeviceCacheImp.h"


// Forwards

class PCEKinectAudioGrabber
{
public:
	PCEKinectAudioGrabber(INuiSensor* m_pNuiSensor);
	~PCEKinectAudioGrabber();

	bool								setupDevice(DWORD dwFlags);
	void								updateBuffer(PCEDeviceCacheImp* pBuffer) { m_pCache = pBuffer; }
	
	void								startThread ();
	void								stopThread();
	void								pauseThread() { m_threadPause = true; }
	bool								isThreadOn() const { return m_threadOn; }

private:
	void								grabThread ();

private:
	PCEKinectAudioGrabber (const PCEKinectAudioGrabber&); // Disabled copy constructor
	PCEKinectAudioGrabber& operator = (const PCEKinectAudioGrabber&); // Disabled assignment operator

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
	bool								m_threadPause;

	PCEDeviceCacheImp*					m_pCache;

	INuiSensor*							m_pNuiSensor;
};

#endif
