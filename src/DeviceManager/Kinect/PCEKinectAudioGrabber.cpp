// File: PCEKinectGrabber.cpp
//
// Dependency Graph Node: PCEKinectGrabber
//
// Author: Tingzhu Zhou
//
#include "PCEKinectAudioGrabber.h"

PCEKinectAudioGrabber::PCEKinectAudioGrabber(INuiSensor* pNuiSensor)
	: m_pNuiSensor(pNuiSensor)
	, m_pCache(NULL)
	, m_threadOn(false)
	, m_threadPause(false)
{
}

PCEKinectAudioGrabber::~PCEKinectAudioGrabber()
{
	stopThread ();
	m_pNuiSensor = NULL;
	m_pCache = NULL;
}

bool PCEKinectAudioGrabber::setupDevice(DWORD dwFlags)
{
	m_threadOn = false;

	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	return true;
}

void PCEKinectAudioGrabber::grabThread ()
{
	while (true)
	{
		boost::this_thread::sleep (boost::posix_time::milliseconds (100));
		if (!m_threadOn)
			break;
		if (m_threadPause)
			continue;
		if (!m_pCache)
			continue;
		
	}
}

void PCEKinectAudioGrabber::startThread ()
{
	if(m_pNuiSensor && !m_threadOn)
	{
		m_Thread.reset (new boost::thread (boost::bind (&PCEKinectAudioGrabber::grabThread, this)));
		m_threadOn = true;
	}
	m_threadPause = false;
}

void PCEKinectAudioGrabber::stopThread ()
{
	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}
}

