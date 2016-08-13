// File: PCEKinectManager.cpp
//
//
// Author: Tingzhu Zhou
//

#include "PCEKinectManager.h"

/// <summary>
/// This function will be called when Kinect device status changed
/// </summary>
void CALLBACK PCEKinectManager::StatusChangeCallback(HRESULT hrStatus, const OLECHAR* instancename, const OLECHAR* uniqueDeviceName, void* pUserData)
{
	PCEKinectManager* hManager = reinterpret_cast<PCEKinectManager*>(pUserData);

	if ( SUCCEEDED( hrStatus ) )
	{

	}
	else
	{

	}
}

PCEKinectManager::PCEKinectManager()
	: m_pNuiSensor(NULL)
	, m_pGrabber(NULL)
	, m_pFaceTracker(NULL)
	, m_pFusion(NULL)
{
	// Set the sensor status callback
	NuiSetDeviceStatusCallback(StatusChangeCallback, reinterpret_cast<void*>(this));
}

PCEKinectManager::~PCEKinectManager()
{
	shutdownDevice ();
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT PCEKinectManager::CreateFirstConnected()
{
	INuiSensor * pNuiSensor = NULL;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr) ) { return hr; }

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL == m_pNuiSensor)
	{
		return E_FAIL;
	}

	return hr;
}

bool PCEKinectManager::initializeDevice (int deviceFlag)
{
	if(!m_pNuiSensor)
	{
		if( FAILED(CreateFirstConnected()) )
			return false;
	}
	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	// Check initialize flag
	DWORD dwFlags = 0;
	if(deviceFlag & EDevice_Depth_On)
	{
		if(deviceFlag & EDevice_PlayIndex_On)
			dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
		else
			dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH;
	}
	if(deviceFlag & EDevice_Color_On)
		dwFlags |= NUI_INITIALIZE_FLAG_USES_COLOR;
	if(deviceFlag & EDevice_Skeleton_On)
		dwFlags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
	if(!isDeviceOn() || (m_pNuiSensor->NuiInitializationFlags() != dwFlags))
	{
		if(!m_pGrabber)
		{
			m_pGrabber = new PCEKinectGrabber(m_pNuiSensor);
		}
		assert(m_pGrabber);
		if(!m_pGrabber)
			return false;
		if( FAILED(m_pGrabber->setupDevice(dwFlags, cDepthResolution, cColorResolution)) )
			return false;
	}

	if(deviceFlag & EDevice_Face_On)
	{
		if(!m_pFaceTracker)
		{
			m_pFaceTracker = new PCEKinectFaceTracker();
		}
		assert(m_pFaceTracker);
		if( !m_pFaceTracker->setupTracker(cDepthResolution, cColorResolution) )
		{
			delete m_pFaceTracker;
			m_pFaceTracker = NULL;
			// printf
		}
		if(m_pGrabber)
			m_pGrabber->setSkeletonSeatedMode(true);
	}
	else
	{
		if(m_pGrabber)
			m_pGrabber->setSkeletonSeatedMode(false);
		if(m_pFaceTracker)
			m_pFaceTracker->stopThread();
		/*if(m_pGrabber)
			m_pGrabber->updateSkeletonSeatedMode(false);*/
	}

	if(deviceFlag & EDevice_Fusion_On)
	{
		if(!m_pFusion)
		{
			m_pFusion = new PCEKinectFusion();
		}
		assert(m_pFusion);
		if( !m_pFusion->initializeKinectFusion(cDepthResolution, cColorResolution) )
		{
			delete m_pFusion;
			m_pFusion = NULL;
			// printf
		}
	}
	else
	{
		if(m_pFusion)
			m_pFusion->stopThread();
	}
	return isDeviceOn();
}

void PCEKinectManager::startDevice ()
{
	if(!isDeviceOn())
	{
		//std::cerr << "Can't start sensor since it's not opened." << std::endl;
		return;
	}

	if(m_pGrabber)
		m_pGrabber->startThread();
	if(m_pFaceTracker)
		m_pFaceTracker->startThread();
	if(m_pFusion)
		m_pFusion->startThread();
}

void PCEKinectManager::pauseDevice ()
{
	if(m_pGrabber)
		m_pGrabber->pauseThread();
	if(m_pFaceTracker)
		m_pFaceTracker->pauseThread();
	if(m_pFusion)
		m_pFusion->pauseThread();
}

void PCEKinectManager::shutdownDevice()
{
	if(m_pGrabber)
		m_pGrabber->pauseThread();
	if (NULL != m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
		m_pNuiSensor = NULL;
	}
	if (NULL != m_pGrabber)
	{
		//m_Grabber->stop();
		delete m_pGrabber;
		m_pGrabber = NULL;
	}
	if (NULL != m_pFaceTracker)
	{
		//m_Grabber->stop();
		delete m_pFaceTracker;
		m_pFaceTracker = NULL;
	}
	if (NULL != m_pFusion)
	{
		//m_Grabber->stop();
		delete m_pFusion;
		m_pFusion = NULL;
	}
}

bool PCEKinectManager::isDeviceOn() const
{
	return ( m_pNuiSensor && (S_OK == m_pNuiSensor->NuiStatus()) && m_pGrabber );
}


void PCEKinectManager::updateBuffer(PCEDeviceCacheImp* pBuffer)
{
	if ( m_pGrabber )
		m_pGrabber->updateBuffer(pBuffer);
	if(m_pFaceTracker)
		m_pFaceTracker->updateBuffer(pBuffer);
	if(m_pFusion)
		m_pFusion->updateBuffer(pBuffer);
}

bool PCEKinectManager::updateNearMode(bool bNearMode)
{
	if(m_pGrabber)
	{
		m_pGrabber->setNearMode(bNearMode);
		return m_pGrabber->getNearMode();
	}
	return false;
}

void PCEKinectManager::updateElevationAngle(LONG tiltAngle)
{
	if ( m_pNuiSensor )
		m_pNuiSensor->NuiCameraElevationSetAngle(CoerceElevationAngle(tiltAngle));
}

void PCEKinectManager::updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius)
{
	if ( m_pGrabber )
		m_pGrabber->updateSkeletonSmoothParams(smoothing, correction, prediction, jitterRadius, maxDeviationRadius);
}
