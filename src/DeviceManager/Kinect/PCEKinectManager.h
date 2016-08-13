#ifndef _PCEKinectManager
#define _PCEKinectManager
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectManager.h
//
//
// Author: Tingzhu Zhou
//
#include "..\PCEDeviceManager.h"
#include "PCEKinectGrabber.h"
#include "PCEKinectFaceTracker.h"
#include "PCEKinectFusion.h"

// PCEKinectManager thread class
class PCEKinectManager : public PCEDeviceManager
{
	static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION   cColorResolution = NUI_IMAGE_RESOLUTION_640x480;
public:
	PCEKinectManager();
	virtual ~PCEKinectManager();

	virtual bool		initializeDevice(int deviceFlag) override;
	virtual void		updateBuffer(PCEDeviceCacheImp* pBuffer) override;
	virtual bool		isDeviceOn() const override;
	virtual void		startDevice() override;
	virtual void		pauseDevice() override;
	virtual void		shutdownDevice() override;

	virtual bool		updateNearMode(bool bNearMode) override;
	virtual void		updateElevationAngle(long tiltAngle) override;
	virtual void		updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius) override;

protected:
	/// <summary>
	/// Create the first connected Kinect found 
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT				CreateFirstConnected();
	/// <summary>
	/// Coerce the requested elevation angle to a valid angle
	/// </summary>
	/// <param name="tiltAngle">The requested angle</param>
	/// <returns>The angle after coerced</returns>
	inline long			CoerceElevationAngle(long tiltAngle)
	{
		return std::min(std::max((long)NUI_CAMERA_ELEVATION_MINIMUM, tiltAngle), (long)NUI_CAMERA_ELEVATION_MAXIMUM);
	}

	void				updateCacheStatus();

	/// <summary>
	/// Called on Kinect device status changed. It will update the sensor chooser UI control
	/// based on the new sensor status. It may also updates the sensor instance if needed
	/// </summary>
	static void CALLBACK StatusChangeCallback(HRESULT hrStatus, const OLECHAR* instancename, const OLECHAR* uniqueDeviceName, void* pUserData);

private:
	INuiSensor*							m_pNuiSensor;

	PCEKinectGrabber*					m_pGrabber;
	PCEKinectFaceTracker*				m_pFaceTracker;
	PCEKinectFusion*					m_pFusion;
};

#endif
