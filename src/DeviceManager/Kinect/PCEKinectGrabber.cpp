// File: PCEKinectGrabber.cpp
//
// Dependency Graph Node: PCEKinectGrabber
//
// Author: Tingzhu Zhou
//
#include <omp.h>

#include "PCEKinectGrabber.h"

#include "../PCECompoundFrame.h"

PCEKinectGrabber::PCEKinectGrabber(INuiSensor* pNuiSensor)
	: m_pNuiSensor(pNuiSensor)
	, m_pMapper(NULL)
	, m_pCache(NULL)
	, m_pDepthFrame(NULL)
	, m_pColorFrame(NULL)
	, m_pSkeletonFrame(NULL)
	, m_depthResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_colorResolution(NUI_IMAGE_RESOLUTION_640x480)
	, m_threadOn(false)
	, m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE)
	, m_pDepthStreamHandle(INVALID_HANDLE_VALUE)
	, m_hNextColorFrameEvent(INVALID_HANDLE_VALUE)
	, m_pColorStreamHandle(INVALID_HANDLE_VALUE)
	, m_hNextSkeletonFrameEvent(INVALID_HANDLE_VALUE)
	, m_bNearMode(false)
	, m_bSkeletonSeatedMode(false)
	, m_minDepth(NUI_IMAGE_DEPTH_MINIMUM>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_maxDepth(NUI_IMAGE_DEPTH_MAXIMUM>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_minNearDepth(NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_maxNearDepth(NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE>>NUI_IMAGE_PLAYER_INDEX_SHIFT)
	, m_depthFrameCount(0)
	, m_lastDepthTime(0)
	, m_colorFrameCount(0)
	, m_lastColorTime(0)
{
	m_skeletonSmoothParams.fSmoothing = 0.5f;
	m_skeletonSmoothParams.fCorrection = 0.5f;
	m_skeletonSmoothParams.fPrediction = 0.5f;
	m_skeletonSmoothParams.fJitterRadius = 0.05f;
	m_skeletonSmoothParams.fMaxDeviationRadius = 0.04f;
}

PCEKinectGrabber::~PCEKinectGrabber()
{
	stopThread ();
	m_pNuiSensor = NULL;
	m_pCache = NULL;
	if(m_pMapper)
	{
		m_pMapper->Release();
		m_pMapper = NULL;
	}
	if(m_pDepthFrame)
	{
		delete m_pDepthFrame;
		m_pDepthFrame = NULL;
	}
	if(m_pColorFrame)
	{
		delete m_pColorFrame;
		m_pColorFrame = NULL;
	}
	if(m_pSkeletonFrame)
	{
		delete m_pSkeletonFrame;
		m_pSkeletonFrame = NULL;
	}
}

bool PCEKinectGrabber::setupDevice(DWORD dwFlags, NUI_IMAGE_RESOLUTION depthResolution, NUI_IMAGE_RESOLUTION colorResolution)
{
	m_threadOn = false;

	assert(m_pNuiSensor);
	if(!m_pNuiSensor)
		return false;

	m_depthResolution = depthResolution;
	m_colorResolution = colorResolution;

	HRESULT hr;
	// Initialize the Kinect and specify that we'll be using depth
	hr = m_pNuiSensor->NuiInitialize(dwFlags);
	if (FAILED(hr) )
	{
		printf_s("Device Initialize failed.\n");
		return false;
	}

	// Initialize the image buffer
	//WCHAR *sensorId = m_pNuiSensor->NuiDeviceConnectionId();

	// Open the streams
	if(dwFlags & (NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|NUI_INITIALIZE_FLAG_USES_DEPTH))
	{
		// Create an event that will be signaled when depth data is available
		if(INVALID_HANDLE_VALUE == m_hNextDepthFrameEvent)
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextDepthFrameEvent);

		// Open a depth image stream to receive depth frames
		NUI_IMAGE_TYPE imageType = (dwFlags & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH;
		hr = m_pNuiSensor->NuiImageStreamOpen(
			imageType,
			m_depthResolution,
			0,
			2,
			m_hNextDepthFrameEvent,
			&m_pDepthStreamHandle);
		if (SUCCEEDED(hr) )
		{
			DWORD depthWidth = 0;
			DWORD depthHeight = 0;
			NuiImageResolutionToSize(m_depthResolution, depthWidth, depthHeight);
			if(m_pDepthFrame)
			{
				if((m_pDepthFrame->getWidth() != depthWidth) || (m_pDepthFrame->getHeight() != depthHeight))
				{
					if(m_pDepthFrame)
					{
						delete m_pDepthFrame;
						m_pDepthFrame = NULL;
					}
				}
			}
			if(!m_pDepthFrame)
			{
				LONGLONG invalidTime = 0;
				m_pDepthFrame = new PCEKinectDepthImageFrame(invalidTime, depthWidth, depthHeight);
			}
		}
		else
		{
			printf_s("Depth Frame Initialize failed.\n");
		}
	}

	if(dwFlags & NUI_INITIALIZE_FLAG_USES_COLOR)
	{
		// Create an event that will be signaled when color data is available
		if(INVALID_HANDLE_VALUE == m_hNextColorFrameEvent)
			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextColorFrameEvent);

		// Open a color image stream to receive color frames
		hr = m_pNuiSensor->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR,
			m_colorResolution,
			0,
			2,
			m_hNextColorFrameEvent,
			&m_pColorStreamHandle );
		if (SUCCEEDED(hr) )
		{
			DWORD colorWidth = 0;
			DWORD colorHeight = 0;
			NuiImageResolutionToSize(m_colorResolution, colorWidth, colorHeight);
			if(m_pColorFrame)
			{
				if((m_pColorFrame->getWidth() != colorWidth) || (m_pColorFrame->getHeight() != colorHeight))
				{
					if(m_pColorFrame)
					{
						delete m_pColorFrame;
						m_pColorFrame = NULL;
					}
				}
			}
			if(!m_pColorFrame)
			{
				LONGLONG invalidTime = 0;
				m_pColorFrame = new PCEKinectColorImageFrame(invalidTime, colorWidth, colorHeight);
			}
		}
		else
		{
			printf_s("Depth Frame Initialize failed.\n");
		}

		if (SUCCEEDED(hr))
		{
			// Create the coordinate mapper for converting color to depth space
			hr = m_pNuiSensor->NuiGetCoordinateMapper(&m_pMapper);
		}
	}
	else if(m_pMapper)
	{
		m_pMapper->Release();
		m_pMapper = NULL;
	}
	

	if(dwFlags & NUI_INITIALIZE_FLAG_USES_SKELETON)
	{
		// Create an event that will be signaled when skeleton data is available
		if(INVALID_HANDLE_VALUE == m_hNextSkeletonFrameEvent)
			m_hNextSkeletonFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		else
			ResetEvent(m_hNextSkeletonFrameEvent);

		hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonFrameEvent, 0);
		if (SUCCEEDED(hr) )
		{
			if(!m_pSkeletonFrame)
				m_pSkeletonFrame = new PCESkeletonFrame();
		}
		else
		{
			printf_s("Skeleton Frame Initialize failed.\n");
		}
	}

	updateNearMode();

	return true;
}

/// <summary>
/// Process depth data received from Kinect
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
bool PCEKinectGrabber::grabDepthFrame()
{
	if(!m_pDepthFrame)
	{
		printf_s("No depth frame available.\n");
		return false;
	}

	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth image.\n");
		return false;
	}

	INuiFrameTexture* pCompressedFrameTexture = imageFrame.pFrameTexture;

	NUI_LOCKED_RECT LockedCompressedRect;
	hr = pCompressedFrameTexture->LockRect(0, &LockedCompressedRect, NULL, 0);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth frame.\n");
		return false;
	}
	if (LockedCompressedRect.Pitch != 0)
	{
		USHORT* dstBuffer = m_pDepthFrame->getCompressedBuffer();
		assert(dstBuffer);
		if(dstBuffer)
		{
			errno_t err = memcpy_s(dstBuffer, m_pDepthFrame->getCompressedBufferSize(), PBYTE(LockedCompressedRect.pBits), pCompressedFrameTexture->BufferLen());
			if(0 != err);
			{
				printf_s("No compressed depth frame buffer available.\n");
			}
		}
	}
	hr = pCompressedFrameTexture->UnlockRect(0);

	/////////////////////////////
	INuiFrameTexture* pFrameTexture = nullptr;
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &m_bNearMode, &pFrameTexture);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth frame.\n");
		return false;
	}

	NUI_LOCKED_RECT LockedRect;
	hr = pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab depth image.\n");
		return false;
	}
	bool returnStatus = false;
	if (LockedRect.Pitch != 0)
	{
		NUI_DEPTH_IMAGE_PIXEL* dstBuffer = m_pDepthFrame->getExtendedBuffer();
		assert(dstBuffer);
		if(dstBuffer)
		{
			errno_t err = memcpy_s(dstBuffer, m_pDepthFrame->getExtendedBufferSize(), PBYTE(LockedRect.pBits), pFrameTexture->BufferLen());
			returnStatus = (0 == err);
			if(returnStatus)
				m_pDepthFrame->setTimeStamp(imageFrame.liTimeStamp.QuadPart);

			if(m_pMapper)
			{
				DWORD colorWidth = 0;
				DWORD colorHeight = 0;
				NuiImageResolutionToSize(m_colorResolution, colorWidth, colorHeight);
				m_pDepthFrame->allocateCoordinates();
				// Get the coordinates to convert color to depth space
				hr = m_pMapper->MapDepthFrameToColorFrame(
					m_depthResolution,
					m_pDepthFrame->getWidth()*m_pDepthFrame->getHeight(),
					(NUI_DEPTH_IMAGE_PIXEL*)LockedRect.pBits,
					NUI_IMAGE_TYPE_COLOR,
					m_colorResolution,
					colorWidth*colorHeight,   // the color coordinates that get set are the same array size as the depth image
					m_pDepthFrame->colorCoordinates() );
				if(FAILED(hr))
				{
					printf_s("Failed to grab color coordinates.\n");
					m_pDepthFrame->deleteCoordinates();
				}
			}
			else
			{
				m_pDepthFrame->deleteCoordinates();
			}
		}
		else
		{
			printf_s("No depth frame buffer available.\n");
		}
	}

	hr = pFrameTexture->UnlockRect(0);
	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to release depth frame.\n");
	}

	return returnStatus;
}

/// <summary>
/// Process color data received from Kinect
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
bool PCEKinectGrabber::grabColorFrame()
{
	if(!m_pColorFrame)
	{
		printf_s("No color frame available.\n");
		return false;
	}

	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab color frame.\n");
		return false;
	}

	NUI_LOCKED_RECT LockedRect;
	hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab color image.\n");
		return false;
	}

	bool returnStatus = false;
	if (LockedRect.Pitch != 0)
	{
		BYTE* dstBuffer = m_pColorFrame->getBuffer();
		assert(dstBuffer);
		if(dstBuffer)
		{
			errno_t err = memcpy_s(dstBuffer, m_pColorFrame->getBufferSize(), PBYTE(LockedRect.pBits), imageFrame.pFrameTexture->BufferLen());
			returnStatus = (0 == err);
			if(returnStatus)
				m_pColorFrame->setTimeStamp(imageFrame.liTimeStamp.QuadPart);
		}
		else
		{
			printf_s("No color frame buffer available.\n");
		}
	}

	hr = imageFrame.pFrameTexture->UnlockRect(0);
	hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to release depth frame.\n");
	}

	return returnStatus;
}

bool PCEKinectGrabber::grabSkeletonFrame()
{
	if(!m_pSkeletonFrame)
	{
		printf_s("No skeleton frame available.\n");
		return false;
	}

	NUI_SKELETON_FRAME skeletonFrame = {0};
	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
	if ( FAILED(hr) )
	{
		printf_s("Failed to grab skeleton frame.\n");
		return false;
	}

	m_pSkeletonFrame->clear();
	m_pSkeletonFrame->setTimeStamp(skeletonFrame.liTimeStamp.QuadPart);
	
	// smooth out the skeleton data
	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, &m_skeletonSmoothParams);

	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
	{
		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

		if (NUI_SKELETON_TRACKED == trackingState)
		{
			const NUI_SKELETON_DATA & skel = skeletonFrame.SkeletonData[i];

			PCESkeletonPosition skeletonJoints;
			for (int jointId = 0; jointId < SKELETON_POSITION_COUNT; jointId++)
			{
				bool isInferred = true;
				NUI_SKELETON_POSITION_TRACKING_STATE jointState = skel.eSkeletonPositionTrackingState[jointId];
				if (jointState == NUI_SKELETON_POSITION_TRACKED)
				{
					isInferred = false;
				}
				else if (jointState == NUI_SKELETON_POSITION_INFERRED)
				{
					isInferred = true;
				}
				else
				{
					continue;
				}

				Vector4 jointInSkeletonSpace = skeletonFrame.SkeletonData[i].SkeletonPositions[jointId];
				skeletonJoints.setJointInSkeletonSpace((PCEJointType)jointId, (float)jointInSkeletonSpace.x, (float)jointInSkeletonSpace.y, (float)jointInSkeletonSpace.z, isInferred);
				LONG pos_x, pos_y;
				USHORT pos_depth;
				NuiTransformSkeletonToDepthImage(
					jointInSkeletonSpace,
					&pos_x,
					&pos_y,
					&pos_depth,
					m_depthResolution );
				if(pos_depth == NUI_IMAGE_DEPTH_NO_VALUE || (pos_x == 0 && pos_y == 0))
					isInferred = true;
				skeletonJoints.setJointData((PCEJointType)jointId, (float)pos_x, (float)pos_y, (float)pos_depth, isInferred);
			}
			m_pSkeletonFrame->pushSkeletonPosition(skeletonJoints);
		}
		else if (NUI_SKELETON_POSITION_ONLY == trackingState)
		{
			LONG pos_x, pos_y;
			USHORT pos_depth;
			NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].Position, &pos_x, &pos_y, &pos_depth);
			PCEJointPosition pos;
			pos.x = (float)pos_x;
			pos.y = (float)pos_y;
			pos.z = (float)pos_depth;
			if(pos_depth != NUI_IMAGE_DEPTH_NO_VALUE && (pos_x != 0 || pos_y != 0))
				m_pSkeletonFrame->pushJointPosition(pos);
		}
	}

	return true;
}

void PCEKinectGrabber::grabThread ()
{
	HANDLE          hEvents[4];

	// Configure events to be listened on
	hEvents[0] = m_hNextDepthFrameEvent;
	hEvents[1] = m_hNextColorFrameEvent;
	hEvents[2] = m_hNextSkeletonFrameEvent;

	while (true)
	{
		if (!m_threadOn)
			break;
		if (!m_pCache)
			continue;
		
		// Wait for an event to be signaled
		WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);

		bool bDepthGrabbed = false;
		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
		{
			bDepthGrabbed = grabDepthFrame();
			if(bDepthGrabbed)
			{
				// Compute FPS
				m_depthFrameCount ++;
				LONGLONG depthTime = m_pDepthFrame->getTimeStamp();
				LONGLONG span      = depthTime - m_lastDepthTime;
				if (span >= 1000)
				{
					m_pDepthFrame->setFPS( (UINT)((double)m_depthFrameCount * 1000.0 / (double)span + 0.5) );
					m_lastDepthTime = depthTime;
					m_depthFrameCount = 0;
				}
			}
			else
			{
				printf_s( "Failed to grab the depth frame.\n" );
			}
		}
		bool bColorGrabbed = false;
		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
		{
			bColorGrabbed = grabColorFrame();
			if(bColorGrabbed)
			{
				// Compute FPS
				m_colorFrameCount ++;
				LONGLONG colorTime = m_pColorFrame->getTimeStamp();
				LONGLONG span      = colorTime - m_lastColorTime;
				if (span >= 1000)
				{
					m_pColorFrame->setFPS( (UINT)((double)m_colorFrameCount * 1000.0 / (double)span + 0.5) );
					m_lastColorTime = colorTime;
					m_colorFrameCount = 0;
				}
			}
			else
			{
				printf_s( "Failed to grab the color frame.\n" );
			}
		}

		// Wait for 0ms, just quickly test if it is time to process a skeleton
		bool bSkeletonGrabbed = false;
		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonFrameEvent, 0) )
		{
			bSkeletonGrabbed = grabSkeletonFrame();
			if(!bSkeletonGrabbed)
			{
				printf_s( "Failed to grab the skeleton frame.\n" );
			}
		}

		if(bDepthGrabbed || bSkeletonGrabbed)
			frameToCache(bDepthGrabbed, bColorGrabbed, bSkeletonGrabbed);
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
	}
}

bool PCEKinectGrabber::frameToCache(bool bDepthGrabbed, bool bColorGrabbed, bool bSkeletonGrabbed)
{
	assert(m_pCache);
	if(!m_pCache)
	{
		printf_s( "No cache available in Grabber.\n" );
		return false;
	}

	PCECompoundFrame* pCompoundFrame = m_pCache->acquireCompoundFrame();
	if(!pCompoundFrame)
	{
		printf_s( "No compound frame available in Grabber.\n" );
		return false;
	}

	Vector4 reading;
	if( SUCCEEDED(m_pNuiSensor->NuiAccelerometerGetCurrentReading(&reading)) )
		pCompoundFrame->setAccelerometerReading(reading);
	
	if(bDepthGrabbed && m_pDepthFrame)
	{
		pCompoundFrame->setTimeStamp(m_pDepthFrame->getTimeStamp());
		pCompoundFrame->setDepthFPS(m_pDepthFrame->getFPS());

		NUI_DEPTH_IMAGE_PIXEL* pDepthBuffer = m_pDepthFrame ? m_pDepthFrame->getExtendedBuffer() : NULL;
		NUI_COLOR_IMAGE_POINT* pColorCoordinates = m_pDepthFrame ? m_pDepthFrame->colorCoordinates() : NULL;
		const UINT depthWidth = m_pDepthFrame ? m_pDepthFrame->getWidth() : 0;
		const UINT depthHeight = m_pDepthFrame ? m_pDepthFrame->getHeight() : 0;
		USHORT* pCachedDepthBuffer = pDepthBuffer ? pCompoundFrame->allocateDepthBuffer(depthWidth, depthHeight) : NULL;

		bool bHasPlayerIndex = (m_pNuiSensor->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
		UINT minDepth = m_bNearMode ? m_minNearDepth : m_minDepth;
		UINT maxDepth = m_bNearMode ? m_maxNearDepth : m_maxDepth;
		
		const UINT colorWidth = m_pColorFrame ? m_pColorFrame->getWidth() : 0;
		const UINT colorHeight = m_pColorFrame ? m_pColorFrame->getHeight() : 0;
		const UINT colorBytesPerPixel = m_pColorFrame ? m_pColorFrame->getBytesPerPixel() : 0;
		BYTE* pColorBuffer = NULL;
		BYTE* pAlignedColorBuffer = NULL;
		if(m_pColorFrame &&
			(std::abs(m_pDepthFrame->getTimeStamp() - m_pColorFrame->getTimeStamp()) < PCEDeviceFrame::sHalfADepthFrameMs) )
		{
			pColorBuffer = m_pColorFrame->getBuffer();
			pAlignedColorBuffer = pColorBuffer ? pCompoundFrame->allocateAlignedColorBuffer(colorWidth, colorHeight) : NULL;
			pCompoundFrame->setColorFPS(m_pColorFrame->getFPS());
		}
		if(pDepthBuffer && pCachedDepthBuffer)
		{
			// Lock
			pCompoundFrame->writeImageLock();
			pCompoundFrame->resetIndices();
			// loop over each row and column of the color
#pragma omp parallel for schedule(dynamic)
			for (UINT y = 0; y < depthHeight; ++y)
			{
				for (UINT x = 0; x < depthWidth; ++x)
				{
					// calculate index into depth array
					UINT depthIndex = x + y * depthWidth;
					NUI_DEPTH_IMAGE_PIXEL depthPixel = pDepthBuffer[depthIndex];
					USHORT depth = depthPixel.depth;
					bool bValidDepth = (depth >= minDepth && depth <= maxDepth);
					pCachedDepthBuffer[depthIndex] = depth;//(bValidDepth ? (depth) : 0);
				
					/*if( x == colorWidth/2 && y == colorHeight/2)
						pnt.x = (float)x;*/
					if(bValidDepth)
					{
						pCompoundFrame->pushValidIndex(depthIndex);
						if(bHasPlayerIndex)
						{
							USHORT player = depthPixel.playerIndex;
							if(1 == player)
								pCompoundFrame->pushFirstPlayerIndex(depthIndex);
							else if(2 == player)
								pCompoundFrame->pushSecondPlayerIndex(depthIndex);
						}
					}

					// calculate index into color array
					if(pColorBuffer && pAlignedColorBuffer)
					{
						UINT colorIndex = depthIndex;
						if(pColorCoordinates)
						{
							// retrieve the depth to color mapping for the current depth pixel
							UINT colorInDepthX = pColorCoordinates[depthIndex].x;
							UINT colorInDepthY = pColorCoordinates[depthIndex].y;

							// make sure the depth pixel maps to a valid point in color space
							if ( colorInDepthX >= 0 && colorInDepthX < colorWidth && colorInDepthY >= 0 && colorInDepthY < colorHeight )
							{
								// calculate index into color array
								colorIndex = colorInDepthX + colorInDepthY * colorWidth;
							}
						}
						BYTE* colorValue = pColorBuffer + colorIndex * colorBytesPerPixel;
						BYTE* alignedColorValue = pAlignedColorBuffer + depthIndex * colorBytesPerPixel;
						alignedColorValue[0] = colorValue[0];
						alignedColorValue[1] = colorValue[1];
						alignedColorValue[2] = colorValue[2];
						alignedColorValue[3] = colorValue[3];
					}
				}
			}
			pCompoundFrame->writeImageUnlock();
		}
		else
		{
			printf_s( "No depth buffer when grabber to cache.\n" );
		}
	}
	
	if(bSkeletonGrabbed && m_pSkeletonFrame)
	{
		pCompoundFrame->cacheSkeletonFrame(*m_pSkeletonFrame);
	}
	/*else
	{
		pCompoundFrame->resetSkeleton();
	}*/
	
	return true;
}

void PCEKinectGrabber::startThread ()
{
	if(m_pNuiSensor && !m_threadOn)
	{
		m_threadOn = true;
		m_Thread.reset (new boost::thread (boost::bind (&PCEKinectGrabber::grabThread, this)));
	}
}

void PCEKinectGrabber::pauseThread()
{
	m_threadOn = false;
	m_Thread->join();
}

void PCEKinectGrabber::stopThread ()
{
	// Reset all the event to nonsignaled state
	ResetEvent(m_hNextDepthFrameEvent);
	ResetEvent(m_hNextColorFrameEvent);
	ResetEvent(m_hNextSkeletonFrameEvent);

	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}
}

void PCEKinectGrabber::updateSkeletonMode()
{
	if ( m_pNuiSensor &&
		INVALID_HANDLE_VALUE != m_hNextSkeletonFrameEvent &&
		(m_pNuiSensor->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_SKELETON) )
	{
		DWORD dwSkeletonFlags = m_bNearMode ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE : 0;
		if (m_bSkeletonSeatedMode)
		{
			dwSkeletonFlags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
		}
		m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonFrameEvent, dwSkeletonFlags);
	}
}

void PCEKinectGrabber::updateNearMode()
{
	if ( m_pNuiSensor && INVALID_HANDLE_VALUE != m_pDepthStreamHandle)
	{
		m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_bNearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
	}
	updateSkeletonMode();
}

void PCEKinectGrabber::setNearMode(bool bNearMode)
{
	m_bNearMode = bNearMode;
	updateNearMode();
}

bool PCEKinectGrabber::getNearMode() const
{
	bool bNearMode = false;
	if ( m_pNuiSensor && INVALID_HANDLE_VALUE != m_pDepthStreamHandle)
	{
		DWORD imageFlag;
		m_pNuiSensor->NuiImageStreamGetImageFrameFlags(m_pDepthStreamHandle, &imageFlag);
		bNearMode = imageFlag & NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
	}
	return bNearMode;
}

void PCEKinectGrabber::setSkeletonSeatedMode(bool bSeatedMode)
{
	m_bSkeletonSeatedMode = bSeatedMode;
	updateSkeletonMode();
}

void PCEKinectGrabber::updateSkeletonSmoothParams(float smoothing, float correction, float prediction, float jitterRadius, float maxDeviationRadius)
{
	//fSmoothing：平滑值(Smoothing)属性，设置处理骨骼数据帧时的平滑量，接受一个0-1的浮点值，值越大，平滑的越多。0表示不进行平滑
	//fCorrection：修正值(Correction)属性，接受一个从0-1的浮点型。值越小，修正越多。
	//fJitterRadius：抖动半径(JitterRadius)属性，设置修正的半径，如果关节点“抖动”超过了设置的这个半径，将会被纠正到这个半径之内。该属性为浮点型，单位为米。
	//fMaxDeviationRadius：最大偏离半径(MaxDeviationRadius)属性，用来和抖动半径一起来设置抖动半径的最大边界。任何超过这一半径的点都不会认为是抖动产生的，而被认定为是一个新的点。该属性为浮点型，单位为米。
	//fPrediction：预测帧大小(Prediction)属性，返回用来进行平滑需要的骨骼帧的数目。
	//对骨骼关节点进行平滑处理会产生性能开销。平滑处理的越多，性能消耗越大。设置平滑参数没有经验可以遵循。需要不断的测试和调试已达到最好的性能和效果。在程序运行的不同阶段，可能需要设置不同的平滑参数。

	// Some smoothing with little latency (defaults).  
	// Only filters out small jitters.  
	// Good for gesture recognition.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS DefaultParams =   
	//{0.5f, 0.5f, 0.5f, 0.05f, 0.04f};  

	// Smoothed with some latency.  
	// Filters out medium jitters.  
	// Good for a menu system that needs to be smooth but  
	// doesn't need the reduced latency as much as gesture recognition does.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS SomewhatLatentParams =   
	//{0.5f, 0.1f, 0.5f, 0.1f, 0.1f};  

	// Very smooth, but with a lot of latency.  
	// Filters out large jitters.  
	// Good for situations where smooth data is absolutely required  
	// and latency is not an issue.  
	//const NUI_TRANSFORM_SMOOTH_PARAMETERS VerySmoothParams =   
	//{0.7f, 0.3f, 1.0f, 1.0f, 1.0f};
	m_skeletonSmoothParams.fSmoothing = smoothing;
	m_skeletonSmoothParams.fCorrection = correction;
	m_skeletonSmoothParams.fPrediction = prediction;
	m_skeletonSmoothParams.fJitterRadius = jitterRadius;
	m_skeletonSmoothParams.fMaxDeviationRadius = maxDeviationRadius;
}

void PCEKinectGrabber::updateDepthRange(UINT minDepth, UINT maxDepth)
{
	if(minDepth > NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_minDepth = minDepth;
	else
		m_minDepth = NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

	if(maxDepth < NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_maxDepth = maxDepth;
	else
		m_maxDepth = NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
}

void PCEKinectGrabber::updateNearDepthRange(UINT minDepth, UINT maxDepth)
{
	if(minDepth > NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_minNearDepth = minDepth;
	else
		m_minNearDepth = NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

	if(maxDepth < NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT)
		m_maxNearDepth = maxDepth;
	else
		m_maxNearDepth = NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
}
