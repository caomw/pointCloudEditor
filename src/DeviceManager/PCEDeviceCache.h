#ifndef _PCEDeviceCache
#define _PCEDeviceCache
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceCache.h
//
// Author: Tingzhu Zhou
//
#include "PCEDeviceBuffer.h"
#include "PCEDataPtrCache.h"
#include "Shape/PCEFusionMesh.h"

class PCEDeviceCache : public PCEDeviceCacheImp
{
	static const size_t						KINECT_CACHE_SIZE = 200;
public:
	PCEDeviceCache ()
		: m_faceTrackingFrame(-1)
		, m_fusionFrame(-1)
	{
		m_framePtrArray.setCapacity(KINECT_CACHE_SIZE);
	}
	virtual ~PCEDeviceCache ()
	{
		clear();
	}
	
	virtual void clear() override
	{
		m_framePtrArray.clear();
		m_faceTrackingFrame = -1;
		m_fusionFrame = -1;
	}

	virtual PCECompoundFrame*	acquireCompoundFrame() override
	{
		PCEDeviceBuffer* pBuffer = new PCEDeviceBuffer();
		assert(pBuffer);
		if( !m_framePtrArray.pushBack(pBuffer) )
		{
			// Memory is full!
			printf_s( "The cache is full!\n" );
			if(pBuffer)
				delete pBuffer;
			return NULL;
		}
		return pBuffer->acquireCompoundFrame();
	}

	virtual PCECompoundFrame*	getLatestCompoundFrame() override
	{
		PCEDeviceBuffer* pBuffer = m_framePtrArray.getBackDataPtr();
		if(!pBuffer)
		{
			printf_s( "Failed to get the latest frame from cache!\n" );
			return NULL;
		}
		return pBuffer->getLatestCompoundFrame();
	}

	virtual PCECompoundFrame*	getCompoundFrame(int frameId) override
	{
		if(frameId < 0)
			return NULL;
		PCEDeviceBuffer* pBuffer = m_framePtrArray.getDataPtr(frameId);
		if(!pBuffer)
		{
			//printf_s( "Failed to get frame from cache!\n" );
			return NULL;
		}
		return pBuffer->getCompoundFrame(frameId);
	}
	
	virtual bool	setFaceShape(PCEFaceTrackingResult* pFaceShape) override
	{
		if(m_faceTrackingFrame < 0 || !pFaceShape)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(m_faceTrackingFrame);
		if(!pFrame)
		{
			printf_s( "Failed to set face shape into cache!\n" );
			return false;
		}
		return pFrame->setFaceShape(pFaceShape);
	}

	virtual int	getFaceTrackingFrame() const override
	{
		return m_faceTrackingFrame;
	}

	virtual int getFaceTrackingFrameToProcess() override
	{
		return (++ m_faceTrackingFrame);
	}

	virtual bool	readFaceShape(int frameId, PCEFaceTrackingResult* pFaceShape) override
	{
		if(frameId < 0 || frameId > m_faceTrackingFrame || !pFaceShape)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(frameId);
		if(!pFrame)
			return false;
		return pFrame->readFaceShape(frameId, pFaceShape);
	}

	virtual bool	setFusionImage(int frameId, unsigned int width, unsigned int height, BYTE* pBuffer) override
	{
		if(frameId < 0 || !pBuffer)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(frameId);
		if(!pFrame)
		{
			printf_s( "Failed to set camera pose into cache!\n" );
			return false;
		}
		return pFrame->setFusionImage(frameId, width, height, pBuffer);
	}
	virtual bool	readFusionImage(int frameId, IplImage* pImage) override
	{
		if(frameId < 0 || !pImage)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(frameId);
		if(!pFrame)
			return false;
		return pFrame->readFusionImage(frameId, pImage);
	}

	virtual int	getFusionFrame() const override
	{
		return m_fusionFrame;
	}

	virtual int getFusionFrameToProcess() override
	{
		return (++ m_fusionFrame);
	}

	virtual bool	setCameraPose(int frameId, const Matrix4& mat) override
	{
		if(frameId < 0)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(frameId);
		if(!pFrame)
		{
			printf_s( "Failed to set camera pose into cache!\n" );
			return false;
		}
		return pFrame->setCameraPose(frameId, mat);
	}
	virtual bool	readCameraPose(int frameId, Matrix4* mat) override
	{
		if(frameId < 0 || !mat)
			return false;
		PCEDeviceBuffer* pFrame = m_framePtrArray.getDataPtr(frameId);
		if(!pFrame)
			return false;
		return pFrame->readCameraPose(frameId, mat);
	}

private:
	/*static*/
	bool isClosestFrame(PCEDeviceFrame* pBaseFrame, PCEDeviceFrame* pToBeComparedFrame, long long toBeComparedTimeStamp)
	{
		if(!pBaseFrame)
			return false;

		long long baseTimeStamp = pBaseFrame->getTimeStamp();
		long long timeDiff = std::abs(baseTimeStamp - toBeComparedTimeStamp);
		if(!pToBeComparedFrame)
		{
			return (timeDiff < PCEDeviceFrame::sHalfADepthFrameMs);
		}

		long long lastToBeComparedTimeStamp = pToBeComparedFrame->getTimeStamp();
		long long lastTimeDiff = std::abs(baseTimeStamp - lastToBeComparedTimeStamp);
		if(timeDiff > lastTimeDiff)
			return false;
		return true;
	}

private:
	PCEDataPtrCache<PCEDeviceBuffer>	m_framePtrArray;
	int									m_faceTrackingFrame;
	int									m_fusionFrame;
};

#endif