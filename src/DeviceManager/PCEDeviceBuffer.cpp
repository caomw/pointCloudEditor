// File: PCEKinectCompoundFrame.cpp
//
//
// Author: Tingzhu Zhou
//
#include "PCEDeviceBuffer.h"

PCEDeviceBuffer::PCEDeviceBuffer()
	: m_pFaceShape(NULL)
	, m_pFusionImage(NULL)
{
	m_cameraPose.M11 = 1; m_cameraPose.M12 = 0; m_cameraPose.M13 = 0; m_cameraPose.M14 = 0;
	m_cameraPose.M21 = 0; m_cameraPose.M22 = 1; m_cameraPose.M23 = 0; m_cameraPose.M24 = 0;
	m_cameraPose.M31 = 0; m_cameraPose.M32 = 0; m_cameraPose.M33 = 1; m_cameraPose.M34 = 0;
	m_cameraPose.M41 = 0; m_cameraPose.M42 = 0; m_cameraPose.M43 = 0; m_cameraPose.M44 = 1;
}

PCEDeviceBuffer::~PCEDeviceBuffer()
{
	clear();
}

void PCEDeviceBuffer::clear()
{
	m_frame.clear();
	if(m_pFaceShape)
	{
		delete m_pFaceShape;
		m_pFaceShape = NULL;
	}
	if(m_pFusionImage)
	{
		cvReleaseImage(&m_pFusionImage);
		m_pFusionImage = NULL;
	}
}

PCECompoundFrame*	PCEDeviceBuffer::acquireCompoundFrame()
{
	return &m_frame;
}

PCECompoundFrame*	PCEDeviceBuffer::getLatestCompoundFrame()
{
	return &m_frame;
}

PCECompoundFrame*	PCEDeviceBuffer::getCompoundFrame(int frameId)
{
	return &m_frame;
}

bool PCEDeviceBuffer::setFaceShape(PCEFaceTrackingResult*	 pShape)
{
	if(!pShape)
		return false;

	boost::mutex::scoped_lock buff_lock (m_bFaceMutex);
	if(pShape == m_pFaceShape)
		return true;
	if(m_pFaceShape)
	{
		printf_s("The face tracking result in the buffer has already been recovered. ");
		delete m_pFaceShape;
	}
	m_pFaceShape = pShape;
	return true;
}

bool PCEDeviceBuffer::readFaceShape(int frameId, PCEFaceTrackingResult* pShape)
{
	if(!pShape)
		return false;
	boost::mutex::scoped_lock buff_lock (m_bFaceMutex);
	if(!m_pFaceShape)
		return false;
	*pShape = *m_pFaceShape;
	return true;
}

bool PCEDeviceBuffer::setFusionImage(int frameId, unsigned int width, unsigned int height, BYTE* pBuffer)
{
	if(!pBuffer)
		return false;

	boost::mutex::scoped_lock buff_lock (m_bFusionImageMutex);

	if(m_pFusionImage)
	{
		if(width != m_pFusionImage->width || height != m_pFusionImage->height)
		{
			cvReleaseImage(&m_pFusionImage);
			m_pFusionImage = NULL;
		}
	}
	if(!m_pFusionImage)
	{
		m_pFusionImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_32S,1);
	}
	if(m_pFusionImage->imageData)
		memcpy(m_pFusionImage->imageData, pBuffer, m_pFusionImage->nSize * 4);
	
	return true;
}

bool PCEDeviceBuffer::readFusionImage(int frameId, IplImage* pImage)
{
	if(!m_pFusionImage)
		return false;

	if(pImage)
	{
		if(m_pFusionImage->width != pImage->width || m_pFusionImage->height != pImage->height || m_pFusionImage->depth != pImage->depth)
		{
			cvReleaseImage(&pImage);
			pImage = NULL;
		}
	}
	if(!pImage)
	{
		pImage = cvCreateImage(cvSize(m_pFusionImage->width,m_pFusionImage->height),IPL_DEPTH_32S,1);
	}

	boost::mutex::scoped_lock buff_lock (m_bFusionImageMutex);
	cvCopy(m_pFusionImage, pImage);
	return true;
}

bool PCEDeviceBuffer::setCameraPose(int frameId, const Matrix4& mat)
{
	boost::mutex::scoped_lock buff_lock (m_bCameraMutex);
	m_cameraPose = mat;
	return true;
}

bool PCEDeviceBuffer::readCameraPose(int frameId, Matrix4* mat)
{
	if(!mat)
		return false;
	boost::mutex::scoped_lock buff_lock (m_bCameraMutex);
	*mat = m_cameraPose;
	return true;
}
