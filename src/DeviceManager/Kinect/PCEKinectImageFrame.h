#ifndef _PCEKinectImageFrame
#define _PCEKinectImageFrame
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectImageFrame.h
//
//
// Author: Tingzhu Zhou
//
#include "../PCEDeviceFrame.h"

#include "opencv2/core.hpp"

#include <FaceTrackLib.h>
#include <NuiApi.h>

class PCEKinectDepthImageFrame : public PCEDeviceFrame
{
public:
	PCEKinectDepthImageFrame (long long timeStamp, UINT width, UINT height)
		: PCEDeviceFrame(timeStamp)
		, m_width(width)
		, m_height(height)
		, m_pColorCoordinates(NULL)
	{
		m_pDepthImagePixelBuffer = new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[m_width*m_height];
		m_pCompressedDepthBuffer = new(std::nothrow) USHORT[m_width*m_height];
	}
	virtual ~PCEKinectDepthImageFrame ()
	{
		release();
	}

	void		release()
	{
		if(m_pDepthImagePixelBuffer)
		{
			delete[] m_pDepthImagePixelBuffer;
			m_pDepthImagePixelBuffer = NULL;
		}

		if(m_pCompressedDepthBuffer)
		{
			delete[] m_pCompressedDepthBuffer;
			m_pCompressedDepthBuffer = NULL;
		}
		
		deleteCoordinates();
	}

	PCEKinectDepthImageFrame (const PCEKinectDepthImageFrame& other)
	{
		deepCopy(other);
	}
	PCEKinectDepthImageFrame& operator = (const PCEKinectDepthImageFrame& other)
	{
		deepCopy(other);
		return *this;
	}
	void deepCopy (const PCEKinectDepthImageFrame& other)
	{
		m_liTimeStamp = other.m_liTimeStamp;
		m_ZoomFactor = other.m_ZoomFactor;
		m_ViewOffset = other.m_ViewOffset;

		m_fps = other.m_fps;

		if(other.m_width != m_width || other.m_height != m_height)
		{
			release();

			m_width = other.m_width;
			m_height = other.m_height;

			m_pDepthImagePixelBuffer = new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[m_width*m_height];
			m_pCompressedDepthBuffer = new(std::nothrow) USHORT[m_width*m_height];
		}
		memcpy(m_pDepthImagePixelBuffer, other.m_pDepthImagePixelBuffer, getExtendedBufferSize());
		memcpy(m_pCompressedDepthBuffer, other.m_pCompressedDepthBuffer, getCompressedBufferSize());

		if(other.m_pColorCoordinates)
		{
			allocateCoordinates();
			memcpy(m_pColorCoordinates, other.m_pColorCoordinates, coordinatesBufferSize());
		}
	}

	UINT		getWidth() const { return m_width; }
	UINT		getHeight() const { return m_height; }

	NUI_DEPTH_IMAGE_PIXEL*	getExtendedBuffer() const { return m_pDepthImagePixelBuffer; }
	UINT	getExtendedBufferSize() const { return (getWidth() * getHeight() * sizeof(NUI_DEPTH_IMAGE_PIXEL)); }

	USHORT*	getCompressedBuffer() const { return m_pCompressedDepthBuffer; }
	UINT	getCompressedBufferSize() const { return (getWidth() * getHeight() * sizeof(USHORT)); }

	void		allocateCoordinates()
	{
		if(!m_pColorCoordinates)
		{
			UINT bufferSize = m_width * m_height;
			m_pColorCoordinates = new(std::nothrow) NUI_COLOR_IMAGE_POINT[bufferSize];
		}
	}
	void		deleteCoordinates()
	{
		if(m_pColorCoordinates)
		{
			delete[] m_pColorCoordinates;
			m_pColorCoordinates = NULL;
		}
	}
	NUI_COLOR_IMAGE_POINT*	colorCoordinates() const { return m_pColorCoordinates; }
	UINT		coordinatesBufferSize() const { return (getWidth() * getHeight() * sizeof(NUI_COLOR_IMAGE_POINT)); }

	void setFPS(UINT fps) { m_fps = fps; }
	UINT getFPS() const { return m_fps; }
private:
	UINT					m_width;
	UINT					m_height;
	NUI_DEPTH_IMAGE_PIXEL*  m_pDepthImagePixelBuffer;
	USHORT*					m_pCompressedDepthBuffer;
	NUI_COLOR_IMAGE_POINT*	m_pColorCoordinates;
	UINT					m_fps;
};

class PCEKinectColorImageFrame : public PCEDeviceFrame
{
public:
	PCEKinectColorImageFrame (long long timeStamp, UINT width, UINT height)
		: PCEDeviceFrame(timeStamp)
	{
		m_pColorImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_32S,1);
	}
	virtual ~PCEKinectColorImageFrame ()
	{
		release();
	}

	PCEKinectColorImageFrame (const PCEKinectColorImageFrame& other)
	{
		deepCopy(other);
	}
	PCEKinectColorImageFrame& operator = (const PCEKinectColorImageFrame& other)
	{
		deepCopy(other);
		return *this;
	}
	void			deepCopy (const PCEKinectColorImageFrame& other)
	{
		m_liTimeStamp = other.m_liTimeStamp;
		m_ZoomFactor = other.m_ZoomFactor;
		m_ViewOffset = other.m_ViewOffset;
		cvCopy(other.m_pColorImage, m_pColorImage);
		//other.m_pColorImage->copyTo(m_pColorImage);
	}
	
	void			release()
	{
		if(m_pColorImage)
			cvReleaseImage(&m_pColorImage);
		m_pColorImage = NULL;
	}

	UINT		getWidth() const { return m_pColorImage ? m_pColorImage->width : 0; }
	UINT		getHeight() const { return m_pColorImage ? m_pColorImage->height : 0; }
	BYTE*		getBuffer() const { return m_pColorImage ? reinterpret_cast<BYTE*>(m_pColorImage->imageData) : NULL; }
	UINT		getBufferSize() const { return m_pColorImage ? m_pColorImage->widthStep*m_pColorImage->height : 0; }
	UINT		getBytesPerPixel() const { return m_pColorImage ? 4 : 0; }

	void			setFPS(UINT fps) { m_fps = fps; }
	UINT			getFPS() const { return m_fps; }
private:
	IplImage*		m_pColorImage;
	UINT			m_fps;
};

#endif
