#ifndef _PCEKinectImageBuffer
#define _PCEKinectImageBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEImageBuffer.h
//
//
// Author: Tingzhu Zhou
//
class PCEKinectImageBuffer
{
public:
	PCEKinectImageBuffer (UINT bytePerPixel, UINT width, UINT height)
	: m_bytesPerPixel(bytePerPixel)
	, m_width(width)
	, m_height(height)
	{
		m_pBuffer = reinterpret_cast<BYTE*>(_malloca(bytePerPixel * m_width * m_height));
	}
	~PCEKinectImageBuffer () {
		release();
	}

	void release()
	{
		if(m_pBuffer)
		{
			_freea(m_pBuffer);
			m_pBuffer = NULL;
		}
		m_width = m_height = 0;
	}

	BYTE* getBuffer() const { return m_pBuffer; }
	UINT getBytesPerPixel() const { return m_bytesPerPixel; }
	UINT getWidth() const { return m_width; }
	UINT getHeight() const { return m_height; }

private:
	BYTE*	m_pBuffer;
	UINT	m_width;
	UINT	m_height;
	UINT	m_bytesPerPixel;

};

#endif
