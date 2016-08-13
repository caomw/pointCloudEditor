#ifndef _PCEPreviewData
#define _PCEPreviewData
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPreviewData.h
//
//
// Author: Tingzhu Zhou
//

class PCEPreviewData
{
public:
	PCEPreviewData ()
	: m_pDepthBuffer(NULL)
	, m_depthWidth(0)
	, m_depthHeight(0)
	, m_depthFPS(0)
	, m_pColorBuffer(NULL)
	, m_colorWidth(0)
	, m_colorHeight(0)
	, m_colorFPS(0)
	{};
	~PCEPreviewData () {};

	void updateDepthBuffer(unsigned short* pBuffer, unsigned int width, unsigned int height, unsigned int fps)
	{
		m_pDepthBuffer = pBuffer;
		m_depthWidth = width;
		m_depthHeight = height;
		m_depthFPS = fps;
	}
	unsigned short* getDepthBufferData() const { return m_pDepthBuffer; }
	unsigned int getDepthWidth() const { return m_depthWidth; }
	unsigned int getDepthHeight() const { return m_depthHeight; }
	unsigned int getDepthFPS() const { return m_depthFPS; }

	void updateColorBuffer(unsigned char* pBuffer, unsigned int width, unsigned int height, unsigned int fps)
	{
		m_pColorBuffer = pBuffer;
		m_colorWidth = width;
		m_colorHeight = height;
		m_colorFPS = fps;
	}
	unsigned char* getColorBufferData() const { return m_pColorBuffer; }
	unsigned int getColorWidth() const { return m_colorWidth; }
	unsigned int getColorHeight() const { return m_colorHeight; }
	unsigned int getColorFPS() const { return m_colorFPS; }
	
private:
	unsigned short*	m_pDepthBuffer;
	unsigned int	m_depthWidth;
	unsigned int	m_depthHeight;
	unsigned int	m_depthFPS;

	unsigned char*	m_pColorBuffer;
	unsigned int	m_colorWidth;
	unsigned int	m_colorHeight;
	unsigned int	m_colorFPS;
};

#endif
