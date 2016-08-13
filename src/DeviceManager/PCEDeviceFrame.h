#ifndef _PCEKinectFrame
#define _PCEKinectFrame
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDeviceFrame.h
//
//
// Author: Tingzhu Zhou
//
struct Point2d
{
	long  x;
	long  y;
};

class PCEDeviceFrame
{
public:

	static const int						sDepthFps = 30;
	static const int						sHalfADepthFrameMs = (1000 / sDepthFps) /*/ 2*/;

public:
	PCEDeviceFrame (long long timeStamp)
		: m_liTimeStamp(timeStamp)
		, m_ZoomFactor(1.0f)
	{
		m_ViewOffset.x = 0;
		m_ViewOffset.y = 0;
	}
	PCEDeviceFrame ()
		: m_liTimeStamp(0)
		, m_ZoomFactor(1.0f)
	{
		m_ViewOffset.x = 0;
		m_ViewOffset.y = 0;
	}
	virtual ~PCEDeviceFrame ()
	{}

	long long				getTimeStamp() const { return m_liTimeStamp; }
	void					setTimeStamp(long long timeStamp) { m_liTimeStamp = timeStamp; }
	float					getZoomFactor() const { return(m_ZoomFactor); }
	Point2d*				getViewOffSet() { return(&m_ViewOffset); }

protected:
	long long				m_liTimeStamp;
	float					m_ZoomFactor;   // video frame zoom factor (it is 1.0f if there is no zoom)
	Point2d					m_ViewOffset;   // Offset of the view from the top left corner.
};

#endif
