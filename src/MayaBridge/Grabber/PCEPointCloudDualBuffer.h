#ifndef _PCEPointCloudDualBuffer
#define _PCEPointCloudDualBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudDualBuffer.h
//
// Author: Tingzhu Zhou
//

#include "PCEKinectFrame.h"

class PCEPointCloudDualBuffer
{
	enum ELockedIndex
	{
		NoLocked = 0;
		FirstLocked = NoLocked +1;
		SecondLocked = FirstLocked + 1;
		TheLastFlag = SecondLocked + 1;
	}
public:
	PCEPointCloudDualBuffer ()
		: m_isFirstBufferLatest(false)
		, m_LockedIndex(NoLocked){}
	~PCEPointCloudDualBuffer () {}
	
	void initializeBuffer(depthResolution, colorResolution)
	{
		allocate(depthResolution)
	}
	
	void locked()
	{
		if(m_isFirstBufferLatest)
			m_LockedIndex = FirstLocked;
		else
			m_LockedIndex = SecondLocked;
	}
	
	void unLocked()
	{
		m_LockedIndex = NoLocked;
	}
	
	void writeDepthBufferHelper(int bufferIndex, BYTE* pBuffer, LONG* coordernates, timeStamp)
	{
		memcpy( m_buffers[bufferIndex].depthFrame.getBuffer );
		memcpy( coordernates );
		m_buffers[bufferIndex].depthFrame.timeStamp = timeStamp;
	}
	
	void writeDepthBuffer(BYTE* pBuffer, LONG* coordernates, timeStamp)
	{
		if(FirstLocked == m_LockedIndex)
		{
			// Write the second buffer
			writeDepthBufferHelper(1, pBuffer, coordernates, timeStamp);
			m_isFirstBufferLatest = false;
		}
		else if(SecondLocked == m_LockedIndex)
		{
			// Write the first buffer
			writeDepthBufferHelper(0, pBuffer, coordernates, timeStamp);
			m_isFirstBufferLatest = true;
		}
		else if(m_isFirstBufferLatest)
		{
			// Write the second buffer
			writeDepthBufferHelper(1, pBuffer, coordernates, timeStamp);
			m_isFirstBufferLatest = false;
		}
		else
		{
			// Write the first buffer
			writeDepthBufferHelper(0, pBuffer, coordernates, timeStamp);
			m_isFirstBufferLatest = true;
		}
	}
	
	BYTE* readDepthBuffer() const
	{
		if(FirstLocked == m_LockedIndex)
		{
			// get the first buffer
			if(0 != m_buffers[0].depthFrame.timeStamp)
				return( m_buffers[0].depthFrame.getBuffer );
			else
				return NULL;
		}
		else if(SecondLocked == m_LockedIndex)
		{
			// get the second buffer
			if(0 != m_buffers[1].depthFrame.timeStamp)
				return( m_buffers[1].depthFrame.getBuffer );
			else
				return NULL;
		}
		return NULL;
	}
	
	void writeColorBufferHelper(int bufferIndex, BYTE* pBuffer, timeStamp)
	{
		memcpy( m_buffers[bufferIndex].colorFrame.getBuffer );
		m_buffers[bufferIndex].colorFrame.timeStamp = timeStamp;
	}
	
	void writeColorBuffer(BYTE* pBuffer, timeStamp)
	{
		if(FirstLocked == m_LockedIndex)
		{
			// Write the second buffer
			writeColorBufferHelper(1, pBuffer, timeStamp);
			m_isFirstBufferLatest = false;
		}
		else if(SecondLocked == m_LockedIndex)
		{
			// Write the first buffer
			writeColorBufferHelper(0, pBuffer, timeStamp);
			m_isFirstBufferLatest = true;
		}
		else if(m_isFirstBufferLatest)
		{
			// Write the second buffer
			writeColorBufferHelper(1, pBuffer, timeStamp);
			m_isFirstBufferLatest = false;
		}
		else
		{
			// Write the first buffer
			writeColorBufferHelper(0, pBuffer, timeStamp);
			m_isFirstBufferLatest = true;
		}
	}
	
	BYTE* readColorBuffer() const
	{
		if(FirstLocked == m_LockedIndex)
		{
			// get the first buffer
			if(0 != m_buffers[0].colorFrame.timeStamp)
				return( m_buffers[0].colorFrame.getBuffer );
			else
				return NULL;
		}
		else if(SecondLocked == m_LockedIndex)
		{
			// get the second buffer
			if(0 != m_buffers[1].colorFrame.timeStamp)
				return( m_buffers[1].colorFrame.getBuffer );
			else
				return NULL;
		}
		return NULL;
	}
	
private:
	PCEKinectFrame 	m_buffers[2];
	
	bool			m_isFirstBufferLatest;
	int				m_LockedIndex;
};

#endif