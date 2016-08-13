#ifndef _PCEKinectCircleBuffer
#define _PCEKinectCircleBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectBuffer.h
//
//
// Author: Tingzhu Zhou
//
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

template<class FrameDataT>
class PCEKinectCircleBuffer
{
private:
	static const size_t GRABBER_BUFFER_SIZE = 200;
public:
	PCEKinectCircleBuffer () {}
	~PCEKinectCircleBuffer () { clear(); }

	
	bool pushBack (FrameDataT* frame)
	{
		bool retVal = false;
		{
			boost::mutex::scoped_lock buff_lock (bMutex);
			if (!m_buffer.full ())
				retVal = true;
			m_buffer.push_back (frame);
		}
		return (retVal);
	}

	FrameDataT*	getFront ()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		FrameDataT* frame = NULL;
		if(!m_buffer.empty ())
		{
			frame = m_buffer.front ();
			m_buffer.pop_front ();
		}
		return frame;
	}

	FrameDataT*	getBackBeforeClear ()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		FrameDataT* frame = NULL;
		if(!m_buffer.empty ())
		{
			frame = m_buffer.back ();
			m_buffer.pop_back ();
			// Clear all
			while (!m_buffer.empty ())
			{
				FrameDataT* frame = m_buffer.back ();
				if(frame)
					delete frame;
				m_buffer.pop_back ();
			}
			m_buffer.clear ();
		}
		return frame;
	}

	bool isFull ()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		return (m_buffer.full ());
	}

	bool isEmpty ()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		return (m_buffer.empty ());
	}

	int	getSize ()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		return (int (m_buffer.size ()));
	}

	int	getCapacity ()
	{
		return (int (m_buffer.capacity ()));
	}

	void setCapacity (int buff_size)
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		m_buffer.set_capacity (buff_size);
	}

	void clear	()
	{
		boost::mutex::scoped_lock buff_lock (bMutex);
		while (!m_buffer.empty ())
		{
			FrameDataT* frame = m_buffer.front ();
			if(frame)
				delete frame;
			m_buffer.pop_front ();
		}
		m_buffer.clear ();
	}

private:
	PCEKinectCircleBuffer (const PCEKinectCircleBuffer&); // Disabled copy constructor
	PCEKinectCircleBuffer& operator = (const PCEKinectCircleBuffer&); // Disabled assignment operator

	boost::mutex							bMutex;
	boost::circular_buffer<FrameDataT*>		m_buffer;
};

#endif
