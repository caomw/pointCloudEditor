// File: PCEPointCloudBuffer.cpp
//
// Dependency Graph Node: PCEPointCloudBuffer
//
// Author: Tingzhu Zhou
//

#include "PCEPointCloudBuffer.h"

#include <pcl/common/time.h>

using namespace pcl;
//////////////////////////////////////////////////////////////////////////////////////////
bool PCEPointCloudGrabberBuffer::isFull ()
{
	boost::mutex::scoped_lock buff_lock (bmutex_);
	return (buffer_.full ());
}

bool PCEPointCloudGrabberBuffer::isEmpty ()
{
	boost::mutex::scoped_lock buff_lock (bmutex_);
	return (buffer_.empty ());
}

int	PCEPointCloudGrabberBuffer::getSize ()
{
	boost::mutex::scoped_lock buff_lock (bmutex_);
	return (int (buffer_.size ()));
}

int	PCEPointCloudGrabberBuffer::getCapacity ()
{
	return (int (buffer_.capacity ()));
}

void PCEPointCloudGrabberBuffer::setCapacity (int buff_size)
{
	boost::mutex::scoped_lock buff_lock (bmutex_);
	buffer_.set_capacity (buff_size);
}

void PCEPointCloudGrabberBuffer::clear ()
{
	boost::mutex::scoped_lock buff_lock (bmutex_);
	buffer_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////
bool PCEPointCloudGrabberBuffer::pushBack (PointCloud<BufferPointT>::ConstPtr cloud)
{
	bool retVal = false;
	{
		boost::mutex::scoped_lock buff_lock (bmutex_);
		if (!buffer_.full ())
			retVal = true;
		buffer_.push_back (cloud);
	}
	buff_empty_.notify_one ();
	return (retVal);
}

//////////////////////////////////////////////////////////////////////////////////////////
PointCloud<BufferPointT>::ConstPtr 
	PCEPointCloudGrabberBuffer::getFront ()
{
	PointCloud<BufferPointT>::ConstPtr cloud;
	{
		boost::mutex::scoped_lock buff_lock (bmutex_);
		if(!buffer_.empty ())
		{
			cloud = buffer_.front ();
			buffer_.pop_front ();
		}
	}
	return (cloud);
}

PointCloud<BufferPointT>::ConstPtr 
	PCEPointCloudGrabberBuffer::getBackBeforeClear ()
{
	PointCloud<BufferPointT>::ConstPtr cloud;
	{
		boost::mutex::scoped_lock buff_lock (bmutex_);
		if(!buffer_.empty ())
		{
			cloud = buffer_.back ();
			buffer_.clear ();
		}
	}
	return (cloud);
}