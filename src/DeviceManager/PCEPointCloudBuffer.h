#ifndef _PCEPointCloudBuffer
#define _PCEPointCloudBuffer
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudBuffer.h
//
// Dependency Graph Node:  PCEPointCloudBuffer
//
// Author: Tingzhu Zhou
//

#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#if defined(__linux__) || defined (TARGET_OS_MAC)
#include <unistd.h>
// Get the available memory size on Linux/BSD systems

size_t 
	getTotalSystemMemory ()
{
	uint64_t memory = std::numeric_limits<size_t>::max ();

#ifdef _SC_AVPHYS_PAGES
	uint64_t pages = sysconf (_SC_AVPHYS_PAGES);
	uint64_t page_size = sysconf (_SC_PAGE_SIZE);

	memory = pages * page_size;

#elif defined(HAVE_SYSCTL) && defined(HW_PHYSMEM)
	// This works on *bsd and darwin.
	unsigned int physmem;
	size_t len = sizeof physmem;
	static int mib[2] = { CTL_HW, HW_PHYSMEM };

	if (sysctl (mib, ARRAY_SIZE (mib), &physmem, &len, NULL, 0) == 0 && len == sizeof (physmem))
	{
		memory = physmem;
	}
#endif

	if (memory > uint64_t (std::numeric_limits<size_t>::max ()))
	{
		memory = std::numeric_limits<size_t>::max ();
	}

	print_info ("Total available memory size: %lluMB.\n", memory / 1048576ull);
	return size_t (memory);
}

const size_t GRABBER_BUFFER_SIZE = size_t (getTotalSystemMemory () / (640 * 480 * sizeof (pcl::PointXYZRGBA)));
#else

const size_t GRABBER_BUFFER_SIZE = 200;
#endif

typedef pcl::PointXYZRGBA BufferPointT;

class PCEPointCloudGrabberBuffer
{
public:
	PCEPointCloudGrabberBuffer () {}

	bool	pushBack (pcl::PointCloud<BufferPointT>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer

	pcl::PointCloud<BufferPointT>::ConstPtr 
			getFront (); // thread-save wrapper for front() method of ciruclar_buffer
	pcl::PointCloud<BufferPointT>::ConstPtr 
			getBackBeforeClear (); // thread-save wrapper for front() method of ciruclar_buffer

	bool	isFull ();
	bool	isEmpty ();
	int		getSize ();
	int		getCapacity ();
	void 	setCapacity (int buff_size);
	void 	clear();

private:
	PCEPointCloudGrabberBuffer (const PCEPointCloudGrabberBuffer&); // Disabled copy constructor
	PCEPointCloudGrabberBuffer& operator = (const PCEPointCloudGrabberBuffer&); // Disabled assignment operator

	boost::mutex bmutex_;
	boost::condition_variable buff_empty_;
	boost::circular_buffer<pcl::PointCloud<BufferPointT>::ConstPtr> buffer_;
};

#endif
