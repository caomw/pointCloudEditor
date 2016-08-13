#ifndef _PCEKinectCompoundFrame
#define _PCEKinectCompoundFrame
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEKinectCompoundFrame.h
//
// Author: Tingzhu Zhou
//
#include "PCEMacros.h"
#include "PCESkeletonFrame.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class PCECompoundFrame : public PCEDeviceFrame
{
public:
	static const float cDepthToFloatCoefficient;
	static const int cColorBytesPerPixel = 4;
public:
	PCECompoundFrame()
		: PCEDeviceFrame()
		, m_depthWidth(0)
		, m_depthHeight(0)
		, m_depthFPS(0)
		, m_pDepthBuffer(NULL)
		, m_colorWidth(0)
		, m_colorHeight(0)
		, m_colorFPS(0)
		, m_pAlignedColorBuffer(NULL)
	{}
	virtual ~PCECompoundFrame();

	void					resetIndices();
	void					resetSkeleton();
	void					clear();
	void					deepCopy (const PCECompoundFrame& other);
	PCECompoundFrame (const PCECompoundFrame& other){ deepCopy(other); }
	PCECompoundFrame& operator = (const PCECompoundFrame& other) {	deepCopy(other); return *this; }
	
	// Image
	unsigned short*			allocateDepthBuffer(unsigned int width, unsigned int height);
	unsigned short*			getDepthBuffer() const { return m_pDepthBuffer; }
	unsigned int			getDepthWidth() const {	return m_depthWidth; }
	unsigned int			getDepthHeight() const { return m_depthHeight; }
	unsigned int			getDepthBufferSize() const { return (sizeof(unsigned short) * m_depthWidth * m_depthHeight); }
	void					setDepthFPS(unsigned int fps) { m_depthFPS = fps; }
	unsigned int			getDepthFPS() const { return m_depthFPS; }

	unsigned char*			allocateAlignedColorBuffer(unsigned int width, unsigned int height);
	unsigned char*			getAlignedColorBuffer() const {	return m_pAlignedColorBuffer; }
	unsigned int			getColorWidth() const {	return m_colorWidth; }
	unsigned int			getColorHeight() const { return m_colorHeight;	}
	unsigned int			getColorBytesPerPixel() const { return (cColorBytesPerPixel * sizeof(unsigned char)); }
	unsigned int			getColorBufferSize() const { return (cColorBytesPerPixel * sizeof(unsigned char) * m_colorWidth * m_colorHeight); }
	void					setColorFPS(unsigned int fps) { m_colorFPS = fps; }
	unsigned int			getColorFPS() const { return m_colorFPS; }

	void					pushValidIndex(unsigned int id){ m_validIndices.push_back(id);	}
	const unsigned int*		getValidIndexBuffer() const;
	unsigned int			getValidIndex(unsigned int id) const { return m_validIndices.at(id); }
	size_t					getValidIndexSize() const { return m_validIndices.size(); }
	void					pushFirstPlayerIndex(unsigned int id){ m_firstPlayerIndices.push_back(id);	}
	const unsigned int*		getFirstPlayerIndexBuffer() const;
	size_t					getFirstPlayerIndexSize() const { return m_firstPlayerIndices.size(); }
	void					pushSecondPlayerIndex(unsigned int id){ m_secondPlayerIndices.push_back(id);	}
	const unsigned int*		getSecondPlayerIndexBuffer() const;
	size_t					getSecondPlayerIndexSize() const { return m_secondPlayerIndices.size(); }

	void					readImageLock() { m_bImageMutex.lock_shared(); }
	void					readImageUnlock() { m_bImageMutex.unlock_shared(); }
	void					writeImageLock() { m_bImageMutex.lock(); }
	void					writeImageUnlock() { m_bImageMutex.unlock(); }

	// Skeleton
	bool					hasSkeleton();
	void					cacheSkeletonFrame(const PCESkeletonFrame& frame);
	bool					readSkeletonPosition(int skeletonId, PCESkeletonPosition* pSkeleton);
	bool					getClosestFaceHint(Vector3* pHint3D);

	void					setAccelerometerReading(const Vector4& reading) { m_accelerometer = reading; }
	const Vector4&			getAccelerometerReading() const { return m_accelerometer; }
private:
	// Image
	unsigned short*				m_pDepthBuffer;
	unsigned int				m_depthWidth;
	unsigned int				m_depthHeight;
	unsigned int				m_depthFPS;

	unsigned char*				m_pAlignedColorBuffer;
	unsigned int				m_colorWidth;
	unsigned int				m_colorHeight;
	unsigned int				m_colorFPS;

	std::vector<unsigned int>	m_validIndices;
	std::vector<unsigned int>	m_firstPlayerIndices;
	std::vector<unsigned int>	m_secondPlayerIndices;

	//typedef boost::shared_lock<boost::shared_mutex> readLock;
	//typedef boost::unique_lock<boost::shared_mutex> writeLock;
	boost::shared_mutex			m_bImageMutex;

	// Skeleton
	boost::mutex				m_bSkeletonMutex;
	PCESkeletonFrame			m_skeletonFrame;

	Vector4						m_accelerometer;
};

#endif