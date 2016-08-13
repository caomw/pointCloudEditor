// File: PCEKinectCompoundFrame.cpp
//
//
// Author: Tingzhu Zhou
//
#include "PCECompoundFrame.h"

const float PCECompoundFrame::cDepthToFloatCoefficient = 1000.0f;

PCECompoundFrame::~PCECompoundFrame()
{
	clear();
}

void PCECompoundFrame::clear()
{
	if(m_pDepthBuffer)
	{
		_freea(m_pDepthBuffer);
		m_pDepthBuffer = NULL;
	}
	if(m_pAlignedColorBuffer)
	{
		_freea(m_pAlignedColorBuffer);
		m_pAlignedColorBuffer = NULL;
	}
	resetSkeleton();
	resetIndices();
}

void PCECompoundFrame::resetIndices()
{
	m_validIndices.clear();
	m_firstPlayerIndices.clear();
	m_secondPlayerIndices.clear();
	m_validIndices.reserve(m_depthWidth*m_depthHeight*0.8);
}

void PCECompoundFrame::deepCopy (const PCECompoundFrame& other)
{
	m_liTimeStamp = other.m_liTimeStamp;
	m_ZoomFactor = other.m_ZoomFactor;
	m_ViewOffset = other.m_ViewOffset;

	m_depthWidth = other.m_depthWidth;
	m_depthHeight = other.m_depthHeight;
	m_depthFPS = other.m_depthFPS;
	if(other.m_pDepthBuffer)
	{
		unsigned short* pDepthBuffer = allocateDepthBuffer(m_depthWidth, m_depthHeight);
		memcpy(pDepthBuffer, other.m_pDepthBuffer, getDepthBufferSize());
	}
	else
	{
		_freea(m_pDepthBuffer);
		m_pDepthBuffer = NULL;
	}

	m_colorWidth = other.m_colorWidth;
	m_colorHeight = other.m_colorHeight;
	m_colorFPS = other.m_colorFPS;
	if(other.m_pAlignedColorBuffer)
	{
		unsigned char* pAlignedColorBuffer = allocateAlignedColorBuffer(m_colorWidth, m_colorHeight);
		memcpy(pAlignedColorBuffer, other.m_pAlignedColorBuffer, getColorBufferSize());
	}
	else
	{
		_freea(m_pAlignedColorBuffer);
		m_pAlignedColorBuffer = NULL;
	}

	m_validIndices = other.m_validIndices;
	m_firstPlayerIndices = other.m_firstPlayerIndices;
	m_secondPlayerIndices = other.m_secondPlayerIndices;
	m_skeletonFrame = other.m_skeletonFrame;
}

unsigned short* PCECompoundFrame::allocateDepthBuffer(unsigned int width, unsigned int height)
{
	if(m_pDepthBuffer)
	{
		if(m_depthWidth != width || m_depthHeight != height)
		{
			_freea(m_pDepthBuffer);
			m_pDepthBuffer = NULL;
		}
	}
	if(!m_pDepthBuffer)
	{
		m_depthWidth = width;
		m_depthHeight = height;
		m_pDepthBuffer = reinterpret_cast<unsigned short*>(_malloca(getDepthBufferSize()));
	}
	return m_pDepthBuffer;
}

BYTE* PCECompoundFrame::allocateAlignedColorBuffer(unsigned int width, unsigned int height)
{
	if(m_pAlignedColorBuffer)
	{
		if(m_colorWidth != width || m_colorHeight != height)
		{
			_freea(m_pAlignedColorBuffer);
			m_pAlignedColorBuffer = NULL;
		}
	}
	if(!m_pAlignedColorBuffer)
	{
		m_colorWidth = width;
		m_colorHeight = height;
		m_pAlignedColorBuffer = reinterpret_cast<BYTE*>(_malloca(getColorBufferSize()));
	}
	return m_pAlignedColorBuffer;
}

const unsigned int* PCECompoundFrame::getValidIndexBuffer() const
{
	if(m_validIndices.empty())
		return NULL;
	else
		return &(m_validIndices.at(0));
}

const unsigned int* PCECompoundFrame::getFirstPlayerIndexBuffer() const
{
	if(m_firstPlayerIndices.empty())
		return NULL;
	else
		return &(m_firstPlayerIndices.at(0));
}

const unsigned int* PCECompoundFrame::getSecondPlayerIndexBuffer() const
{
	if(m_secondPlayerIndices.empty())
		return NULL;
	else
		return &(m_secondPlayerIndices.at(0));
}

bool	PCECompoundFrame::hasSkeleton()
{
	boost::mutex::scoped_lock buff_lock (m_bSkeletonMutex);
	return m_skeletonFrame.isSkeletonEmpty();
}

void PCECompoundFrame::resetSkeleton()
{
	boost::mutex::scoped_lock buff_lock (m_bSkeletonMutex);
	m_skeletonFrame.clear();
}

void PCECompoundFrame::cacheSkeletonFrame(const PCESkeletonFrame& frame)
{
	boost::mutex::scoped_lock buff_lock (m_bSkeletonMutex);
	m_skeletonFrame = frame;
}

bool PCECompoundFrame::readSkeletonPosition(int skeletonId, PCESkeletonPosition* pSkeleton)
{
	if(!pSkeleton)
		return false;
	boost::mutex::scoped_lock buff_lock (m_bSkeletonMutex);
	return m_skeletonFrame.readSkeletonPosition(skeletonId, pSkeleton);
}

bool PCECompoundFrame::getClosestFaceHint(Vector3* pHint3D)
{
	if (!pHint3D)
	{
		return false;
	}

	int selectedSkeleton = -1;
	float smallestDistance = 0;
	PCESkeletonPosition skeletonPos;

	// Lock
	boost::mutex::scoped_lock buff_lock (m_bSkeletonMutex);

	if (pHint3D[1].x == 0 && pHint3D[1].y == 0 && pHint3D[1].z == 0)
	{
		// Get the skeleton closest to the camera
		for (int i = 0 ; i < m_skeletonFrame.getSkeletonSize() ; i++ )
		{
			if( !m_skeletonFrame.readSkeletonPosition(i, &skeletonPos) )
				continue;
			float headZ = skeletonPos.getJointPositionZInSkeletonSpace(SKELETON_POSITION_HEAD);
			if (smallestDistance == 0 || headZ < smallestDistance)
			{
				smallestDistance = headZ;
				selectedSkeleton = i;
			}
		}
	}
	else
	{   // Get the skeleton closest to the previous position
		for (int i = 0 ; i < m_skeletonFrame.getSkeletonSize() ; i++ )
		{
			if( !m_skeletonFrame.readSkeletonPosition(i, &skeletonPos) )
				continue;
			float headX = skeletonPos.getJointPositionXInSkeletonSpace(SKELETON_POSITION_HEAD);
			float headY = skeletonPos.getJointPositionYInSkeletonSpace(SKELETON_POSITION_HEAD);
			float headZ = skeletonPos.getJointPositionZInSkeletonSpace(SKELETON_POSITION_HEAD);
			float d = abs(headX - pHint3D[1].x) +
				abs(headY - pHint3D[1].y) +
				abs(headZ - pHint3D[1].z);
			if (smallestDistance == 0 || d < smallestDistance)
			{
				smallestDistance = d;
				selectedSkeleton = i;
			}
		}
	}
	if (selectedSkeleton != -1)
	{
		m_skeletonFrame.readSkeletonPosition(selectedSkeleton, &skeletonPos);
		pHint3D[0].x = skeletonPos.getJointPositionXInSkeletonSpace(SKELETON_POSITION_SHOULDER_CENTER);
		pHint3D[0].y = skeletonPos.getJointPositionYInSkeletonSpace(SKELETON_POSITION_SHOULDER_CENTER);
		pHint3D[0].z = skeletonPos.getJointPositionZInSkeletonSpace(SKELETON_POSITION_SHOULDER_CENTER);
		pHint3D[1].x = skeletonPos.getJointPositionXInSkeletonSpace(SKELETON_POSITION_HEAD);
		pHint3D[1].y = skeletonPos.getJointPositionYInSkeletonSpace(SKELETON_POSITION_HEAD);
		pHint3D[1].z = skeletonPos.getJointPositionZInSkeletonSpace(SKELETON_POSITION_HEAD);
		return true;
	}

	return false;
}
