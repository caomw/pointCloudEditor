#ifndef _PCESkeletonFrame
#define _PCESkeletonFrame
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletonFrame.h
//
//
// Author: Tingzhu Zhou
//
#include <vector>

#include "PCEDeviceFrame.h"
#include "Shape/PCESkeletonPosition.h"

class PCESkeletonFrame : public PCEDeviceFrame
{
public:
	PCESkeletonFrame ()
		: PCEDeviceFrame()
	{}
	PCESkeletonFrame (long long timeStamp)
		: PCEDeviceFrame(timeStamp)
	{}
	virtual ~PCESkeletonFrame ()
	{
		clear();
	}
	/*PCEKinectSkeletonFrame (const PCEKinectSkeletonFrame& other)
	{
		deepCopy(other);
	}
	PCEKinectSkeletonFrame& operator = (const PCEKinectSkeletonFrame& other)
	{
		deepCopy(other);
		return *this;

	void deepCopy (const PCEKinectSkeletonFrame& other)
	{
		if(!other.m_pSkeletonData)
			clear();
		if(!m_pSkeletonData)
			m_pSkeletonData = new PCEKinectSkeletonData();
		*m_pSkeletonData = *other.m_pSkeletonData;
	}*/

	void clear()
	{
		m_detailSkeletons.clear();
		m_positionJoints.clear();
	}

	bool isEmpty() const { return m_detailSkeletons.empty() && m_positionJoints.empty(); }
	bool isSkeletonEmpty() const { return m_detailSkeletons.empty(); }

	void pushSkeletonPosition(const PCESkeletonPosition& skel)
	{
		m_detailSkeletons.push_back(skel);
	}
	bool readSkeletonPosition(int skeletonId, PCESkeletonPosition* pSkPosition) const
	{
		if(!pSkPosition)
			return false;

		if(skeletonId >= m_detailSkeletons.size())
			return false;
		
		*pSkPosition = m_detailSkeletons.at(skeletonId);
		return true;
	}
	size_t getSkeletonSize() const { return m_detailSkeletons.size(); }

	void pushJointPosition(const PCEJointPosition& joint)
	{
		m_positionJoints.push_back(joint);
	}

	/*void setSkeletonData(PCEKinectSkeletonData*	pSkeletonData)
	{
		if(!pSkeletonData || (pSkeletonData == m_pSkeletonData))
			return;
		if(m_pSkeletonData)
			delete m_pSkeletonData;
		m_pSkeletonData = pSkeletonData;
	}*/

private:
	std::vector<PCESkeletonPosition>	m_detailSkeletons;
	std::vector<PCEJointPosition>		m_positionJoints;
};

#endif
