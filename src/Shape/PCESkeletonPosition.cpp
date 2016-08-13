#include "PCESkeletonPosition.h"

PCESkeletonPosition::PCESkeletonPosition()
{
	for (int i = 0; i < SKELETON_POSITION_COUNT; ++i)
	{
		_joints[i].x      = 0.0f;
		_joints[i].y      = 0.0f;
		_joints[i].z      = 0.0f;
		_joints[i].isInferred = true;
	}
	for (int i = 0; i < SKELETON_POSITION_COUNT; ++i)
	{
		_jointsInSkeletonSpace[i].x      = 0.0f;
		_jointsInSkeletonSpace[i].y      = 0.0f;
		_jointsInSkeletonSpace[i].z      = 0.0f;
		_jointsInSkeletonSpace[i].isInferred = true;
	}
}

void PCESkeletonPosition::setJointData(PCEJointType jointType, float x, float y, float z, bool isInferred /*= false*/)
{
	_joints[jointType].x = x;
	_joints[jointType].y = y;
	_joints[jointType].z = z;
	_joints[jointType].isInferred = isInferred;
}

void PCESkeletonPosition::setJointInSkeletonSpace(PCEJointType jointType, float x, float y, float z, bool isInferred /*= false*/)
{
	_jointsInSkeletonSpace[jointType].x = x;
	_jointsInSkeletonSpace[jointType].y = y;
	_jointsInSkeletonSpace[jointType].z = z;
	_jointsInSkeletonSpace[jointType].isInferred = isInferred;
}

PCESkeletonPosition& PCESkeletonPosition::operator=( const PCESkeletonPosition& other )
{
	if ( &other != this ) {
		for (int i = 0; i < SKELETON_POSITION_COUNT; ++i)
		{
			_joints[i]      = other._joints[i];
		}
		for (int i = 0; i < SKELETON_POSITION_COUNT; ++i)
		{
			_jointsInSkeletonSpace[i]      = other._jointsInSkeletonSpace[i];
		}
	}

	return *this;
}