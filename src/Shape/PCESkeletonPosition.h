#ifndef _PCESkeletonPosition
#define _PCESkeletonPosition


/** Available joints in skeleton */
typedef enum
{
	SKELETON_POSITION_HIP_CENTER	= 0,
	SKELETON_POSITION_SPINE	= ( SKELETON_POSITION_HIP_CENTER + 1 ) ,
	SKELETON_POSITION_SHOULDER_CENTER	= ( SKELETON_POSITION_SPINE + 1 ) ,
	SKELETON_POSITION_HEAD	= ( SKELETON_POSITION_SHOULDER_CENTER + 1 ) ,
	SKELETON_POSITION_SHOULDER_LEFT	= ( SKELETON_POSITION_HEAD + 1 ) ,
	SKELETON_POSITION_ELBOW_LEFT	= ( SKELETON_POSITION_SHOULDER_LEFT + 1 ) ,
	SKELETON_POSITION_WRIST_LEFT	= ( SKELETON_POSITION_ELBOW_LEFT + 1 ) ,
	SKELETON_POSITION_HAND_LEFT	= ( SKELETON_POSITION_WRIST_LEFT + 1 ) ,
	SKELETON_POSITION_SHOULDER_RIGHT	= ( SKELETON_POSITION_HAND_LEFT + 1 ) ,
	SKELETON_POSITION_ELBOW_RIGHT	= ( SKELETON_POSITION_SHOULDER_RIGHT + 1 ) ,
	SKELETON_POSITION_WRIST_RIGHT	= ( SKELETON_POSITION_ELBOW_RIGHT + 1 ) ,
	SKELETON_POSITION_HAND_RIGHT	= ( SKELETON_POSITION_WRIST_RIGHT + 1 ) ,
	SKELETON_POSITION_HIP_LEFT	= ( SKELETON_POSITION_HAND_RIGHT + 1 ) ,
	SKELETON_POSITION_KNEE_LEFT	= ( SKELETON_POSITION_HIP_LEFT + 1 ) ,
	SKELETON_POSITION_ANKLE_LEFT	= ( SKELETON_POSITION_KNEE_LEFT + 1 ) ,
	SKELETON_POSITION_FOOT_LEFT	= ( SKELETON_POSITION_ANKLE_LEFT + 1 ) ,
	SKELETON_POSITION_HIP_RIGHT	= ( SKELETON_POSITION_FOOT_LEFT + 1 ) ,
	SKELETON_POSITION_KNEE_RIGHT	= ( SKELETON_POSITION_HIP_RIGHT + 1 ) ,
	SKELETON_POSITION_ANKLE_RIGHT	= ( SKELETON_POSITION_KNEE_RIGHT + 1 ) ,
	SKELETON_POSITION_FOOT_RIGHT	= ( SKELETON_POSITION_ANKLE_RIGHT + 1 ) ,
	SKELETON_POSITION_COUNT	= ( SKELETON_POSITION_FOOT_RIGHT + 1 )
} PCEJointType;

struct PCEJointPosition
{
	float x;
	float y;
	float z;

	bool isInferred;
};

class PCESkeletonPosition
{
public:
	PCESkeletonPosition();
	~PCESkeletonPosition() {}

	void	setJointData(PCEJointType jointType, float x, float y, float z, bool isInferred = false);
	void	setJointInSkeletonSpace(PCEJointType jointType, float x, float y, float z, bool isInferred = false);

	bool	isInferred(PCEJointType jointType) const { return _joints[jointType].isInferred; }
	float	getJointPositionX(PCEJointType jointType) const { return _joints[jointType].x; }
	float	getJointPositionY(PCEJointType jointType) const { return _joints[jointType].y; }
	float	getJointPositionZ(PCEJointType jointType) const { return _joints[jointType].z; }

	float	getJointPositionXInSkeletonSpace(PCEJointType jointType) const { return _jointsInSkeletonSpace[jointType].x; }
	float	getJointPositionYInSkeletonSpace(PCEJointType jointType) const { return _jointsInSkeletonSpace[jointType].y; }
	float	getJointPositionZInSkeletonSpace(PCEJointType jointType) const { return _jointsInSkeletonSpace[jointType].z; }

	PCESkeletonPosition& operator=( const PCESkeletonPosition& other );

private:
	PCEJointPosition		_joints[SKELETON_POSITION_COUNT];
	PCEJointPosition		_jointsInSkeletonSpace[SKELETON_POSITION_COUNT];
};

#endif