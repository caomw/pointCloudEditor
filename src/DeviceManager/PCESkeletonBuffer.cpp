#include "PCESkeletonBuffer.h"
#include "Shape/PCEPointCloud.h"

#include <maya/MGlobal.h>

using namespace pcl;

bool PCESkeletonBuffer::initialize()
{
	if( fUserTracker_.isValid())
	{
		return true;
	}
	nite::Status niteRc;
	nite::NiTE::initialize();

	niteRc = fUserTracker_.create();
	if (niteRc != nite::STATUS_OK)
	{
		MGlobal::displayError( "Couldn't create user tracker");
		return false;
	}
	else
	{
		MStatus stat = MGlobal::executeCommand("source PCESkeletonReady");
		if (!stat) {
			stat.perror("prepare HIK.");
		}
	}
	
	MGlobal::displayInfo( "Ready for user tracker");
	return true;
}

void PCESkeletonBuffer::shutDown()
{
	nite::NiTE::shutdown();
	fUserTracker_.destroy();
}

bool PCESkeletonBuffer::isValid() const
{
	return fUserTracker_.isValid();
}

void PCESkeletonBuffer::grabSkeletonCenter(const nite::UserData& userData, const PointCloud<BufferPointT>::ConstPtr& cloud)
{
	float coordinates[2] = {0};
	fUserTracker_.convertJointCoordinatesToDepth(userData.getCenterOfMass().x, userData.getCenterOfMass().y, userData.getCenterOfMass().z, &coordinates[0], &coordinates[1]);
	
	int indexInCloud = (int)coordinates[1] * cloud->width + (int)coordinates[0];
	if(indexInCloud < cloud->size())
	{
		fSkeletonData.setJointData(SKELETON_POSITION_HIP_CENTER, cloud->at(indexInCloud).x, cloud->at(indexInCloud).y, cloud->at(indexInCloud).z);
	}
}

void PCESkeletonBuffer::grabJoint(const nite::UserData& userData, const PointCloud<BufferPointT>::ConstPtr& cloud, nite::JointType niteType, PCEJointType pceType)
{
	float coordinates[2] = {0};
	const nite::SkeletonJoint& joint = userData.getSkeleton().getJoint(niteType);
	fUserTracker_.convertJointCoordinatesToDepth(joint.getPosition().x, joint.getPosition().y, joint.getPosition().z, &coordinates[0], &coordinates[1]);

	int indexInCloud = (int)coordinates[1] * cloud->width + (int)coordinates[0];
	if(indexInCloud < cloud->size())
	{
		fSkeletonData.setJointData(pceType, cloud->at(indexInCloud).x, cloud->at(indexInCloud).y, cloud->at(indexInCloud).z, (joint.getPositionConfidence() < 0.5f));
	}
}

bool PCESkeletonBuffer::grabAndUpdate(const PointCloud<BufferPointT>::ConstPtr& cloud)
{
	if(!fUserTracker_.isValid() || !cloud)
	{
		return false;
	}
	// Lock
	boost::mutex::scoped_lock buff_lock (bmutex_);

	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status rc = fUserTracker_.readFrame( &userTrackerFrame );
	if (rc != nite::STATUS_OK)
	{
		return false;
	}

	const nite::Array<nite::UserData>& aUsers = userTrackerFrame.getUsers();
	for( int i = 0; i < aUsers.getSize(); ++ i )
	{
		const nite::UserData& rUser = aUsers[i];

		if(i >= PCE_MAX_USERS)
			continue;
		const nite::Skeleton& rSkeleton = rUser.getSkeleton();
		fGrabState[i] = rSkeleton.getState();

		if( rUser.isNew() )
		{
			//MGlobal::displayInfo( MString("New User [") + rUser.getId() + "] found." );
			// 5a. start tracking skeleton
			fUserTracker_.startSkeletonTracking( rUser.getId() );
		}
		else if( !rUser.isLost() && (fGrabState[i] == nite::SKELETON_TRACKED) )
		{
			grabJoint(rUser, cloud, nite::JOINT_HEAD, SKELETON_POSITION_HEAD);
			grabJoint(rUser, cloud, nite::JOINT_NECK, SKELETON_POSITION_SHOULDER_CENTER);

			grabJoint(rUser, cloud, nite::JOINT_LEFT_SHOULDER, SKELETON_POSITION_SHOULDER_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_SHOULDER, SKELETON_POSITION_SHOULDER_RIGHT);
			grabJoint(rUser, cloud, nite::JOINT_LEFT_ELBOW, SKELETON_POSITION_ELBOW_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_ELBOW, SKELETON_POSITION_ELBOW_RIGHT);
			grabJoint(rUser, cloud, nite::JOINT_LEFT_HAND, SKELETON_POSITION_HAND_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_HAND, SKELETON_POSITION_HAND_RIGHT);

			grabJoint(rUser, cloud, nite::JOINT_TORSO, SKELETON_POSITION_SPINE);

			grabJoint(rUser, cloud, nite::JOINT_LEFT_HIP, SKELETON_POSITION_HIP_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_HIP, SKELETON_POSITION_HIP_RIGHT);
			grabJoint(rUser, cloud, nite::JOINT_LEFT_KNEE, SKELETON_POSITION_KNEE_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_KNEE, SKELETON_POSITION_KNEE_RIGHT);
			grabJoint(rUser, cloud, nite::JOINT_LEFT_FOOT, SKELETON_POSITION_FOOT_LEFT);
			grabJoint(rUser, cloud, nite::JOINT_RIGHT_FOOT, SKELETON_POSITION_FOOT_RIGHT);
			grabJoint(rUser, cloud, nite::JOINT_HEAD, SKELETON_POSITION_HEAD);
			grabJoint(rUser, cloud, nite::JOINT_HEAD, SKELETON_POSITION_HEAD);

			grabSkeletonCenter(rUser, cloud);
			
			//MGlobal::displayInfo( MString("User [") + rUser.getId() + "] lost." );
		}
	}
	fUserLabels = userTrackerFrame.getUserMap();
	return true;
}

bool	PCESkeletonBuffer::getSkeletonData(int skeletonId, PCESkeletonPosition* pSkeletonData)
{
	if(!pSkeletonData)
		return false;

	if(!fUserTracker_.isValid())
	{
		MGlobal::displayError( "Skeleton capture failed." );
		return false;
	}
	// Lock
	boost::mutex::scoped_lock buff_lock (bmutex_);

	switch( fGrabState[skeletonId] )
	{
	case nite::SKELETON_NONE:
		MGlobal::displayInfo("Stopped tracking.");
		return false;
	case nite::SKELETON_CALIBRATING:
		MGlobal::displayInfo("Calibrating...");
		return false;
	case nite::SKELETON_TRACKED:
		MGlobal::displayInfo("Tracking!");
		return false;
	case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
	case nite::SKELETON_CALIBRATION_ERROR_HANDS:
	case nite::SKELETON_CALIBRATION_ERROR_LEGS:
	case nite::SKELETON_CALIBRATION_ERROR_HEAD:
	case nite::SKELETON_CALIBRATION_ERROR_TORSO:
		MGlobal::displayInfo("Calibration Failed... :-|");
		return false;
	}

	*pSkeletonData = fSkeletonData;
	return true;
}

bool PCESkeletonBuffer::getUserLabels()
{
	if(!fUserTracker_.isValid())
	{
		MGlobal::displayError( "Skeleton capture failed." );
		return false;
	}
	// Lock
	boost::mutex::scoped_lock buff_lock (bmutex_);

	// Assemble the user map
	int widthMap = fUserLabels.getWidth();
	int heightMap = fUserLabels.getHeight();

	pcl::PointIndices	indicesMap;
	const nite::UserId* pLabels = fUserLabels.getPixels();
	for (int y = 0; y < heightMap; ++y)
	{
		for (int x = 0; x < widthMap; ++x, ++pLabels)
		{
			if (*pLabels != 0)
			{
				indicesMap.indices.push_back(y * widthMap + x);
			}
		}
	}
	//geomPtr->indicesClusters.push_back(indicesMap);

	return true;
}

void PCESkeletonBuffer::updateSkeletonSmoothFactor(float smoothFactor)
{
	fUserTracker_.setSkeletonSmoothingFactor(smoothFactor);
}