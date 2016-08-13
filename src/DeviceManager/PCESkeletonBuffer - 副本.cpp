#include "PCESkeletonBuffer.h"
#include "PointCloudShape/PCEPointCloud.h"

#include <maya/MGlobal.h>

MStatus PCESkeletonBuffer::initialize()
{
	if( fUserTracker_.isValid())
	{
		return MS::kSuccess;
	}
	nite::Status niteRc;
	nite::NiTE::initialize();

	niteRc = fUserTracker_.create();
	if (niteRc != nite::STATUS_OK)
	{
		MGlobal::displayError( "Couldn't create user tracker");
		return MS::kFailure;
	}
	
	MGlobal::displayInfo( "Ready for user tracker");
	return MS::kSuccess;
}

void PCESkeletonBuffer::shutDown()
{
	nite::NiTE::shutdown();
	fUserTracker_.destroy();
}

MStatus PCESkeletonBuffer::grabAndUpdate()
{
	if(!fUserTracker_.isValid())
	{
		return MS::kFailure;
	}
	// Lock
	//boost::mutex::scoped_lock buff_lock (bmutex_);

	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status rc = fUserTracker_.readFrame( &userTrackerFrame );
	if (rc != nite::STATUS_OK)
	{
		MGlobal::displayError( "GetNextData failed." );
		return MS::kFailure;
	}

	const nite::Array<nite::UserData>& aUsers = userTrackerFrame.getUsers();
	for( int i = 0; i < aUsers.getSize(); ++ i )
	{
		const nite::UserData& rUser = aUsers[i];
		if( rUser.isNew() )
		{
			MGlobal::displayInfo( MString("New User [") + rUser.getId() + "] found." );
			// 5a. start tracking skeleton
			fUserTracker_.startSkeletonTracking( rUser.getId() );
		}

		const nite::Skeleton& rSkeleton = rUser.getSkeleton();
		switch( rSkeleton.getState() )
		{
		case nite::SKELETON_NONE:
			MGlobal::displayInfo("Stopped tracking.");
			break;
		case nite::SKELETON_CALIBRATING:
			MGlobal::displayInfo("Calibrating...");
			break;
		case nite::SKELETON_TRACKED:
			MGlobal::displayInfo("Tracking!");
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			MGlobal::displayInfo("Calibration Failed... :-|");
			break;
		}
	}
	return MS::kSuccess;
}

MStatus	PCESkeletonBuffer::recieveSkeletonData(PCEPointCloud* geomPtr)
{
	if(!fUserTracker_.isValid())
	{
		MGlobal::displayError( "Skeleton capture failed." );
		return MS::kFailure;
	}
	// Lock
	//boost::mutex::scoped_lock buff_lock (bmutex_);

	nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status rc = fUserTracker_.readFrame( &userTrackerFrame );
	if (rc != nite::STATUS_OK)
	{
		MGlobal::displayError( "GetNextData failed." );
		return MS::kFailure;
	}
	return MS::kFailure;
	const nite::Array<nite::UserData>& aUsers = userTrackerFrame.getUsers();
	for( int i = 0; i < aUsers.getSize(); ++ i )
	{
		const nite::UserData& rUser = aUsers[i];
		if( rUser.isNew() )
		{
			MGlobal::displayInfo( MString("New User [") + rUser.getId() + "] found." );
			// 5a. start tracking skeleton
			fUserTracker_.startSkeletonTracking( rUser.getId() );
		}
		if( rUser.isLost() )
		{
			geomPtr->skeletonJointPositions._center_Mass[0] = rUser.getCenterOfMass().x;
			geomPtr->skeletonJointPositions._center_Mass[1] = rUser.getCenterOfMass().y;
			geomPtr->skeletonJointPositions._center_Mass[2] = rUser.getCenterOfMass().z;

			geomPtr->skeletonJointPositions._min[0] = rUser.getBoundingBox().min.x;
			geomPtr->skeletonJointPositions._min[1] = rUser.getBoundingBox().min.y;
			geomPtr->skeletonJointPositions._min[2] = rUser.getBoundingBox().min.z;

			geomPtr->skeletonJointPositions._max[0] = rUser.getBoundingBox().max.x;
			geomPtr->skeletonJointPositions._max[1] = rUser.getBoundingBox().max.y;
			geomPtr->skeletonJointPositions._max[2] = rUser.getBoundingBox().max.z;

			MGlobal::displayInfo( MString("User [") + rUser.getId() + "] lost." );
		}

		const nite::Skeleton& rSkeleton = rUser.getSkeleton();
		switch( rSkeleton.getState() )
		{
		case nite::SKELETON_NONE:
			MGlobal::displayInfo("Stopped tracking.");
			break;
		case nite::SKELETON_CALIBRATING:
			MGlobal::displayInfo("Calibrating...");
			break;
		case nite::SKELETON_TRACKED:
			MGlobal::displayInfo("Tracking!");
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			MGlobal::displayInfo("Calibration Failed... :-|");
			break;
		}

		// 5b. get skeleton
		if( rSkeleton.getState() == nite::SKELETON_TRACKED )
		{
			// if is tracked, get joints
			const nite::SkeletonJoint& rHead
				= rSkeleton.getJoint( nite::JOINT_HEAD );
			const nite::Point3f& rPos = rHead.getPosition();
			PCEJointPos& posJoint = geomPtr->skeletonJointPositions._joints.at(JOINT_HEAD);
			posJoint.x = rPos.x;
			posJoint.y = rPos.y;
			posJoint.z = rPos.z;
			posJoint.confidence = rHead.getPositionConfidence();

			MGlobal::displayInfo( MString(" > ") + rPos.x + "/" + rPos.y + "/" + rPos.z + "/ Confidence: " + rHead.getPositionConfidence() );
		}
	}
	// Assemble the user map
	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
	int widthMap = userLabels.getWidth();
	int heightMap = userLabels.getHeight();

	if((widthMap != geomPtr->pntCloud.width) ||
		(heightMap != geomPtr->pntCloud.height))
	{
		MGlobal::displayError( "The skeleton map cannot match the cloud data." );
		return MS::kFailure;
	}
	pcl::PointIndices	indicesMap;
	const nite::UserId* pLabels = userLabels.getPixels();
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
	geomPtr->indicesClusters.push_back(indicesMap);

	return MS::kSuccess;
}