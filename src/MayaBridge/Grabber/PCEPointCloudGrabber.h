#ifndef _PCEPointCloudGrabber
#define _PCEPointCloudGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudGrabber.h
//
// Dependency Graph Node:  PCEPointCloudGrabber
//
// Author: Tingzhu Zhou
//

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

#include "DeviceManager/PCEDeviceCache.h"
#include "DeviceManager/PCEDeviceManager.h"
#include "DeviceManager/PCEPreviewTimer.h"

//Forwards
class PCEPointCloud;
class PCESkeletonPosition;
class PCEFaceTrackingResult;
class PCEFusionMesh;

class PCEPointCloudGrabber : public MPxNode
{
public:
	enum PCEGrabberDeviceMode
	{
		EDeviceMode_DepthColor				= 0,
		EDeviceMode_DepthColorPlayer		= 1,
		EDeviceMode_DepthColorSkeleton		= 2,
		EDeviceMode_DepthColorSkeletonFace	= 3,
		EDeviceMode_Depth					= 4,
		EDeviceMode_Skeleton				= 5,
		EDeviceMode_Fusion					= 6,

		EDeviceMode_LastFlag				= 7,
	};

	enum PCEGrabberPlayMode
	{
		EPlayMode_Preview	= 0,
		EPlayMode_Record	= 1,
		EPlayMode_Review	= 2,

		EPlayMode_LastFlag	= 3,
	};

							PCEPointCloudGrabber();
	virtual					~PCEPointCloudGrabber(); 
	// Override functions
	virtual void			postConstructor();
	virtual MStatus			compute( const MPlug& plug, MDataBlock& datablock );
	
	static  void*			creator();
	static  MStatus			initialize();

	// Callbacks
	static void				attrChangedCB(MNodeMessage::AttributeMessage msg, MPlug & plug, MPlug & otherPlug, void* clientData);
	static void				playingCB( bool isPlaying, void* clientData );

	// Helper functions
	bool					updateDevice();
	bool					updateNearMode();
	void					updatePlayMode();
	void					updatePreviewer();
	void					updatePreviewerMode();
	void					updateaElevationAngle();
	void					updateaSkeletonSmoothParams();

	size_t					cachedPointCloudSize() { return 0; }
	PCEPointCloud*			getCachedPointCloud(int frameId) { return NULL; }
public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject			aTime;
	static  MObject			aDeviceOn;
	static  MObject			aDeviceMode;
	static  MObject			aPlayMode;
	static  MObject			aPreviewer;
	static  MObject			aNearMode;
	static  MObject			aElevationAngle;
	static  MObject			aSkeletonSmooth;

	static  MObject			aOutputCloudData;
	static  MObject			aOutputSkeletonData;
	static  MObject			aOutputFaceTrackingData;
	static  MObject			aOutputFusionMesh;

	static  MObject			aCameraScaleX;
	static  MObject			aCameraScaleY;
	static  MObject			aCameraScaleZ;
	static  MObject			aCameraScale;
	static  MObject			aCameraRotateX;
	static  MObject			aCameraRotateY;
	static  MObject			aCameraRotateZ;
	static  MObject			aCameraRotate;
	static  MObject			aCameraTranslateX;
	static  MObject			aCameraTranslateY;
	static  MObject			aCameraTranslateZ;
	static  MObject			aCameraTranslate;


	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId			id;

private:
	MCallbackIdArray		fCallbackIds;

	void					addCallbacks();
	void					removeCallbacks();

	// Helper functions
	MStatus					computeFusionMesh( MDataBlock& datablock, MObject& meshData );
	void					createEmptyMesh( MObject& out_empytMesh );

	void					updateBuffer();
	bool					readPointCloudData(int frameId, PCEPointCloud* pPointCloud);
	bool					readSkeletonData(int frameId, int skeletonId, PCESkeletonPosition* pSkeleton);
	bool					readFaceTrackingResult(int frameId, PCEFaceTrackingResult* pFaceResult);
	bool					readFusionMesh(PCEFusionMesh* pMesh);
	bool					readCameraPose(int frameId, Matrix4* mat);

	void					startPreviewer();
	void					stopPreviewer();
	static void             PreviewerCallingBack(void* lpParam);

private:	
	PCEDeviceManager*		m_Device;
	PCEPreviewTimer*		m_pPreviewer;

	PCEDeviceCache			m_cache;
	PCEDeviceBuffer			m_buffer;
	bool					m_bUseCache;
};

#endif
