//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudGrabber.cpp
//
// Dependency Graph Node: pointCloudGrabber
//
// Author: Tingzhu Zhou
//

#include "PCEPointCloudGrabber.h"
#include "DeviceManager/OpenNI/PCEOpenNI2Manager.h"
#include "DeviceManager/Kinect/PCEKinectManager.h"
#include "../PointCloudShape/PCEPointCloudData.h"
#include "../Skeleton//PCESkeletonData.h"
#include "../Skeleton/PCEFaceTrackingData.h"
#include "PCEGrabberRenderOverride.h"
#include "PCEGrabberPreView.h"
#include "PCEGrabberPreViewCmd.h"

#include <Eigen/Core>

#include "api_macros.h"

#include <maya/MTime.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>
#include <maya/MAnimControl.h>
#include <maya/MConditionMessage.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatPointArray.h>

using namespace pcl;

MTypeId     PCEPointCloudGrabber::id( 0x80089 );

// Attributes
// 
MObject     PCEPointCloudGrabber::aTime;
MObject		PCEPointCloudGrabber::aDeviceOn;
MObject		PCEPointCloudGrabber::aDeviceMode;
MObject		PCEPointCloudGrabber::aPlayMode;
MObject		PCEPointCloudGrabber::aPreviewer;
MObject		PCEPointCloudGrabber::aNearMode;
MObject		PCEPointCloudGrabber::aElevationAngle;
MObject     PCEPointCloudGrabber::aSkeletonSmooth;

MObject     PCEPointCloudGrabber::aOutputCloudData;
MObject     PCEPointCloudGrabber::aOutputSkeletonData;
MObject     PCEPointCloudGrabber::aOutputFaceTrackingData;
MObject     PCEPointCloudGrabber::aOutputFusionMesh;
MObject		PCEPointCloudGrabber::aCameraScaleX;
MObject		PCEPointCloudGrabber::aCameraScaleY;
MObject		PCEPointCloudGrabber::aCameraScaleZ;
MObject		PCEPointCloudGrabber::aCameraScale;
MObject		PCEPointCloudGrabber::aCameraRotateX;
MObject		PCEPointCloudGrabber::aCameraRotateY;
MObject		PCEPointCloudGrabber::aCameraRotateZ;
MObject		PCEPointCloudGrabber::aCameraRotate;
MObject		PCEPointCloudGrabber::aCameraTranslateX;
MObject		PCEPointCloudGrabber::aCameraTranslateY;
MObject		PCEPointCloudGrabber::aCameraTranslateZ;
MObject		PCEPointCloudGrabber::aCameraTranslate;

PCEPointCloudGrabber::PCEPointCloudGrabber()
	: m_Device(NULL)
	, m_pPreviewer(NULL)
	, m_bUseCache(false)
{
}
PCEPointCloudGrabber::~PCEPointCloudGrabber()
{
	if(m_pPreviewer)
	{
		delete m_pPreviewer;
		m_pPreviewer = NULL;
	}
	if(m_Device)
	{
		m_Device->shutdownDevice();
		delete m_Device;
		m_Device = NULL;
	}
	m_cache.clear();
	m_buffer.clear();
	removeCallbacks();
}

void PCEPointCloudGrabber::attrChangedCB(MNodeMessage::AttributeMessage msg, MPlug & plug, MPlug & otherPlug, void* clientData)
{
	PCEPointCloudGrabber *wrapper = static_cast<PCEPointCloudGrabber *>(clientData);
	if(!wrapper)
		return;

	MStatus stat;
	MObject attr = plug.attribute(&stat);
	MFnAttribute fnAttr(attr);

	if (fnAttr.name() == "deviceOn" ||
		fnAttr.name() == "deviceMode" ) {
		wrapper->updateDevice();
	}
	else if (fnAttr.name() == "previewerOn") {
		wrapper->updatePreviewer();
	}
	else if (fnAttr.name() == "nearMode") {
		wrapper->updateNearMode();
	}
	else if (fnAttr.name() == "playMode") {
		wrapper->updatePlayMode();
	}
	else if (fnAttr.name() == "elevationAngle") {
		wrapper->updateaElevationAngle();
	}
	else if (fnAttr.name() == "skeletonSmooth") {
		wrapper->updateaSkeletonSmoothParams();
	}
}

void PCEPointCloudGrabber::playingCB( bool isPlaying, void* clientData )
{
	PCEPointCloudGrabber *wrapper = static_cast<PCEPointCloudGrabber *>(clientData);
	if(!wrapper)
		return;

	if(isPlaying && MAnimControl::isPlaying())
		wrapper->updateDevice();
	/*else
		wrapper->pauseDevice();*/
}

void PCEPointCloudGrabber::addCallbacks()
{
	MStatus status;
	
	MObject node = thisMObject();
	fCallbackIds.append( MNodeMessage::addAttributeChangedCallback(node, attrChangedCB, (void*)this) );
	//fCallbackIds.append( MConditionMessage::addConditionCallback( MString("playingBack"), playingCB, (void*)this) );
}

void PCEPointCloudGrabber::removeCallbacks()
{
	MMessage::removeCallbacks(fCallbackIds);
}

void PCEPointCloudGrabber::postConstructor()
{
	addCallbacks();
	m_Device = new PCEKinectManager();
	m_Device->updateSkeletonSmoothParams(0.1f, 0.5f, 0.5f, 0.05f, 0.04f);
	/*MStatus stat = MGlobal::executeCommand("source PCESkeletonSetup");
	if (!stat) {
	stat.perror("setup HIK.");
	}*/
}

void PCEPointCloudGrabber::createEmptyMesh( MObject& out_empytMesh )
{
	MStatus status;
	MFnMeshData meshData;
	out_empytMesh = meshData.create( &status );
	CHECK_MSTATUS( status );

	MFloatPointArray  	vertexArray;
	MIntArray  	        polygonCounts;
	MIntArray  	        polygonConnects;

	MFnMesh meshCreator;
	MObject newMeshObject = meshCreator.create( 
		0, // nb vertices
		0, // nb triangles 
		vertexArray, 
		polygonCounts, 
		polygonConnects, 
		out_empytMesh );
}

MStatus PCEPointCloudGrabber::computeFusionMesh( 
	MDataBlock&			datablock,
	MObject&			meshData)
//
// Description
//
//     This function takes an input surface of type kMeshData and converts
//     the geometry into this nodes attributes.
//     Returns kFailure if nothing is connected.
//
{
	MStatus stat;

	PCEFusionMesh fusionMesh;
	if(!readFusionMesh(&fusionMesh))
		return MS::kFailure;

	unsigned int vertexCount = fusionMesh.getVertexCount();
	Vector3* pVertices = fusionMesh.getVertices();
	if(!pVertices || 0 == vertexCount)
		return MS::kFailure;

	MFloatPointArray vertexArray;
	MIntArray polygonCounts;
	MIntArray polygonConnects;

	for (unsigned int vtId = 0; vtId < vertexCount; vtId++)
	{
		Vector3 pts = pVertices[vtId];
		vertexArray.append(pts.x, pts.y, pts.z);
	}

	int* pTriangleIndices = fusionMesh.getTriangleIndices();
	if(pTriangleIndices)
	{
		for (unsigned int trId = 0; trId < fusionMesh.getTriangleIndexCount(); trId++)
		{
			int triIndex = pTriangleIndices[trId];
			polygonConnects.append(triIndex);
			if(trId % 3 == 2)
				polygonCounts.append(3);
		}
	}

	MFnMesh meshFn;
	meshFn.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, meshData, &stat);

	return stat;
}

MStatus PCEPointCloudGrabber::compute( const MPlug& plug, MDataBlock& datablock )
//
//	Description:
//		This method computes the value of the given output plug based
//		on the values of the input attributes.
//
//	Arguments:
//		plug - the plug to compute
//		data - object that provides access to the attributes for this node
//
{
	MStatus returnStatus;
 
	// Check which output attribute we have been asked to compute.  If this 
	// node doesn't know how to compute it, we must return 
	// MS::kUnknownParameter.
	// 
	if ( plug == aOutputCloudData ) {

		/* Get time */
		MDataHandle timeData = datablock.inputValue( aTime, &returnStatus ); 
		MCHECKERROR(returnStatus, "Error getting time data handle\n");
		MTime time = timeData.asTime();
		//!< 30 frames per second
		int	  frame = (int)time.as( MTime::kNTSCFrame ) - 1;//Noted: The first frame in MAYA is 1;

		// Create some user defined geometry data and access the
		// geometry so we can set it
		//
		MFnPluginData fnDataCreator;
		MTypeId tmpid( PCEPointCloudData::id );

		fnDataCreator.create( tmpid, &returnStatus );
		MCHECKERROR( returnStatus, "compute : error creating pointCloudData")

		PCEPointCloudData * newData = (PCEPointCloudData*)fnDataCreator.data( &returnStatus );
		MCHECKERROR( returnStatus, "compute : error gettin at proxy pointCloudData object")

		PCEPointCloud* geomPtr = newData->pCloudData.get();

		if(readPointCloudData(frame, geomPtr))
		{
			MDataHandle outHandle = datablock.outputValue( aOutputCloudData );
			outHandle.set( newData );
			datablock.setClean( plug );
		}
		else
		{
			//MGlobal::displayError( " Cannot receive the point data from the cache." );
			return MS::kFailure;
		}
		/*if(MS::kSuccess == returnStatus)
		{
			if(fDevice && fDevice->recieveSkeletonData(geomPtr))
			{
				fCache.insertSkeletonData(frame, geomPtr->skeletonJointPositions);
				fCache.insertClusterData(frame, geomPtr->indicesClusters);
			}
		}*/

		// There is no grabbed data so check the shapeType attribute
		// and create either a cube or a sphere.
		//
		/*if ( !returnStatus ) {
			loadCloudData(frame, geomPtr);
		}*/

		// Assign the new data to the outputSurface handle
		//
	}
	else if ( plug == aOutputSkeletonData ) {

		/* Get time */
		MDataHandle timeData = datablock.inputValue( aTime, &returnStatus ); 
		MCHECKERROR(returnStatus, "Error getting time data handle\n");
		MTime time = timeData.asTime();
		//!< 30 frames per second
		int	  frame = (int)time.as( MTime::kNTSCFrame ) - 1;//Noted: The first frame in MAYA is 1;

		// Create some user defined geometry data and access the
		// geometry so we can set it
		//
		MFnPluginData fnDataCreator;
		MTypeId tmpid( PCESkeletonData::id );

		fnDataCreator.create( tmpid, &returnStatus );
		MCHECKERROR( returnStatus, "compute : error creating skeleton data")

		PCESkeletonData * newData = (PCESkeletonData*)fnDataCreator.data( &returnStatus );
		MCHECKERROR( returnStatus, "compute : error gettin at proxy pointCloudData object")

		PCESkeletonPosition* skeletonPtr = newData->pSkeletonData;

		if(readSkeletonData(frame, 0, skeletonPtr))
		{
			// Assign the new data to the outputSurface handle
			//
			MDataHandle outHandle = datablock.outputValue( aOutputSkeletonData );
			outHandle.set( newData );
			datablock.setClean( plug );
		}
		else
		{
			//MGlobal::displayError( " Cannot receive the skeleton from the cache." );
			return MS::kFailure;
		}
	}
	else if ( plug == aOutputFaceTrackingData ) {
		
		/* Get time */
		MDataHandle timeData = datablock.inputValue( aTime, &returnStatus ); 
		MCHECKERROR(returnStatus, "Error getting time data handle\n");
		MTime time = timeData.asTime();
		//!< 30 frames per second
		int	  frame = (int)time.as( MTime::kNTSCFrame ) - 1;//Noted: The first frame in MAYA is 1;

		// Create some user defined geometry data and access the
		// geometry so we can set it
		//
		MFnPluginData fnDataCreator;
		MTypeId tmpid( PCEFaceTrackingData::id );

		fnDataCreator.create( tmpid, &returnStatus );
		MCHECKERROR( returnStatus, "compute : error creating faceData")

		PCEFaceTrackingData * newData = (PCEFaceTrackingData*)fnDataCreator.data( &returnStatus );
		MCHECKERROR( returnStatus, "compute : error gettin at proxy face data object")

		PCEFaceTrackingResult* faceResultPtr = newData->pFaceResult;
		if(readFaceTrackingResult(frame, faceResultPtr))
		{
			// Assign the new data to the outputSurface handle
			//
			MDataHandle outHandle = datablock.outputValue( aOutputFaceTrackingData );
			outHandle.set( newData );
			datablock.setClean( plug );
		}
		else
		{
			//MGlobal::displayError( " Cannot receive the face tracking from the cache." );
			return MS::kFailure;
		}
	}
	else if ( plug == aOutputFusionMesh ) {

		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&returnStatus);
		if (!returnStatus) {
			cerr << "ERROR creating outputData\n";
			return returnStatus;
		}

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		returnStatus = computeFusionMesh(datablock, newOutputData);
		if(returnStatus != MS::kSuccess)
		{
			createEmptyMesh( newOutputData );
		}
		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( aOutputFusionMesh );
		outHandle.set( newOutputData );
		datablock.setClean( plug );
		return MS::kSuccess;
	}
	else if(plug == aCameraScale ||
		plug == aCameraScaleX ||
		plug == aCameraScaleY ||
		plug == aCameraScaleZ ||
		plug == aCameraRotate ||
		plug == aCameraRotateX || 
		plug == aCameraRotateY || 
		plug == aCameraRotateZ ||
		plug == aCameraTranslate ||
		plug == aCameraTranslateX || 
		plug == aCameraTranslateY || 
		plug == aCameraTranslateZ)
	{
		/* Get time */
		MDataHandle timeData = datablock.inputValue( aTime, &returnStatus ); 
		MCHECKERROR(returnStatus, "Error getting time data handle\n");
		MTime time = timeData.asTime();
		//!< 30 frames per second
		int	  frame = (int)time.as( MTime::kNTSCFrame ) - 1;//Noted: The first frame in MAYA is 1;

		Matrix4 cameraMatrix;
		if(readCameraPose(frame, &cameraMatrix))
		{
			MDataHandle otHandle = datablock.outputValue( aCameraTranslate ); 
			otHandle.set( (double)cameraMatrix.M14,
				(double)cameraMatrix.M24,
				(double)cameraMatrix.M34 );
			datablock.setClean(aCameraTranslate);

			Eigen::Matrix3f transform;
			transform(0,0) = cameraMatrix.M11;
			transform(0,1) = cameraMatrix.M12;
			transform(0,2) = cameraMatrix.M13;
			transform(1,0) = cameraMatrix.M21;
			transform(1,1) = cameraMatrix.M22;
			transform(1,2) = cameraMatrix.M23;
			transform(2,0) = cameraMatrix.M31;
			transform(2,1) = cameraMatrix.M32;
			transform(2,2) = cameraMatrix.M33;

			Eigen::Quaternion<float> cameraPose (transform);
			float scale = cameraPose.norm();
			cameraPose.normalize();
			otHandle = datablock.outputValue( aCameraRotate ); 
			otHandle.set( (double)cameraPose.x(),
				(double)cameraPose.y(),
				(double)cameraPose.z() );
			datablock.setClean(aCameraRotate);

			otHandle = datablock.outputValue( aCameraScale );
			otHandle.set( (double)scale,
				(double)scale,
				(double)scale );
			datablock.setClean(aCameraScale);
		}
		else
		{
			//MGlobal::displayError( " Cannot receive the face tracking from the cache." );
			return MS::kFailure;
		}
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

bool PCEPointCloudGrabber::updateDevice()
{
	assert(m_Device);
	if(!m_Device)
		return false;

	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool isDeviceOn = false;
	MDataHandle inputHandle = datablock.inputValue( aDeviceOn, &returnStatus );
	if(returnStatus == MS::kSuccess)
		isDeviceOn = inputHandle.asBool();
	
	if(isDeviceOn)
	{
		short deviceMode = EDeviceMode_DepthColor;
		MDataHandle inputHandle = datablock.inputValue( aDeviceMode, &returnStatus );
		if(returnStatus == MS::kSuccess)
			deviceMode = inputHandle.asShort();

		int deviceFlag = 0;
		switch (deviceMode)
		{
		case EDeviceMode_DepthColor:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_Color_On;
			break;
		case EDeviceMode_DepthColorPlayer:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_Color_On | PCEDeviceManager::EDevice_PlayIndex_On;
			break;
		case EDeviceMode_DepthColorSkeleton:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_Color_On | PCEDeviceManager::EDevice_PlayIndex_On | PCEDeviceManager::EDevice_Skeleton_On;
			break;
		case EDeviceMode_DepthColorSkeletonFace:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_Color_On | PCEDeviceManager::EDevice_PlayIndex_On | PCEDeviceManager::EDevice_Skeleton_On | PCEDeviceManager::EDevice_Face_On;
			break;
		case EDeviceMode_Depth:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_PlayIndex_On;
			break;
		case EDeviceMode_Skeleton:
			deviceFlag = PCEDeviceManager::EDevice_Skeleton_On;
			break;
		case EDeviceMode_Fusion:
			deviceFlag = PCEDeviceManager::EDevice_Depth_On | PCEDeviceManager::EDevice_Color_On | PCEDeviceManager::EDevice_Fusion_On;
			break;
		default:
			assert(0);
			return false;
		}
		if(m_Device && m_Device->initializeDevice(deviceFlag))
		{
			MGlobal::displayInfo( " Initialize the device." );
			if(m_Device->isDeviceOn())
			{
				updateBuffer();
				updateNearMode();
				m_Device->startDevice();
			}
			else
			{
				MGlobal::displayError( " Failed to start the device." );
				return false;
			}
		}
		else
		{
			MGlobal::displayError( " Failed to setup the device." );
			return false;
		}
	}
	else if(m_Device)
	{
		m_Device->shutdownDevice();
	}
	return true;
}

void PCEPointCloudGrabber::updateaElevationAngle()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	int elevationAngle = 0;
	MDataHandle inputHandle = datablock.inputValue( aElevationAngle, &returnStatus );
	if(returnStatus == MS::kSuccess)
		elevationAngle = inputHandle.asInt();

	if(m_Device)
		m_Device->updateElevationAngle(elevationAngle);
}

bool PCEPointCloudGrabber::updateNearMode()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bNearMode = false;
	MDataHandle inputHandle = datablock.inputValue( aNearMode, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bNearMode = inputHandle.asBool();

	return (m_Device ? m_Device->updateNearMode(bNearMode) : false);
}

void PCEPointCloudGrabber::updatePlayMode()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	short playMode = EPlayMode_Preview;
	MDataHandle inputHandle = datablock.inputValue( aPlayMode, &returnStatus );
	if(returnStatus == MS::kSuccess)
		playMode = inputHandle.asShort();
	 m_bUseCache = (EPlayMode_Preview != playMode);
	 updateBuffer();
}

void PCEPointCloudGrabber::updateBuffer()
{
	if(m_Device)
		m_Device->updateBuffer(m_bUseCache ? reinterpret_cast<PCEDeviceCacheImp*>(&m_cache) : reinterpret_cast<PCEDeviceCacheImp*>(&m_buffer));
	if(m_pPreviewer)
		m_pPreviewer->updateBuffer(m_bUseCache ? reinterpret_cast<PCEDeviceCacheImp*>(&m_cache) : reinterpret_cast<PCEDeviceCacheImp*>(&m_buffer));
}

void PCEPointCloudGrabber::updateaSkeletonSmoothParams()
{
	if(!m_Device)
		return;

	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	float skeletonSmoothFactor = 0.1f;
	MDataHandle inputHandle = datablock.inputValue( aSkeletonSmooth, &returnStatus );
	if(returnStatus == MS::kSuccess)
		skeletonSmoothFactor = inputHandle.asFloat();

	m_Device->updateSkeletonSmoothParams(skeletonSmoothFactor, 0.5f, 0.5f, 0.05f, 0.04f);
}


bool PCEPointCloudGrabber::readPointCloudData(int frameId, PCEPointCloud* pPointCloud)
{
	if(!pPointCloud)
		return false;

	PCECompoundFrame* pFrame = NULL;
	if ( m_bUseCache )
		pFrame = m_cache.getCompoundFrame(frameId);
	else
		pFrame = m_buffer.getCompoundFrame(frameId);
	if (!pFrame)
	{
		printf_s( "Failed to read compound frame.\n" );
		return false;
	}

	bool readStatus = false;
	// Lock
	pFrame->readImageLock();
	
	const UINT depthWidth = pFrame->getDepthWidth();
	const UINT depthHeight = pFrame->getDepthHeight();
	USHORT* pDepthBuffer = pFrame->getDepthBuffer();
	BYTE* pAlignedColorBuffer = pFrame->getAlignedColorBuffer();
	if(pDepthBuffer)
	{
		pPointCloud->initialize(depthWidth, depthHeight);

		// loop over each row and column of the color
#pragma omp parallel for schedule(dynamic)
		for (UINT y = 0; y < depthHeight; ++y)
		{
			for (UINT x = 0; x < depthWidth; ++x)
			{
				// calculate index into depth array
				UINT depthIndex = x + y * depthWidth;

				PCEPointT pnt;
				pnt.x = (float)(x)-depthWidth/2;
				pnt.y = -(float)(y)+depthHeight/2;
				pnt.z = -(float)(pDepthBuffer[depthIndex]);

				if(pAlignedColorBuffer)
				{
					BYTE* colorValue = pAlignedColorBuffer + depthIndex * pFrame->getColorBytesPerPixel();
					pnt.b = colorValue[0];
					pnt.g = colorValue[1];
					pnt.r = colorValue[2];
					pnt.a = colorValue[3];
				}

				// compute initilize UV
				pnt.u = float(x % depthWidth) / depthWidth;
				pnt.v = float(x / depthWidth) / depthHeight;
				pPointCloud->setPtPosition(x, y, pnt);
			}
		}
	}
	UINT indexSize = pFrame->getValidIndexSize();
	const unsigned int* validBuffer = pFrame->getValidIndexBuffer();
	if(indexSize > 0 && validBuffer)
	{
		pPointCloud->resizeValidIndex(indexSize);
		//memcpy_s((void*)(pPointCloud->getValidIndexBuffer()), pPointCloud->getValidIndexSize(), (void*)(validBuffer), indexSize);
		for (unsigned int i = 0; i < indexSize; ++i)
		{
			pPointCloud->pushValidIndex(pFrame->getValidIndex(i));
		}
	}
	indexSize = pFrame->getFirstPlayerIndexSize();
	const unsigned int* firstPlayerBuffer = pFrame->getFirstPlayerIndexBuffer();
	if(indexSize > 0 && firstPlayerBuffer)
	{
		pPointCloud->resizeFirstPlayerIndex(indexSize);
		memcpy_s((void*)(pPointCloud->getFirstPlayerIndexBuffer()), pPointCloud->getFirstPlayerIndexSize(), (void*)(firstPlayerBuffer), indexSize);
	}
	indexSize = pFrame->getSecondPlayerIndexSize();
	const unsigned int* secondPlayerBuffer = pFrame->getSecondPlayerIndexBuffer();
	if(indexSize > 0 && secondPlayerBuffer)
	{
		pPointCloud->resizeFirstPlayerIndex(indexSize);
		memcpy_s((void*)(pPointCloud->getSecondPlayerIndexBuffer()), pPointCloud->getSecondPlayerIndexSize(), (void*)(secondPlayerBuffer), indexSize);
	}
	
	pFrame->readImageUnlock();

	return true;
}

bool PCEPointCloudGrabber::readSkeletonData(int frameId, int skeletonId, PCESkeletonPosition* pSkeleton)
{
	if(!pSkeleton)
		return false;

	PCECompoundFrame* pFrame = NULL;
	if ( m_bUseCache )
		pFrame = m_cache.getCompoundFrame(frameId);
	else
		pFrame = m_buffer.getCompoundFrame(frameId);
	if (!pFrame)
	{
		printf_s( "Failed to read compound frame.\n" );
		return false;
	}
	return pFrame->readSkeletonPosition(skeletonId, pSkeleton);
}

bool PCEPointCloudGrabber::readFaceTrackingResult(int frameId, PCEFaceTrackingResult* pFaceResult)
{
	if(!pFaceResult)
		return false;

	if ( m_bUseCache )
		return m_cache.readFaceShape(frameId, pFaceResult);
	else
		return m_buffer.readFaceShape(frameId, pFaceResult);
}

bool PCEPointCloudGrabber::readFusionMesh(PCEFusionMesh* pMesh)
{
	if(!pMesh)
		return false;

	if ( !m_bUseCache )
		return false;
	/*return m_cache.readFusionMesh(pMesh);
	else
		return m_buffer.readFusionMesh(frameId, pMesh);*/
}

bool PCEPointCloudGrabber::readCameraPose(int frameId, Matrix4* mat)
{
	if(!mat)
		return false;

	if ( !m_bUseCache )
		return false;
	return m_cache.readCameraPose(frameId, mat);
	/*else
		return m_buffer.readFusionMesh(frameId, pMesh);*/
}

void* PCEPointCloudGrabber::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new PCEPointCloudGrabber();
}

MStatus PCEPointCloudGrabber::initialize()
//
//	Description:
//		This method is called to create and initialize all of the attributes
//      and attribute dependencies for this node type.  This is only called 
//		once when the node type is registered with Maya.
//
//	Return Values:
//		MS::kSuccess
//		MS::kFailure
//		
{
	// This sample creates a single input float attribute and a single
	// output float attribute.
	//
	MFnNumericAttribute nAttr;
	MFnTypedAttribute	typedAttr;
	MFnUnitAttribute	unitAttr;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;

	aTime = unitAttr.create( "time", "tm",
							MFnUnitAttribute::kTime,
							0.0, &stat );
	stat = addAttribute( aTime );
	if (!stat) { stat.perror("addAttribute time"); return stat;}

	aDeviceOn = nAttr.create( "deviceOn", "do", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aDeviceOn );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aDeviceMode = enumAttr.create( "deviceMode", "dm", EDeviceMode_DepthColorPlayer, &stat );
	if (!stat) { stat.perror("create DeviceMode attribute"); return stat;}
	stat = enumAttr.addField( "Depth,Color", EDeviceMode_DepthColor );
	if (!stat) { stat.perror("add enum type DepthColor"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Player", EDeviceMode_DepthColorPlayer );
	if (!stat) { stat.perror("add enum type DepthColorPlayer"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Skeleton", EDeviceMode_DepthColorSkeleton );
	if (!stat) { stat.perror("add enum type DepthColorSkeleton"); return stat;}
	stat = enumAttr.addField( "Depth,Color,Skeleton,Face", EDeviceMode_DepthColorSkeletonFace );
	if (!stat) { stat.perror("add enum type DepthColorSkeletonFace"); return stat;}
	stat = enumAttr.addField( "Depth", EDeviceMode_Depth );
	if (!stat) { stat.perror("add enum type Depth"); return stat;}
	stat = enumAttr.addField( "Skeleton", EDeviceMode_Skeleton );
	if (!stat) { stat.perror("add enum type Skeleton"); return stat;}
	stat = enumAttr.addField( "Fusion", EDeviceMode_Fusion );
	if (!stat) { stat.perror("add enum type fusion"); return stat;}
	CHECK_MSTATUS( enumAttr.setHidden( false ) );
	CHECK_MSTATUS( enumAttr.setKeyable( false ) );
	stat = addAttribute( aDeviceMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aPlayMode = enumAttr.create( "playMode", "pm", EPlayMode_Preview, &stat );
	if (!stat) { stat.perror("create PlayMode attribute"); return stat;}
	stat = enumAttr.addField( "Preview", EPlayMode_Preview );
	if (!stat) { stat.perror("add enum type Preview"); return stat;}
	stat = enumAttr.addField( "Record", EPlayMode_Record );
	if (!stat) { stat.perror("add enum type Record"); return stat;}
	stat = enumAttr.addField( "PlayBack", EPlayMode_Review );
	if (!stat) { stat.perror("add enum type PlayBack"); return stat;}
	CHECK_MSTATUS( enumAttr.setHidden( false ) );
	CHECK_MSTATUS( enumAttr.setKeyable( false ) );
	stat = addAttribute( aPlayMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aPreviewer = nAttr.create( "previewerOn", "po", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aPreviewer );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aNearMode = nAttr.create( "nearMode", "ne", MFnNumericData::kBoolean, false );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aNearMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aElevationAngle = nAttr.create( "elevationAngle", "ea", MFnNumericData::kInt, 0 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(-27);
	nAttr.setMax(27);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aElevationAngle );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aSkeletonSmooth = nAttr.create("skeletonSmooth", "ssm", MFnNumericData::kFloat, 0.1, &stat);
	MCHECKERROR( stat, "create aSkeletonSmooth attribute" )
		nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	nAttr.setMin(0.0);
	nAttr.setSoftMin(0.01);
	nAttr.setSoftMax(3.0);
	ADD_ATTRIBUTE( aSkeletonSmooth );


	// ----------------------- OUTPUTS -------------------------
	aOutputCloudData = typedAttr.create( "outputCloudData", "ocd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputCloudData attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputSkeletonData = typedAttr.create( "outputSkeletonData", "osd",
		PCESkeletonData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputSkeletonData attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputSkeletonData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputFaceTrackingData = typedAttr.create( "outputFaceTrackingData", "oft",
		PCEFaceTrackingData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputFaceTrackingData attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputFaceTrackingData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOutputFusionMesh = typedAttr.create( "outputFusionMesh", "ofm",
		MFnData::kMesh,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputFusionMesh attribute"); return stat;}
	typedAttr.setWritable( false );
	stat = addAttribute( aOutputFusionMesh );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCameraScaleX = nAttr.create( "cameraScaleX", "csX", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create cameraScaleX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraScaleY = nAttr.create( "cameraScaleY", "csY", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create cameraScaleY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraScaleZ = nAttr.create( "cameraScaleZ", "csZ", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create cameraScaleZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraScale = nAttr.create( "cameraScale", "cs", aCameraScaleX, aCameraScaleY, aCameraScaleZ, &stat );
	if (!stat) { stat.perror("create cameraScale attribute"); return stat;}
	nAttr.setDefault(1.0f, 1.0f, 1.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCameraScale );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCameraRotateX = nAttr.create( "cameraRotateX", "crX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraRotateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotateY = nAttr.create( "cameraRotateY", "crY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotateZ = nAttr.create( "cameraRotateZ", "crZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraRotate = nAttr.create( "cameraRotate", "cr", aCameraRotateX, aCameraRotateY, aCameraRotateZ, &stat );
	if (!stat) { stat.perror("create cameraRotate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCameraRotate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCameraTranslateX = nAttr.create( "cameraTranslateX", "ctX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslateY = nAttr.create( "cameraTranslateY", "ctY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslateZ = nAttr.create( "cameraTranslateZ", "ctZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create cameraTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCameraTranslate = nAttr.create( "cameraTranslate", "ct", aCameraTranslateX, aCameraTranslateY, aCameraTranslateZ, &stat );
	if (!stat) { stat.perror("create cameraTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCameraTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aTime, aOutputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aOutputSkeletonData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aOutputFaceTrackingData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aOutputFusionMesh );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraScaleX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraScaleY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraScaleZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraScale );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aTime, aCameraTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aPlayMode, aOutputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aPlayMode, aOutputSkeletonData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aPlayMode, aOutputFaceTrackingData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	
	return MS::kSuccess;
}

void PCEPointCloudGrabber::startPreviewer()
{
	if(!m_pPreviewer)
	{
		m_pPreviewer = new PCEPreviewTimer(PreviewerCallingBack);
	}
	assert(m_pPreviewer);
	updateBuffer();
	int interval = 100;
	m_pPreviewer->start(interval);
}

void PCEPointCloudGrabber::stopPreviewer()
{
	if(m_pPreviewer)
	{
		m_pPreviewer->stop();
	}
}

void PCEPointCloudGrabber::updatePreviewer()
{
	MStatus returnStatus;
	MDataBlock datablock = forceCache();

	bool bPreviewerOn = false;
	MDataHandle inputHandle = datablock.inputValue( aPreviewer, &returnStatus );
	if(returnStatus == MS::kSuccess)
		bPreviewerOn = inputHandle.asBool();

	updatePreviewerMode();
	if(bPreviewerOn)
		startPreviewer();
	else
		stopPreviewer();
}

void PCEPointCloudGrabber::updatePreviewerMode()
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer)
		return;

	PCEGrabberRenderOverride* renderOverrideInstance = (PCEGrabberRenderOverride*)
		renderer->findRenderOverride(PCEGrabberRenderOverride::kPCEGrabberPreviewerName);
	if (!renderOverrideInstance)
		return;

	int flags = PCEGrabberRenderOverride::EPreviewNone;
	flags |= PCEGrabberRenderOverride::EFusionMode;
	flags |= PCEGrabberRenderOverride::EColorMode;
	renderOverrideInstance->updatePreviewerMode((PCEGrabberRenderOverride::PreviewModeFlag)flags);
}

void PCEPointCloudGrabber::PreviewerCallingBack(PVOID pVoid)
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer)
		return;

	PCEGrabberRenderOverride* renderOverrideInstance = (PCEGrabberRenderOverride*)
		renderer->findRenderOverride(PCEGrabberRenderOverride::kPCEGrabberPreviewerName);
	if (!renderOverrideInstance)
		return;

	PCEDeviceCacheImp* pCache = reinterpret_cast<PCEDeviceCacheImp*>(pVoid);
	if (!pCache)
		return;
	
	if( renderOverrideInstance->updatePreviewerTexture(pCache) )
		renderOverrideInstance->refreshView();

	//MStatus status;
	//MPx3dModelView *user3dModelView = PCEGrabberPreView::getModelView(kPCEGrabberPreviewerTypeName, &status);
	//if (NULL != user3dModelView) {
	//	
	//	//	This is now safe to do, since the above test passed.
	//	//
	//	PCEGrabberPreView *dView = (PCEGrabberPreView *)user3dModelView;
	//	if(dView)
	//	{

	//		M3dView view;
	//		status = dView->getAsM3dView(view);
	//		if (status == MStatus::kSuccess && (PCEGrabberRenderOverride::kPCEGrabberPreviewerName == view.renderOverrideName()))
	//		{
	//			view.scheduleRefresh();
	//		}
	//	}
	//}
	//else
	//{
	//	MGlobal::displayError("NULL == user3dModelView!");
	//}

	/*M3dView view;
	MStatus status = M3dView::getM3dViewFromModelEditor(kPCEGrabberPreViewCmdName, view);
	if (status == MStatus::kSuccess && (PCEGrabberRenderOverride::kPCEGrabberPreviewerName == view.renderOverrideName()))
	{
		view.scheduleRefresh();
	}*/
}
