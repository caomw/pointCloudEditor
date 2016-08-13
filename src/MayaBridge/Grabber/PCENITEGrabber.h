#ifndef _PCENITEGrabber
#define _PCENITEGrabber
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCENITEGrabber.h
//
// Dependency Graph Node:  PCENITEGrabber
//
// Author: Tingzhu Zhou
//

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>

#include <XnCppWrapper.h>

class PCENITEGrabber : public MPxNode
{
public:
	PCENITEGrabber();
	virtual				~PCENITEGrabber(); 
	// Override functions
	virtual void		postConstructor();
	virtual MStatus		compute( const MPlug& plug, MDataBlock& datablock );

	static  void*		creator();
	static  MStatus		initialize();

public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject		aTime;
	static  MObject		aDeviceOn;
	static  MObject		aRecordMode;
	static  MObject		aSkipFrameNum;
	static  MObject		aSave;
	static  MObject		aLoad;
	static  MObject     aOutputCloudData;


	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

private:
	void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2);

private:
	static void FindPlayer();
	static void LostPlayer();
	static XnBool AssignPlayer(XnUserID user);

	static void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie);
	static void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie);
	static void XN_CALLBACK_TYPE UserExit(xn::UserGenerator& generator, XnUserID user, void* pCookie);
	static void XN_CALLBACK_TYPE UserReEnter(xn::UserGenerator& generator, XnUserID user, void* pCookie);
	static void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt);
	static void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt);
	static void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt);
	
	xn::Context g_Context;
	xn::ScriptNode g_ScriptNode;
	static xn::UserGenerator g_UserGenerator;
	static xn::DepthGenerator g_DepthGenerator;

	static XnUserID g_nPlayer;
	static XnBool g_bCalibrated;
};

#endif
