#include "PCENITEGrabber.h"
#include "../PointCloudShape/PCEPointCloudData.h"

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>

MTypeId     PCENITEGrabber::id( 0x80389 );

// Attributes
// 
MObject     PCENITEGrabber::aTime;
MObject		PCENITEGrabber::aDeviceOn;
MObject		PCENITEGrabber::aRecordMode;
MObject		PCENITEGrabber::aSkipFrameNum;
MObject		PCENITEGrabber::aSave;
MObject		PCENITEGrabber::aLoad;
MObject     PCENITEGrabber::aOutputCloudData;

xn::UserGenerator	PCENITEGrabber::g_UserGenerator;
xn::DepthGenerator	PCENITEGrabber::g_DepthGenerator;

XnUserID			PCENITEGrabber::g_nPlayer = 0;
XnBool				PCENITEGrabber::g_bCalibrated = false;

#define SAMPLE_XML_PATH "F:\\SourceCode\\Tingzhu_maya\\build\\myPlugins\\pointCloudEditor\\pointCloudEditor\\Sample-User.xml"

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
{																\
	printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	return;													\
}

#define CHECK_ERRORS(rc, errors, what)		\
	if (rc == XN_STATUS_NO_NODE_PRESENT)	\
{										\
	XnChar strError[1024];				\
	errors.ToString(strError, 1024);	\
	printf("%s\n", strError);			\
	return;						\
}

XnBool PCENITEGrabber::AssignPlayer(XnUserID user)
{
	if (g_nPlayer != 0)
		return FALSE;

	XnPoint3D com;
	g_UserGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return FALSE;

	printf("Matching for existing calibration\n");
	g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	g_UserGenerator.GetSkeletonCap().StartTracking(user);
	g_nPlayer = user;
	return TRUE;

}

void XN_CALLBACK_TYPE PCENITEGrabber::NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	if (!g_bCalibrated) // check on player0 is enough
	{
		printf("Look for pose\n");
		if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
		}
		return;
	}

	AssignPlayer(user);
	// 	if (g_nPlayer == 0)
	// 	{
	// 		printf("Assigned user\n");
	// 		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	// 		g_UserGenerator.GetSkeletonCap().StartTracking(user);
	// 		g_nPlayer = user;
	// 	}
}
void PCENITEGrabber::FindPlayer()
{
	if (g_nPlayer != 0)
	{
		return;
	}
	XnUserID aUsers[20];
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}
void PCENITEGrabber::LostPlayer()
{
	g_nPlayer = 0;
	FindPlayer();

}
void XN_CALLBACK_TYPE PCENITEGrabber::LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("Lost user %d\n", user);
	if (g_nPlayer == user)
	{
		LostPlayer();
	}
}
void XN_CALLBACK_TYPE PCENITEGrabber::UserExit(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d exit\n", user);
}

void XN_CALLBACK_TYPE PCENITEGrabber::UserReEnter(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	printf("User %d reenter\n", user);
}
void XN_CALLBACK_TYPE PCENITEGrabber::PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt)
{
	printf("Found pose \"%s\" for user %d\n", strPose, user);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE PCENITEGrabber::CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt)
{
	printf("Calibration started\n");
}

void XN_CALLBACK_TYPE PCENITEGrabber::CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt)
{
	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!g_bCalibrated)
		{
			g_UserGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			g_nPlayer = user;
			g_UserGenerator.GetSkeletonCap().StartTracking(user);
			g_bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
	else if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
	{
		printf("Manual abort occurred, stop attempting to calibrate!");
	}
	else if (skeleton.NeedPoseForCalibration())
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
	}
	else
	{
		skeleton.RequestCalibration(user, TRUE);
	}
}

PCENITEGrabber::PCENITEGrabber()
{
}
PCENITEGrabber::~PCENITEGrabber()
{
	g_Context.Release();
}

void PCENITEGrabber::postConstructor()
{
	g_nPlayer = 0;
	g_bCalibrated = FALSE;

	XnStatus rc = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_ScriptNode, &errors);
	CHECK_ERRORS(rc, errors, "InitFromXmlFile");
	CHECK_RC(rc, "InitFromXml");

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	CHECK_RC(rc, "Find user generator");

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	{
		printf("User generator doesn't support either skeleton or pose detection.\n");
		return;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");

	XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	g_UserGenerator.RegisterUserCallbacks(PCENITEGrabber::NewUser, PCENITEGrabber::LostUser, NULL, hUserCBs);
	g_UserGenerator.RegisterToUserExit(UserExit, NULL, hUserCBs);
	g_UserGenerator.RegisterToUserReEnter(UserReEnter, NULL, hUserCBs);
	rc = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(CalibrationStarted, NULL, hCalibrationStartCB);
	CHECK_RC(rc, "Register to calbiration start");
	rc = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, NULL, hCalibrationCompleteCB);
	CHECK_RC(rc, "Register to calibration complete");
	rc = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, NULL, hPoseCBs);
	CHECK_RC(rc, "Register to pose detected");
}

void PCENITEGrabber::DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsCalibrated(player))
	{
		printf("not calibrated!\n");
		return;
	}
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;
}

MStatus PCENITEGrabber::compute( const MPlug& plug, MDataBlock& datablock )
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
		bool isDeviceOn = false;
		MDataHandle inputHandle = datablock.inputValue( aDeviceOn, &returnStatus );
		if(returnStatus == MS::kSuccess)
			isDeviceOn = inputHandle.asBool();

		if(isDeviceOn)
		{
			// Read next available data
			g_Context.WaitOneUpdateAll(g_DepthGenerator);

			xn::SceneMetaData sceneMD;
			xn::DepthMetaData depthMD;
			g_DepthGenerator.GetMetaData(depthMD);
			g_UserGenerator.GetUserPixels(0, sceneMD);

			// Draw the players, label index means color index
			const XnLabel* pLabels = sceneMD.Data();
			for (int nY=0; nY< 480; nY++)
			{
				for (int nX=0; nX < 640; nX++)
				{
					XnLabel label = *pLabels;
					if(label > 0)
						label = *pLabels;
				}
			}

			// Draw string of player
			char strLabel[20] = "";
			XnUserID aUsers[15];
			XnUInt16 nUsers = 15;
			g_UserGenerator.GetUsers(aUsers, nUsers);
			for (int i = 0; i < nUsers; ++i)
			{
				XnPoint3D com;
				g_UserGenerator.GetCoM(aUsers[i], com);
				
			}
			// Draw skeleton of user
			if (g_nPlayer != 0)
			{
				DrawLimb(g_nPlayer, XN_SKEL_HEAD, XN_SKEL_NECK);
			}
		}
		else
		{
			// Stop
			g_Context.StopGeneratingAll();
		}
	}
	return MS::kSuccess;
}

void* PCENITEGrabber::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new PCENITEGrabber();
}

MStatus PCENITEGrabber::initialize()
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

	aRecordMode = enumAttr.create( "recordMode", "rm", 0, &stat );
	if (!stat) { stat.perror("create shapeType attribute"); return stat;}
	stat = enumAttr.addField( "record", 0 );
	if (!stat) { stat.perror("add enum type record"); return stat;}
	stat = enumAttr.addField( "immediate", 1 );
	if (!stat) { stat.perror("add enum type immediate"); return stat;}
	CHECK_MSTATUS( enumAttr.setHidden( false ) );
	CHECK_MSTATUS( enumAttr.setKeyable( true ) );
	stat = addAttribute( aRecordMode );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aSkipFrameNum = nAttr.create( "skipFrameNum", "sfn", MFnNumericData::kInt, 10 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	nAttr.setKeyable(true);
	// Attribute is keyable and will show up in the channel box
	stat = addAttribute( aSkipFrameNum );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// ----------------------- OUTPUTS -------------------------
	aOutputCloudData = typedAttr.create( "outputCloudData", "ocd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputCloudData attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable(false);
	stat = addAttribute( aOutputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aTime, aOutputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aDeviceOn, aOutputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aRecordMode, aOutputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;

}
