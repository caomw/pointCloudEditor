//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletonDriver.cpp
//
// Dependency Graph Node: PCESkeletonDriver
//
// Author: Tingzhu Zhou
//

#include "PCESkeletonDriver.h"
#include "PCESkeletonData.h"

#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>

MTypeId     PCESkeletonDriver::id( 0x80629 );

// Attributes
// 
MObject		PCESkeletonDriver::aHeadTranslate;
MObject		PCESkeletonDriver::aHeadTranslateX;
MObject		PCESkeletonDriver::aHeadTranslateY;
MObject		PCESkeletonDriver::aHeadTranslateZ;
MObject		PCESkeletonDriver::aNeckTranslate;
MObject		PCESkeletonDriver::aNeckTranslateX;
MObject		PCESkeletonDriver::aNeckTranslateY;
MObject		PCESkeletonDriver::aNeckTranslateZ;

MObject		PCESkeletonDriver::aLeftShoulderTranslate;
MObject		PCESkeletonDriver::aLeftShoulderTranslateX;
MObject		PCESkeletonDriver::aLeftShoulderTranslateY;
MObject		PCESkeletonDriver::aLeftShoulderTranslateZ;
MObject		PCESkeletonDriver::aRightShoulderTranslate;
MObject		PCESkeletonDriver::aRightShoulderTranslateX;
MObject		PCESkeletonDriver::aRightShoulderTranslateY;
MObject		PCESkeletonDriver::aRightShoulderTranslateZ;
MObject		PCESkeletonDriver::aLeftElbowTranslate;
MObject		PCESkeletonDriver::aLeftElbowTranslateX;
MObject		PCESkeletonDriver::aLeftElbowTranslateY;
MObject		PCESkeletonDriver::aLeftElbowTranslateZ;
MObject		PCESkeletonDriver::aRightElbowTranslate;
MObject		PCESkeletonDriver::aRightElbowTranslateX;
MObject		PCESkeletonDriver::aRightElbowTranslateY;
MObject		PCESkeletonDriver::aRightElbowTranslateZ;
MObject		PCESkeletonDriver::aLeftHeadTranslate;
MObject		PCESkeletonDriver::aLeftHeadTranslateX;
MObject		PCESkeletonDriver::aLeftHeadTranslateY;
MObject		PCESkeletonDriver::aLeftHeadTranslateZ;
MObject		PCESkeletonDriver::aRightHeadTranslate;
MObject		PCESkeletonDriver::aRightHeadTranslateX;
MObject		PCESkeletonDriver::aRightHeadTranslateY;
MObject		PCESkeletonDriver::aRightHeadTranslateZ;

MObject		PCESkeletonDriver::aTorsoTranslate;
MObject		PCESkeletonDriver::aTorsoTranslateX;
MObject		PCESkeletonDriver::aTorsoTranslateY;
MObject		PCESkeletonDriver::aTorsoTranslateZ;

MObject		PCESkeletonDriver::aLeftHipTranslate;
MObject		PCESkeletonDriver::aLeftHipTranslateX;
MObject		PCESkeletonDriver::aLeftHipTranslateY;
MObject		PCESkeletonDriver::aLeftHipTranslateZ;
MObject		PCESkeletonDriver::aRightHipTranslate;
MObject		PCESkeletonDriver::aRightHipTranslateX;
MObject		PCESkeletonDriver::aRightHipTranslateY;
MObject		PCESkeletonDriver::aRightHipTranslateZ;
MObject		PCESkeletonDriver::aLeftKneeTranslate;
MObject		PCESkeletonDriver::aLeftKneeTranslateX;
MObject		PCESkeletonDriver::aLeftKneeTranslateY;
MObject		PCESkeletonDriver::aLeftKneeTranslateZ;
MObject		PCESkeletonDriver::aRightKneeTranslate;
MObject		PCESkeletonDriver::aRightKneeTranslateX;
MObject		PCESkeletonDriver::aRightKneeTranslateY;
MObject		PCESkeletonDriver::aRightKneeTranslateZ;
MObject		PCESkeletonDriver::aLeftFootTranslate;
MObject		PCESkeletonDriver::aLeftFootTranslateX;
MObject		PCESkeletonDriver::aLeftFootTranslateY;
MObject		PCESkeletonDriver::aLeftFootTranslateZ;
MObject		PCESkeletonDriver::aRightFootTranslate;
MObject		PCESkeletonDriver::aRightFootTranslateX;
MObject		PCESkeletonDriver::aRightFootTranslateY;
MObject		PCESkeletonDriver::aRightFootTranslateZ;

MObject		PCESkeletonDriver::aCenterMassTranslate;
MObject		PCESkeletonDriver::aCenterMassTranslateX;
MObject		PCESkeletonDriver::aCenterMassTranslateY;
MObject		PCESkeletonDriver::aCenterMassTranslateZ;

PCESkeletonDriver::PCESkeletonDriver()
{
}
PCESkeletonDriver::~PCESkeletonDriver()
{
	
}

MStatus PCESkeletonDriver::compute( const MPlug& plug, MDataBlock& datablock )
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
	// TODO: if...
	{
		/* Get skeleton data */
		MDataHandle skeletonHandle = datablock.inputValue( aInputData, &returnStatus ); 
		MCHECKERROR(returnStatus, "Error getting skeleton data handle\n");
		PCESkeletonPosition* pSkeletonData = (PCESkeletonPosition*)skeletonHandle.asPluginData();
		if ( NULL == pSkeletonData ) {
			cerr << "NULL skeleton data found\n";
			return returnStatus;
		}
		
		// Head
		MDataHandle otHandle = datablock.outputValue( aHeadTranslate ); 
		otHandle.set( pSkeletonData->getJointPoistionX(JOINT_HEAD),
			pSkeletonData->getJointPoistionY(JOINT_HEAD),
			pSkeletonData->getJointPoistionZ(JOINT_HEAD) );
		datablock.setClean(aHeadTranslate);

		// Neck
		otHandle = datablock.outputValue( aNeckTranslate ); 
		otHandle.set( pSkeletonData->getJointPoistionX(JOINT_NECK),
			pSkeletonData->getJointPoistionY(JOINT_NECK),
			pSkeletonData->getJointPoistionZ(JOINT_NECK) );
		datablock.setClean(aNeckTranslate);
	}
	/*else {
		return MS::kUnknownParameter;
	}*/

	return MS::kSuccess;
}


void* PCESkeletonDriver::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new PCESkeletonDriver();
}

MStatus PCESkeletonDriver::initialize()
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
	MStatus				stat;

	// ----------------------- INPUTS --------------------------
	aInputData = typedAttr.create( "inputData", "isd",
		PCESkeletonData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create inputCloudData attribute" )
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	CHECK_MSTATUS( addAttribute( aInputData ) );

	// ----------------------- OUTPUTS -------------------------
	// Head
	aHeadTranslateX = nAttr.create( "headTranslateX", "htX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslateY = nAttr.create( "headTranslateY", "htY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslateZ = nAttr.create( "headTranslateZ", "htZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create headTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aHeadTranslate = nAttr.create( "headTranslate", "ht", aHeadTranslateX, aHeadTranslateY, aHeadTranslateZ, &stat );
	if (!stat) { stat.perror("create headTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aHeadTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Neck
	aNeckTranslateX = nAttr.create( "neckTranslateX", "ntX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckTranslateY = nAttr.create( "neckTranslateY", "ntY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckTranslateZ = nAttr.create( "neckTranslateZ", "ntZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create neckTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aNeckTranslate = nAttr.create( "neckTranslate", "nt", aNeckTranslateX, aNeckTranslateY, aNeckTranslateZ, &stat );
	if (!stat) { stat.perror("create neckTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aNeckTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftShoulder
	aLeftShoulderTranslateX = nAttr.create( "leftShoulderTranslateX", "lstX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderTranslateY = nAttr.create( "leftShoulderTranslateY", "lstY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderTranslateZ = nAttr.create( "leftShoulderTranslateZ", "lstZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftShoulderTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftShoulderTranslate = nAttr.create( "leftShoulderTranslate", "lst", aLeftShoulderTranslateX, aLeftShoulderTranslateY, aLeftShoulderTranslateZ, &stat );
	if (!stat) { stat.perror("create leftShoulderTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftShoulderTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightShoulder
	aRightShoulderTranslateX = nAttr.create( "rightShoulderTranslateX", "rstX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderTranslateY = nAttr.create( "rightShoulderTranslateY", "rstY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderTranslateZ = nAttr.create( "rightShoulderTranslateZ", "rstZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightShoulderTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightShoulderTranslate = nAttr.create( "rightShoulderTranslate", "rst", aRightShoulderTranslateX, aRightShoulderTranslateY, aRightShoulderTranslateZ, &stat );
	if (!stat) { stat.perror("create rightShoulderTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightShoulderTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftElbow
	aLeftElbowTranslateX = nAttr.create( "leftElbowTranslateX", "letX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslateY = nAttr.create( "leftElbowTranslateY", "letY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslateZ = nAttr.create( "leftElbowTranslateZ", "letZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftElbowTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftElbowTranslate = nAttr.create( "leftElbowTranslate", "let", aLeftElbowTranslateX, aLeftElbowTranslateY, aLeftElbowTranslateZ, &stat );
	if (!stat) { stat.perror("create leftElbowTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftElbowTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightElbow
	aRightElbowTranslateX = nAttr.create( "rightElbowTranslateX", "retX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslateY = nAttr.create( "rightElbowTranslateY", "retY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslateZ = nAttr.create( "rightElbowTranslateZ", "retZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightElbowTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightElbowTranslate = nAttr.create( "rightElbowTranslate", "ret", aRightElbowTranslateX, aRightElbowTranslateY, aRightElbowTranslateZ, &stat );
	if (!stat) { stat.perror("create rightElbowTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightElbowTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftHead
	aLeftHeadTranslateX = nAttr.create( "leftHeadTranslateX", "lhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHeadTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHeadTranslateY = nAttr.create( "leftHeadTranslateY", "lhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHeadTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHeadTranslateZ = nAttr.create( "leftHeadTranslateZ", "lhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHeadTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHeadTranslate = nAttr.create( "leftHeadTranslate", "lht", aLeftHeadTranslateX, aLeftHeadTranslateY, aLeftHeadTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHeadTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHeadTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightHead
	aRightHeadTranslateX = nAttr.create( "rightHeadTranslateX", "rhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHeadTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHeadTranslateY = nAttr.create( "rightHeadTranslateY", "rhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHeadTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHeadTranslateZ = nAttr.create( "rightHeadTranslateZ", "rhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHeadTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHeadTranslate = nAttr.create( "rightHeadTranslate", "rht", aRightHeadTranslateX, aRightHeadTranslateY, aRightHeadTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHeadTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHeadTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Torso
	aTorsoTranslateX = nAttr.create( "torsoTranslateX", "ttX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoTranslateY = nAttr.create( "torsoTranslateY", "ttY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoTranslateZ = nAttr.create( "torsoTranslateZ", "ttZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create torsoTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTorsoTranslate = nAttr.create( "torsoTranslate", "tt", aTorsoTranslateX, aTorsoTranslateY, aTorsoTranslateZ, &stat );
	if (!stat) { stat.perror("create torsoTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aTorsoTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftHip
	aLeftHipTranslateX = nAttr.create( "leftHipTranslateX", "lhpX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipTranslateY = nAttr.create( "leftHipTranslateY", "lhpY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipTranslateZ = nAttr.create( "leftHipTranslateZ", "lhpZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHipTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHipTranslate = nAttr.create( "leftHipTranslate", "lhp", aLeftHipTranslateX, aLeftHipTranslateY, aLeftHipTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHipTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHipTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightHip
	aRightHipTranslateX = nAttr.create( "rightHipTranslateX", "rhpX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipTranslateY = nAttr.create( "rightHipTranslateY", "rhpY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipTranslateZ = nAttr.create( "rightHipTranslateZ", "rhpZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHipTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHipTranslate = nAttr.create( "rightHipTranslate", "rhp", aRightHipTranslateX, aRightHipTranslateY, aRightHipTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHipTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHipTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftKnee
	aLeftKneeTranslateX = nAttr.create( "leftKneeTranslateX", "lktX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeTranslateY = nAttr.create( "leftKneeTranslateY", "lktY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeTranslateZ = nAttr.create( "leftKneeTranslateZ", "lktZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftKneeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftKneeTranslate = nAttr.create( "leftKneeTranslate", "lkt", aLeftKneeTranslateX, aLeftKneeTranslateY, aLeftKneeTranslateZ, &stat );
	if (!stat) { stat.perror("create leftKneeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftKneeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightKnee
	aRightKneeTranslateX = nAttr.create( "rightKneeTranslateX", "rktX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeTranslateY = nAttr.create( "rightKneeTranslateY", "rktY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeTranslateZ = nAttr.create( "rightKneeTranslateZ", "rktZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightKneeTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightKneeTranslate = nAttr.create( "rightKneeTranslate", "rkt", aRightKneeTranslateX, aRightKneeTranslateY, aRightKneeTranslateZ, &stat );
	if (!stat) { stat.perror("create rightKneeTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightKneeTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftFoot
	aLeftFootTranslateX = nAttr.create( "leftFootTranslateX", "lftX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootTranslateY = nAttr.create( "leftFootTranslateY", "lftY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootTranslateZ = nAttr.create( "leftFootTranslateZ", "lftZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftFootTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftFootTranslate = nAttr.create( "leftFootTranslate", "lft", aLeftFootTranslateX, aLeftFootTranslateY, aLeftFootTranslateZ, &stat );
	if (!stat) { stat.perror("create leftFootTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftFootTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightFoot
	aRightFootTranslateX = nAttr.create( "rightFootTranslateX", "rftX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootTranslateY = nAttr.create( "rightFootTranslateY", "rftY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootTranslateZ = nAttr.create( "rightFootTranslateZ", "rftZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightFootTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightFootTranslate = nAttr.create( "rightFootTranslate", "rft", aRightFootTranslateX, aRightFootTranslateY, aRightFootTranslateZ, &stat );
	if (!stat) { stat.perror("create rightFootTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightFootTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// CenterMass
	aCenterMassTranslateX = nAttr.create( "centerMassTranslateX", "cmtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslateY = nAttr.create( "centerMassTranslateY", "cmtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslateZ = nAttr.create( "centerMassTranslateZ", "cmtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerMassTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterMassTranslate = nAttr.create( "centerMassTranslate", "cmt", aCenterMassTranslateX, aCenterMassTranslateY, aCenterMassTranslateZ, &stat );
	if (!stat) { stat.perror("create centerMassTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCenterMassTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aInputData, aCenterMassTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterMassTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterMassTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterMassTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;

}

