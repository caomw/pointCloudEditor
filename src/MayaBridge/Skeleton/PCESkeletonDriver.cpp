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

MTypeId     PCESkeletonDriver::id( 0x08102B );

// Attributes
//
MObject		PCESkeletonDriver::aInputData;

MObject		PCESkeletonDriver::aHeadTranslate;
MObject		PCESkeletonDriver::aHeadTranslateX;
MObject		PCESkeletonDriver::aHeadTranslateY;
MObject		PCESkeletonDriver::aHeadTranslateZ;
MObject		PCESkeletonDriver::aHeadValid;
MObject		PCESkeletonDriver::aNeckTranslate;
MObject		PCESkeletonDriver::aNeckTranslateX;
MObject		PCESkeletonDriver::aNeckTranslateY;
MObject		PCESkeletonDriver::aNeckTranslateZ;
MObject		PCESkeletonDriver::aNeckValid;

MObject		PCESkeletonDriver::aLeftShoulderTranslate;
MObject		PCESkeletonDriver::aLeftShoulderTranslateX;
MObject		PCESkeletonDriver::aLeftShoulderTranslateY;
MObject		PCESkeletonDriver::aLeftShoulderTranslateZ;
MObject		PCESkeletonDriver::aLeftShoulderValid;
MObject		PCESkeletonDriver::aRightShoulderTranslate;
MObject		PCESkeletonDriver::aRightShoulderTranslateX;
MObject		PCESkeletonDriver::aRightShoulderTranslateY;
MObject		PCESkeletonDriver::aRightShoulderTranslateZ;
MObject		PCESkeletonDriver::aRightShoulderValid;
MObject		PCESkeletonDriver::aLeftElbowTranslate;
MObject		PCESkeletonDriver::aLeftElbowTranslateX;
MObject		PCESkeletonDriver::aLeftElbowTranslateY;
MObject		PCESkeletonDriver::aLeftElbowTranslateZ;
MObject		PCESkeletonDriver::aLeftElbowValid;
MObject		PCESkeletonDriver::aRightElbowTranslate;
MObject		PCESkeletonDriver::aRightElbowTranslateX;
MObject		PCESkeletonDriver::aRightElbowTranslateY;
MObject		PCESkeletonDriver::aRightElbowTranslateZ;
MObject		PCESkeletonDriver::aRightElbowValid;
MObject		PCESkeletonDriver::aLeftHandTranslate;
MObject		PCESkeletonDriver::aLeftHandTranslateX;
MObject		PCESkeletonDriver::aLeftHandTranslateY;
MObject		PCESkeletonDriver::aLeftHandTranslateZ;
MObject		PCESkeletonDriver::aLeftHandValid;
MObject		PCESkeletonDriver::aRightHandTranslate;
MObject		PCESkeletonDriver::aRightHandTranslateX;
MObject		PCESkeletonDriver::aRightHandTranslateY;
MObject		PCESkeletonDriver::aRightHandTranslateZ;
MObject		PCESkeletonDriver::aRightHandValid;

MObject		PCESkeletonDriver::aTorsoTranslate;
MObject		PCESkeletonDriver::aTorsoTranslateX;
MObject		PCESkeletonDriver::aTorsoTranslateY;
MObject		PCESkeletonDriver::aTorsoTranslateZ;
MObject		PCESkeletonDriver::aTorsoValid;

MObject		PCESkeletonDriver::aLeftHipTranslate;
MObject		PCESkeletonDriver::aLeftHipTranslateX;
MObject		PCESkeletonDriver::aLeftHipTranslateY;
MObject		PCESkeletonDriver::aLeftHipTranslateZ;
MObject		PCESkeletonDriver::aLeftHipValid;
MObject		PCESkeletonDriver::aRightHipTranslate;
MObject		PCESkeletonDriver::aRightHipTranslateX;
MObject		PCESkeletonDriver::aRightHipTranslateY;
MObject		PCESkeletonDriver::aRightHipTranslateZ;
MObject		PCESkeletonDriver::aRightHipValid;
MObject		PCESkeletonDriver::aLeftKneeTranslate;
MObject		PCESkeletonDriver::aLeftKneeTranslateX;
MObject		PCESkeletonDriver::aLeftKneeTranslateY;
MObject		PCESkeletonDriver::aLeftKneeTranslateZ;
MObject		PCESkeletonDriver::aLeftKneeValid;
MObject		PCESkeletonDriver::aRightKneeTranslate;
MObject		PCESkeletonDriver::aRightKneeTranslateX;
MObject		PCESkeletonDriver::aRightKneeTranslateY;
MObject		PCESkeletonDriver::aRightKneeTranslateZ;
MObject		PCESkeletonDriver::aRightKneeValid;
MObject		PCESkeletonDriver::aLeftFootTranslate;
MObject		PCESkeletonDriver::aLeftFootTranslateX;
MObject		PCESkeletonDriver::aLeftFootTranslateY;
MObject		PCESkeletonDriver::aLeftFootTranslateZ;
MObject		PCESkeletonDriver::aLeftFootValid;
MObject		PCESkeletonDriver::aRightFootTranslate;
MObject		PCESkeletonDriver::aRightFootTranslateX;
MObject		PCESkeletonDriver::aRightFootTranslateY;
MObject		PCESkeletonDriver::aRightFootTranslateZ;
MObject		PCESkeletonDriver::aRightFootValid;

MObject		PCESkeletonDriver::aCenterMassTranslate;
MObject		PCESkeletonDriver::aCenterMassTranslateX;
MObject		PCESkeletonDriver::aCenterMassTranslateY;
MObject		PCESkeletonDriver::aCenterMassTranslateZ;
MObject		PCESkeletonDriver::aCenterMassValid;

PCESkeletonDriver::PCESkeletonDriver()
{
}
PCESkeletonDriver::~PCESkeletonDriver()
{
}

void PCESkeletonDriver::postConstructor()
{

}

void PCESkeletonDriver::setSkeleton(MDataBlock& datablock)
{
	// Center
	MDataHandle otHandle = datablock.outputValue( aCenterMassTranslate ); 
	otHandle.set( (double)m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_CENTER),
		(double)m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_CENTER),
		(double)m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_CENTER) );
	datablock.setClean(aCenterMassTranslate);

	otHandle = datablock.outputValue( aCenterMassValid ); 
	otHandle.set( !m_validSkeleton.isInferred(SKELETON_POSITION_HIP_CENTER) );
	datablock.setClean(aCenterMassValid);

	// Spine
	double retiaveX = 0.0;
	double retiaveY = 0.0;
	double retiaveZ = 0.0;
	otHandle = datablock.outputValue( aTorsoValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_SPINE))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_SPINE) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_SPINE) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SPINE) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_CENTER);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aTorsoValid);
	
	otHandle = datablock.outputValue( aTorsoTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aTorsoTranslate);

	// Neck
	otHandle = datablock.outputValue( aNeckValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_SHOULDER_CENTER))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SPINE);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SPINE);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_CENTER) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SPINE);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aNeckValid);

	otHandle = datablock.outputValue( aNeckTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aNeckTranslate);

	// Head
	otHandle = datablock.outputValue( aHeadValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_HEAD))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_HEAD) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_HEAD) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HEAD) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_CENTER);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aHeadValid);

	otHandle = datablock.outputValue( aHeadTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aHeadTranslate);

	// Left shoulder
	otHandle = datablock.outputValue( aLeftShoulderValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_SHOULDER_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_CENTER);
		otHandle.set( true );
	}
	else
	{
		retiaveX = retiaveY = retiaveZ = 0.0;
		otHandle.set( false );
	}
	datablock.setClean(aLeftShoulderValid);

	otHandle = datablock.outputValue( aLeftShoulderTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftShoulderTranslate);

	// Left elbow
	otHandle = datablock.outputValue( aLeftElbowValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_ELBOW_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_ELBOW_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_LEFT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_ELBOW_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_LEFT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_ELBOW_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_LEFT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aLeftElbowValid);

	otHandle = datablock.outputValue( aLeftElbowTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftElbowTranslate);

	// Left hand
	otHandle = datablock.outputValue( aLeftHandValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_HAND_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_HAND_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_ELBOW_LEFT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_HAND_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_ELBOW_LEFT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HAND_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_ELBOW_LEFT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aLeftHandValid);

	otHandle = datablock.outputValue( aLeftHandTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftHandTranslate);

	// Right shoulder
	otHandle = datablock.outputValue( aRightShoulderValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_SHOULDER_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_CENTER);
		otHandle.set( true );
	}
	else
	{
		retiaveX = retiaveY = retiaveZ = 0.0;
		otHandle.set( false );
	}
	datablock.setClean(aRightShoulderValid);

	otHandle = datablock.outputValue( aRightShoulderTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightShoulderTranslate);

	// Right elbow
	otHandle = datablock.outputValue( aRightElbowValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_ELBOW_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_ELBOW_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_SHOULDER_RIGHT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_ELBOW_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_SHOULDER_RIGHT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_ELBOW_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_SHOULDER_RIGHT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aRightElbowValid);

	otHandle = datablock.outputValue( aRightElbowTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightElbowTranslate);

	// Right hand
	otHandle = datablock.outputValue( aRightHandValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_HAND_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_HAND_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_ELBOW_RIGHT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_HAND_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_ELBOW_RIGHT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HAND_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_ELBOW_RIGHT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aRightHandValid);

	otHandle = datablock.outputValue( aRightHandTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightHandTranslate);

	// Left hip
	otHandle = datablock.outputValue( aLeftHipValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_HIP_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_CENTER);
		otHandle.set( true );
	}
	else
	{
		retiaveX = retiaveY = retiaveZ = 0.0;
		otHandle.set( false );
	}
	datablock.setClean(aLeftHipValid);

	otHandle = datablock.outputValue( aLeftHipTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftHipTranslate);

	// Left knee
	otHandle = datablock.outputValue( aLeftKneeValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_KNEE_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_KNEE_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_LEFT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_KNEE_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_LEFT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_KNEE_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_LEFT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aLeftKneeValid);

	otHandle = datablock.outputValue( aLeftKneeTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftKneeTranslate);

	// Left foot
	otHandle = datablock.outputValue( aLeftFootValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_FOOT_LEFT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_FOOT_LEFT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_KNEE_LEFT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_FOOT_LEFT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_KNEE_LEFT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_FOOT_LEFT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_KNEE_LEFT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aLeftFootValid);

	otHandle = datablock.outputValue( aLeftFootTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aLeftFootTranslate);

	// Right hip
	otHandle = datablock.outputValue( aRightHipValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_HIP_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_CENTER);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_CENTER);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_CENTER);
		otHandle.set( true );
	}
	else
	{
		retiaveX = retiaveY = retiaveZ = 0.0;
		otHandle.set( false );
	}
	datablock.setClean(aRightHipValid);

	otHandle = datablock.outputValue( aRightHipTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightHipTranslate);

	// Right knee
	otHandle = datablock.outputValue( aRightKneeValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_KNEE_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_KNEE_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_HIP_RIGHT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_KNEE_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_HIP_RIGHT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_KNEE_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_HIP_RIGHT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aRightKneeValid);

	otHandle = datablock.outputValue( aRightKneeTranslate ); 
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightKneeTranslate);

	// Right foot
	otHandle = datablock.outputValue( aRightFootValid );
	if(!m_validSkeleton.isInferred(SKELETON_POSITION_FOOT_RIGHT))
	{
		retiaveX = m_validSkeleton.getJointPositionX(SKELETON_POSITION_FOOT_RIGHT) - m_validSkeleton.getJointPositionX(SKELETON_POSITION_KNEE_RIGHT);
		retiaveY = m_validSkeleton.getJointPositionY(SKELETON_POSITION_FOOT_RIGHT) - m_validSkeleton.getJointPositionY(SKELETON_POSITION_KNEE_RIGHT);
		retiaveZ = m_validSkeleton.getJointPositionZ(SKELETON_POSITION_FOOT_RIGHT) - m_validSkeleton.getJointPositionZ(SKELETON_POSITION_KNEE_RIGHT);
		otHandle.set( true );
	}
	else
		otHandle.set( false );
	datablock.setClean(aRightFootValid);

	otHandle = datablock.outputValue( aRightFootTranslate );
	otHandle.set( retiaveX, retiaveY, retiaveZ );
	datablock.setClean(aRightFootTranslate);
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
		if (!returnStatus) {
			returnStatus.perror("Error getting skeleton data handle\n");
			return returnStatus;
		}
		PCESkeletonData* pData = (PCESkeletonData*)skeletonHandle.asPluginData();
		if ( NULL == pData ) {
			cerr << "NULL skeleton data found\n";
			return returnStatus;
		}
		PCESkeletonPosition* pSkeletonData = pData->pSkeletonData;
		if ( NULL == pSkeletonData ) {
			cerr << "NULL skeleton data found\n";
			return returnStatus;
		}

		for(int i = 0; i < SKELETON_POSITION_COUNT; ++i)
		{
			//if( !pSkeletonData->isInferred((PCEJointType)i) )
			{
				m_validSkeleton.setJointData(
					(PCEJointType)i,
					pSkeletonData->getJointPositionX((PCEJointType)i),
					pSkeletonData->getJointPositionY((PCEJointType)i),
					pSkeletonData->getJointPositionZ((PCEJointType)i) );
			}
		}
		setSkeleton(datablock);
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
	if (!stat) { stat.perror("create inputData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

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
	aHeadValid = nAttr.create( "headValid", "hv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create headValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aHeadValid );
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
	aNeckValid = nAttr.create( "neckValid", "nv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create neckValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aNeckValid );
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
	aLeftShoulderValid = nAttr.create( "leftShoulderValid", "lsv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftShoulderValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftShoulderValid );
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
	aRightShoulderValid = nAttr.create( "rightShoulderValid", "rsv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightShoulderValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightShoulderValid );
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
	aLeftElbowValid = nAttr.create( "leftElbowValid", "lev", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftElbowValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftElbowValid );
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
	aRightElbowValid = nAttr.create( "rightElbowValid", "rev", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightElbowValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightElbowValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// LeftHead
	aLeftHandTranslateX = nAttr.create( "leftHandTranslateX", "lhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandTranslateY = nAttr.create( "leftHandTranslateY", "lhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandTranslateZ = nAttr.create( "leftHandTranslateZ", "lhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create leftHandTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aLeftHandTranslate = nAttr.create( "leftHandTranslate", "lht", aLeftHandTranslateX, aLeftHandTranslateY, aLeftHandTranslateZ, &stat );
	if (!stat) { stat.perror("create leftHandTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHandTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aLeftHandValid = nAttr.create( "leftHandValid", "lhv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftHandValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHandValid );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// RightHead
	aRightHandTranslateX = nAttr.create( "rightHandTranslateX", "rhtX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslateY = nAttr.create( "rightHandTranslateY", "rhtY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslateZ = nAttr.create( "rightHandTranslateZ", "rhtZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rightHandTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRightHandTranslate = nAttr.create( "rightHandTranslate", "rht", aRightHandTranslateX, aRightHandTranslateY, aRightHandTranslateZ, &stat );
	if (!stat) { stat.perror("create rightHandTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}
	aRightHandValid = nAttr.create( "rightHandValid", "rhv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightHandValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHandValid );
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
	aTorsoValid = nAttr.create( "torsoValid", "tv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create torsoValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aTorsoValid );
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
	aLeftHipValid = nAttr.create( "leftHipValid", "lhpv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftHipValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftHipValid );
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
	aRightHipValid = nAttr.create( "rightHipValid", "rhpv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightHipValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightHipValid );
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
	aLeftKneeValid = nAttr.create( "leftKneeValid", "lkv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftKneeValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftKneeValid );
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
	aRightKneeValid = nAttr.create( "rightKneeValid", "rkv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightKneeValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightKneeValid );
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
	aLeftFootValid = nAttr.create( "leftFootValid", "lfv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create leftFootValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLeftFootValid );
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
	aRightFootValid = nAttr.create( "rightFootValid", "rfv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create rightFootValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRightFootValid );
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
	aCenterMassValid = nAttr.create( "centerMassValid", "cmv", MFnNumericData::kBoolean, false, &stat );
	if (!stat) { stat.perror("create centerMassValid attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCenterMassValid );
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
	stat = attributeAffects( aInputData, aCenterMassValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aTorsoTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTorsoTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTorsoTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTorsoTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTorsoValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aNeckTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aNeckTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aNeckTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aNeckTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aNeckValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aHeadTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aHeadTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aHeadTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aHeadTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aHeadValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftShoulderTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftShoulderTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftShoulderTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftShoulderTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftShoulderValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightShoulderTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightShoulderTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightShoulderTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightShoulderTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightShoulderValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftElbowTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftElbowTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftElbowTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftElbowTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftElbowValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightElbowTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightElbowTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightElbowTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightElbowTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightElbowValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftHandTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHandTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHandTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHandTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHandValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightHandTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHandTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHandTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHandTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHandValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftHipTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHipTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHipTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHipTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftHipValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightHipTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHipTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHipTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHipTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightHipValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftKneeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftKneeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftKneeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftKneeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftKneeValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightKneeTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightKneeTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightKneeTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightKneeTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightKneeValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aLeftFootTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftFootTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftFootTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftFootTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLeftFootValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	stat = attributeAffects( aInputData, aRightFootTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightFootTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightFootTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightFootTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRightFootValid );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;

}

