#ifndef _PCESkeletonDriver
#define _PCESkeletonDriver
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletonDriver.h
//
// Dependency Graph Node:  PCESkeletonDriver
//
// Author: Tingzhu Zhou
//
#include "Shape/PCESkeletonPosition.h"

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

class PCESkeletonDriver : public MPxNode
{
public:
						PCESkeletonDriver();
	virtual				~PCESkeletonDriver();
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
	static  MObject		aInputData;

	static  MObject		aHeadTranslate;
	static  MObject		aHeadTranslateX;
	static  MObject		aHeadTranslateY;
	static  MObject		aHeadTranslateZ;
	static  MObject		aHeadValid;
	static  MObject		aNeckTranslate;
	static  MObject		aNeckTranslateX;
	static  MObject		aNeckTranslateY;
	static  MObject		aNeckTranslateZ;
	static  MObject		aNeckValid;

	static  MObject		aLeftShoulderTranslate;
	static  MObject		aLeftShoulderTranslateX;
	static  MObject		aLeftShoulderTranslateY;
	static  MObject		aLeftShoulderTranslateZ;
	static  MObject		aLeftShoulderValid;
	static  MObject		aRightShoulderTranslate;
	static  MObject		aRightShoulderTranslateX;
	static  MObject		aRightShoulderTranslateY;
	static  MObject		aRightShoulderTranslateZ;
	static  MObject		aRightShoulderValid;
	static  MObject		aLeftElbowTranslate;
	static  MObject		aLeftElbowTranslateX;
	static  MObject		aLeftElbowTranslateY;
	static  MObject		aLeftElbowTranslateZ;
	static  MObject		aLeftElbowValid;
	static  MObject		aRightElbowTranslate;
	static  MObject		aRightElbowTranslateX;
	static  MObject		aRightElbowTranslateY;
	static  MObject		aRightElbowTranslateZ;
	static  MObject		aRightElbowValid;
	static  MObject		aLeftHandTranslate;
	static  MObject		aLeftHandTranslateX;
	static  MObject		aLeftHandTranslateY;
	static  MObject		aLeftHandTranslateZ;
	static  MObject		aLeftHandValid;
	static  MObject		aRightHandTranslate;
	static  MObject		aRightHandTranslateX;
	static  MObject		aRightHandTranslateY;
	static  MObject		aRightHandTranslateZ;
	static  MObject		aRightHandValid;

	static  MObject		aTorsoTranslate;
	static  MObject		aTorsoTranslateX;
	static  MObject		aTorsoTranslateY;
	static  MObject		aTorsoTranslateZ;
	static  MObject		aTorsoValid;

	static  MObject		aLeftHipTranslate;
	static  MObject		aLeftHipTranslateX;
	static  MObject		aLeftHipTranslateY;
	static  MObject		aLeftHipTranslateZ;
	static  MObject		aLeftHipValid;
	static  MObject		aRightHipTranslate;
	static  MObject		aRightHipTranslateX;
	static  MObject		aRightHipTranslateY;
	static  MObject		aRightHipTranslateZ;
	static  MObject		aRightHipValid;
	static  MObject		aLeftKneeTranslate;
	static  MObject		aLeftKneeTranslateX;
	static  MObject		aLeftKneeTranslateY;
	static  MObject		aLeftKneeTranslateZ;
	static  MObject		aLeftKneeValid;
	static  MObject		aRightKneeTranslate;
	static  MObject		aRightKneeTranslateX;
	static  MObject		aRightKneeTranslateY;
	static  MObject		aRightKneeTranslateZ;
	static  MObject		aRightKneeValid;
	static  MObject		aLeftFootTranslate;
	static  MObject		aLeftFootTranslateX;
	static  MObject		aLeftFootTranslateY;
	static  MObject		aLeftFootTranslateZ;
	static  MObject		aLeftFootValid;
	static  MObject		aRightFootTranslate;
	static  MObject		aRightFootTranslateX;
	static  MObject		aRightFootTranslateY;
	static  MObject		aRightFootTranslateZ;
	static  MObject		aRightFootValid;

	static  MObject		aCenterMassTranslate;
	static  MObject		aCenterMassTranslateX;
	static  MObject		aCenterMassTranslateY;
	static  MObject		aCenterMassTranslateZ;
	static  MObject		aCenterMassValid;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

private:
	void				setSkeleton(MDataBlock& datablock);
private:
	PCESkeletonPosition m_validSkeleton;
};

#endif
