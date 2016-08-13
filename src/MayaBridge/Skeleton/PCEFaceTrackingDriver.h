#ifndef _PCEFaceDriver
#define _PCEFaceDriver
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEFaceTrackingDriver.h
//
// Author: Tingzhu Zhou
//

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MCallbackIdArray.h>
#include <maya/MNodeMessage.h>

class PCEFaceTrackingDriver : public MPxNode
{
public:
						PCEFaceTrackingDriver();
	virtual				~PCEFaceTrackingDriver();
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

	static  MObject		aOutputMesh;
	static  MObject		aCenterTranslateX;
	static  MObject		aCenterTranslateY;
	static  MObject		aCenterTranslateZ;
	static  MObject		aCenterTranslate;

	static  MObject		aScaleX;
	static  MObject		aScaleY;
	static  MObject		aScaleZ;
	static  MObject		aScale;

	static  MObject		aRotateX;
	static  MObject		aRotateY;
	static  MObject		aRotateZ;
	static  MObject		aRotate;

	static  MObject		aTranslateX;
	static  MObject		aTranslateY;
	static  MObject		aTranslateZ;
	static  MObject		aTranslate;

	static  MObject		aUpperLipRaiser;
	static  MObject		aJawLowerer;
	static  MObject		aLipStretcher;
	static  MObject		aBrowLowerer;
	static  MObject		aLipCornerDepressor;
	static  MObject		aOuterBrowRaiser;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

private:
	MStatus				computeOutputMesh( MDataBlock& datablock, MObject& meshData );
	void				createEmptyMesh( MObject& out_empytMesh );

};

#endif
