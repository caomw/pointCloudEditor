#ifndef _PCEPointCloudFilter
#define _PCEPointCloudFilter
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudFilter.h
//
// Dependency Graph Node:  PCEPointCloudFilter
//
// Author: Tingzhu Zhou
//

#include <maya/MPxNode.h>
#include <maya/MTypeId.h> 

class PCEPointCloud;
class PCEPointCloudFilter : public MPxNode
{
public:
	PCEPointCloudFilter();
	virtual				~PCEPointCloudFilter(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& datablock );

	static  void*		creator();
	static  MStatus		initialize();

	MStatus				computeFilteredPointCloud( const MPlug& plug,
													MDataBlock& datablock,
													PCEPointCloud* geomPtr);

public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static  MObject		aMethod;
	static  MObject		aLeafSize;
	static  MObject		aField;
	static  MObject		aMin;
	static  MObject		aMax;
	static  MObject     inputCloudData;
	static  MObject     outputCloudData;


	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;
};

#endif