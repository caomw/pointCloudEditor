#ifndef _PCEPointCloudExtraction
#define _PCEPointCloudExtraction
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudExtraction.h
//
// Dependency Graph Node:  PCEPointCloudExtraction
//
// Author: Tingzhu Zhou
//

#include <maya/MPxNode.h>
#include <maya/MTypeId.h> 

class PCEPointCloud;
class PCEPointCloudExtraction : public MPxNode
{
public:
	PCEPointCloudExtraction();
	virtual				~PCEPointCloudExtraction(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& datablock );
	virtual MStatus		setDependentsDirty( const MPlug& plug, MPlugArray& plugArray);
	virtual MStatus		connectionMade( const MPlug& plug,
										const MPlug& otherPlug,
										bool asSrc );
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
	static  MObject		aClusterIndex;
	static  MObject		aIsRemovedIndices;
	static  MObject     aInputCloudData;
	static  MObject     aOutputCloudData;


	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

private:
	bool fNeedDirty;
};

#endif