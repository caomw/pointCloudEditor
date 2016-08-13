//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#ifndef POINTCLOUDTEXTURENODE_H
#define POINTCLOUDTEXTURENODE_H

#include <maya/MPxNode.h>

#include "../PointCloudShape/PCEPointCloudData.h"

class PCEPointCloudTextureNode : public MPxNode
{
public:
	PCEPointCloudTextureNode();
	~PCEPointCloudTextureNode();

	static void* creator();
	static MStatus initialize();
	virtual MStatus compute( const MPlug&, MDataBlock& );
	virtual void    postConstructor();

	PCEPointCloud* getTextureData() { return geomPtr; };

	static const MTypeId m_TypeId;
	static const MString m_TypeName;
	static const MString m_drawDbClassification;

private:
	// Input attributes
	static  MObject     inputCloudData;
	static	MObject		aUVCoord;
	// Output attributes
	static  MObject		aOutColor;

	PCEPointCloud* geomPtr;
};


#endif