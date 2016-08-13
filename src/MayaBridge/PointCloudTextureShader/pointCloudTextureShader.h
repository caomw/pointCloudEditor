//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#ifndef POINTCLOUDTEXTURENODE_H
#define POINTCLOUDTEXTURENODE_H

#include <maya/MPxHardwareShader.h>

class pointCloudTextureShader : public MPxHardwareShader
{
public:
	pointCloudTextureShader();
	~pointCloudTextureShader();

	static void* creator();
	static MStatus initialize();
	virtual MStatus compute( const MPlug&, MDataBlock& );
	virtual MStatus	render(MGeometryList& iterator);

	static  MObject     inputCloudData;
	static  MObject		aOutColor;

	static const MTypeId m_TypeId;
	static const MString m_TypeName;
	static const MString m_RegistrantId;
	static const MString m_drawDbClassification;
};


#endif