//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#include "pointCloudTextureShader.h"
#include "../PointCloudShape/PCEPointCloudData.h"

#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFloatVector.h>
#include <maya/M3dView.h>

const MTypeId pointCloudTextureShader::m_TypeId(0x00081057);
const MString pointCloudTextureShader::m_TypeName("pointCloudTextureShader");
const MString pointCloudTextureShader::m_RegistrantId("pointCloudTextureShaderRegistrantId");
const MString pointCloudTextureShader::m_drawDbClassification("drawdb/shader/surface/pointCloudTextureShader");

// Attributes
MObject     pointCloudTextureShader::inputCloudData;
MObject		pointCloudTextureShader::aOutColor;

pointCloudTextureShader::pointCloudTextureShader()
{
}

pointCloudTextureShader::~pointCloudTextureShader()
{
}

MStatus pointCloudTextureShader::compute( const MPlug& plug, MDataBlock& block)
{
	//TRACE_API_CALLS("vp2BlinnShader::compute");
	if (plug == outColor)
	{
		// Set a dummy result when outColor is queried
		MStatus status;
		MDataHandle outColorHandle = block.outputValue(outColor, &status);
		CHECK_MSTATUS(status);
		MFloatVector& outColorData = outColorHandle.asFloatVector();
		outColorData = MFloatVector(1.0f, 0.0f, 0.0f);
		outColorHandle.setClean();

		return MS::kSuccess;
	}
	return MS::kUnknownParameter;
}

MStatus pointCloudTextureShader::initialize()
{
	MFnTypedAttribute	typedAttr;
	MFnNumericAttribute nAttr;
	MStatus				stat;

	// Input attributes
	inputCloudData = typedAttr.create( "inputCloudData", "icd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputCloudData attribute"); return stat;}
	typedAttr.setHidden( true );
	typedAttr.setStorable(false);
	stat = addAttribute( inputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	attributeAffects (inputCloudData,			outColor);

	return MS::kSuccess;
}

void* pointCloudTextureShader::creator()
{
	return new pointCloudTextureShader();
}

MStatus	pointCloudTextureShader::render( MGeometryList& iterator)
{
	MStatus result = MStatus::kFailure;

	// for now we render nothing in OpenGL in default viewport
	return result;
}