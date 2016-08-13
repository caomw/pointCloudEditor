//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#include "PCEPointCloudTextureNode.h"
#include "api_macros.h"


#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFloatVector.h>
#include <maya/M3dView.h>

const MTypeId PCEPointCloudTextureNode::m_TypeId(0x00081057);
const MString PCEPointCloudTextureNode::m_TypeName("pointCloudTextureNode");
const MString PCEPointCloudTextureNode::m_drawDbClassification("drawdb/shader/texture/2d/checkerTexture");

// Attributes
MObject     PCEPointCloudTextureNode::inputCloudData;
MObject		PCEPointCloudTextureNode::aUVCoord;
MObject		PCEPointCloudTextureNode::aOutColor;

PCEPointCloudTextureNode::PCEPointCloudTextureNode()
	: geomPtr(NULL)
{
}

PCEPointCloudTextureNode::~PCEPointCloudTextureNode()
{
	geomPtr = NULL;
}

MStatus PCEPointCloudTextureNode::compute( const MPlug& plug, MDataBlock& block)
{
	// outColor or individial R, G, B channel
	if((plug != aOutColor) && (plug.parent() != aOutColor))
		return MS::kUnknownParameter;

	MStatus stat = MS::kSuccess;
	MDataHandle inputHandle = block.inputValue( inputCloudData, &stat );
	MCHECKERROR( stat, "computeInputSurface error getting inputSurface")

	PCEPointCloudData* surf = (PCEPointCloudData*) inputHandle.asPluginData();
	if ( NULL == surf ) {
		cerr << "NULL inputSurface data found\n";
		return stat;
	}

	geomPtr = surf->pCloudData.get();

	// Set a dummy result when outColor is queried
	MDataHandle outColorHandle = block.outputValue( aOutColor );
	MFloatVector& outColor = outColorHandle.asFloatVector();
	outColor = MFloatVector(1.0f, 0.0f, 0.0f);
	outColorHandle.setClean();

	return MS::kSuccess;
}

MStatus PCEPointCloudTextureNode::initialize()
{
	MFnTypedAttribute	typedAttr;
	MFnNumericAttribute nAttr;
	MStatus				stat;

	// Input attributes
	inputCloudData = typedAttr.create( "inputCloudData", "icd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputCloudData attribute"); return stat;}
	typedAttr.setKeyable(true);
	typedAttr.setStorable(true);
	typedAttr.setReadable(false);
	typedAttr.setWritable(true);
	stat = addAttribute( inputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	MObject child1 = nAttr.create( "uCoord", "u", MFnNumericData::kFloat);
	MObject child2 = nAttr.create( "vCoord", "v", MFnNumericData::kFloat);
	aUVCoord = nAttr.create( "uvCoord","uv", child1, child2);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setReadable(false);
	nAttr.setWritable(true);
	nAttr.setHidden(true);
	stat = addAttribute( aUVCoord );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Output attributes
	aOutColor = nAttr.createColor("outColor", "oc");
	nAttr.setKeyable(false);
	nAttr.setStorable(false);
	nAttr.setReadable(true);
	nAttr.setWritable(false);
	stat = addAttribute( aOutColor );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	attributeAffects (inputCloudData,			aOutColor);

	return MS::kSuccess;
}

void* PCEPointCloudTextureNode::creator()
{
	return new PCEPointCloudTextureNode();
}

void PCEPointCloudTextureNode::postConstructor( )
{
	setMPSafe(true);
}