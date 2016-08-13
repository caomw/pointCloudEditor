//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEFaceDriver.cpp
//
// Author: Tingzhu Zhou
//

#include "PCEFaceTrackingDriver.h"

#include "Shape/PCEFaceTrackingResult.h"
#include "PCEFaceTrackingData.h"

#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatPointArray.h>

MTypeId     PCEFaceTrackingDriver::id( 0x08471B );

// Attributes
//
MObject		PCEFaceTrackingDriver::aInputData;
MObject		PCEFaceTrackingDriver::aOutputMesh;
MObject		PCEFaceTrackingDriver::aCenterTranslateX;
MObject		PCEFaceTrackingDriver::aCenterTranslateY;
MObject		PCEFaceTrackingDriver::aCenterTranslateZ;
MObject		PCEFaceTrackingDriver::aCenterTranslate;
MObject		PCEFaceTrackingDriver::aScaleX;
MObject		PCEFaceTrackingDriver::aScaleY;
MObject		PCEFaceTrackingDriver::aScaleZ;
MObject		PCEFaceTrackingDriver::aScale;
MObject		PCEFaceTrackingDriver::aRotateX;
MObject		PCEFaceTrackingDriver::aRotateY;
MObject		PCEFaceTrackingDriver::aRotateZ;
MObject		PCEFaceTrackingDriver::aRotate;
MObject		PCEFaceTrackingDriver::aTranslateX;
MObject		PCEFaceTrackingDriver::aTranslateY;
MObject		PCEFaceTrackingDriver::aTranslateZ;
MObject		PCEFaceTrackingDriver::aTranslate;
MObject		PCEFaceTrackingDriver::aUpperLipRaiser;
MObject		PCEFaceTrackingDriver::aJawLowerer;
MObject		PCEFaceTrackingDriver::aLipStretcher;
MObject		PCEFaceTrackingDriver::aBrowLowerer;
MObject		PCEFaceTrackingDriver::aLipCornerDepressor;
MObject		PCEFaceTrackingDriver::aOuterBrowRaiser;

PCEFaceTrackingDriver::PCEFaceTrackingDriver()
{
}
PCEFaceTrackingDriver::~PCEFaceTrackingDriver()
{
}

void PCEFaceTrackingDriver::postConstructor()
{

}


void PCEFaceTrackingDriver::createEmptyMesh( MObject& out_empytMesh )
{
	MStatus status;
	MFnMeshData meshData;
	out_empytMesh = meshData.create( &status );
	CHECK_MSTATUS( status );

	MFloatPointArray  	vertexArray;
	MIntArray  	        polygonCounts;
	MIntArray  	        polygonConnects;

	MFnMesh meshCreator;
	MObject newMeshObject = meshCreator.create( 
		0, // nb vertices
		0, // nb triangles 
		vertexArray, 
		polygonCounts, 
		polygonConnects, 
		out_empytMesh );
}

MStatus PCEFaceTrackingDriver::computeOutputMesh( 
	MDataBlock&			datablock,
	MObject&			meshData)
//
// Description
//
//     This function takes an input surface of type kMeshData and converts
//     the geometry into this nodes attributes.
//     Returns kFailure if nothing is connected.
//
{
	MStatus stat;

	/* Get face data */
	MDataHandle faceTrackingHandle = datablock.inputValue( aInputData, &stat );
	if (!stat) {
		stat.perror("Error getting skeleton data handle\n");
		return stat;
	}
	PCEFaceTrackingData* pData = (PCEFaceTrackingData*)faceTrackingHandle.asPluginData();
	if ( NULL == pData ) {
		cerr << "NULL face data found\n";
		return MS::kFailure;
	}
	PCEFaceTrackingResult* pFaceShape = pData->pFaceResult;
	if ( NULL == pFaceShape ) {
		cerr << "NULL face shape found\n";
		return MS::kFailure;
	}

	if( pFaceShape->isMeshEmpty() )
		return MS::kFailure;

	MFloatPointArray vertexArray;
	MIntArray polygonCounts;
	MIntArray polygonConnects;

	for (UINT vtId = 0; vtId < pFaceShape->sizeOfPts3D(); vtId++)
	{
		Vector3 pts = pFaceShape->getPts3D(vtId);
		vertexArray.append(pts.x, pts.y, pts.z);
	}

	for (UINT trId = 0; trId < pFaceShape->sizeOfTriangles(); trId++)
	{
		Triangle triangle = pFaceShape->getTriangle(trId);
		polygonCounts.append(3);
		polygonConnects.append(triangle.i);
		polygonConnects.append(triangle.j);
		polygonConnects.append(triangle.k);
	}

	MFnMesh meshFn;
	meshFn.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, meshData, &stat);

	return stat;
}

MStatus PCEFaceTrackingDriver::compute( const MPlug& plug, MDataBlock& datablock )
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
	if (plug == aOutputMesh)
	{
		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&returnStatus);
		if (!returnStatus) {
			cerr << "ERROR creating outputData\n";
			return returnStatus;
		}

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		returnStatus = computeOutputMesh(datablock, newOutputData);
		if(returnStatus != MS::kSuccess)
		{
			createEmptyMesh( newOutputData );
		}
		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( aOutputMesh );
		outHandle.set( newOutputData );
		datablock.setClean( plug );
	}
	else if(plug == aCenterTranslate ||
		plug == aCenterTranslateX || 
		plug == aCenterTranslateY || 
		plug == aCenterTranslateZ )
	{
		/* Get face data */
		MDataHandle faceTrackingHandle = datablock.inputValue( aInputData, &returnStatus );
		if (!returnStatus) {
			returnStatus.perror("Error getting skeleton data handle\n");
			return returnStatus;
		}
		PCEFaceTrackingData* pData = (PCEFaceTrackingData*)faceTrackingHandle.asPluginData();
		if ( NULL == pData ) {
			cerr << "NULL face data found\n";
			return returnStatus;
		}
		PCEFaceTrackingResult* pFaceShape = pData->pFaceResult;
		if ( NULL == pFaceShape ) {
			cerr << "NULL face shape found\n";
			return returnStatus;
		}

		Vector3		centerPt = pFaceShape->getCenterPt();

		MDataHandle otHandle = datablock.outputValue( aCenterTranslate ); 
		otHandle.set( (double)centerPt.x,
			(double)centerPt.y,
			(double)centerPt.z );
		datablock.setClean(aCenterTranslate);
	}
	else if(plug == aScale ||
		plug == aScaleX ||
		plug == aScaleY ||
		plug == aScaleZ ||
		plug == aRotate ||
		plug == aRotateX || 
		plug == aRotateY || 
		plug == aRotateZ ||
		plug == aTranslate ||
		plug == aTranslateX || 
		plug == aTranslateY || 
		plug == aTranslateZ)
	{
		/* Get face data */
		MDataHandle faceTrackingHandle = datablock.inputValue( aInputData, &returnStatus );
		if (!returnStatus) {
			returnStatus.perror("Error getting skeleton data handle\n");
			return returnStatus;
		}
		PCEFaceTrackingData* pData = (PCEFaceTrackingData*)faceTrackingHandle.asPluginData();
		if ( NULL == pData ) {
			cerr << "NULL face data found\n";
			return returnStatus;
		}
		PCEFaceTrackingResult* pFaceShape = pData->pFaceResult;
		if ( NULL == pFaceShape ) {
			cerr << "NULL face shape found\n";
			return returnStatus;
		}

		float	scale = pFaceShape->getScale() * 100.0f;
		MDataHandle otHandle = datablock.outputValue( aScale );
		otHandle.set( (double)scale,
			(double)scale,
			(double)scale );
		datablock.setClean(aScale);

		float rotationXYZ[3];
		pFaceShape->getRotationXYZ(rotationXYZ);
		otHandle = datablock.outputValue( aRotate ); 
		otHandle.set( (double)rotationXYZ[0],
			(double)rotationXYZ[1],
			(double)rotationXYZ[2] );
		datablock.setClean(aRotate);

		float translationXYZ[3];
		pFaceShape->getTranslationXYZ(translationXYZ);
		otHandle = datablock.outputValue( aTranslate ); 
		otHandle.set( (double)(translationXYZ[0] * 100.0f),
			(double)(translationXYZ[1] * 100.0f),
			(double)(translationXYZ[2] * 100.0f) );
		datablock.setClean(aTranslate);
	}
	else if(plug == aUpperLipRaiser ||
		plug == aJawLowerer || 
		plug == aLipStretcher || 
		plug == aBrowLowerer ||
		plug == aLipCornerDepressor || 
		plug == aOuterBrowRaiser )
	{
		/* Get face data */
		MDataHandle faceTrackingHandle = datablock.inputValue( aInputData, &returnStatus );
		if (!returnStatus) {
			returnStatus.perror("Error getting skeleton data handle\n");
			return returnStatus;
		}
		PCEFaceTrackingData* pData = (PCEFaceTrackingData*)faceTrackingHandle.asPluginData();
		if ( NULL == pData ) {
			cerr << "NULL face data found\n";
			return returnStatus;
		}
		PCEFaceTrackingResult* pFaceShape = pData->pFaceResult;
		if ( NULL == pFaceShape ) {
			cerr << "NULL face shape found\n";
			return returnStatus;
		}

		float* pAUs = pFaceShape->getAUs();
		unsigned int auCount =  pFaceShape->getAUcount();
		if(!pAUs || auCount == 0)
			return returnStatus;

		MDataHandle otHandle = datablock.outputValue( aUpperLipRaiser );
		if(pAUs[0] >= -1.0f && pAUs[0] <= 1.0f)
			otHandle.set( (double)pAUs[0] );
		datablock.setClean(aUpperLipRaiser);

		if(auCount > 1 && (pAUs[1] >= -1.0f && pAUs[1] <= 1.0f))
		{
			otHandle = datablock.outputValue( aJawLowerer ); 
			otHandle.set( (double)pAUs[1] );
		}
		datablock.setClean(aJawLowerer);

		if(auCount > 2 && (pAUs[2] >= -1.0f && pAUs[2] <= 1.0f))
		{
			otHandle = datablock.outputValue( aLipStretcher ); 
			otHandle.set( (double)pAUs[2] );
		}
		datablock.setClean(aLipStretcher);

		if(auCount > 3 && (pAUs[3] >= -1.0f && pAUs[3] <= 1.0f))
		{
			otHandle = datablock.outputValue( aBrowLowerer ); 
			otHandle.set( (double)pAUs[3] );
		}
		datablock.setClean(aBrowLowerer);

		if(auCount > 4 && (pAUs[4] >= -1.0f && pAUs[4] <= 1.0f))
		{
			otHandle = datablock.outputValue( aLipCornerDepressor );
			otHandle.set( (double)pAUs[4] );
		}
		datablock.setClean(aLipCornerDepressor);

		if(auCount > 5 && (pAUs[5] >= -1.0f && pAUs[5] <= 1.0f))
		{
			otHandle = datablock.outputValue( aOuterBrowRaiser ); 
			otHandle.set( (double)pAUs[5] );
		}
		datablock.setClean(aOuterBrowRaiser);
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}


void* PCEFaceTrackingDriver::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new PCEFaceTrackingDriver();
}

MStatus PCEFaceTrackingDriver::initialize()
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
		PCEFaceTrackingData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create inputData attribute"); return stat;}
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	stat = addAttribute( aInputData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// ----------------------- OUTPUTS -------------------------
	aOutputMesh = typedAttr.create( "outputMesh", "otm",
		MFnData::kMesh,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create outputMesh attribute"); return stat;}
	typedAttr.setWritable( false );
	stat = addAttribute( aOutputMesh );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aCenterTranslateX = nAttr.create( "centerTranslateX", "ctX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerTranslateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterTranslateY = nAttr.create( "centerTranslateY", "ctY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerTranslateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterTranslateZ = nAttr.create( "centerTranslateZ", "ctZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create centerTranslateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aCenterTranslate = nAttr.create( "centerTranslate", "ht", aCenterTranslateX, aCenterTranslateY, aCenterTranslateZ, &stat );
	if (!stat) { stat.perror("create centerTranslate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aCenterTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aScaleX = nAttr.create( "scaleX", "sX", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create scaleX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aScaleY = nAttr.create( "scaleY", "sY", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create scaleY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aScaleZ = nAttr.create( "scaleZ", "sZ", MFnNumericData::kDouble, 1.0, &stat );
	if (!stat) { stat.perror("create scaleZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aScale = nAttr.create( "scale", "sc", aScaleX, aScaleY, aScaleZ, &stat );
	if (!stat) { stat.perror("create scale attribute"); return stat;}
	nAttr.setDefault(1.0f, 1.0f, 1.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aScale );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aRotateX = nAttr.create( "rotateX", "rX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotateY = nAttr.create( "rotateY", "rY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotateZ = nAttr.create( "rotateZ", "rZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create rotateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aRotate = nAttr.create( "rotate", "ro", aRotateX, aRotateY, aRotateZ, &stat );
	if (!stat) { stat.perror("create rotate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aRotate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aTranslateX = nAttr.create( "translateX", "tX", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create translateX attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTranslateY = nAttr.create( "translateY", "tY", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create translateY attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTranslateZ = nAttr.create( "translateZ", "tZ", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create translateZ attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	aTranslate = nAttr.create( "translate", "t", aTranslateX, aTranslateY, aTranslateZ, &stat );
	if (!stat) { stat.perror("create translate attribute"); return stat;}
	nAttr.setDefault(0.0f, 0.0f, 0.0f);
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aTranslate );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aUpperLipRaiser = nAttr.create( "upperLipRaiser", "ulr", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create upperLipRaiser attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aUpperLipRaiser );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aJawLowerer = nAttr.create( "jawLowerer", "jl", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create jawLowerer attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aJawLowerer );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipStretcher = nAttr.create( "lipStretcher", "ls", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create lipStretcher attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipStretcher );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aBrowLowerer = nAttr.create( "browLowerer", "bl", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create browLowerer attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aBrowLowerer );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLipCornerDepressor = nAttr.create( "lipCornerDepressor", "lcd", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create lipCornerDepressor attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aLipCornerDepressor );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aOuterBrowRaiser = nAttr.create( "outerBrowRaiser", "obr", MFnNumericData::kDouble, 0.0, &stat );
	if (!stat) { stat.perror("create outerBrowRaiser attribute"); return stat;}
	nAttr.setReadable( true );
	nAttr.setWritable( false );
	nAttr.setStorable(true);
	stat = addAttribute( aOuterBrowRaiser );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aInputData, aOutputMesh );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aCenterTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aScaleX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aScaleY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aScaleZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aScale );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aRotate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTranslateX );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTranslateY );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTranslateZ );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aTranslate );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aUpperLipRaiser );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aJawLowerer );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLipStretcher );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aBrowLowerer );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aLipCornerDepressor );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aInputData, aOuterBrowRaiser );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;

}

