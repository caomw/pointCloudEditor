//
// Copyright (C) Tingzhu Zhou
// 
// File: pointCloudShape.cpp
//
// Author: Tingzhu Zhou
//

///////////////////////////////////////////////////////////////////////////////
//
// pointCloudShape.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <math.h>

#include "PCEPointCloudShape.h"
#include "PCEPointCloudData.h"
#include "PCEPointCloudDataIterator.h"
#include "api_macros.h"

#include <maya/MFnPluginData.h>
#include <maya/MFnAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MDrawRegistry.h>
#include <maya/MViewport2Renderer.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MAttributeSpecArray.h>
#include <maya/MAttributeSpec.h>
#include <maya/MAttributeIndex.h>
#include <maya/MObjectArray.h>
#include <maya/MVectorArray.h>
#include <maya/MPointArray.h>
#include <maya/MPlane.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>

////////////////////////////////////////////////////////////////////////////////
//
// Shape implementation
//
////////////////////////////////////////////////////////////////////////////////

MObject PCEPointCloudShape::aInputCloudData;
MObject PCEPointCloudShape::aOutputCloudData;
MObject PCEPointCloudShape::aCachedCloudData;

MObject PCEPointCloudShape::bboxCorner1;
MObject PCEPointCloudShape::bboxCorner2;
MObject PCEPointCloudShape::useWeightedTransformUsingFunction;
MObject PCEPointCloudShape::useWeightedTweakUsingFunction;

MObject PCEPointCloudShape::aShowNormals;
MObject PCEPointCloudShape::aNormalLength;

MObject PCEPointCloudShape::aShowInvalid;
MObject PCEPointCloudShape::aShowCluster;
MObject PCEPointCloudShape::aUseLighting;

MTypeId PCEPointCloudShape::id( 0x80099 );

PCEPointCloudShape::PCEPointCloudShape() {}
PCEPointCloudShape::~PCEPointCloudShape() {}


///////////////////////////////////////////////////////////////////////////////
//
// Overrides
//
///////////////////////////////////////////////////////////////////////////////

/* override */
void PCEPointCloudShape::postConstructor()
//
// Description
//
//    When instances of this node are created internally, the MObject associated
//    with the instance is not created until after the constructor of this class
//    is called. This means that no member functions of MPxSurfaceShape can
//    be called in the constructor.
//    The postConstructor solves this problem. Maya will call this function
//    after the internal object has been created.
//    As a general rule do all of your initialization in the postConstructor.
//
{
	// This call allows the shape to have shading groups assigned
	//
	setRenderable( true );

	// Is there input history to this node
	//
	fHasHistoryOnCreate = false;

	// Used by VP2.0 sub-scene evaluator
	//
	fShapeDirty = true;
	fMaterialDirty = true;
}

/* override */
MStatus PCEPointCloudShape::compute( const MPlug& plug, MDataBlock& datablock )
//
// Description
//
//    When input attributes are dirty this method will be called to
//    recompute the output attributes.
//
// Arguments
//
//    plug      - the attribute that triggered the compute
//    datablock - the nodes data
//
// Returns
//
//    kSuccess          - this method could compute the dirty attribute,
//    kUnknownParameter - the dirty attribute can not be handled at this level
//
{
	if ( plug == aOutputCloudData ) {
		return computeOutputCloudData( plug, datablock );
	}
	else if ( plug == aCachedCloudData ) {
		return computeOutputCloudData( plug, datablock );
	}
	else {
		return MS::kUnknownParameter;
	}
}

/* override */
//
// Description
//
//	Horribly abuse the purpose of this method to notify the Viewport 2.0
//  renderer that something about this shape has changed and that it should
//  be retranslated.
//
MStatus PCEPointCloudShape::setDependentsDirty( const MPlug& plug, MPlugArray& plugArray)
{
	// if the dirty attribute is the output mesh then we need to signal the
	// the renderer that it needs to update the object

	if ( plug == aInputCloudData ||
		plug == mControlPoints ||
		plug == mControlValueX ||
		plug == mControlValueY ||
		plug == mControlValueZ ||
		plug == aShowInvalid ||
		plug == aShowNormals ||
		plug == aShowCluster ||
		plug == aUseLighting ||
		plug == aNormalLength)
	{
		signalDirtyToViewport();
	}
	return MS::kSuccess;
}

/* override */
//
// Description
//
//    Handle internal attributes.
//
//    Attributes that require special storage, bounds checking,
//    or other non-standard behavior can be marked as "Internal" by
//    using the "MFnAttribute::setInternal" method.
//
//    The get/setInternalValue methods will get called for internal
//    attributes whenever the attribute values are stored or retrieved
//    using getAttr/setAttr or MPlug getValue/setValue.
//
//    The inherited attribute mControlPoints is internal and we want
//    its values to get stored only if there is input history. Otherwise
//    any changes to the vertices are stored in the cachedData and outputData
//    directly.
//
//    If values are retrieved then we want the controlPoints value
//    returned if there is history, this will be the offset or tweak.
//    In the case of no history, the vertex position of the cached mesh
//    is returned.
//
bool PCEPointCloudShape::getInternalValue( const MPlug& plug, MDataHandle& result )
{
	bool isOk = true;

	if( (plug == mControlPoints) ||
		(plug == mControlValueX) ||
		(plug == mControlValueY) ||
		(plug == mControlValueZ) )
	{
		// If there is input history then the control point value is
		// directly returned. This is the tweak or offset that
		// was applied to the vertex.
		//
		// If there is no input history then return the actual vertex
		// position and ignore the controlPoints attribute.
		//
		if ( hasHistory() )	{
			return MPxNode::getInternalValue( plug, result );
		}
		else {
			double val = 0.0;
			if ( (plug == mControlPoints) && !plug.isArray() ) {
				double pnt_x = 0.0;
				double pnt_y = 0.0;
				double pnt_z = 0.0;
				int index = plug.logicalIndex();
				value( index, pnt_x, pnt_y, pnt_z );
				result.set( pnt_x, pnt_y, pnt_z );
			}
			else if ( plug == mControlValueX ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				value( index, 0, val );
				result.set( val );
			}
			else if ( plug == mControlValueY ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				value( index, 1, val );
				result.set( val );
			}
			else if ( plug == mControlValueZ ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				value( index, 2, val );
				result.set( val );
			}
		}
	}
	// This inherited attribute is used to specify whether or
	// not this shape has history. During a file read, the shape
	// is created before any input history can get connected.
	// This attribute, also called "tweaks", provides a way to
	// for the shape to determine if there is input history
	// during file reads.
	//
	else if ( plug == mHasHistoryOnCreate ) {
		result.set( fHasHistoryOnCreate );
	}
	else {
		isOk = MPxSurfaceShape::getInternalValue( plug, result );
	}

	return isOk;
}

/* override */
//
// Description
//
//    Handle internal attributes.
//
//    Attributes that require special storage, bounds checking,
//    or other non-standard behavior can be marked as "Internal" by
//    using the "MFnAttribute::setInternal" method.
//
//    The get/setInternalValue methods will get called for internal
//    attributes whenever the attribute values are stored or retrieved
//    using getAttr/setAttr or MPlug getValue/setValue.
//
//    The inherited attribute mControlPoints is internal and we want
//    its values to get stored only if there is input history. Otherwise
//    any changes to the vertices are stored in the cachedMesh and outputMesh
//    directly.
//
//    If values are retrieved then we want the controlPoints value
//    returned if there is history, this will be the offset or tweak.
//    In the case of no history, the vertex position of the cached mesh
//    is returned.
//
bool PCEPointCloudShape::setInternalValue( const MPlug& plug, const MDataHandle& handle )
{
	bool isOk = true;

	if( (plug == mControlPoints) ||
		(plug == mControlValueX) ||
		(plug == mControlValueY) ||
		(plug == mControlValueZ) )
	{
		// If there is input history then set the control points value
		// using the normal mechanism. In this case we are setting
		// the tweak or offset that will get applied to the input
		// history.
		//
		// If there is no input history then ignore the controlPoints
		// attribute and set the vertex position directly in the
		// cachedMesh.
		//
		if ( hasHistory() )	{
			verticesUpdated();
			return MPxNode::setInternalValue( plug, handle );
		}
		else {
			if( plug == mControlPoints && !plug.isArray()) {
				int index = plug.logicalIndex();
				double3& ptData = handle.asDouble3();
				setValue( index, ptData );
			}
			else if( plug == mControlValueX ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				setValue( index, 0, handle.asDouble() );
			}
			else if( plug == mControlValueY ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				setValue( index, 1, handle.asDouble() );
			}
			else if( plug == mControlValueZ ) {
				MPlug parentPlug = plug.parent();
				int index = parentPlug.logicalIndex();
				setValue( index, 2, handle.asDouble() );
			}
		}
	}
	// This inherited attribute is used to specify whether or
	// not this shape has history. During a file read, the shape
	// is created before any input history can get connected.
	// This attribute, also called "tweaks", provides a way to
	// for the shape to determine if there is input history
	// during file reads.
	//
	else if ( plug == mHasHistoryOnCreate ) {
		fHasHistoryOnCreate = handle.asBool();
	}
	else {
		isOk = MPxSurfaceShape::setInternalValue( plug, handle );
	}

	return isOk;
}


/* override */
MStatus PCEPointCloudShape::connectionMade( const MPlug& plug,
			const MPlug& otherPlug,
			bool asSrc )
//
// Description
//
//    Whenever a connection is made to this node, this method
//    will get called.
//
{
	MObject thisObj = thisMObject();
	MFnDependencyNode dgNode( thisObj );
	MPlug instObjGroups( dgNode.findPlug("instObjGroups") );
	if ( plug == instObjGroups ) {
		setMaterialDirty(true);
	}
	else if ( plug == aInputCloudData ) {
		MStatus stat;
		MPlug historyPlug( thisObj, mHasHistoryOnCreate );
		stat = historyPlug.setValue( true );
		MCHECKERROR( stat, "connectionMade: setValue(mHasHistoryOnCreate)" );
	}
	return MPxNode::connectionMade( plug, otherPlug, asSrc );
}

/* override */
MStatus PCEPointCloudShape::connectionBroken( const MPlug& plug,
			const MPlug& otherPlug,
			bool asSrc )
//
// Description
//
//    Whenever a connection to this node is broken, this method
//    will get called.
//
{
	MObject thisObj = thisMObject();
	MFnDependencyNode dgNode( thisObj );
	MPlug instObjGroups( dgNode.findPlug("instObjGroups") );
	if ( plug == instObjGroups ) {
		setMaterialDirty(true);
	}
	else if ( plug == aInputCloudData ) {
		MStatus stat;
		MPlug historyPlug( thisObj, mHasHistoryOnCreate );
		stat = historyPlug.setValue( false );
		MCHECKERROR( stat, "connectionBroken: setValue(mHasHistoryOnCreate)" );
	}

	return MPxNode::connectionBroken( plug, otherPlug, asSrc );
}

/* override */
MStatus PCEPointCloudShape::shouldSave( const MPlug& plug, bool& result )
//
// Description
//
//    During file save this method is called to determine which
//    attributes of this node should get written. The default behavior
//    is to only save attributes whose values differ from the default.
//
//
//
{
	MStatus status = MS::kSuccess;

	if( plug == mControlPoints || plug == mControlValueX ||
		plug == mControlValueY || plug == mControlValueZ )
	{
		if( hasHistory() ) {
			// Calling this will only write tweaks if they are
			// different than the default value.
			//
			status = MPxNode::shouldSave( plug, result );
		}
		else {
			result = false;
		}
	}
	else if ( plug == aCachedCloudData ) {
		if ( hasHistory() ) {
			result = false;
		}
		else {
			MObject data;
			status = plug.getValue( data );
			MCHECKERROR( status, "shouldSave: MPlug::getValue" );
			result = ( ! data.isNull() );
		}
	}
	else {
		status = MPxNode::shouldSave( plug, result );
	}

	return status;
}

PCEPointCloudShape::SchedulingType	PCEPointCloudShape::schedulingType()const
// PCL_TODO: tO
{
	return SchedulingType::kDefaultScheduling;//kGloballySerialize;
}

/* override */
void PCEPointCloudShape::componentToPlugs( MObject & component,
											MSelectionList & list ) const
//
// Description
//
//    Converts the given component values into a selection list of plugs.
//    This method is used to map components to attributes.
//
// Arguments
//
//    component - the component to be translated to a plug/attribute
//    list      - a list of plugs representing the passed in component
//
{
	if ( component.hasFn(MFn::kSingleIndexedComponent) ) {

		MFnSingleIndexedComponent fnVtxComp( component );
		MObject thisNode = thisMObject();
		MPlug plug( thisNode, mControlPoints );
		// If this node is connected to a tweak node, reset the
		// plug to point at the tweak node.
		//
		convertToTweakNodePlug(plug);

		int len = fnVtxComp.elementCount();

		for ( int i = 0; i < len; i++ )
		{
			plug.selectAncestorLogicalIndex(fnVtxComp.element(i),
											plug.attribute());
			list.add(plug);
		}
	}
}

/* override */
MPxSurfaceShape::MatchResult
PCEPointCloudShape::matchComponent( const MSelectionList& item,
									const MAttributeSpecArray& spec,
									MSelectionList& list )
//
// Description:
//
//    Component/attribute matching method.
//    This method validates component names and indices which are
//    specified as a string and adds the corresponding component
//    to the passed in selection list.
//
//    For instance, select commands such as "select shape1.vtx[0:7]"
//    are validated with this method and the corresponding component
//    is added to the selection list.
//
// Arguments
//
//    item - DAG selection item for the object being matched
//    spec - attribute specification object
//    list - list to add components to
//
// Returns
//
//    the result of the match
//
{
	MPxSurfaceShape::MatchResult result = MPxSurfaceShape::kMatchOk;
	MAttributeSpec attrSpec = spec[0];
	int dim = attrSpec.dimensions();

	// Look for attributes specifications of the form :
	//     vtx[ index ]
	//     vtx[ lower:upper ]
	//
	if ( (1 == spec.length()) && (dim > 0) && (attrSpec.name() == "vtx") ) {
		int numVertices = meshGeom()->pntCloud.size();
		MAttributeIndex attrIndex = attrSpec[0];

		int upper = 0;
		int lower = 0;
		if ( attrIndex.hasLowerBound() ) {
			attrIndex.getLower( lower );
		}
		if ( attrIndex.hasUpperBound() ) {
			attrIndex.getUpper( upper );
		}

		// Check the attribute index range is valid
		//
		if ( (lower > upper) || (upper >= numVertices) ) {
			result = MPxSurfaceShape::kMatchInvalidAttributeRange;
		}
		else {
			MDagPath path;
			item.getDagPath( 0, path );
			MFnSingleIndexedComponent fnVtxComp;
			MObject vtxComp = fnVtxComp.create( MFn::kMeshVertComponent );

			for ( int i=lower; i<=upper; i++ )
			{
				fnVtxComp.addElement( i );
			}
			list.add( path, vtxComp );
		}
	}
	else {
		// Pass this to the parent class
		return MPxSurfaceShape::matchComponent( item, spec, list );
	}

	return result;
}

/* override */
bool PCEPointCloudShape::match( const MSelectionMask & mask,
		const MObjectArray& componentList ) const
//
// Description:
//
//		Check for matches between selection type / component list, and
//		the type of this shape / or it's components
//
//      This is used by sets and deformers to make sure that the selected
//      components fall into the "vertex only" category.
//
// Arguments
//
//		mask          - selection type mask
//		componentList - possible component list
//
// Returns
//		true if matched any
//
{
	bool result = false;

	if( componentList.length() == 0 ) {
		result = mask.intersects( MSelectionMask::kSelectMeshes );
	}
	else {
		for ( int i=0; i<(int)componentList.length(); i++ ) {
			if ( (componentList[i].apiType() == MFn::kMeshVertComponent) &&
				(mask.intersects(MSelectionMask::kSelectMeshVerts))
				) {
					result = true;
					break;
			}
		}
	}
	return result;
}

/* override */
MSelectionMask PCEPointCloudShape::getShapeSelectionMask() const
//
// Description
//     This method is overriden to support interactive object selection in Viewport 2.0
//
// Returns
//
//    The selection mask of the shape
//
{
	MSelectionMask::SelectionType selType = MSelectionMask::kSelectMeshes;
	return MSelectionMask( selType );
}

/* override */
MSelectionMask PCEPointCloudShape::getComponentSelectionMask() const
//
// Description
//     This method is overriden to support interactive component selection in Viewport 2.0
//
// Returns
//
//    The selection mask of the shape components
//
{
	MSelectionMask retVal(MSelectionMask::kSelectMeshVerts);
	return retVal;
}

/* override */
MObject PCEPointCloudShape::createFullVertexGroup() const
//
// Description
//     This method is used by maya when it needs to create a component
//     containing every vertex (or control point) in the shape.
//     This will get called if you apply some deformer to the whole
//     shape, i.e. select the shape in object mode and add a deformer to it.
//
// Returns
//
//    A "complete" component representing all vertices in the shape.
//
{
	// Create a vertex component
	//
	MFnSingleIndexedComponent fnComponent;
	MObject fullComponent = fnComponent.create( MFn::kMeshVertComponent );

	// Set the component to be complete, i.e. the elements in
	// the component will be [0:numVertices-1]
	//
	int numVertices = ((PCEPointCloudShape*)this)->meshGeom()->pntCloud.size();
	fnComponent.setCompleteData( numVertices );

	return fullComponent;
}

/* override */
MObject PCEPointCloudShape::localShapeInAttr() const
//
// Description
//
//    Returns the input attribute of the shape. This is used by
//    maya to establish input connections for deformers etc.
//    This attribute must be data of type kGeometryData.
//
// Returns
//
//    input attribute for the shape
//
{
	return aInputCloudData;
}

/* override */
MObject PCEPointCloudShape::localShapeOutAttr() const
//
// Description
//
//    Returns the output attribute of the shape. This is used by
//    maya to establish out connections for deformers etc.
//    This attribute must be data of tye kGeometryData.
//
// Returns
//
//    output attribute for the shape
//
//
{
	return aOutputCloudData;
}

/* override */
MObject PCEPointCloudShape::cachedShapeAttr() const
//
// Description
//
//    Returns the cached shape attribute of the shape.
//    This attribute must be data of type kGeometryData.
//
// Returns
//
//    cached shape attribute
//
{
	return aCachedCloudData;
}

/* override */
MObject PCEPointCloudShape::geometryData() const
//
// Description
//
//    Returns the data object for the surface. This gets
//    called internally for grouping (set) information.
//
{
	PCEPointCloudShape* nonConstThis = (PCEPointCloudShape*)this;
	MDataBlock datablock = nonConstThis->forceCache();
	MDataHandle handle = datablock.inputValue( aInputCloudData );
	return handle.data();
}

/*override */
void PCEPointCloudShape:: closestPoint ( const MPoint & toThisPoint, \
		MPoint & theClosestPoint, double tolerance ) const
//
// Description
//
//		Returns the closest point to the given point in space.
//		Used for rigid bind of skin.  Currently returns wrong results;
//		override it by implementing a closest point calculation.
{
	// Iterate through the geometry to find the closest point within
	// the given tolerance.
	//
	double minDistance = DBL_MAX;
	PCEPointCloud* geomPtr = ((PCEPointCloudShape*)this)->meshGeom();
	int numVertices = geomPtr->pntCloud.size();
	for (int ii=0; ii<numVertices; ii++)
	{
		MPoint pt(geomPtr->pntCloud.at(ii).data);
		double distance = pt.distanceTo(toThisPoint);

		if(minDistance > distance)
		{
			minDistance = distance;
			// Set the output point to the result
			theClosestPoint = pt;
		}
	}
}


/* override */
void PCEPointCloudShape::transformUsing( const MMatrix & mat,
										const MObjectArray & componentList )
//
// Description
//
//    Transforms by the matrix the given components, or the entire shape
//    if the componentList is empty. This method is used by the freezeTransforms command.
//
// Arguments
//
//    mat           - matrix to tranform the components by
//    componentList - list of components to be transformed,
//                    or an empty list to indicate the whole surface
//
{
	// Let the other version of transformUsing do the work for us.
	//
	transformUsing( mat,
		componentList,
		MPxSurfaceShape::kNoPointCaching,
		NULL);
}


//
// Description
//
//    Transforms the given components. This method is used by
//    the move, rotate, and scale tools in component mode.
//    The bounding box has to be updated here, so do the normals and
//    any other attributes that depend on vertex positions.
//
// Arguments
//    mat           - matrix to tranform the components by
//    componentList - list of components to be transformed,
//                    or an empty list to indicate the whole surface
//    cachingMode   - how to use the supplied pointCache (kSavePoints, kRestorePoints)
//    pointCache    - if non-null, save or restore points from this list base
//					  on the cachingMode
//
void PCEPointCloudShape::transformUsing( const MMatrix & mat,
										const MObjectArray & componentList,
										MVertexCachingMode cachingMode,
										MPointArray* pointCache)
{
	MStatus stat;
	PCEPointCloud* geomPtr = meshGeom();

	// Create cachingMode boolean values for clearer reading of conditional code below
	//
	bool savePoints    = (cachingMode == MPxSurfaceShape::kSavePoints);
	bool restorePoints = (cachingMode == MPxSurfaceShape::kRestorePoints);

	unsigned int i=0,j=0;
	unsigned int len = componentList.length();

	if ( restorePoints ) {
		// restore the points based on the data provided in the pointCache attribute
		//
		unsigned int cacheLen = pointCache->length();
		if (len > 0) {
			// traverse the component list
			//
			for ( i = 0; i < len && j < cacheLen; i++ )
			{
				MObject comp = componentList[i];
				MFnSingleIndexedComponent fnComp( comp );
				int elemCount = fnComp.elementCount();
				for ( int idx=0; idx<elemCount && j < cacheLen; idx++, ++j ) {
					int elemIndex = fnComp.element( idx );
					PCEPointT& pnt = geomPtr->pntCloud.at(elemIndex);
					pnt.x = (*pointCache)[j].x;
					pnt.y = (*pointCache)[j].y;
					pnt.z = (*pointCache)[j].z;
				}
			}
		} else {
			// if the component list is of zero-length, it indicates that we
			// should transform the entire surface
			//
			len = geomPtr->pntCloud.size();
			for ( unsigned int idx = 0; idx < len && j < cacheLen; ++idx, ++j ) {
				PCEPointT& pnt = geomPtr->pntCloud.at(idx);
				pnt.x = (*pointCache)[j].x;
				pnt.y = (*pointCache)[j].y;
				pnt.z = (*pointCache)[j].z;
			}
		}
	} else {
		// Transform the surface vertices with the matrix.
		// If savePoints is true, save the points to the pointCache.
		//
		if (len > 0) {
			// Traverse the componentList
			//
			for ( i=0; i<len; i++ )
			{
				MObject comp = componentList[i];
				MFnSingleIndexedComponent fnComp( comp );
				int elemCount = fnComp.elementCount();

				if (savePoints && 0 == i) {
					pointCache->setSizeIncrement(elemCount);
				}
				for ( int idx=0; idx<elemCount; idx++ )
				{
					int elemIndex = fnComp.element( idx );
					PCEPointT& pt = geomPtr->pntCloud.at(elemIndex);
					MPoint pnt = MPoint(pt.data);
					if (savePoints) {
						pointCache->append(pnt);
					}
					// transform the vertex
					pnt *= mat;
					pt.x = pnt.x;
					pt.y = pnt.y;
					pt.z = pnt.z;
					// PCL_TODO: need to recompute the local normals
					// transform the normal
					MVector norm = MVector(pt.normal_x, pt.normal_y, pt.normal_z);
					norm = norm.transformAsNormal( mat );
					pt.normal_x = norm[0];
					pt.normal_y = norm[1];
					pt.normal_z = norm[2];
				}
			}
		} else {
			// If the component list is of zero-length, it indicates that we
			// should transform the entire surface
			//
			len = geomPtr->pntCloud.size();
			if (savePoints) {
				pointCache->setSizeIncrement(len);
			}
			for ( unsigned int idx = 0; idx < len; ++idx ) {
				PCEPointT& pt = geomPtr->pntCloud.at(idx);
				MPoint pnt = MPoint(pt.data);
				if (savePoints) {
					pointCache->append(pnt);
				}
				// transform the vertex
				pnt *= mat;
				pt.x = pnt.x;
				pt.y = pnt.y;
				pt.z = pnt.z;
				// PCL_TODO: need to recompute the local normals
				// transform the normal
				MVector norm = MVector(pt.normal_x, pt.normal_y, pt.normal_z);
				norm = norm.transformAsNormal( mat );
				pt.normal_x = norm[0];
				pt.normal_y = norm[1];
				pt.normal_z = norm[2];
			}
		}
	}

	// Update the surface
	updateCachedData( geomPtr, componentList );
}

//
// Description
//
//    Update the cached cloudData attribute, handle the tweak history as appropriate,
//    and trigger a bounding box change calculation.
//
// Arguments
//    geomPtr       - the modified geometry to apply to the cached surface attribute
//
void PCEPointCloudShape::updateCachedData( const PCEPointCloud* geomPtr, const MObjectArray & componentList )
{
	MStatus stat;
	unsigned int len = componentList.length();

	// Retrieve the value of the cached cloudData attribute.
	// We will set the new geometry data into the cached surface attribute
	//
	// Access the datablock directly. This code has to be efficient
	// and so we bypass the compute mechanism completely.
	// NOTE: In general we should always go though compute for getting
	// and setting attributes.
	//
	MDataBlock datablock = forceCache();

	MDataHandle cachedHandle = datablock.outputValue( aCachedCloudData, &stat );
	MCHECKERRORNORET( stat, "computeInputCloudData error getting cachedCloudData")
	PCEPointCloudData* cached = (PCEPointCloudData*) cachedHandle.asPluginData();

	MDataHandle dHandle = datablock.outputValue( mControlPoints, &stat );
	MCHECKERRORNORET( stat, "updateCachedCloudData get dHandle" )

	// If there is history then calculate the tweaks necessary for
	// setting the final positions of the vertices.
	//
	if ( hasHistory() && (NULL != cached) ) {
		// Since the shape has history, we need to store the tweaks (deltas)
		// between the input shape and the tweaked shape in the control points
		// attribute.
		//
		stat = buildControlPoints( datablock, geomPtr->pntCloud.size() );
		MCHECKERRORNORET( stat, "updateCachedCloudData buildControlPoints" )

		MArrayDataHandle cpHandle( dHandle, &stat );
		MCHECKERRORNORET( stat, "updateCachedCloudData get cpHandle" )

		// Loop through the component list and transform each vertex.
		//
		for ( unsigned int i=0; i<len; i++ )
		{
			MObject comp = componentList[i];
			MFnSingleIndexedComponent fnComp( comp );
			int elemCount = fnComp.elementCount();
			for ( int idx=0; idx<elemCount; idx++ )
			{
				int elemIndex = fnComp.element( idx );
				cpHandle.jumpToElement( elemIndex );
				MDataHandle pntHandle = cpHandle.outputValue();
				double3& pnt = pntHandle.asDouble3();

				MPoint oldPnt = MPoint( cached->pCloudData->pntCloud.at(elemIndex).data );
				MPoint newPnt = MPoint( geomPtr->pntCloud.at(elemIndex).data );
				MPoint offset = newPnt - oldPnt;

				pnt[0] += offset[0];
				pnt[1] += offset[1];
				pnt[2] += offset[2];
			}
		}
	}

	// Copy outputSurface to cachedSurface
	//
	if ( NULL == cached ) {
		cerr << "NULL cachedSurface data found\n";
	}
	else {
		*(cached->pCloudData) = *geomPtr;
	}

	MPlug pCPs(thisMObject(),mControlPoints);
	pCPs.setValue(dHandle);

	// Moving vertices will likely change the bounding box.
	//
	computeBoundingBox( datablock );

	// Tell maya the bounding box for this object has changed
	// and thus "boundingBox()" needs to be called.
	//
	childChanged( MPxSurfaceShape::kBoundingBoxChanged );

	// Signal to the viewport that it needs to update the object
	signalDirtyToViewport();
}

//
// Description
//
//    Transforms the given components. This method is used by
//    the move, rotate, and scale tools in component mode when the
//    tweaks for the shape are stored on a separate tweak node.
//    The bounding box has to be updated here, so do the normals and
//    any other attributes that depend on vertex positions.
//
// Arguments
//    mat           - matrix to tranform the components by
//    componentList - list of components to be transformed,
//                    or an empty list to indicate the whole surface
//    cachingMode   - how to use the supplied pointCache (kSavePoints, kRestorePoints, kUpdatePoints)
//    pointCache    - if non-null, save or restore points from this list base
//					  on the cachingMode
//    handle	    - handle to the attribute on the tweak node where the
//					  tweaks should be stored
//
/* override */
void
PCEPointCloudShape::tweakUsing( const MMatrix & mat,
								const MObjectArray & componentList,
								MVertexCachingMode cachingMode,
								MPointArray* pointCache,
								MArrayDataHandle& handle )
{
	PCEPointCloud* geomPtr = meshGeom();

	// Create cachingMode boolean values for clearer reading of conditional code below
	//
	bool savePoints    = (cachingMode == MPxSurfaceShape::kSavePoints);
	bool updatePoints  = (cachingMode == MPxSurfaceShape::kUpdatePoints);
	bool restorePoints = (cachingMode == MPxSurfaceShape::kRestorePoints);

	MArrayDataBuilder builder = handle.builder();

	MPoint delta, currPt, newPt;
	unsigned int i=0;
	unsigned int len = componentList.length();
	unsigned int cacheIndex = 0;
	unsigned int cacheLen = (NULL != pointCache) ? pointCache->length() : 0;

	if ( restorePoints ) {
		// restore points from the pointCache
		//
		if (len > 0) {
			// traverse the component list
			//
			for ( i=0; i<len; i++ )
			{
				MObject comp = componentList[i];
				MFnSingleIndexedComponent fnComp( comp );
				int elemCount = fnComp.elementCount();
				for ( int idx=0; idx<elemCount && cacheIndex < cacheLen; idx++, cacheIndex++) {
					int elemIndex = fnComp.element( idx );
					double3 & pt = builder.addElement( elemIndex ).asDouble3();
					MPoint& cachePt = (*pointCache)[cacheIndex];
					pt[0] += cachePt.x;
					pt[1] += cachePt.y;
					pt[2] += cachePt.z;
				}
			}
		} else {
			// if the component list is of zero-length, it indicates that we
			// should transform the entire surface
			//
			len = geomPtr->pntCloud.size();
			for ( unsigned int idx = 0; idx < len && idx < cacheLen; ++idx ) {
				double3 & pt = builder.addElement( idx ).asDouble3();
				MPoint& cachePt = (*pointCache)[cacheIndex];
				pt[0] += cachePt.x;
				pt[1] += cachePt.y;
				pt[2] += cachePt.z;
			}
		}
	} else {
		// Tweak the points. If savePoints is true, also save the tweaks in the
		// pointCache. If updatePoints is true, add the new tweaks to the existing
		// data in the pointCache.
		//
		if (len > 0) {
			for ( i=0; i<len; i++ )
			{
				MObject comp = componentList[i];
				MFnSingleIndexedComponent fnComp( comp );
				int elemCount = fnComp.elementCount();
				if (savePoints) {
					pointCache->setSizeIncrement(elemCount);
				}
				for ( int idx=0; idx<elemCount; idx++ )
				{
					int elemIndex = fnComp.element( idx );
					double3 & pt = builder.addElement( elemIndex ).asDouble3();
					currPt = newPt = MPoint( geomPtr->pntCloud.at(elemIndex).data );
					newPt *= mat;
					delta.x = newPt.x - currPt.x;
					delta.y = newPt.y - currPt.y;
					delta.z = newPt.z - currPt.z;
					pt[0] += delta.x;
					pt[1] += delta.y;
					pt[2] += delta.z;
					if (savePoints) {
						// store the points in the pointCache for undo
						//
						pointCache->append(delta*(-1.0));
					} else if (updatePoints && cacheIndex < cacheLen) {
						MPoint& cachePt = (*pointCache)[cacheIndex];
						cachePt[0] -= delta.x;
						cachePt[1] -= delta.y;
						cachePt[2] -= delta.z;
						cacheIndex++;
					}
				}
			}
		} else {
			// if the component list is of zero-length, it indicates that we
			// should transform the entire surface
			//
			len = geomPtr->pntCloud.size();
			if (savePoints) {
				pointCache->setSizeIncrement(len);
			}
			for ( unsigned int idx = 0; idx < len; ++idx ) {
				double3 & pt = builder.addElement( idx ).asDouble3();
				currPt = newPt = MPoint( geomPtr->pntCloud.at(idx).data );
				newPt *= mat;
				delta.x = newPt.x - currPt.x;
				delta.y = newPt.y - currPt.y;
				delta.z = newPt.z - currPt.z;
				pt[0] += delta.x;
				pt[1] += delta.y;
				pt[2] += delta.z;
				if (savePoints) {
					// store the points in the pointCache for undo
					//
					pointCache->append(delta*-1.0);
				} else if (updatePoints && idx < cacheLen) {
					MPoint& cachePt = (*pointCache)[idx];
					cachePt[0] -= delta.x;
					cachePt[1] -= delta.y;
					cachePt[2] -= delta.z;
				}
			}
		}
	}
	// Set the builder into the handle.
	//
	handle.set(builder);

	// Tell maya the bounding box for this object has changed
	// and thus "boundingBox()" needs to be called.
	//
	childChanged( MPxSurfaceShape::kBoundingBoxChanged );

	// Signal to the viewport that it needs to update the object
	signalDirtyToViewport();
}

/* override */
//
// Description
//
//    Transforms the given soft-selected components interpolated using the specified weights.
//    This method is used by the move, rotate, and scale tools in component mode.
//    The bounding box has to be updated here, so do the normals and
//    any other attributes that depend on vertex positions.
//    It is similar to the transformUsing() virtual function.
//
// Arguments
//
//    xform           the matrix representing the transformation that is to be applied to the components
//    space           the matrix representing the transformation space to perform the interpolated transformation.
//                    A value of NULL indicates it should be ignored.
//    componentList   a list of components to be transformed and their weights.  This list will not be empty.
//    cachingMode     whether the points should be added/updated in the pointCache, or restored from
//                    the pointCache, or transform using the original values in the pointCache.
//    pointCache      used to store for undo and restore points during undo
//    freezePlane     used for symmetric transformation of components.  A value of NULL indicates
//                    it is not used and there is no symmetric transformation.
//
void PCEPointCloudShape::weightedTransformUsing(	const MTransformationMatrix& xform,
													const MMatrix* space,
													const MObjectArray& componentList,
													MVertexCachingMode cachingMode,
													MPointArray* pointCache,
													const MPlane* freezePlane )
{
	// For example purposes only, use the default MPxSurfaceShape::weightedTransformUsing() if the
	// useWeightedTransformUsingFunction is false
	//
	MPlug plg_useWeightedTransformUsingFunction( thisMObject(), useWeightedTransformUsingFunction );
	bool val_useWeightedTransformUsingFunction = plg_useWeightedTransformUsingFunction.asBool();
	if (!val_useWeightedTransformUsingFunction)
	{
		MPxSurfaceShape::weightedTransformUsing(xform, space, componentList, cachingMode, pointCache, freezePlane);
		signalDirtyToViewport();
		return;
	}

	// Create cachingMode boolean values for clearer reading of conditional code below
	//
	bool savePoints    = (cachingMode == MPxSurfaceShape::kSavePoints);
	bool updatePoints  = (cachingMode == MPxSurfaceShape::kUpdatePoints);
	bool restorePoints  = (cachingMode == MPxSurfaceShape::kRestorePoints);
	bool transformOrigPoints  = (cachingMode == MPxSurfaceShape::kTransformOriginalPoints);

	// Pre-calculate parameters
	MMatrix spaceInv;
	if (space) {
		spaceInv = space->inverse();
	}

	// Traverse the componentList and modify the control points
	//
	PCEPointCloud* geomPtr = meshGeom();
	float almostZero = 1.0e-5f; // Hardcoded tolerance
	int pointCacheIndex = 0;
	unsigned int len = componentList.length();
	for ( unsigned int i=0; i<len; i++ )
	{
		MObject comp = componentList[i];
		MFnSingleIndexedComponent fnComp( comp );
		int elemCount = fnComp.elementCount();
		bool hasWeights = fnComp.hasWeights();
		bool hasSeam = (NULL != freezePlane);

		if (savePoints && (0 == i) ) {
			pointCache->setSizeIncrement(elemCount);
		}

		for ( int idx=0; idx<elemCount; idx++ )
		{
			int elemIndex = fnComp.element( idx );
			float perc = (hasWeights) ? fnComp.weight(idx).influence() : 1.0f;

			// Only act upon points (store in pointCache, transform, etc) that have a non-zero weight
			if (perc > almostZero) { // if the point has enough weight to be transformed
				if (restorePoints) {
					// restore the original point from the point cache
					PCEPointT& pt = geomPtr->pntCloud[elemIndex];
					MPoint ptCache = (*pointCache)[pointCacheIndex];
					pt.x = ptCache.x;
					pt.y = ptCache.y;
					pt.z = ptCache.z;
					pointCacheIndex++;
				}
				else { // perform point transformation
					// Update the pointCache with the original value
					if (savePoints) {
						pointCache->append( MPoint(geomPtr->pntCloud.at(elemIndex).data) );
					}
					else if ( transformOrigPoints ) { // start by reverting points back to their original values stored in the pointCache for the transformation
						PCEPointT& pt = geomPtr->pntCloud[elemIndex];
						MPoint ptCache = (*pointCache)[pointCacheIndex];
						pt.x = ptCache.x;
						pt.y = ptCache.y;
						pt.z = ptCache.z;
					}
					else if ( updatePoints ) { // update the pointCache with the current values
						const PCEPointT& pt = geomPtr->pntCloud[elemIndex];
						MPoint& ptCache = (*pointCache)[pointCacheIndex];
						ptCache.x = pt.x;
						ptCache.y = pt.y;
						ptCache.z = pt.z;
					}

					// Compute interpolated transformation matrix
					MMatrix mat;
					if (perc == 1.0) {
						mat = xform.asMatrix();
					}
					else {
						mat = (space) ? (*space) * xform.asMatrix(perc) * (spaceInv) : xform.asMatrix(perc) ;
					}

					// transform to new position
					PCEPointT& pnt = geomPtr->pntCloud[elemIndex];
					MPoint pt = MPoint(pnt.data);
					MPoint newp(pt);
					newp *= mat;

					// handle symmetry and reflection
					if( hasSeam && fnComp.weight(idx).seam() > 0.0f)
					{
						newp += freezePlane->normal() * (fnComp.weight(idx).seam() * (freezePlane->directedDistance(pt) - freezePlane->directedDistance( newp)));
					}

					// Update the geomPtr with the new point
					pnt.x = newp.x;
					pnt.y = newp.y;
					pnt.z = newp.z;
					pointCacheIndex++;
				}
			}
		}
	}

	// Update the surface
	updateCachedData( geomPtr, componentList );
}

/* override */
//
// Description
//
//    Transforms the given soft-selected components interpolated using the specified weights.
//    This method is used by the move, rotate, and scale tools in component mode when the
//    tweaks for the shape are stored on a separate tweak node.
//    The bounding box has to be updated here, so do the normals and
//    any other attributes that depend on vertex positions.
//
//    It is similar to the tweakUsing() virtual function and is based on apiMesh::tweakUsing().
//
//
// Arguments
//
//    xform           the matrix representing the transformation that is to be applied to the components
//    space           the matrix representing the transformation space to perform the interpolated transformation.
//                    A value of NULL indicates it should be ignored.
//    componentList   a list of components to be transformed and their weights.  This list will not be empty.
//    cachingMode     whether the points should be added/updated in the pointCache, or restored from
//                    the pointCache, or transform using use the original values in the pointCache.
//    pointCache      used to store for undo and restore points during undo
//    freezePlane     used for symmetric transformation of components.  A value of NULL indicates
//                    it is not used and there is no symmetric transformation.
//    handle	    - handle to the attribute on the tweak node where the
//					  tweaks should be stored
//
void PCEPointCloudShape::weightedTweakUsing(
											const MTransformationMatrix& xform,
											const MMatrix* space,
											const MObjectArray& componentList,
											MVertexCachingMode cachingMode,
											MPointArray* pointCache,
											const MPlane* freezePlane,
											MArrayDataHandle& handle )
{
	// For example purposes only, use the default MPxSurfaceShape::weightedTransformUsing() if the
	// useWeightedTweakUsingFunction is false
	//
	MPlug plg_useWeightedTweakUsingFunction( thisMObject(), useWeightedTweakUsingFunction );
	bool val_useWeightedTweakUsingFunction = plg_useWeightedTweakUsingFunction.asBool();
	if (!val_useWeightedTweakUsingFunction) {
		return MPxSurfaceShape::weightedTweakUsing(xform, space, componentList, cachingMode, pointCache, freezePlane, handle);
	}

	PCEPointCloud* geomPtr = meshGeom();

	// Create cachingMode boolean values for clearer reading of conditional code below
	//
	bool savePoints    = (cachingMode == MPxSurfaceShape::kSavePoints);
	bool updatePoints  = (cachingMode == MPxSurfaceShape::kUpdatePoints);
	bool restorePoints = (cachingMode == MPxSurfaceShape::kRestorePoints);
	bool transformOrigPoints  = (cachingMode == MPxSurfaceShape::kTransformOriginalPoints);

	MArrayDataBuilder builder = handle.builder();

	MPoint delta, currPt, newPt;
	unsigned int i=0;
	unsigned int len = componentList.length();
	unsigned int cacheIndex = 0;
	unsigned int cacheLen = (NULL != pointCache) ? pointCache->length() : 0;

	if ( restorePoints ) {
		// restore points from the pointCache
		//
		// traverse the component list
		//
		for ( i=0; i<len; i++ )
		{
			MObject comp = componentList[i];
			MFnSingleIndexedComponent fnComp( comp );
			int elemCount = fnComp.elementCount();
			for ( int idx=0; idx<elemCount && cacheIndex < cacheLen; idx++, cacheIndex++) {
				int elemIndex = fnComp.element( idx );
				double3 & pt = builder.addElement( elemIndex ).asDouble3();
				MPoint& cachePt = (*pointCache)[cacheIndex];
				pt[0] += cachePt.x;
				pt[1] += cachePt.y;
				pt[2] += cachePt.z;
			}
		}
	} else {
		// Tweak the points. If savePoints is true, also save the tweaks in the
		// pointCache. If updatePoints is true, add the new tweaks to the existing
		// data in the pointCache.
		//

		// Specify a few parameters (for weighted transformation)
		float almostZero = 1.0e-5f; // Hardcoded tolerance
		MMatrix spaceInv;
		if (space) {
			spaceInv = space->inverse();
		}

		for ( i=0; i<len; i++ )
		{
			MObject comp = componentList[i];
			MFnSingleIndexedComponent fnComp( comp );
			int elemCount = fnComp.elementCount();
			bool hasWeights = fnComp.hasWeights(); // (for weighted transformation)
			bool hasSeam = (NULL != freezePlane);  // (for weighted transformation)
			if (savePoints) {
				pointCache->setSizeIncrement(elemCount);
			}
			for ( int idx=0; idx<elemCount; idx++ )
			{
				int elemIndex = fnComp.element( idx );
				float perc = (hasWeights) ? fnComp.weight(idx).influence() : 1.0f; // get the weight for the component

				// Only act upon points (store in pointCache, transform, etc) that have a non-zero weight
				if (perc > almostZero) { // if the point has enough weight to be transformed (for weighted transformation)

					// Compute interpolated transformation matrix (for weighted transformation)
					//
					MMatrix mat;
					if (perc == 1.0) {
						mat = xform.asMatrix();
					}
					else {
						mat = (space) ? (*space) * xform.asMatrix(perc) * (spaceInv) : xform.asMatrix(perc) ;
					}

					// Start by reverting points back to their original values stored in
					// the pointCache for the transformation
					//
					if ( transformOrigPoints ) {
						PCEPointT& pt = geomPtr->pntCloud[elemIndex];
						const MPoint& ptCache = (*pointCache)[cacheIndex];
						pt.x = ptCache.x;
						pt.y = ptCache.y;
						pt.z = ptCache.z;
					}

					// Perform transformation of the point
					//
					double3 & pt = builder.addElement( elemIndex ).asDouble3();
					currPt = newPt = MPoint(geomPtr->pntCloud.at(elemIndex).data);
					newPt *= mat;

					// Handle symmetry and reflection (for weighted transformation)
					//
					if( hasSeam && fnComp.weight(idx).seam() > 0.0f)
					{
						newPt += freezePlane->normal() * (fnComp.weight(idx).seam() * (freezePlane->directedDistance(currPt) - freezePlane->directedDistance( newPt)));
					}

					// Calculate deltas and final positions
					delta.x = newPt.x - currPt.x;
					delta.y = newPt.y - currPt.y;
					delta.z = newPt.z - currPt.z;
					pt[0] += delta.x;
					pt[1] += delta.y;
					pt[2] += delta.z;
					if (savePoints) {
						// store the points in the pointCache for undo
						//
						pointCache->append(delta*(-1.0));
					} else if (updatePoints && cacheIndex < cacheLen) {
						MPoint& cachePt = (*pointCache)[cacheIndex];
						cachePt[0] -= delta.x;
						cachePt[1] -= delta.y;
						cachePt[2] -= delta.z;
						cacheIndex++;
					}
				}

			}
		}
	}
	// Set the builder into the handle.
	//
	handle.set(builder);

	// Tell maya the bounding box for this object has changed
	// and thus "boundingBox()" needs to be called.
	//
	childChanged( MPxSurfaceShape::kBoundingBoxChanged );
}


/* override */
//
// Description
//
//    Returns offsets for the given components to be used my the
//    move tool in normal/u/v mode.
//
// Arguments
//
//    component - components to calculate offsets for
//    direction - array of offsets to be filled
//    mode      - the type of offset to be calculated
//    normalize - specifies whether the offsets should be normalized
//
// Returns
//
//    true if the offsets could be calculated, false otherwise
//
bool PCEPointCloudShape::vertexOffsetDirection( MObject & component,
												MVectorArray & direction,
												MVertexOffsetMode mode,
												bool normalize )
{
	MStatus stat;
	bool offsetOkay = false ;

	MObject vtxComp = component;
	MFnSingleIndexedComponent fnComp( vtxComp, &stat );
	if ( !stat || (component.apiType() != MFn::kMeshVertComponent) ) {
		return false;
	}

	offsetOkay = true ;

	PCEPointCloud * geomPtr = meshGeom();
	if ( NULL == geomPtr ) {
		return false;
	}

	// For each vertex add the appropriate offset
	//
	int count = fnComp.elementCount();
	for ( int idx=0; idx<count; idx++ )
	{
		const PCEPointT& pt = geomPtr->pntCloud.at( fnComp.element(idx) );
		MVector normal = MVector(pt.normal_x, pt.normal_y, pt.normal_z);

		if( mode == MPxSurfaceShape::kNormal ) {
			if( normalize ) normal.normalize() ;
			direction.append( normal );
		}
		else {
			// Construct an orthonormal basis from the normal
			// uAxis, and vAxis are the new vectors.
			//
			MVector uAxis, vAxis ;
			int    i, j, k;
			double a;
			normal.normalize();

			i = 0;  a = fabs( normal[0] );
			if ( a < fabs(normal[1]) ) { i = 1; a = fabs(normal[1]); }
			if ( a < fabs(normal[2]) ) i = 2;
			j = (i+1)%3;  k = (j+1)%3;
			a = sqrt(normal[i]*normal[i] + normal[j]*normal[j]);
			uAxis[i] = -normal[j]/a; uAxis[j] = normal[i]/a; uAxis[k] = 0.0;
			vAxis = normal^uAxis;

			if ( mode == MPxSurfaceShape::kUTangent ||
				mode == MPxSurfaceShape::kUVNTriad )
			{
				if( normalize ) uAxis.normalize() ;
				direction.append( uAxis );
			}

			if ( mode == MPxSurfaceShape::kVTangent ||
				mode == MPxSurfaceShape::kUVNTriad )
			{
				if( normalize ) vAxis.normalize() ;
				direction.append( vAxis );
			}

			if ( mode == MPxSurfaceShape::kUVNTriad ) {
				if( normalize ) normal.normalize() ;
				direction.append( normal );
			}
		}
	}

	return offsetOkay ;
}

/* override */
bool PCEPointCloudShape::isBounded() const
//
// Description
//
//    Specifies that this object has a boundingBox.
//
{
	return true;
}

/* override */
MBoundingBox PCEPointCloudShape::boundingBox() const
//
// Description
//
//    Returns the bounding box for this object.
//    It is a good idea not to recompute here as this funcion is called often.
//
{
	if ( fShapeDirty )
	{
		// Cast away the constant
		PCEPointCloudShape *msPtr = (PCEPointCloudShape *)this;

		// Force update
		msPtr->meshDataRef();
	}

	MObject thisNode = thisMObject();
	MPlug   c1Plug( thisNode, bboxCorner1 );
	MPlug   c2Plug( thisNode, bboxCorner2 );
	MObject corner1Object;
	MObject corner2Object;
	c1Plug.getValue( corner1Object );
	c2Plug.getValue( corner2Object );

	double3 corner1, corner2;

	MFnNumericData fnData;
	fnData.setObject( corner1Object );
	fnData.getData( corner1[0], corner1[1], corner1[2] );
	fnData.setObject( corner2Object );
	fnData.getData( corner2[0], corner2[1], corner2[2] );

	MPoint corner1Point( corner1[0], corner1[1], corner1[2] );
	MPoint corner2Point( corner2[0], corner2[1], corner2[2] );

	return MBoundingBox( corner1Point, corner2Point );
}

/* override */
MPxGeometryIterator* PCEPointCloudShape::geometryIteratorSetup(MObjectArray& componentList,
																MObject& components,
																bool forReadOnly )
//
// Description
//
//    Creates a geometry iterator compatible with his shape.
//
// Arguments
//
//    componentList - list of components to be iterated
//    components    - component to be iterator
//    forReadOnly   -
//
// Returns
//
//    An iterator for the components
//
{
	PCEPointCloudDataIterator * result = NULL;
	if ( components.isNull() ) {
		result = new PCEPointCloudDataIterator( meshGeom(), componentList );
	}
	else {
		result = new PCEPointCloudDataIterator( meshGeom(), components );
	}
	return result;
}

/* override */
bool PCEPointCloudShape::acceptsGeometryIterator( bool writeable )
//
// Description
//
//    Specifies that this shape can provide an iterator for getting/setting
//    control point values.
//
// Arguments
//
//    writable - maya asks for an iterator that can set points if this is true
//
{
	return true;
}

/* override */
bool PCEPointCloudShape::acceptsGeometryIterator( MObject&, bool writeable,
	bool forReadOnly )
//
// Description
//
//    Specifies that this shape can provide an iterator for getting/setting
//    control point values.
//
// Arguments
//
//    writable   - maya asks for an iterator that can set points if this is true
//    forReadOnly - maya asking for an iterator for querying only
//
{
	return true;
}


///////////////////////////////////////////////////////////////////////////////
//
// Helper functions
//
///////////////////////////////////////////////////////////////////////////////

bool PCEPointCloudShape::hasHistory()
//
// Description
//
//    Returns true if the shape has input history, false otherwise.
//
{
	return fHasHistoryOnCreate;
}

bool PCEPointCloudShape::shapeDirty()
//
// Description
//
//    Returns true if the input surface of the shape has been dirtied since
//    the last reset of the flag
//
{
	return fShapeDirty;
}

void PCEPointCloudShape::resetShapeDirty()
//
// Description
//
//    Reset the shape dirty state of the node
//
{
	fShapeDirty = false;
}

bool PCEPointCloudShape::materialDirty() const
//
// Description
//
//    Returns true if the shading group of the shape has been changed since
//    the last reset of the flag
//
{
	return fMaterialDirty;
}

void PCEPointCloudShape::setMaterialDirty(bool dirty)
//
// Description
//
//    Reset the material dirty state of the node
//
{
	fMaterialDirty = dirty;
}

float PCEPointCloudShape::getFloatValue(const MObject &attribute)
{
	MDataBlock block = forceCache();
	MDataHandle handle = block.inputValue(attribute);
	return handle.asFloat();
}

bool PCEPointCloudShape::getBooleanValue(const MObject &attribute)
{
	MDataBlock block = forceCache();
	MDataHandle handle = block.inputValue(attribute);
	return handle.asBool();
}


MStatus PCEPointCloudShape::computeBoundingBox( MDataBlock& datablock )
//
// Description
//
//    Use the larges/smallest vertex positions to set the corners
//    of the bounding box.
//
{
	MStatus stat = MS::kSuccess;

	// Update bounding box
	//
	MDataHandle lowerHandle = datablock.outputValue( bboxCorner1 );
	MDataHandle upperHandle = datablock.outputValue( bboxCorner2 );
	double3 &lower = lowerHandle.asDouble3();
	double3 &upper = upperHandle.asDouble3();

	PCEPointCloud* geomPtr = meshGeom();
	int cnt = geomPtr->pntCloud.size();
	if ( cnt == 0 ) return stat;

	// This clears any old bbox values
	//
	lower[0] = lower[1] = lower[2] = std::numeric_limits<double>::max ();
	upper[0] = upper[1] = upper[2] = -std::numeric_limits<double>::max ();


	for ( int i=0; i<cnt; i++ )
	{
		PCEPointT pnt = geomPtr->pntCloud[i];
		if (!pcl_isfinite (pnt.x) || 
			!pcl_isfinite (pnt.y) || 
			!pcl_isfinite (pnt.z))
			continue;

		if ( pnt.x < lower[0] ) lower[0] = pnt.x;
		if ( pnt.y < lower[1] ) lower[1] = pnt.y;
		if ( pnt.z < lower[2] ) lower[2] = pnt.z;
		if ( pnt.x > upper[0] ) upper[0] = pnt.x;
		if ( pnt.y > upper[1] ) upper[1] = pnt.y;
		if ( pnt.z > upper[2] ) upper[2] = pnt.z;
	}

	lowerHandle.setClean();
	upperHandle.setClean();

	// Signal that the bounding box has changed.
	//
	childChanged( MPxSurfaceShape::kBoundingBoxChanged );

	return stat;
}

MStatus PCEPointCloudShape::computeInputCloudData( const MPlug& plug, MDataBlock& datablock )
//
// Description
//
//    If there is input history, evaluate the input attribute
//
{
	MStatus stat = MS::kSuccess;

	// Get the input surface if there is history
	//
	if ( hasHistory() ) {
		MDataHandle inputHandle = datablock.inputValue( aInputCloudData, &stat );
		MCHECKERROR( stat, "computeInputSurface error getting inputSurface")

		PCEPointCloudData* surf = (PCEPointCloudData*) inputHandle.asPluginData();
		if ( NULL == surf ) {
			cerr << "NULL inputSurface data found\n";
			return stat;
		}

		PCEPointCloud* geomPtr = surf->pCloudData.get();

		// Create the cachedSurface and copy the input surface into it
		//
		MFnPluginData fnDataCreator;
		MTypeId tmpid( PCEPointCloudData::id );
		fnDataCreator.create( tmpid, &stat );
		MCHECKERROR( stat, "compute : error creating Cached apiMeshData")
		PCEPointCloudData * newCachedData = (PCEPointCloudData*)fnDataCreator.data( &stat );
		MCHECKERROR( stat, " error gettin proxy cached apiMeshData object")
		*(newCachedData->pCloudData) = *geomPtr;

		MDataHandle cachedHandle = datablock.outputValue( aCachedCloudData,&stat );
		MCHECKERROR( stat, "computeInputSurface error getting cachedSurface")
		cachedHandle.set( newCachedData );

		/*bool showSkeleton = false;
		inputHandle = datablock.inputValue( aShowSkeleton, &stat );
		if(stat == MS::kSuccess)
			showSkeleton = inputHandle.asBool();

		if(showSkeleton)
		{
			stat = MGlobal::executeCommand("source PCESkeletonUpdate");
			if (!stat) {
				stat.perror("update HIK.");
			}
		}*/
	}
	return stat;
}

MStatus PCEPointCloudShape::computeOutputCloudData( const MPlug& plug,
	MDataBlock& datablock )
//
// Description
//
//    Compute the outputSurface attribute.
//
//    If there is no history, use cachedSurface as the
//    input surface. All tweaks will get written directly
//    to it. Output is just a copy of the cached surface
//    that can be connected etc.
//
{
	MStatus stat;

	// Check for an input surface. The input surface, if it
	// exists, is copied to the cached surface.
	//
	if ( ! computeInputCloudData( plug, datablock ) ) {
		return MS::kFailure;
	}

	// Get a handle to the cached data
	//
	MDataHandle cachedHandle = datablock.outputValue( aCachedCloudData, &stat );
	MCHECKERROR( stat, "computeInputSurface error getting cachedSurface")
	PCEPointCloudData* cached = (PCEPointCloudData*) cachedHandle.asPluginData();
	if ( NULL == cached ) {
		cerr << "NULL cachedSurface data found\n";
	}

	datablock.setClean( plug );

	// Apply any vertex offsets.
	//
	if ( hasHistory() ) {
		if(cached)
			applyTweaks( datablock, cached->pCloudData.get() );
	}
	else {
		MArrayDataHandle cpHandle = datablock.inputArrayValue( mControlPoints,
			&stat );
		cpHandle.setAllClean();
	}

	// Create some output data

	//
	MFnPluginData fnDataCreator;
	MTypeId tmpid( PCEPointCloudData::id );
	fnDataCreator.create( tmpid, &stat );
	MCHECKERROR( stat, "compute : error creating pointCloudShape")
	PCEPointCloudData * newData = (PCEPointCloudData*)fnDataCreator.data( &stat );
	MCHECKERROR( stat, "compute : error gettin at proxy pointCloud data object")

	// Copy the data
	//
	if ( NULL != cached ) {
		*(newData->pCloudData) = *(cached->pCloudData);
	}
	else {
		cerr << "computeOutputSurface: NULL cachedSurface data\n";
	}

	// Assign the new data to the outputSurface handle
	//
	MDataHandle outHandle = datablock.outputValue( aOutputCloudData );
	outHandle.set( newData );

	// Update the bounding box attributes
	//
	stat = computeBoundingBox( datablock );
	MCHECKERROR( stat, "computeBoundingBox" )

	//if(getBooleanValue(PCEPointCloudShape::aShowSkeleton))
	//{
	//	MStatus stat = MGlobal::executeCommand("source PCESkeletonUpdate");//PCL_TODO
	//	if (!stat) {
	//		stat.perror("update HIK.");
	//	}
	//}

	return stat;
}

MStatus PCEPointCloudShape::applyTweaks( MDataBlock& datablock, PCEPointCloud* geomPtr )
//
// Description
//
//    If the shape has history, apply any tweaks (offsets) made
//    to the control points.
//
{
	MStatus stat;

	MArrayDataHandle cpHandle = datablock.inputArrayValue( mControlPoints,
		&stat );
	MCHECKERROR( stat, "applyTweaks get cpHandle" )

	// Loop through the component list and transform each vertex.
	//
	int elemCount = cpHandle.elementCount();
	for ( int idx=0; idx<elemCount; idx++ )
	{
		int elemIndex = cpHandle.elementIndex();
		MDataHandle pntHandle = cpHandle.outputValue();
		double3& offset = pntHandle.asDouble3();

		// Apply the tweaks to the output surface
		//
		PCEPointT& oldPnt = geomPtr->pntCloud.at(elemIndex);
		oldPnt.x = oldPnt.x + offset[0];
		oldPnt.y = oldPnt.y + offset[1];
		oldPnt.z = oldPnt.z + offset[2];

		cpHandle.next();
	}

	return stat;
}

bool PCEPointCloudShape::value( int pntInd, int vlInd, double & val ) const
//
// Description
//
//	  Helper function to return the value of a given vertex
//    from the cachedMesh.
//
{
	bool result = false;

	PCEPointCloudShape* nonConstThis = (PCEPointCloudShape*)this;
	PCEPointCloud* geomPtr = nonConstThis->cachedGeom();
	if ( NULL != geomPtr ) {
		val = geomPtr->pntCloud.at(pntInd).data[ vlInd ];
		result = true;
	}

	return result;
}

bool PCEPointCloudShape::value( int pntInd, double & pnt_x, double & pnt_y, double & pnt_z ) const
//
// Description
//
//	  Helper function to return the value of a given vertex
//    from the cachedMesh.
//
{
	bool result = false;

	PCEPointCloudShape* nonConstThis = (PCEPointCloudShape*)this;
	PCEPointCloud* geomPtr = nonConstThis->cachedGeom();
	if ( NULL != geomPtr ) {
		PCEPointT point = geomPtr->pntCloud.at( pntInd );
		pnt_x = point.x;
		pnt_y = point.y;
		pnt_z = point.z;
		result = true;
	}

	return result;
}

bool PCEPointCloudShape::setValue( int pntInd, int vlInd, double val )
//
// Description
//
//	  Helper function to set the value of a given vertex
//    in the cachedMesh.
//
{
	bool result = false;

	PCEPointCloudShape* nonConstThis = (PCEPointCloudShape*)this;
	PCEPointCloud* geomPtr = nonConstThis->cachedGeom();
	if ( NULL != geomPtr ) {
		PCEPointT& point = geomPtr->pntCloud.at( pntInd );
		point.data[ vlInd ] = val;
		result = true;
	}

	verticesUpdated();

	return result;
}

bool PCEPointCloudShape::setValue( int pntInd, double3 val )
//
// Description
//
//	  Helper function to set the value of a given vertex
//    in the cachedMesh.
//
{
	bool result = false;

	PCEPointCloudShape* nonConstThis = (PCEPointCloudShape*)this;
	PCEPointCloud* geomPtr = nonConstThis->cachedGeom();
	if ( NULL != geomPtr ) {
		PCEPointT& point = geomPtr->pntCloud.at( pntInd );
		point.x = val[0];
		point.y = val[1];
		point.z = val[2];
		result = true;
	}

	verticesUpdated();

	return result;
}

MObject PCEPointCloudShape::meshDataRef()
	//
	// Description
	//
	//    Get a reference to the mesh data (outputSurface)
	//    from the datablock. If dirty then an evaluation is
	//    triggered.
	//
{
	// Get the datablock for this node
	//
	MDataBlock datablock = forceCache();

	// Calling inputValue will force a recompute if the
	// connection is dirty. This means the most up-to-date
	// mesh data will be returned by this method.
	//
	MDataHandle handle = datablock.inputValue( aOutputCloudData );
	return handle.data();
}

PCEPointCloud* PCEPointCloudShape::meshGeom()
	//
	// Description
	//
	//    Returns a pointer to the PointCloudTPtr underlying the shape.
	//
{
	MStatus stat;
	PCEPointCloud* result = NULL;

	MObject tmpObj = meshDataRef();
	MFnPluginData fnData( tmpObj );
	PCEPointCloudData * data = (PCEPointCloudData*)fnData.data( &stat );
	MCHECKERRORNORET( stat, "meshGeom : Failed to get apiMeshData");

	if ( NULL != data ) {
		result = data->pCloudData.get();
	}

	return result;
}

MObject PCEPointCloudShape::cachedDataRef()
//
// Description
//
//    Get a reference to the mesh data (cachedSurface)
//    from the datablock. No evaluation is triggered.
//
{
	// Get the datablock for this node
	//
	MDataBlock datablock = forceCache();
	MDataHandle handle = datablock.outputValue( aCachedCloudData );
	return handle.data();
}

PCEPointCloud* PCEPointCloudShape::cachedGeom()
//
// Description
//
//    Returns a pointer to the apiMeshGeom underlying the shape.
//
{
	MStatus stat;
	PCEPointCloud* result = NULL;

	MObject tmpObj = cachedDataRef();
	MFnPluginData fnData( tmpObj );
	PCEPointCloudData * data = (PCEPointCloudData*)fnData.data( &stat );
	MCHECKERRORNORET( stat, "cachedGeom : Failed to get apiMeshData");

	if ( NULL != data ) {
		result = data->pCloudData.get();
	}

	return result;
}

MStatus PCEPointCloudShape::buildControlPoints( MDataBlock& datablock, int count )
//
// Description
//
//    Check the controlPoints array. If there is input history
//    then we will use this array to store tweaks (vertex movements).
//
{
	MStatus stat;

	MArrayDataHandle cpH = datablock.outputArrayValue( mControlPoints, &stat );
	MCHECKERROR( stat, "compute get cpH" )

		MArrayDataBuilder oldBuilder = cpH.builder();
	if ( count != (int)oldBuilder.elementCount() )
	{
		// Make and set the new builder based on the
		// info from the old builder.
		MArrayDataBuilder builder( oldBuilder );
		MCHECKERROR( stat, "compute - create builder" )

			for ( int vtx=0; vtx<count; vtx++ )
			{
				/* double3 & pt = */ builder.addElement( vtx ).asDouble3();
			}

			cpH.set( builder );
	}

	cpH.setAllClean();

	return stat;
}

void PCEPointCloudShape::verticesUpdated()
//
// Description
//
//    Helper function to tell maya that this shape's
//    vertices have updated and that the bbox needs
//    to be recalculated and the shape redrawn.
//
{
	childChanged( MPxSurfaceShape::kBoundingBoxChanged );
	childChanged( MPxSurfaceShape::kObjectChanged );
}

void PCEPointCloudShape::signalDirtyToViewport()
{
	fShapeDirty = true;
	MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
}

void* PCEPointCloudShape::creator()
//
// Description
//
//    Called internally to create a new instance of the users MPx node.
//
{
	return new PCEPointCloudShape();
}

MStatus PCEPointCloudShape::initialize()
//
// Description
//
//    Attribute (static) initialization.
//    See api_macros.h.
//
{
	MStatus				stat;
	MFnTypedAttribute	typedAttr;
	MFnNumericAttribute	numericAttr;

	// ----------------------- INPUTS --------------------------
	aInputCloudData = typedAttr.create( "inputCloudData", "icd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create inputCloudData attribute" )
	typedAttr.setReadable( false );
	typedAttr.setWritable( true );
	typedAttr.setStorable( false );
	CHECK_MSTATUS( addAttribute( aInputCloudData ) );

	useWeightedTransformUsingFunction = numericAttr.create( "useWeightedTransformUsingFunction", "utru", MFnNumericData::kBoolean, true, &stat);
	MCHECKERROR( stat, "create useWeightedTransformUsingFunction attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( useWeightedTransformUsingFunction );

	useWeightedTweakUsingFunction = numericAttr.create( "useWeightedTweakUsingFunction", "utwu", MFnNumericData::kBoolean, true, &stat);
	MCHECKERROR( stat, "create useWeightedTweakUsingFunction attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( useWeightedTweakUsingFunction );

	aShowInvalid = numericAttr.create("showInvalid", "siv", MFnNumericData::kBoolean, false, &stat);
	MCHECKERROR( stat, "create showInvalid attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowInvalid );

	aShowCluster = numericAttr.create("showCluster", "sc", MFnNumericData::kBoolean, 0, &stat);
	MCHECKERROR( stat, "create showCluster attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowCluster );

	aUseLighting = numericAttr.create("useLighting", "ul", MFnNumericData::kBoolean, 0, &stat);
	MCHECKERROR( stat, "create useLighting attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( aUseLighting );

	aShowNormals = numericAttr.create("showNormals", "sn", MFnNumericData::kBoolean, 0, &stat);
	MCHECKERROR( stat, "create showNormals attribute" )
	numericAttr.setKeyable(true);
	ADD_ATTRIBUTE( aShowNormals );

	aNormalLength = numericAttr.create("NormalLength", "nl", MFnNumericData::kFloat, 0.02, &stat);
	MCHECKERROR( stat, "create aNormalLength attribute" )
	numericAttr.setConnectable(true);
	numericAttr.setKeyable(true);
	numericAttr.setMin(0.0);
	numericAttr.setSoftMin(0.01);
	numericAttr.setSoftMax(10.0);
	ADD_ATTRIBUTE( aNormalLength );

	// ----------------------- OUTPUTS -------------------------
	// local output surface attributes
	//
	aOutputCloudData = typedAttr.create( "outputCloudData", "ocd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create output point cloud attribute" )
	typedAttr.setReadable( true );
	typedAttr.setWritable( false );
	typedAttr.setStorable( false );
	CHECK_MSTATUS( addAttribute( aOutputCloudData ) );

	// bbox attributes
	//
	MAKE_NUMERIC_ATTR(	bboxCorner1, "bboxCorner1", "bb1",
		MFnNumericData::k3Double, 0,
		false, false, false );
	MAKE_NUMERIC_ATTR(	bboxCorner2, "bboxCorner2", "bb2",
		MFnNumericData::k3Double, 0,
		false, false, false );

	// Cached surface used for file IO
	//
	aCachedCloudData = typedAttr.create( "cachedCloudData", "ccd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create cachedCloudData attribute" )
	typedAttr.setReadable( true );
	typedAttr.setWritable( true );
	typedAttr.setStorable( true );
	ADD_ATTRIBUTE( aCachedCloudData );

	// ---------- Specify what inputs affect the outputs ----------
	//
	ATTRIBUTE_AFFECTS( aInputCloudData, aOutputCloudData );
	ATTRIBUTE_AFFECTS( aCachedCloudData, aOutputCloudData );

	ATTRIBUTE_AFFECTS( mControlPoints, aOutputCloudData );
	ATTRIBUTE_AFFECTS( mControlValueX, aOutputCloudData );
	ATTRIBUTE_AFFECTS( mControlValueY, aOutputCloudData );
	ATTRIBUTE_AFFECTS( mControlValueZ, aOutputCloudData );
	ATTRIBUTE_AFFECTS( mControlPoints, aCachedCloudData );
	ATTRIBUTE_AFFECTS( mControlValueX, aCachedCloudData );
	ATTRIBUTE_AFFECTS( mControlValueY, aCachedCloudData );
	ATTRIBUTE_AFFECTS( mControlValueZ, aCachedCloudData );

	return MS::kSuccess;
}
