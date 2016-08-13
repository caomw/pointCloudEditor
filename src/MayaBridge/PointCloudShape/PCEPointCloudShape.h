//
// Copyright (C) Tingzhu Zhou
// 
// File: pointCloudShape.h
//
// Dependency Graph Node: pointCloudShape
//
// Author: Tingzhu Zhou
//

#ifndef _PCEPointCloudShape
#define _PCEPointCloudShape

///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudShape.h
//
// Implements a new type of shape node in maya called pointCloudShape.
//
// INPUTS
//     inputCloudData    - input point cloud data
//     mControlPoints    - inherited control vertices for the mesh. These values
//                         are tweaks (offsets) that will be applied to the
//                         vertices of the input shape.
//    
// OUTPUTS
//	   outputCloudData   - output point cloud data
//     bboxCorner1       - bounding box upper left corner
//     bboxCorner2       - bounding box lower right corner
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxSurfaceShape.h>
#include <maya/MSelectionList.h>


class PCEPointCloud;
class PCEPointCloudShape : public MPxSurfaceShape
{
public:
	PCEPointCloudShape();
	virtual ~PCEPointCloudShape();

	//////////////////////////////////////////////////////////
	//
	// Overrides
	//
	//////////////////////////////////////////////////////////

	// From MPxNode
	//
	virtual void			postConstructor();
	virtual MStatus   		compute( const MPlug& plug, MDataBlock& data );
	virtual MStatus			setDependentsDirty( const MPlug& plug, MPlugArray& plugArray);
	virtual bool			getInternalValue( const MPlug&,
		MDataHandle&);
	virtual bool			setInternalValue( const MPlug&,
		const MDataHandle&);
	virtual MStatus			connectionMade( const MPlug& plug,
		const MPlug& otherPlug,
		bool asSrc );
	virtual MStatus			connectionBroken( const MPlug& plug,
		const MPlug& otherPlug,
		bool asSrc );
	virtual MStatus			shouldSave( const MPlug& plug, bool& result );
	virtual SchedulingType	schedulingType()const;

	// Attribute to component (components)
	//
	virtual void			componentToPlugs( MObject &,
		MSelectionList & ) const;
	virtual MatchResult		matchComponent( const MSelectionList& item,
		const MAttributeSpecArray& spec,
		MSelectionList& list );
	virtual bool			match(	const MSelectionMask & mask,
		const MObjectArray& componentList ) const;

	virtual MSelectionMask	getShapeSelectionMask() const;
	virtual MSelectionMask	getComponentSelectionMask() const;

	// Support deformers (components)
	//
	virtual MObject			createFullVertexGroup() const;
	virtual MObject 		localShapeInAttr() const;
	virtual MObject 		localShapeOutAttr() const;
	virtual MObject 		cachedShapeAttr() const;

	virtual MObject			geometryData() const;
	virtual void			closestPoint ( const MPoint & toThisPoint,
		MPoint & theClosestPoint,
		double tolerance ) const;


	// Support the translate/rotate/scale tool (components)
	//
	virtual void		    transformUsing( const MMatrix & mat,
		const MObjectArray & componentList );
	virtual void		    transformUsing( const MMatrix & mat,
		const MObjectArray & componentList,
		MPxSurfaceShape::MVertexCachingMode cachingMode,
		MPointArray* pointCache);
	virtual void			tweakUsing( const MMatrix & mat,
		const MObjectArray & componentList,
		MVertexCachingMode cachingMode,
		MPointArray* pointCache,
		MArrayDataHandle& handle );

	// Support the soft-select translate/rotate/scale tool (components)
	//
	virtual void			weightedTransformUsing(	const MTransformationMatrix& mat,
		const MMatrix* space,
		const MObjectArray& componentList,
		MVertexCachingMode cachingMode,
		MPointArray* pointCache,
		const MPlane* freezePlane );
	virtual void            weightedTweakUsing(     const MTransformationMatrix& xform,
		const MMatrix* space,
		const MObjectArray& componentList,
		MVertexCachingMode cachingMode,
		MPointArray* pointCache,
		const MPlane* freezePlane,
		MArrayDataHandle& handle );

	// Support the move tools normal/u/v mode (components)
	//
	virtual bool			vertexOffsetDirection( MObject & component,
		MVectorArray & direction,
		MVertexOffsetMode mode,
		bool normalize );

	// Bounding box methods
	//
	virtual bool            isBounded() const;
	virtual MBoundingBox    boundingBox() const;

	// Associates a user defined iterator with the shape (components)
	//
	virtual	MPxGeometryIterator*
		geometryIteratorSetup( MObjectArray&, MObject&,
		bool forReadOnly = false );
	virtual bool			acceptsGeometryIterator( bool  writeable=true );
	virtual bool			acceptsGeometryIterator( MObject&,
		bool writeable=true,
		bool forReadOnly = false);

	//////////////////////////////////////////////////////////
	//
	// Helper methods
	//
	//////////////////////////////////////////////////////////
	bool					hasHistory();

	bool					shapeDirty();
	void					resetShapeDirty();

	bool					materialDirty() const;
	void					setMaterialDirty(bool dirty);

	MStatus 		  		computeInputCloudData( const MPlug&, MDataBlock& );
	MStatus 		  		computeOutputCloudData( const MPlug&, MDataBlock& );
	MStatus 		  		computeBoundingBox( MDataBlock& );

	MStatus					applyTweaks( MDataBlock&, PCEPointCloud* );

	void 					updateCachedData( const PCEPointCloud* geomPtr, const MObjectArray & componentList );

	bool					value( int pntInd, int vlInd, double & val ) const;
	bool					value( int pntInd, double & pnt_x, double & pnt_y, double & pnt_z ) const;
	bool					setValue( int pntInd, int vlInd, double val );
	bool					setValue( int pntInd, double3 val );

	float					getFloatValue(const MObject &attribute);
	bool					getBooleanValue(const MObject &attribute);

	MObject					meshDataRef();
	PCEPointCloud*			meshGeom();

	MObject					cachedDataRef();
	PCEPointCloud*			cachedGeom();

	MStatus					buildControlPoints( MDataBlock&, int count );
	void					verticesUpdated();

	static  void *          creator();
	static  MStatus         initialize();

public:
	//////////////////////////////////////////////////////////
	//
	// Attributes
	//
	//////////////////////////////////////////////////////////
	static  MObject         aInputCloudData;
	static  MObject         aOutputCloudData;

	static  MObject         useWeightedTransformUsingFunction;
	static  MObject         useWeightedTweakUsingFunction;

	// used to support tweaking of points, the inputSurface attribute data is
	// transferred into the cached surface when it is dirty. The control points
	// tweaks are added into it there.
	//
	static  MObject         aCachedCloudData;

	static  MObject         bboxCorner1;
	static  MObject         bboxCorner2;

	static  MObject			aShowNormals;
	static  MObject			aNormalLength;

	static  MObject			aShowInvalid;
	static  MObject			aShowCluster;
	static  MObject			aUseLighting;

	static	MTypeId			id;

private:
	void					signalDirtyToViewport();

	bool fHasHistoryOnCreate;
	bool fShapeDirty;
	bool fMaterialDirty;
};

#endif /* _PCEPointCloudShape */