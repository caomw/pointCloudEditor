
///////////////////////////////////////////////////////////////////////////////
//
// Provides a data type for some arbitrary user geometry.
// 
// A users geometry class can exist in the DAG by creating an
// MPxSurfaceShape (and UI) class for it and can also be passed through
// DG connections by creating an MPxGeometryData class for it.
// 
// MPxGeometryData is the same as MPxData except it provides 
// additional methods to modify the geometry data via an iterator.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef _PCEPointCloudData
#define _PCEPointCloudData

#include "Shape/PCEPointCloud.h"
#include <memory>

#include <maya/MPxGeometryData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class PCEPointCloudData : public MPxGeometryData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	PCEPointCloudData();
	virtual ~PCEPointCloudData();

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxGeometryData
	//
	//////////////////////////////////////////////////////////////////

	virtual MPxGeometryIterator* iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents);
	virtual MPxGeometryIterator* iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents,
											bool world) const;

	virtual bool	updateCompleteVertexGroup( MObject & component ) const;

	//////////////////////////////////////////////////////////////////
	//
	// Helper methods
	//
	//////////////////////////////////////////////////////////////////

	MStatus					readVerticesASCII( const MArgList&, unsigned& );

	MStatus					writeVerticesASCII( ostream& out );

	static void * creator();

public:
	static const MString typeName;
	static const MTypeId id;

	// This is the geometry our data will pass though the DG
	//
	std::shared_ptr<PCEPointCloud>	  pCloudData;
};

#endif /* _PCEPointCloudData */
