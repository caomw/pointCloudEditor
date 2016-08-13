
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

#ifndef _PCEFaceTrackingData
#define _PCEFaceTrackingData

#include "Shape/PCEFaceTrackingResult.h"

#include <maya/MPxData.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>


class PCEFaceTrackingData : public MPxData
{
public:
	//////////////////////////////////////////////////////////////////
	//
	// Overrides from MPxData
	//
	//////////////////////////////////////////////////////////////////

	PCEFaceTrackingData();
	virtual ~PCEFaceTrackingData();

	virtual MStatus			readASCII( const MArgList& argList, unsigned& idx );
	virtual MStatus			readBinary( istream& in, unsigned length );
	virtual MStatus			writeASCII( ostream& out );
	virtual MStatus			writeBinary( ostream& out );

	virtual	void			copy ( const MPxData& );

	virtual MTypeId         typeId() const;
	virtual MString         name() const;

	static void * creator();

public:
	static const MString	typeName;
	static const MTypeId	id;

	// This is the geometry our data will pass though the DG
	//
	PCEFaceTrackingResult*			pFaceResult;
};

#endif /* _PCEFaceTrackingData */
