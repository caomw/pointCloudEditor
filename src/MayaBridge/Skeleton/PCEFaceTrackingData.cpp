///////////////////////////////////////////////////////////////////////////////
//
// PCEFaceTrackingData.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include <maya/MIOStream.h>
#include "PCEFaceTrackingData.h"

//////////////////////////////////////////////////////////////////////

// Ascii file IO defines
//
#define kDblQteChar				"\""
#define kSpaceChar				"	"
#define kWrapString				"\n\t\t"

//////////////////////////////////////////////////////////////////////

const MTypeId PCEFaceTrackingData::id( 0x85774 );
const MString PCEFaceTrackingData::typeName( "faceTrackingData" );

PCEFaceTrackingData::PCEFaceTrackingData() : pFaceResult(NULL)
{
	pFaceResult = new PCEFaceTrackingResult;
}

PCEFaceTrackingData::~PCEFaceTrackingData()
{
	if ( NULL != pFaceResult ) {
		delete pFaceResult;
		pFaceResult = NULL;
	}
}

/* override */
MStatus PCEFaceTrackingData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{
	
	return MS::kSuccess;
}

/* override */
MStatus PCEFaceTrackingData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCEFaceTrackingData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{
	
	return MS::kSuccess;
}

/* override */
MStatus PCEFaceTrackingData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void PCEFaceTrackingData::copy ( const MPxData& other )
{
	pFaceResult->deepCopy(*((const PCEFaceTrackingData &)other).pFaceResult) ;
}

/* override */
MTypeId PCEFaceTrackingData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return PCEFaceTrackingData::id;
}

/* override */
MString PCEFaceTrackingData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return PCEFaceTrackingData::typeName;
}

void * PCEFaceTrackingData::creator()
{
	return new PCEFaceTrackingData;
}
