///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudData.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include <maya/MIOStream.h>
#include "PCESkeletonData.h"

//////////////////////////////////////////////////////////////////////

// Ascii file IO defines
//
#define kDblQteChar				"\""
#define kSpaceChar				"	"
#define kWrapString				"\n\t\t"

//////////////////////////////////////////////////////////////////////

const MTypeId PCESkeletonData::id( 0x80774 );
const MString PCESkeletonData::typeName( "skeletonData" );

PCESkeletonData::PCESkeletonData() : pSkeletonData(NULL)
{
	pSkeletonData = new PCESkeletonPosition;
}

PCESkeletonData::~PCESkeletonData()
{
	if ( NULL != pSkeletonData ) {
		delete pSkeletonData;
		pSkeletonData = NULL;
	}
}

/* override */
MStatus PCESkeletonData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{
	
	return MS::kSuccess;
}

/* override */
MStatus PCESkeletonData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCESkeletonData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{
	
	return MS::kSuccess;
}

/* override */
MStatus PCESkeletonData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void PCESkeletonData::copy ( const MPxData& other )
{
	*pSkeletonData = *((const PCESkeletonData &)other).pSkeletonData;
}

/* override */
MTypeId PCESkeletonData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return PCESkeletonData::id;
}

/* override */
MString PCESkeletonData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return PCESkeletonData::typeName;
}

void * PCESkeletonData::creator()
{
	return new PCESkeletonData;
}
