///////////////////////////////////////////////////////////////////////////////
//
// apiMeshData.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include <maya/MIOStream.h>
#include "PCEPointCloudTextureData.h"


const MTypeId PCEPointCloudTextureData::id( 0x85777 );
const MString PCEPointCloudTextureData::typeName( "pointCloudTextureData" );

PCEPointCloudTextureData::PCEPointCloudTextureData() : fpTextureData(NULL)
{
	fpTextureData = new pcl::PointCloud<pcl::PointXYZRGBA>;
}

PCEPointCloudTextureData::~PCEPointCloudTextureData()
{
	if ( NULL != fpTextureData ) {
		delete fpTextureData;
		fpTextureData = NULL;
	}
}

/* override */
MStatus PCEPointCloudTextureData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCEPointCloudTextureData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCEPointCloudTextureData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCEPointCloudTextureData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void PCEPointCloudTextureData::copy ( const MPxData& other )
{
	fpTextureData = (((const PCEPointCloudTextureData &)other).fpTextureData);
}

/* override */
MTypeId PCEPointCloudTextureData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return PCEPointCloudTextureData::id;
}

/* override */
MString PCEPointCloudTextureData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return PCEPointCloudTextureData::typeName;
}

void * PCEPointCloudTextureData::creator()
{
	return new PCEPointCloudTextureData;
}