///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudData.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include <maya/MIOStream.h>
#include "PCEPointCloudData.h"
#include "PCEPointCloudDataIterator.h"

#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MArgList.h>

#include <pcl/io/pcd_io.h>

//////////////////////////////////////////////////////////////////////

// Ascii file IO defines
//
#define kDblQteChar				"\""
#define kSpaceChar				"	"
#define kWrapString				"\n\t\t"
#define kPCDKeyword				"PointCloudData"
#define kPCDName				"PointCloudDataName"

//////////////////////////////////////////////////////////////////////

const MTypeId PCEPointCloudData::id( 0x80006 );
const MString PCEPointCloudData::typeName( "pointCloudData" );

PCEPointCloudData::PCEPointCloudData() : pCloudData(new PCEPointCloud())
{
}

PCEPointCloudData::~PCEPointCloudData()
{
}

/* override */
MStatus PCEPointCloudData::readASCII( const MArgList& argList, unsigned& index )
//
// Description
//    ASCII file input method.
//
{
	MStatus result;
	MString typeStr;

	result = argList.get( index, typeStr );

	if ( result && (typeStr == kPCDKeyword) ) {
		result = argList.get( ++index, typeStr );

		if ( result && pCloudData && (typeStr == kPCDName) ) {
			if (pcl::io::loadPCDFile<PCEPointT> (kPCDName, pCloudData->pntCloud) == -1) //* load the file
				return MS::kFailure;
		}
	}

	return result;
}

/* override */
MStatus PCEPointCloudData::readBinary( istream& /*in*/, unsigned /*length*/ )
//
// Description
//     NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
MStatus PCEPointCloudData::writeASCII( ostream& out )
//
// Description
//    ASCII file output method.
//
{
	out << "\n";
	out << kWrapString;
	out << kDblQteChar << kPCDKeyword << kDblQteChar
		<< kSpaceChar << kPCDName;

	if(pCloudData)
		pcl::io::savePCDFileASCII (kPCDName, pCloudData->pntCloud);

	return MS::kSuccess;
}

/* override */
MStatus PCEPointCloudData::writeBinary( ostream& /*out*/ )
//
// Description
//    NOT IMPLEMENTED
//
{
	return MS::kSuccess;
}

/* override */
void PCEPointCloudData::copy ( const MPxData& other )
{
	pCloudData = (((const PCEPointCloudData &)other).pCloudData);
}

/* override */
MTypeId PCEPointCloudData::typeId() const
//
// Description
//    Binary tag used to identify this kind of data
//
{
	return PCEPointCloudData::id;
}

/* override */
MString PCEPointCloudData::name() const
//
// Description
//    String name used to identify this kind of data
//
{
	return PCEPointCloudData::typeName;
}

void * PCEPointCloudData::creator()
{
	return new PCEPointCloudData;
}

/* override */
MPxGeometryIterator* PCEPointCloudData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents)
//
// Description
//
{
	PCEPointCloudDataIterator * result = NULL;
	if ( useComponents ) {
		result = new PCEPointCloudDataIterator( static_cast<void*>(pCloudData.get()), componentList );
	}
	else {
		result = new PCEPointCloudDataIterator( static_cast<void*>(pCloudData.get()), component );
	}
	return result;
}

/* override */
MPxGeometryIterator* PCEPointCloudData::iterator( MObjectArray & componentList,
											MObject & component,
											bool useComponents,
											bool /*world*/) const
//
// Description
//
{
	PCEPointCloudDataIterator * result = NULL;
	if ( useComponents ) {
		result = new PCEPointCloudDataIterator( static_cast<void*>(pCloudData.get()), componentList );
	}
	else {
		result = new PCEPointCloudDataIterator( static_cast<void*>(pCloudData.get()), component );
	}
	return result;
}

/* override */
bool PCEPointCloudData::updateCompleteVertexGroup( MObject & component ) const
//
// Description
//     Make sure complete vertex group data is up-to-date.
//     Returns true if the component was updated, false if it was already ok.
//
//     This is used by deformers when deforming the "whole" object and
//     not just selected components.
//
{
	MStatus stat;
	MFnSingleIndexedComponent fnComponent( component, &stat );

	// Make sure there is non-null geometry and that the component
	// is "complete". A "complete" component represents every 
	// vertex in the shape.
	//
	if ( stat && pCloudData && (fnComponent.isComplete()) ) {
	
		int maxVerts ;
		fnComponent.getCompleteData( maxVerts );
		int numVertices = pCloudData->pntCloud.size();

		if ( (numVertices > 0) && (maxVerts != numVertices) ) {
			// Set the component to be complete, i.e. the elements in
			// the component will be [0:numVertices-1]
			//
			fnComponent.setCompleteData( numVertices );
			return true;
		}
	}

	return false;
}
