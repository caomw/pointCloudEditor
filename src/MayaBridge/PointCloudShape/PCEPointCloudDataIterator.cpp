
///////////////////////////////////////////////////////////////////////////////
//
// pointCloudVertexIterator.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudDataIterator.h"
#include <maya/MIOStream.h>
#include <maya/MPoint.h>

PCEPointCloudDataIterator::PCEPointCloudDataIterator( void * geom, MObjectArray & comps )
	: MPxGeometryIterator( geom, comps ),
	geomPtr( reinterpret_cast<PCEPointCloud*>(geom) )
{
	reset();
}

PCEPointCloudDataIterator::PCEPointCloudDataIterator( void * geom, MObject & comps )
	: MPxGeometryIterator( geom, comps ),
	geomPtr( reinterpret_cast<PCEPointCloud*>(geom) )
{
	reset();
}

/* override */
void PCEPointCloudDataIterator::reset()
//
// Description
//
//  	
//   Resets the iterator to the start of the components so that another
//   pass over them may be made.
//
{
	MPxGeometryIterator::reset();
	setCurrentPoint( 0 );
	if ( NULL != geomPtr ) {
		int maxVertex = geomPtr->pntCloud.size();
		setMaxPoints( maxVertex );
	}
}

/* override */
MPoint PCEPointCloudDataIterator::point() const
//
// Description
//
//    Returns the point for the current element in the iteration.
//    This is used by the transform tools for positioning the
//    manipulator in component mode. It is also used by deformers.	 
//
{
	MPoint pnt;
	if ( NULL != geomPtr ) {
		const PCEPointT& pt = geomPtr->pntCloud.at( index() );
		pnt = MPoint(pt.data);
	}
	return pnt;
}

/* override */
void PCEPointCloudDataIterator::setPoint( const MPoint & pnt ) const
//
// Description
//
//    Set the point for the current element in the iteration.
//    This is used by deformers.	 
//
{
	if ( NULL != geomPtr ) {
		PCEPointT& pt = geomPtr->pntCloud.at( index() );
		pt.x = pnt.x;
		pt.y = pnt.y;
		pt.z = pnt.z;
	}
}

/* override */
int PCEPointCloudDataIterator::iteratorCount() const
{
//
// Description
//
//    Return the number of vertices in the iteration.
//    This is used by deformers such as smooth skinning
//
	return geomPtr ? geomPtr->pntCloud.size() : 0;
	
}

/* override */
bool PCEPointCloudDataIterator::hasPoints() const
//
// Description
//
//    Returns true since the shape data has points.
//
{
	return true;
}

