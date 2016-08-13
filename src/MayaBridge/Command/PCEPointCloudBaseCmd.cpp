///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudBaseCmd.cpp
//
///////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudBaseCmd.h"
#include "Shape/PCEPointCloud.h"
#include "../PointCloudShape/PCEPointCloudShape.h"

#include <maya/MGlobal.h>
#include <maya/MItSelectionList.h>
#include <maya/MObject.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MViewport2Renderer.h>

PCEPointCloudBaseCmd::PCEPointCloudBaseCmd()
{
}

PCEPointCloudBaseCmd::~PCEPointCloudBaseCmd()
{
	previousSelectionList.clear();
}

bool PCEPointCloudBaseCmd::isUndoable() const
{
	return true;
}

MStatus PCEPointCloudBaseCmd::doIt(const MArgList& args)
{
	MStatus status = parseArgs(args);
	if(status !=MS::kSuccess){
		MGlobal::displayError( "Parse arguments error" );
		return status;
	}

	MGlobal::getActiveSelectionList(previousSelectionList);
	return redoIt();
}

MStatus PCEPointCloudBaseCmd::redoIt()
{
	return doHelper(false);
}

MStatus PCEPointCloudBaseCmd::undoIt()
{
	MGlobal::setActiveSelectionList(previousSelectionList, MGlobal::kReplaceList);
	return needRecoverData() ? doHelper(true) : MS::kSuccess;
}

MStatus PCEPointCloudBaseCmd::doHelper(bool isRecoverData)
{
	MStatus     stat;				// Status code

	MItSelectionList selListIter( previousSelectionList,	MFn::kInvalid, &stat );
	if ( MS::kSuccess != stat )
		return stat;

	for ( ; !selListIter.isDone(); selListIter.next() )
	{
		//MObject 	dependNode;		// Selected dependency node
		//// Get the selected dependency node and create
		//// a function set for it
		////
		//if ( MS::kSuccess != selListIter.getDependNode( dependNode ) ) {
		//	cerr << "Error getting the dependency node" << endl;
		//	continue;
		//}

		//MFnDependencyNode dgNode( dependNode, &stat );
		//if ( MS::kSuccess != stat ) {
		//	cerr << "Error creating MFnDependencyNode" << endl;
		//	continue;
		//}

		MObject component;
		MDagPath dagPath;
		if ( MS::kSuccess !=  selListIter.getDagPath(dagPath, component) ){
			cerr << "Error getting the dag path" << endl;
			continue;
		}
		dagPath.extendToShape();
		MFnDagNode dagNode( dagPath.node() );
		if(dagNode.typeId() != PCEPointCloudShape::id)
			continue;
		PCEPointCloudShape* shape = dynamic_cast<PCEPointCloudShape*>(dagNode.userNode());
		if(!shape)
		{
			cerr << "Error getting the shape in cmd" << endl;
			continue;
		}
		PCEPointCloud* cloud = shape->meshGeom();
		if(!cloud)
		{
			cerr << "Error getting the point cloud data in cmd" << endl;
			continue;
		}

		PCEIndicesPtr indices( new std::vector<int>);
		if ( !component.isNull() )
		{
			// Has component
			if(component.apiType() != MFn::kMeshVertComponent)
				continue;

			MFnSingleIndexedComponent vtxComp(component);
			for (int i = 0 ; i < vtxComp.elementCount(); ++i )
			{
				// Check the invalid points
				int index = vtxComp.element(i);
				PCEPointT pnt = cloud->pntCloud[index];
				if (!pcl_isfinite (pnt.x) || 
					!pcl_isfinite (pnt.y) || 
					!pcl_isfinite (pnt.z))
					continue;
				indices->push_back(index);
			}
		}

		if(isRecoverData)
			recoverData(cloud, indices);
		else
			compute(cloud, indices);
		//Update the viewport
		MHWRender::MRenderer::setGeometryDrawDirty(shape->thisMObject());
	}

	return MS::kSuccess;
}

/*virtual*/
bool PCEPointCloudBaseCmd::needRecoverData()
{
	return false;
}

void PCEPointCloudBaseCmd::recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices)
{
}
