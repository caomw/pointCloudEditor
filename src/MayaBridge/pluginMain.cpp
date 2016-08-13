//
// Copyright (C) Tingzhu Zhou
// 
// File: pluginMain.cpp
//
// Author: Tingzhu Zhou
//

#include "Grabber/PCEPointCloudGrabber.h"
#include "Grabber/PCENITEGrabber.h"
#include "Grabber/PCEGrabberPreView.h"
#include "Grabber/PCEGrabberPreViewCmd.h"
#include "Grabber/PCEGrabberRenderOverride.h"
#include "PCEPointCloudToMesh.h"
#include "PCEPointCloudExtraction.h"
#include "PCEPointCloudFilter.h"
#include "Skeleton/PCESkeletonData.h"
#include "Skeleton/PCESkeletonDriver.h"
#include "Skeleton/PCEFaceTrackingData.h"
#include "Skeleton/PCEFaceTrackingDriver.h"
#include "PointCloudShape/PCEPointCloudData.h"
#include "PointCloudShape/PCEPointCloudShape.h"
#include "PointCloudShape/PCEPointCloudShapeUI.h"
#include "PointCloudShape/PCEPointCloudGeometryOverride.h"
#include "PointCloudTextureShader/PCEPointCloudTextureNode.h"
#include "PointCloudTextureShader/PCEPointCloudTextureOverride.h"
#include "Command/PCEPointCloudNormalCmd.h"
#include "Command/PCEPointCloudSegmentCmd.h"
#include "Command/PCEPointCloudKeyPointsCmd.h"
#include "Command/PCEPointCloudIndicesCmd.h"
#include "Command/PCEGrabberCmd.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MDrawRegistry.h>

#define	PLUGIN_VENDOR "Tingzhu"
#define	PLUGIN_VERSION "2015"

static MString sDrawDbClassification("drawdb/geometry/pointCloudShape");
static MString sDrawRegistrantId("pointCloudEditorPlugin");

static PCEGrabberRenderOverride* sGrabberRenderOverrideInstance = NULL;

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{ 
	MStatus stat1, stat2, stat3, stat4;
	MFnPlugin plugin( obj, PLUGIN_VENDOR, PLUGIN_VERSION, "Any");

	stat1 = plugin.registerData( "pointCloudData", PCEPointCloudData::id,
		&PCEPointCloudData::creator,
		MPxData::kGeometryData );
	if ( ! stat1 ) {
		cerr << "Failed to register geometry data : pointCloudData \n";
		return stat1;
	}

	stat2 = plugin.registerShape( "pointCloudShape", PCEPointCloudShape::id,
		&PCEPointCloudShape::creator,
		&PCEPointCloudShape::initialize,
		&PCEPointCloudShapeUI::creator,
		&sDrawDbClassification );
	if ( ! stat2 ) {
		cerr << "Failed to register shape : pointCloudShape\n";
		if ( stat1) plugin.deregisterData( PCEPointCloudData::id );
		return stat2;
	}

	stat3 = plugin.registerData( "skeletonData", PCESkeletonData::id,
		&PCESkeletonData::creator,
		MPxData::kData );
	if ( ! stat3 ) {
		cerr << "Failed to register geometry data : skeletonData \n";
		if ( stat2 ) {
			plugin.deregisterNode( PCEPointCloudShape::id );
			plugin.deregisterData( PCEPointCloudData::id );
		}
		return stat3;
	}

	stat3 = plugin.registerData( "faceTrackingData", PCEFaceTrackingData::id,
		&PCEFaceTrackingData::creator,
		MPxData::kData );
	if ( ! stat3 ) {
		cerr << "Failed to register geometry data : faceTrackingData \n";
		if ( stat2 ) {
			plugin.deregisterNode( PCEPointCloudShape::id );
			plugin.deregisterData( PCEPointCloudData::id );
		}
		return stat3;
	}

	stat4 = plugin.registerNode( "pointCloudGrabber", PCEPointCloudGrabber::id, PCEPointCloudGrabber::creator,
								  PCEPointCloudGrabber::initialize );
	if (!stat4) {
		stat4.perror("registerNode");
		if ( stat3 ) {
			plugin.deregisterNode( PCEPointCloudShape::id );
			plugin.deregisterData( PCEPointCloudData::id );
			plugin.deregisterData( PCESkeletonData::id );
			plugin.deregisterData( PCEFaceTrackingData::id );
		}
		return stat4;
	}

	stat4 = MHWRender::MDrawRegistry::registerGeometryOverrideCreator(
		sDrawDbClassification,
		sDrawRegistrantId,
		PCEPointCloudGeometryOverride::Creator);
	if (!stat4) {
		stat4.perror("Failed to register Viewport 2.0 geometry override\n");
		return stat4;
	}

	stat1 = plugin.registerNode( "skeletonDriver", PCESkeletonDriver::id, PCESkeletonDriver::creator,
		PCESkeletonDriver::initialize );
	if (!stat1) {
		stat1.perror("registerNode: skeleton driver node");
		return stat1;
	}

	stat1 = plugin.registerNode( "faceTrackingDriver", PCEFaceTrackingDriver::id, PCEFaceTrackingDriver::creator,
		PCEFaceTrackingDriver::initialize );
	if (!stat1) {
		stat1.perror("registerNode: face tracking driver node");
		return stat1;
	}

	stat1 = plugin.registerNode( "pointCloudToMesh", PCEPointCloudToMesh::id, PCEPointCloudToMesh::creator,
		PCEPointCloudToMesh::initialize );
	if (!stat1) {
		stat1.perror("registerNode pointCloudToMesh");
		return stat1;
	}

	stat1 = plugin.registerNode( "pointCloudExtraction", PCEPointCloudExtraction::id, PCEPointCloudExtraction::creator,
		PCEPointCloudExtraction::initialize );
	if (!stat1) {
		stat1.perror("registerNode pointCloudExtraction");
		return stat1;
	}

	stat1 = plugin.registerNode( "pointCloudFilter", PCEPointCloudFilter::id, PCEPointCloudFilter::creator,
		PCEPointCloudFilter::initialize );
	if (!stat1) {
		stat1.perror("registerNode PCEPointCloudFilter");
		return stat1;
	}

	stat1 = plugin.registerNode( "NITEGrabber", PCENITEGrabber::id, PCENITEGrabber::creator,
		PCENITEGrabber::initialize );
	if (!stat1) {
		stat1.perror("registerNode NITEGrabber");
		return stat1;
	}

	// pointCloudTextureShader
	MString TextureClassify = MString("texture/2d:");
	TextureClassify += PCEPointCloudTextureNode::m_drawDbClassification;

	// Register node
	stat1 = plugin.registerNode(
		PCEPointCloudTextureNode::m_TypeName,
		PCEPointCloudTextureNode::m_TypeId,
		PCEPointCloudTextureNode::creator,
		PCEPointCloudTextureNode::initialize,
		MPxNode::kDependNode,
		&TextureClassify);
	if (stat1 != MS::kSuccess)
	{
		stat1.perror("registerNode pointCloudTextureNode");
		return stat1;
	}

	// Register a shader override for this node
	stat1 = MHWRender::MDrawRegistry::registerShadingNodeOverrideCreator(
		PCEPointCloudTextureNode::m_drawDbClassification,
		sDrawRegistrantId,
		PCEPointCloudTextureOverride::Creator);
	if (stat1 != MS::kSuccess)
	{
		stat1.perror("registerShaderOverrideCreator");
		return stat1;
	}

	// Register override
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (renderer)
	{
		if (!sGrabberRenderOverrideInstance)
		{
			sGrabberRenderOverrideInstance = new PCEGrabberRenderOverride( PCEGrabberRenderOverride::kPCEGrabberPreviewerName );
			renderer->registerOverride(sGrabberRenderOverrideInstance);
		}
	}

	stat1 = plugin.registerModelEditorCommand(
		PCEGrabberPreViewCmd::kGrabberPreViewCmd,
		PCEGrabberPreViewCmd::creator,
		PCEGrabberPreView::creator);
	if (!stat1) {
		stat1.perror("registerCommand : PCEGrabberPreViewCmd");
		return stat1;
	}

	stat1 = plugin.registerCommand(  PCEPointCloudNormalCmd::kPointCloudNormalCmd,
		PCEPointCloudNormalCmd::creator,
		PCEPointCloudNormalCmd::newSyntax );
	if (!stat1) {
		stat1.perror("registerCommand : PointCloudNormalCmd");
		return stat1;
	}

	stat1 = plugin.registerCommand(  PCEPointCloudSegmentCmd::kPointCloudSegmentCmd,
		PCEPointCloudSegmentCmd::creator,
		PCEPointCloudSegmentCmd::newSyntax );
	if (!stat1) {
		stat1.perror("registerCommand : PCEPointCloudSegmentCmd");
		return stat1;
	}

	stat1 = plugin.registerCommand(  PCEPointCloudKeyPointsCmd::kPointCloudKeyPointsCmd,
		PCEPointCloudKeyPointsCmd::creator,
		PCEPointCloudKeyPointsCmd::newSyntax );
	if (!stat1) {
		stat1.perror("registerCommand : PCEPointCloudKeyPointsCmd");
		return stat1;
	}

	stat1 = plugin.registerCommand(  PCEPointCloudIndicesCmd::kPointCloudIndicesCmd,
		PCEPointCloudIndicesCmd::creator,
		PCEPointCloudIndicesCmd::newSyntax );
	if (!stat1) {
		stat1.perror("registerCommand : PCEPointCloudIndicesCmd");
		return stat1;
	}

	stat1 = plugin.registerCommand(  PCEGrabberCmd::kGrabberCmd,
		PCEGrabberCmd::creator,
		PCEGrabberCmd::newSyntax );
	if (!stat1) {
		stat1.perror("registerCommand : PCEPcdFileIOCmd");
		return stat1;
	}

	// set the mel procs to be run when the plugin is loaded / unloaded
	if (MGlobal::mayaState() == MGlobal::kInteractive) {
		// We have to explicitly source this script here. In
		// particular, it can't be done from the UI creation script
		// below. The UI creation script is executed by
		// TloadPluginAction through a deferred evaluation. The
		// deferred evaluation can cause the UI deletion script to be
		// executed before the creation UI one!
		stat4 = MGlobal::executeCommand("source pointCloudEditor");
		if (!stat4) {
			stat4.perror("pointCloudEditorInitUI");
			return stat4;
		}
	}

	return stat3;
}

MStatus uninitializePlugin( MObject obj)
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = MHWRender::MDrawRegistry::deregisterGeometryOverrideCreator(
		sDrawDbClassification,
		sDrawRegistrantId);
	if (!status) {
		status.perror("Failed to deregister sub-scene override : pointCloudGeometryOverride \n");
	}

	status = plugin.deregisterNode( PCEPointCloudShape::id );
	if ( ! status ) {
		cerr << "Failed to deregister shape : pointCloudShape \n";
	}

	status = plugin.deregisterData( PCEPointCloudData::id );
	if ( ! status ) {
		cerr << "Failed to deregister geometry data : pointCloudData \n";
	}

	status = plugin.deregisterNode( PCEPointCloudGrabber::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : pointCloudGrabber \n";
	}

	status = plugin.deregisterData( PCESkeletonData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : skeleton Data \n";
	}

	status = plugin.deregisterNode( PCESkeletonDriver::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : skeleton driver \n";
	}

	status = plugin.deregisterData( PCEFaceTrackingData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : face tracking Data \n";
	}

	status = plugin.deregisterNode( PCEFaceTrackingDriver::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : face tracking driver \n";
	}

	status = plugin.deregisterNode( PCENITEGrabber::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : NITEGrabber \n";
	}

	status = plugin.deregisterNode( PCEPointCloudToMesh::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : pointCloudToMesh \n";
	}

	status = plugin.deregisterNode( PCEPointCloudExtraction::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : pointCloudExtraction \n";
	}

	status = plugin.deregisterNode( PCEPointCloudFilter::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : PCEPointCloudFilter \n";
	}

	status = MHWRender::MDrawRegistry::deregisterShadingNodeOverrideCreator(
		PCEPointCloudTextureNode::m_drawDbClassification,
		sDrawRegistrantId);
	if (status != MS::kSuccess)
	{
		status.perror("deregisterShaderOverrideCreator");
		return status;
	}

	status = plugin.deregisterNode(PCEPointCloudTextureNode::m_TypeId);
	if (status != MS::kSuccess)
	{
		status.perror("deregisterNode : pointCloudTextureShader");
		return status;
	}

	status = plugin.deregisterModelEditorCommand( PCEGrabberPreViewCmd::kGrabberPreViewCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (renderer)
	{
		if (sGrabberRenderOverrideInstance)
		{
			renderer->deregisterOverride(sGrabberRenderOverrideInstance);
			delete sGrabberRenderOverrideInstance;
		}
		sGrabberRenderOverrideInstance = NULL;
	}

	status = plugin.deregisterCommand( PCEPointCloudNormalCmd::kPointCloudNormalCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	status = plugin.deregisterCommand( PCEPointCloudSegmentCmd::kPointCloudSegmentCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	status = plugin.deregisterCommand( PCEPointCloudKeyPointsCmd::kPointCloudKeyPointsCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	status = plugin.deregisterCommand( PCEPointCloudIndicesCmd::kPointCloudIndicesCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	status = plugin.deregisterCommand( PCEGrabberCmd::kGrabberCmd );
	if (!status) {
		status.perror("deregisterCommand");
	}

	return status;
}
