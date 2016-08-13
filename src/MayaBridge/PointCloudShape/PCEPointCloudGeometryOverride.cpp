//
// Copyright (C) Tingzhu Zhou
// 
// File: pointCloudShape.h
//
// Dependency Graph Node: pointCloudShape
//
// Author: Tingzhu Zhou
//

///////////////////////////////////////////////////////////////////////////////
//
// pointCloudGeometryOverride.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudGeometryOverride.h"
#include "PCEPointCloudShape.h"
#include "Shape/PCEPointCloud.h"

#include <maya/MFnDagNode.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MHWGeometry.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MObjectArray.h>
#include <maya/MShaderManager.h>

#include <maya/MGlobal.h>

#define SCALE_FACTOR 1.0;

const MString PCEPointCloudGeometryOverride::sBoundingBoxItemName = "BoundingBoxItem";
const MString PCEPointCloudGeometryOverride::sVertexItemName = "pointCloudVertices";
const MString PCEPointCloudGeometryOverride::sVertexNormalItemName = "pointCloudVertexNormals";
const MString PCEPointCloudGeometryOverride::sActiveVertexItemName = "pointCloudActiveVertices";
const MString PCEPointCloudGeometryOverride::sClusterItemName = "pointCloudClusters";
const MString PCEPointCloudGeometryOverride::sFirstPlayerItemName = "pointCloudFirstPlayer";
const MString PCEPointCloudGeometryOverride::sSecondPlayerItemName = "pointCloudSecondPlayer";

PCEPointCloudGeometryOverride::PCEPointCloudGeometryOverride(const MObject& obj)
	: MPxGeometryOverride(obj)
	, fShape(NULL)
{
	// get the real apiMesh object from the MObject
	MStatus status;
	MFnDependencyNode node(obj, &status);
	if (status)
	{
		fShape = dynamic_cast<PCEPointCloudShape*>(node.userNode());
	}
}

PCEPointCloudGeometryOverride::~PCEPointCloudGeometryOverride()
{
	fShape = NULL;
}

MHWRender::DrawAPI PCEPointCloudGeometryOverride::supportedDrawAPIs() const
{
	// this plugin supports both GL and DX
	return (MHWRender::kOpenGL | MHWRender::kDirectX11);
}

void PCEPointCloudGeometryOverride::updateDG()
{
	if(!fShape)
		return;

	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;
	if(!geomPtr->indicesClusters.empty())
		fClusterIndices = geomPtr->indicesClusters.at(0);//PCL_TODO
	// Pull the actual outMesh from the shape, as well
	// as any active components
	fActiveVertices.clear();
	if (fShape)
	{
		if (fShape->hasActiveComponents())
		{
			MObjectArray clist = fShape->activeComponents();
			if (clist.length())
			{
				MFnSingleIndexedComponent fnComponent( clist[0] );
				if (fnComponent.elementCount())
				{
					fnComponent.getElements( fActiveVertices );
				}
			}
		}
	}
}

/*
	Test to see if active components should be enabled.
	Based on active vertices + non-template state
*/
bool PCEPointCloudGeometryOverride::enableActiveComponentDisplay(const MDagPath &path) const
{
	bool enable = true;

	// If no active components then disable the active
	// component render item
	if (fActiveVertices.length() == 0)
	{
		enable = false;
	}
	else
	{
		// If there are components then we need to check
		// either the display status of the object, or
		// in the case of a templated object make sure
		// to hide components to be consistent with how
		// internal objects behave
		//
		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);
		if (displayStatus == MHWRender::kTemplate ||
			displayStatus == MHWRender::kActiveTemplate)
		{
			enable = false;
		}
		else
		{
			// Do an explicit path test for templated
			// since display status does not indicate this.
			if (path.isTemplated())
				enable = false;
		}
	}
	return enable;
}

/*
	Set the point size for solid color shaders
*/
void PCEPointCloudGeometryOverride::setSolidPointSize(MHWRender::MShaderInstance* shaderInstance, float pointSize)
{
	if (!shaderInstance)
		return;

	float pointSizeArray[2] = {0};
	pointSizeArray[0] = pointSize;
	pointSizeArray[1] = pointSize;

	const MString pointSizeParameterName = "pointSize";
	shaderInstance->setParameter( pointSizeParameterName, pointSizeArray );
}

void PCEPointCloudGeometryOverride::updateSelectedBoundingBoxRenderItem(
	const MDagPath& path, MHWRender::MRenderItemList& list)
{
	// Update render item
	int index = list.indexOf(sBoundingBoxItemName);
	if(index < 0)
		return;

	MHWRender::MRenderItem* boundingBoxItem = list.itemAt(index);
	if( !boundingBoxItem ) return;

	boundingBoxItem->setDrawMode(MHWRender::MGeometry::kAll);
	MHWRender::DisplayStatus displayStatus =
		MHWRender::MGeometryUtilities::displayStatus(path);
	switch (displayStatus) {
		case MHWRender::kActive:
		case MHWRender::kActiveAffected:
		case MHWRender::kLead:
		case MHWRender::kTemplate:
		case MHWRender::kActiveTemplate:
		case MHWRender::kIntermediateObject:
			boundingBoxItem->enable(true);
			break;
		default:
			boundingBoxItem->enable(false);
			break;
	}
}

/*
	Create a render item for dormant vertices if it does not exist. Updating
	shading parameters as necessary.
*/
void PCEPointCloudGeometryOverride::updateVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	MHWRender::MRenderItem* vertexItem = NULL;
	int index = list.indexOf(sVertexItemName);
	if (index < 0)
	{
		vertexItem = MHWRender::MRenderItem::Create(
			sVertexItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		vertexItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		vertexItem->depthPriority( MHWRender::MRenderItem::sDormantWireDepthPriority );
		list.append(vertexItem);
	}
	else
	{
		vertexItem = list.itemAt(index);
	}

	if (vertexItem)
	{
		bool useLighting = fShape->getBooleanValue(PCEPointCloudShape::aUseLighting);
		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			useLighting ? MHWRender::MShaderManager::k3dShadedPointCloudShader :
			MHWRender::MShaderManager::k3dCPVFatPointShader );
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			// assign shader
			vertexItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}

		MHWRender::DisplayStatus displayStatus =
			MHWRender::MGeometryUtilities::displayStatus(path);

		// Generally if the display status is hilite then we
		// draw components.
		if (displayStatus == MHWRender::kHilite)
		{
			// In case the object is templated
			// we will hide the components to be consistent
			// with how internal objects behave.
			if (path.isTemplated())
				vertexItem->enable(false);
			else
				vertexItem->enable(true);
		}
		else
		{
			vertexItem->enable(true);
		}
	}
}

/*
	Create a render item for active vertices if it does not exist. Updating
	shading parameters as necessary.
*/
void PCEPointCloudGeometryOverride::updateActiveVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr, bool enableActiveDisplay)
{
	MHWRender::MRenderItem* activeItem = NULL;
	int index = list.indexOf(sActiveVertexItemName);
	if (index < 0)
	{
		activeItem = MHWRender::MRenderItem::Create(
			sActiveVertexItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		activeItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority to be active point. This should offset it
		// to be visible above items with "dormant point" priority.
		activeItem->depthPriority( MHWRender::MRenderItem::sActivePointDepthPriority );
		list.append(activeItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			// Set the point size parameter. Make it slightly larger for active vertices
			static const float pointSize = 5.0f;
			setSolidPointSize( shader, pointSize );

			// Assign shader. Use a named stream if we want to supply a different
			// set of "shared" vertices for drawing active vertices
			activeItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		activeItem = list.itemAt(index);
	}

	if (activeItem)
	{
		MHWRender::MShaderInstance* shader = activeItem->getShader();
		if (shader)
		{
			// Set active color
			static const float theColor[] = { 1.0f, 1.0f, 0.0f, 1.0f };
			shader->setParameter("solidColor", theColor);
		}

		activeItem->enable( enableActiveDisplay );
	}
}

/*
	Create a render item for vertex normals if it does not exist. Updating
	shading parameters as necessary.
*/
void PCEPointCloudGeometryOverride::updateVertexNormalsItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	MHWRender::MRenderItem* vertexNormalItem = NULL;
	int index = list.indexOf(sVertexNormalItemName);
	if (index < 0)
	{
		vertexNormalItem = MHWRender::MRenderItem::Create(
			sVertexNormalItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		vertexNormalItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		vertexNormalItem->depthPriority( MHWRender::MRenderItem::sDormantWireDepthPriority );
		list.append(vertexNormalItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dPointNormalShader );
		if (shader)
		{
			// Set the point size parameter
			static const float pointSize = 3.0f;
			setSolidPointSize( shader, pointSize );

			// assign shader
			vertexNormalItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		vertexNormalItem = list.itemAt(index);
	}

	if (vertexNormalItem)
	{
		if(!fShape->getBooleanValue(PCEPointCloudShape::aShowNormals))
		{
			vertexNormalItem->enable(false);
			return;
		}
		// Set parameters
		MHWRender::MShaderInstance* shader = vertexNormalItem->getShader();
		if (shader)
		{
			float scale = fShape->getFloatValue(PCEPointCloudShape::aNormalLength);
			const MString vectorSclaeParameterName = "vectorScale";
			shader->setParameter( vectorSclaeParameterName, scale );

			static const float theColor[] = { 0.0f, 0.0f, 1.0f, 1.0f };
			const MString colorParameterName = "solidColor";
			shader->setParameter( colorParameterName, theColor );
		}
		// Enable
		vertexNormalItem->enable(true);
	}
}

/*
	Create a render item for vertex normals if it does not exist. Updating
	shading parameters as necessary.
*/
void PCEPointCloudGeometryOverride::updateClusterItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	MHWRender::MRenderItem* clusterItem = NULL;
	int index = list.indexOf(sClusterItemName);
	if (index < 0)
	{
		clusterItem = MHWRender::MRenderItem::Create(
			sClusterItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		clusterItem->setDrawMode((MHWRender::MGeometry::DrawMode)(MHWRender::MGeometry::kShaded | MHWRender::MGeometry::kTextured));
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		clusterItem->depthPriority( MHWRender::MRenderItem::sActivePointDepthPriority );
		list.append(clusterItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			shader->setIsTransparent(true);
			// assign shader
			clusterItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		clusterItem = list.itemAt(index);
	}

	if (clusterItem)
	{
		if(!fShape->getBooleanValue(PCEPointCloudShape::aShowCluster) || fClusterIndices.indices.empty())
		{
			clusterItem->enable(false);
			return;
		}
		// Set parameters
		MHWRender::MShaderInstance* shader = clusterItem->getShader();
		if (shader)
		{
			// Set the point size parameter
			const float pointSize = 5.0f;
			setSolidPointSize( shader, pointSize );

			static const float theColor[] = { 1.0f, 0.0f, 1.0f, 0.3f };
			const MString colorParameterName = "solidColor";
			shader->setParameter( colorParameterName, theColor );
		}
		// Enable
		clusterItem->enable(true);
	}
}

void PCEPointCloudGeometryOverride::updateFirstPlayerItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	if(!fShape)
		return;

	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* firstPlayerItem = NULL;
	int index = list.indexOf(sFirstPlayerItemName);
	if (index < 0)
	{
		firstPlayerItem = MHWRender::MRenderItem::Create(
			sFirstPlayerItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		firstPlayerItem->setDrawMode(MHWRender::MGeometry::kWireframe);
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		firstPlayerItem->depthPriority( MHWRender::MRenderItem::sActiveWireDepthPriority );
		list.append(firstPlayerItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			shader->setIsTransparent(true);
			// assign shader
			firstPlayerItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		firstPlayerItem = list.itemAt(index);
	}

	if (firstPlayerItem)
	{
		if(0 == geomPtr->getFirstPlayerIndexSize())
		{
			firstPlayerItem->enable(false);
			return;
		}
		// Set parameters
		MHWRender::MShaderInstance* shader = firstPlayerItem->getShader();
		if (shader)
		{
			// Set the point size parameter
			const float pointSize = 4.0f;
			setSolidPointSize( shader, pointSize );

			static const float theColor[] = { 1.0f, 1.0f, 0.0f, 0.2f };
			const MString colorParameterName = "solidColor";
			shader->setParameter( colorParameterName, theColor );
		}
		// Enable
		firstPlayerItem->enable(true);
	}
}

void PCEPointCloudGeometryOverride::updateSecondPlayerItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr)
{
	if(!fShape)
		return;

	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MRenderItem* secondPlayerItem = NULL;
	int index = list.indexOf(sSecondPlayerItemName);
	if (index < 0)
	{
		secondPlayerItem = MHWRender::MRenderItem::Create(
			sSecondPlayerItemName,
			MHWRender::MRenderItem::DecorationItem,
			MHWRender::MGeometry::kPoints);
		secondPlayerItem->setDrawMode(MHWRender::MGeometry::kWireframe);
		// Set depth priority higher than wireframe and shaded render items,
		// but lower than active points.
		// Raising higher than wireframe will make them not seem embedded into the surface
		secondPlayerItem->depthPriority( MHWRender::MRenderItem::sActiveWireDepthPriority );
		list.append(secondPlayerItem);

		MHWRender::MShaderInstance* shader = shaderMgr->getStockShader(
			MHWRender::MShaderManager::k3dFatPointShader );
		if (shader)
		{
			shader->setIsTransparent(true);
			// assign shader
			secondPlayerItem->setShader(shader);

			// once assigned, no need to hold on to shader instance
			shaderMgr->releaseShader(shader);
		}
	}
	else
	{
		secondPlayerItem = list.itemAt(index);
	}

	if (secondPlayerItem)
	{
		if(0 == geomPtr->getSecondPlayerIndexSize())
		{
			secondPlayerItem->enable(false);
			return;
		}
		// Set parameters
		MHWRender::MShaderInstance* shader = secondPlayerItem->getShader();
		if (shader)
		{
			// Set the point size parameter
			const float pointSize = 4.0f;
			setSolidPointSize( shader, pointSize );

			static const float theColor[] = { 0.0f, 1.0f, 1.0f, 0.2f };
			const MString colorParameterName = "solidColor";
			shader->setParameter( colorParameterName, theColor );
		}
		// Enable
		secondPlayerItem->enable(true);
	}
}

/*
	Update render items. Shaded render item is provided so this
	method will be adding and updating UI render items only.
*/
void PCEPointCloudGeometryOverride::updateRenderItems(
	const MDagPath& path,
	MHWRender::MRenderItemList& list)
{
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer) return;
	const MHWRender::MShaderManager* shaderMgr = renderer->getShaderManager();
	if (!shaderMgr) return;

	// Disable all the items
	for (int i = 0;i < list.length(); i++)
	{
		list.itemAt(i)->enable(false);
	}

	if(!fShape)
		return;
	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	int numVerts = geomPtr->pntCloud.size();
	if(0 == numVerts)
		return;
	
	bool enableActiveDisplay = enableActiveComponentDisplay(path);

	// Update vertex render items
	updateSelectedBoundingBoxRenderItem(path, list);
	updateVerticesItem(path, list, shaderMgr);
	updateActiveVerticesItem(path, list, shaderMgr, enableActiveDisplay);
	updateVertexNormalsItem(path, list, shaderMgr);
	updateClusterItem(path, list, shaderMgr);
	updateFirstPlayerItem(path, list, shaderMgr);
	updateSecondPlayerItem(path, list, shaderMgr);
}

/*
	Examine the geometry requirements and create / update the
	appropriate data streams. As render items specify both named and
	unnamed data streams, both need to be handled here.
*/
void PCEPointCloudGeometryOverride::updateGeometryRequirements(
		const MHWRender::MGeometryRequirements& requirements,
		MHWRender::MGeometry& data,
		bool debugPopulateGeometry)
{
	if(!fShape)
		return;
	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	// Get the active vertex count
	unsigned int activeVertexCount = fActiveVertices.length();
	int numVerts = geomPtr->pntCloud.size();

	// Vertex data
	MHWRender::MVertexBuffer* positionBuffer = NULL;
	float* positions = NULL;
	MHWRender::MVertexBuffer* normalBuffer = NULL;
	float* normals = NULL;
	MHWRender::MVertexBuffer* cpvBuffer = NULL;
	float* cpv = NULL;
	
	const MHWRender::MVertexBufferDescriptorList& descList =
		requirements.vertexRequirements();
	int numVertexReqs = descList.length();
	MHWRender::MVertexBufferDescriptor desc;
	for (int reqNum=0; reqNum<numVertexReqs; reqNum++)
	{
		if (!descList.getDescriptor(reqNum, desc))
		{
			continue;
		}

		// Fill vertex stream data used for dormant vertex, wireframe and shaded drawing.
		// Fill also for active vertices if (fDrawSharedActiveVertices=false)
		if (debugPopulateGeometry)
		{
			printf(">>> Fill in data for requirement[%d] with name %s. Semantic = %d\n",
				reqNum, desc.name().asChar(), desc.semantic() );
		}
		switch (desc.semantic())
		{
		case MHWRender::MGeometry::kPosition:
			{
				if (!positionBuffer)
				{
					positionBuffer = data.createVertexBuffer(desc);
					if (positionBuffer)
					{
						if (debugPopulateGeometry)
							printf("Acquire unnamed position buffer\n");
						positions = (float*)positionBuffer->acquire(numVerts, true /*writeOnly - we don't need the current buffer values*/);
					}
				}
			}
			break;
		case MHWRender::MGeometry::kNormal:
			{
				if (!normalBuffer)
				{
					normalBuffer = data.createVertexBuffer(desc);
					if (normalBuffer)
					{
						normals = (float*)normalBuffer->acquire(numVerts, true /*writeOnly - we don't need the current buffer values*/);
					}
				}
			}
			break;
		case MHWRender::MGeometry::kColor:
			{
				if (!cpvBuffer)
				{
					cpvBuffer = data.createVertexBuffer(desc);
					if (cpvBuffer)
					{
						cpv = (float*)cpvBuffer->acquire(numVerts, true /*writeOnly - we don't need the current buffer values*/);
					}
				}
			}
			break;
		default:
			// do nothing for stuff we don't understand
			break;
		}
	}

	int pid = 0;
	int nid = 0;
	int cid = 0;
#pragma omp parallel for schedule(dynamic)
	for (int i=0; i<numVerts; i++)
	{
		const PCEPointT& pnt = geomPtr->pntCloud.at( i );
		if (positions)
		{
			// Position used as position
			if (positions)
			{
				positions[pid] = (float)pnt.x * SCALE_FACTOR;
				positions[pid+1] = (float)pnt.y * SCALE_FACTOR;
				positions[pid+2] = (float)pnt.z * SCALE_FACTOR;
			}
			pid += 3;
		}

		if (normals)
		{
			normals[nid++] = (float)pnt.normal_x;
			normals[nid++] = (float)pnt.normal_y;
			normals[nid++] = (float)pnt.normal_z;
		}

		// color-per-vertex (CPV)
		if (cpv)
		{
			cpv[cid++] = (float)pnt.r/255;
			cpv[cid++] = (float)pnt.g/255;
			cpv[cid++] = (float)pnt.b/255;
			cpv[cid++] = (float)1.0;//PCL_TODO:ALPHA
		}
	}

	if (positions)
	{
		positionBuffer->commit(positions);
	}

	if (normals)
	{
		normalBuffer->commit(normals);
	}
	if (cpv)
	{
		cpvBuffer->commit(cpv);
	}
}

/*
	Create / update indexing for render items which draw active vertices
*/
void PCEPointCloudGeometryOverride::updateIndexingForActiveVertices(const MHWRender::MRenderItem* item,
															  MHWRender::MGeometry& data)
{
	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		// Get the active vertex count
		unsigned int activeVertexCount = fActiveVertices.length();
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(activeVertexCount, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			for (int idx=0; idx<activeVertexCount; idx++)
			{
				buffer[idx] = fActiveVertices[idx];
			}

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}


/*
	Create / update indexing for render items which draw dormant vertices
*/
void PCEPointCloudGeometryOverride::updateIndexingForDormantVertices(const MHWRender::MRenderItem* item,
															  MHWRender::MGeometry& data)
{
	if(!fShape)
		return;
	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		bool needInvalid = fShape->getBooleanValue(PCEPointCloudShape::aShowInvalid);

		int numVerts = needInvalid ? geomPtr->getCloudSize() : geomPtr->getValidIndexSize();
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(numVerts, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			for (int idx=0; idx<numVerts; idx++)
			{
				buffer[idx] = needInvalid ? idx : geomPtr->getValidIndex(idx);
			}

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}

/*
	Create / update indexing for render items which draw cluster
*/
void PCEPointCloudGeometryOverride::updateIndexingForCluster(const MHWRender::MRenderItem* item,
															  MHWRender::MGeometry& data)
{
	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		// Get the active vertex count
		unsigned int clusterCount = fClusterIndices.indices.size();
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(clusterCount, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			for (int idx=0; idx<clusterCount; idx++)
			{
				buffer[idx] = fClusterIndices.indices.at(idx);
			}

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}

void PCEPointCloudGeometryOverride::updateIndexingForFirstPlayer(const MHWRender::MRenderItem* item,
	MHWRender::MGeometry& data)
{
	if(!fShape)
		return;
	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	unsigned int indexCount = geomPtr->getFirstPlayerIndexSize();
	if(0 == indexCount)
		return;

	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		// Get the active vertex count
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(indexCount, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			for (int idx=0; idx<indexCount; idx++)
			{
				buffer[idx] = geomPtr->getFirstPlayerIndex(idx);
			}

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}

void PCEPointCloudGeometryOverride::updateIndexingForSecondPlayer(const MHWRender::MRenderItem* item,
	MHWRender::MGeometry& data)
{
	if(!fShape)
		return;
	PCEPointCloud* geomPtr = fShape->meshGeom();
	if(!geomPtr)
		return;

	unsigned int indexCount = geomPtr->getSecondPlayerIndexSize();
	if(0 == indexCount)
		return;

	MHWRender::MIndexBuffer* indexBuffer =
		data.createIndexBuffer(MHWRender::MGeometry::kUnsignedInt32);
	if (indexBuffer)
	{
		// Get the active vertex count
		unsigned int* buffer = (unsigned int*)indexBuffer->acquire(indexCount, true /*writeOnly - we don't need the current buffer values*/);
		if (buffer)
		{
			for (int idx=0; idx<indexCount; idx++)
			{
				buffer[idx] = geomPtr->getSecondPlayerIndex(idx);
			}

			indexBuffer->commit(buffer);
			item->associateWithIndexBuffer(indexBuffer);
		}
	}
}

/*
	Fill in data and index streams based on the requirements passed in.
	Associate indexing with the render items passed in.

	Note that we leave both code paths to either draw shared or non-shared active vertices.
	The choice of which to use is up to the circumstances per plug-in.
	When drawing shared vertices, this requires an additional position buffer to be
	created so will use more memory. If drawing unshared vertices redundent extra
	vertices are drawn but will use less memory. The data member fDrawSharedActiveVertices
	can be set to decide on which implementation to use.
*/
void PCEPointCloudGeometryOverride::populateGeometry(
	const MHWRender::MGeometryRequirements& requirements,
    const MHWRender::MRenderItemList& renderItems,
	MHWRender::MGeometry& data)
{
	static bool debugPopulateGeometry = false;
	if (debugPopulateGeometry)
		printf("> Begin populate geometry\n");

	/////////////////////////////////////////////////////////////////////
	// Update data streams based on geometry requirements
	/////////////////////////////////////////////////////////////////////
	updateGeometryRequirements(requirements, data,
		debugPopulateGeometry);

	int numItems = renderItems.length();
	for (int i=0; i<numItems; i++)
	{
        const MHWRender::MRenderItem* item = renderItems.itemAt(i);
		if (!item) continue;

		// Enable to debug vertex buffers that are associated with each render item.
		// Can also use to generate indexing better, but we don't need that here.
		// Also debugs custom data on the render item.
		static const bool debugStuff = false;
		if (debugStuff)
		{
			const MHWRender::MVertexBufferDescriptorList& itemBuffers =
				item->requiredVertexBuffers();
			int numBufs = itemBuffers.length();
			MHWRender::MVertexBufferDescriptor desc;
			for (int bufNum=0; bufNum<numBufs; bufNum++)
			{
				if (itemBuffers.getDescriptor(bufNum, desc))
				{
					printf("Buffer Required for Item #%d ('%s'):\n", i, item->name().asChar());
					printf("\tBufferName: %s\n", desc.name().asChar());
					printf("\tDataType: %s (dimension %d)\n", MHWRender::MGeometry::dataTypeString(desc.dataType()).asChar(), desc.dimension());
					printf("\tSemantic: %s\n", MHWRender::MGeometry::semanticString(desc.semantic()).asChar());
					printf("\n");
				}
			}

			// Just print a message for illustration purposes. Note that the custom data is also
			// accessible from the MRenderItem in MPxShaderOverride::draw().
			/*apiMeshUserData* myCustomData = dynamic_cast<apiMeshUserData*>(item->customData());
			if (myCustomData)
			{
				printf("Custom data on Item #%d: '%s', modified count='%d'\n\n", i, myCustomData->fMessage.asChar(), myCustomData->fNumModifications);
			}
			else
			{
				printf("No custom data on Item #%d\n\n", i);
			}*/
		}

		// Update indexing for active vertex item
		//
		if (item->name() == sActiveVertexItemName)
		{
			updateIndexingForActiveVertices( item, data);
		}

		// Create indexing for dormant vertex render items
		//
		else if (item->name() == sVertexItemName ||
			item->name() == sVertexNormalItemName)
		{
			updateIndexingForDormantVertices( item, data );
		}
		// Create indexing for cluster items
		//
		else if (item->name() == sClusterItemName)
		{
			updateIndexingForCluster( item, data );
		}
		else if (item->name() == sFirstPlayerItemName)
		{
			updateIndexingForFirstPlayer(item, data );
		}
		else if (item->name() == sSecondPlayerItemName)
		{
			updateIndexingForSecondPlayer(item, data );
		}
	}

	if (debugPopulateGeometry)
		printf("> End populate geometry\n");
}

void PCEPointCloudGeometryOverride::cleanUp()
{
	fActiveVertices.clear();
	fClusterIndices.indices.clear();
}
