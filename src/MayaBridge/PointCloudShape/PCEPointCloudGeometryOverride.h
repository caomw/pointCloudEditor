//
// Copyright (C) Tingzhu Zhou
// 
// File: pointCloudShape.h
//
// Dependency Graph Node: pointCloudShape
//
// Author: Tingzhu Zhou
//

#ifndef _pointCloudGeometryOverride
#define _pointCloudGeometryOverride

///////////////////////////////////////////////////////////////////////////////
//
// pointCloudGeometryOverride.h
//
// Handles vertex data preparation for drawing the user defined shape in
// Viewport 2.0.
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxGeometryOverride.h>
#include <maya/MIntArray.h>
#include <maya/MStateManager.h>

#include <pcl/common/common.h>

class PCEPointCloudShape;
namespace MHWRender
{
	class MIndexBuffer;
}

class PCEPointCloudGeometryOverride : public MHWRender::MPxGeometryOverride
{
public:
	static MPxGeometryOverride* Creator(const MObject& obj)
	{
		return new PCEPointCloudGeometryOverride(obj);
	}

	virtual ~PCEPointCloudGeometryOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;

	virtual void updateDG();
	virtual void updateRenderItems(
		const MDagPath& path,
		MHWRender::MRenderItemList& list);
	virtual void populateGeometry(
		const MHWRender::MGeometryRequirements& requirements,
		const MHWRender::MRenderItemList& renderItems,
		MHWRender::MGeometry& data);
	virtual void cleanUp();

protected:
	PCEPointCloudGeometryOverride(const MObject& obj);

	void setSolidPointSize(MHWRender::MShaderInstance* shaderInstance, float value);
	bool enableActiveComponentDisplay(const MDagPath &path) const;

	// Render item handling methods
	void updateSelectedBoundingBoxRenderItem(const MDagPath& path, MHWRender::MRenderItemList& list);
	void updateVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateActiveVerticesItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr, bool enableActiveDisplay);
	void updateVertexNormalsItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateClusterItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateFirstPlayerItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);
	void updateSecondPlayerItem(const MDagPath& path, MHWRender::MRenderItemList& list, const MHWRender::MShaderManager* shaderMgr);

	// Data stream (geometry requirements) handling
	void updateGeometryRequirements(const MHWRender::MGeometryRequirements& requirements,
		MHWRender::MGeometry& data,
		bool debugPopulateGeometry);

	// Indexing for render item handling methods
	void updateIndexingForDormantVertices(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);
	void updateIndexingForActiveVertices(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);
	void updateIndexingForCluster(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);
	void updateIndexingForFirstPlayer(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);
	void updateIndexingForSecondPlayer(const MHWRender::MRenderItem* item, MHWRender::MGeometry& data);
	
	PCEPointCloudShape* fShape;
	MIntArray fActiveVertices;
	pcl::PointIndices fClusterIndices;

	// Render item names
	static const MString sBoundingBoxItemName;
	static const MString sVertexItemName;
	static const MString sVertexNormalItemName;
	static const MString sActiveVertexItemName;
	static const MString sClusterItemName;
	static const MString sFirstPlayerItemName;
	static const MString sSecondPlayerItemName;
};
#endif /* _pointCloudGeometryOverride */
