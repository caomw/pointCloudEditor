//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#ifndef POINTCLOUDTEXTUREOVERRIDE_H
#define POINTCLOUDTEXTUREOVERRIDE_H

#include <maya/MPxShadingNodeOverride.h>
#include <maya/MShaderManager.h>

class PCEPointCloud;
class PCEPointCloudTextureOverride : public MHWRender::MPxShadingNodeOverride
{
public:
	static MHWRender::MPxShadingNodeOverride* Creator(const MObject& obj);

	virtual ~PCEPointCloudTextureOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;
	virtual MString fragmentName() const;

	virtual void updateDG();
	virtual void updateShader(
		MHWRender::MShaderInstance& shader,
		const MHWRender::MAttributeParameterMappingList& mappings);

private:
	PCEPointCloudTextureOverride(const MObject& obj);

	void updateShaderInstance();
	void generateCustomTexture(PCEPointCloud* geomPtr);
	void buildAndUpdateCustomDataTextureViaMaya(MHWRender::MShaderInstance& shader);

	MObject fObject;
	MString fFragmentName;

	// custom texture data:
	unsigned char* m_customTextureData;
	MHWRender::MTextureDescription m_desc;
};

#endif