//-
// Copyright 2012 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk license agreement
// provided at the time of installation or download, or which otherwise
// accompanies this software in either electronic or hard copy form.
//+

#include "PCEPointCloudTextureOverride.h"
#include "PCEPointCloudTextureNode.h"
#include "../PointCloudShape/PCEPointCloudData.h"

#include <maya/MString.h>
#include <maya/MFragmentManager.h>
#include <maya/MFnDependencyNode.h>

PCEPointCloudTextureOverride::PCEPointCloudTextureOverride(const MObject& obj)
: MPxShadingNodeOverride(obj)
, fFragmentName("")
, fObject(obj)
, m_customTextureData(NULL)
{
	// Fragments are defined in separate XML files, add the checker node
	// directory to the search path and load from the files.
	static const MString sFragmentName("pointCloudTextureNodeFragment");
	static const MString sFragmentOutputName("pointCloudTextureNodeFragmentOutput");
	static const MString sFragmentGraphName("pointCloudTextureNodeGraph");
	MHWRender::MRenderer* theRenderer = MHWRender::MRenderer::theRenderer();
	if (theRenderer)
	{
		MHWRender::MFragmentManager* fragmentMgr =
			theRenderer->getFragmentManager();
		if (fragmentMgr)
		{
			if (fragmentMgr->hasFragment(fFragmentName))
			{
				return;
			}

			// Add search path (once only)
			static bool sAdded = false;
			if (!sAdded)
			{
				static const MString loc(
					MString(getenv("MAYA_LOCATION")) +
					MString("/devkit/myPluginShaders"));
				fragmentMgr->addFragmentPath(loc);
				sAdded = true;
			}

			// Add fragments if needed
			bool fragAdded = fragmentMgr->hasFragment(sFragmentName);
			bool structAdded = fragmentMgr->hasFragment(sFragmentOutputName);
			bool graphAdded = fragmentMgr->hasFragment(sFragmentGraphName);
			if (!fragAdded)
			{
				fragAdded = (sFragmentName == fragmentMgr->addShadeFragmentFromFile(sFragmentName + ".xml", false));
			}
			if (!structAdded)
			{
				structAdded = (sFragmentOutputName == fragmentMgr->addShadeFragmentFromFile(sFragmentOutputName + ".xml", false));
			}
			if (!graphAdded)
			{
				graphAdded = (sFragmentGraphName == fragmentMgr->addFragmentGraphFromFile(sFragmentGraphName + ".xml"));
			}

			// If we have them all, use the final graph for the override
			if (fragAdded && structAdded && graphAdded)
			{
				fFragmentName = sFragmentGraphName;
			}
		}
	}
}

MHWRender::MPxShadingNodeOverride* PCEPointCloudTextureOverride::Creator(const MObject& obj)
{
	return new PCEPointCloudTextureOverride(obj);
}

PCEPointCloudTextureOverride::~PCEPointCloudTextureOverride()
{
	// clean custom texture data:
	delete m_customTextureData;
	m_customTextureData = NULL;
}

MString PCEPointCloudTextureOverride::fragmentName() const
{
	return fFragmentName;
}

void PCEPointCloudTextureOverride::updateDG()
{
	MStatus status;
	MFnDependencyNode node(fObject, &status);
	if (status == MStatus::kSuccess)
	{
		PCEPointCloudTextureNode* material = dynamic_cast<PCEPointCloudTextureNode*>(node.userNode());

		generateCustomTexture( material->getTextureData() );
	}
}

MHWRender::DrawAPI PCEPointCloudTextureOverride::supportedDrawAPIs() const
{
	return MHWRender::kOpenGL | MHWRender::kDirectX11;
}

/*!
	Update shader parameters for specific mappings
*/
void PCEPointCloudTextureOverride::updateShader(
	MHWRender::MShaderInstance& shader,
	const MHWRender::MAttributeParameterMappingList& mappings)
{
	if (fFragmentName.length() == 0)
		return;

	buildAndUpdateCustomDataTextureViaMaya(shader);
}

void PCEPointCloudTextureOverride::buildAndUpdateCustomDataTextureViaMaya(MHWRender::MShaderInstance& shader)
{
	if(!m_customTextureData)
		return;

	MHWRender::MRenderer *renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer)
		return;

	MHWRender::MTextureManager* mt = renderer->getTextureManager();

	MString newTextureName = MString("PointCloudTexture");
	MHWRender::MTexture* pointCloudTex = mt->acquireTexture(newTextureName, m_desc, m_customTextureData, false);

	if (pointCloudTex)
	{
		MHWRender::MTextureAssignment texResource;
		texResource.texture = pointCloudTex;
		MStatus status = shader.setParameter("floatMap", texResource);
	}
}

void PCEPointCloudTextureOverride::generateCustomTexture(PCEPointCloud* geomPtr)
{
	int numSlice = 1; // Number of array slices. e.g. 6 would be required for a cube-map

	// First time, we create a contiguous block of data for the texture.
	if (!m_customTextureData && geomPtr)
	{
		int width = geomPtr->pntCloud.width;
		int height = geomPtr->pntCloud.height;
		m_customTextureData = new unsigned char[4*width*height*numSlice];

		// init values
		// do whatever you would do with your custom texture data. E.g. generate procedural texture.
		float intensity = 1.0f;
		int index = 0;
		for (int face=0 ; face<numSlice ; face++)
		{
			for (int j=0 ; j<height ; j++)
			{
				for (int i=0 ; i<width ; i++)
				{
					int ix = 4 * index;
					m_customTextureData[ix] = (unsigned char)(geomPtr->pntCloud.at(index).r);
					m_customTextureData[ix+1] = (unsigned char)(geomPtr->pntCloud.at(index).g);
					m_customTextureData[ix+2] = (unsigned char)(geomPtr->pntCloud.at(index).b);
					m_customTextureData[ix+3] = 255;
					index ++;
				}
			}
		}

		// we still have to setup a texture description for Maya, even when using custom data:
		m_desc.setToDefault2DTexture();
		m_desc.fWidth = width;
		m_desc.fHeight = height;
		m_desc.fDepth = 1;
		m_desc.fBytesPerRow = 4*width;
		m_desc.fBytesPerSlice = 4*width*height;
		m_desc.fMipmaps = 1;
		m_desc.fArraySlices = numSlice;
		m_desc.fFormat = MHWRender::kR8G8B8A8_UNORM;
		m_desc.fTextureType = MHWRender::kImage2D;
	}
}
