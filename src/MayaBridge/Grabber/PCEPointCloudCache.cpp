//
// Copyright (C) Tingzhu Zhou
// 
// File: pointCloudShape.cpp
//
// Author: Tingzhu Zhou
//

#include "PCEPointCloudCache.h"

PCEPointCloudCache::PCEPointCloudCache()
{
}
PCEPointCloudCache::~PCEPointCloudCache()
{
	clear();
}


void PCEPointCloudCache::clear()
{
	for (std::vector<PCEPointCloud*>::iterator iter=m_arrPointCloud.begin();iter!=m_arrPointCloud.end();++iter)
	{
		PCEPointCloud* pData = *iter;
		if(pData)
			delete pData;
	}
	m_arrPointCloud.clear();
}

size_t PCEPointCloudCache::size() const
{
	return m_arrPointCloud.size();
}

void PCEPointCloudCache::pushBack(PCEPointCloud* frame)
{
	if(frame)
		m_arrPointCloud.push_back(frame);
}

PCEPointCloud* PCEPointCloudCache::getFrame(int frameId)
{
	if(frameId < size() )
		return m_arrPointCloud.at(frameId);

	return NULL;
}
