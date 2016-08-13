#ifndef _PCEDataCache
#define _PCEDataCache
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDataCache.h
//
//
// Author: Tingzhu Zhou
//
#include "PCEDataPtrCache.h"
#include "src/Shape/PCEPointCloud.h"

class PCEDataCache
{
public:
	PCEDataCache(){};
	~PCEDataCache(){ clear(); }

	void clear(){	fPointCloudCache.clear(); }

	// For PointCloud
	PCEPointCloud* getPointCloudPtr(int frameId)
	{
		return fPointCloudCache.getDataPtr(frameId);
	}
	PCEPointCloud* getFrontPointCloudPtr()
	{
		return fPointCloudCache.getFrontDataPtr();
	}
	void pushbackPointCloudPtr(PCEPointCloud* pData)
	{
		fPointCloudCache.pushBack(pData);
	}
	size_t pointCloudPtrSize()
	{
		return fPointCloudCache.size();
	}

private:
	
	PCEDataPtrCache<PCEPointCloud>	fPointCloudCache;
};



#endif
