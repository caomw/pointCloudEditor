#ifndef _PCEDataCacheManager
#define _PCEDataCacheManager
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEDataCacheManager.h
//
//
// Author: Tingzhu Zhou
//
#include "PCEDataPtrCache.h"
#include "Shape/PCEPointCloud.h"

class PCEDataCacheManager
{
public:
	~PCEDataCacheManager(){	fPointCloudCache.clear(); }

	static PCEDataCacheManager* getDataCacheManager()
	{
		if (sDataCacheManager == NULL)
		{
			sDataCacheManager = new PCEDataCacheManager();
		}

		return sDataCacheManager;
	}

	static void releaseDataCacheManager()
	{
		if(sDataCacheManager)
		{
			delete sDataCacheManager;
			sDataCacheManager = NULL;
		}
	}

	// For PointCloud
	PCEPointCloud* getPointCloudPtr(int frameId)
	{
		return fPointCloudCache.getDataPtr(frameId);
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
	PCEDataCacheManager(){};

	static PCEDataCacheManager* sDataCacheManager;

	PCEDataPtrCache<PCEPointCloud>	fPointCloudCache;
};



#endif
