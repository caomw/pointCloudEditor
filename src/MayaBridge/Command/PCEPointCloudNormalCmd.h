#ifndef _PCEPointCloudNormalCmd
#define _PCEPointCloudNormalCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudNormalCmd.h
//
// Dependency Graph Node:  PCEPointCloudNormalCmd
//
// Author: Tingzhu Zhou
//
#include "PCEPointCloudBaseCmd.h"

class PCEPointCloudNormalCmd : public PCEPointCloudBaseCmd
{
public:

	PCEPointCloudNormalCmd();
	virtual ~PCEPointCloudNormalCmd();

	static void* creator();
	static MSyntax newSyntax();
	
	static const MString kPointCloudNormalCmd;
protected:
	virtual bool needRecoverData();
	virtual MStatus parseArgs( const MArgList& args);
	virtual void compute(PCEPointCloud* cloud, PCEIndicesPtr indices);

private:
	int _methodValue;
	int _kNearest;
	double _searchRadius;
};

#endif