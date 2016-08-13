#ifndef _PCEPointCloudKeyPointsCmd
#define _PCEPointCloudKeyPointsCmd
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

class PCEPointCloudKeyPointsCmd : public PCEPointCloudBaseCmd
{
public:

	PCEPointCloudKeyPointsCmd();
	virtual ~PCEPointCloudKeyPointsCmd();

	static void* creator();
	static MSyntax newSyntax();

	static const MString kPointCloudKeyPointsCmd;
protected:
	virtual MStatus parseArgs( const MArgList& args);
	virtual void compute(PCEPointCloud* cloud, PCEIndicesPtr indices);
	virtual bool needRecoverData();
	virtual void recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices);

private:
	int _methodValue;
	double _min;
	double _max;
	double _searchRadius;
};

#endif
