#ifndef _PCEPointCloudSegmentCmd
#define _PCEPointCloudSegmentCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudSegmentCmd.h
//
// Dependency Graph Node:  PCEPointCloudSegmentCmd
//
// Author: Tingzhu Zhou
//
#include "PCEPointCloudBaseCmd.h"

class PCEPointCloudSegmentCmd : public PCEPointCloudBaseCmd
{
public:

	PCEPointCloudSegmentCmd();
	virtual ~PCEPointCloudSegmentCmd();

	static void* creator();
	static MSyntax newSyntax();

	static const MString kPointCloudSegmentCmd;

protected:
	virtual MStatus parseArgs( const MArgList& args);
	virtual void compute(PCEPointCloud* cloud, PCEIndicesPtr indices);
	virtual bool needRecoverData();
	virtual void recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices);

private:
	int _methodValue;
	int _min;
	int _max;
	double _tolerance;
};

#endif