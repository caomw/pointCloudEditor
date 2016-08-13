#ifndef _PCEPointCloudIndicesCmd
#define _PCEPointCloudIndicesCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudIndicesCmd.h
//
// Dependency Graph Node:  PCEPointCloudIndicesCmd
//
// Author: Tingzhu Zhou
//

#include "PCEPointCloudBaseCmd.h"

class PCEPointCloudIndicesCmd : public PCEPointCloudBaseCmd
{
public:

	PCEPointCloudIndicesCmd();
	virtual ~PCEPointCloudIndicesCmd();

	static void* creator();
	static MSyntax newSyntax();

	static const MString kPointCloudIndicesCmd;
protected:
	virtual bool needRecoverData();
	virtual void recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices);
	virtual MStatus parseArgs( const MArgList& args);
	virtual void compute(PCEPointCloud* cloud, PCEIndicesPtr indices);

private:
	int _clearIndex;
	bool _clearAll;
};

#endif