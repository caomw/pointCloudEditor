#ifndef _PCEPointCloudBaseCmd
#define _PCEPointCloudBaseCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPointCloudBaseCmd.h
//
// Dependency Graph Node:  PCEPointCloudBaseCmd
//
// Author: Tingzhu Zhou
//
#include <maya/MPxCommand.h>
#include <maya/MSelectionList.h>

#include <Eigen/StdVector>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr <std::vector<int> > PCEIndicesPtr;

class PCEPointCloud;
class PCEPointCloudBaseCmd : public MPxCommand
{
public:

	PCEPointCloudBaseCmd();
	virtual ~PCEPointCloudBaseCmd();

	virtual bool isUndoable() const;
	virtual MStatus doIt(const MArgList&);
	virtual MStatus redoIt();
	virtual MStatus undoIt();

protected:
	virtual MStatus parseArgs( const MArgList& args) = 0;
	virtual void compute(PCEPointCloud* cloud, PCEIndicesPtr indices) = 0;
	virtual bool needRecoverData();
	virtual void recoverData(PCEPointCloud* cloud, PCEIndicesPtr indices);

	MStatus doHelper(bool isRecoverData);

	MSelectionList previousSelectionList;
};

#endif