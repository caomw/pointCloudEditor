#ifndef _PCESkeletonJointInfoCmd
#define _PCESkeletonJointInfoCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletionJointInfoCmd.h
//
// Dependency Graph Node:  PCESkeletionJointInfoCmd
//
// Author: Tingzhu Zhou
//
#include <maya/MPxCommand.h>

class PCESkeletonJointInfoCmd : public MPxCommand
{
public:

	PCESkeletonJointInfoCmd(){}
	virtual ~PCESkeletonJointInfoCmd(){}

	virtual MStatus doIt(const MArgList&);

	static void* creator();
	static MSyntax newSyntax();

	static const MString kSkeletionJointInfoCmd;
private:
	MStatus parseArgs( const MArgList& args);

	int _BodyPart;
};

#endif
