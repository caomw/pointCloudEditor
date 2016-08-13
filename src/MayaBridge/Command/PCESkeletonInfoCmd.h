#ifndef _PCESkeletonInfoCmd
#define _PCESkeletonInfoCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCESkeletonInfoCmd.h
//
// Author: Tingzhu Zhou
//
#include <maya/MPxCommand.h>

//////////////////////////////////////////////////////////////////////////////////////////
// PCEPcdWriter thread class
class PCESkeletonInfoCmd : public MPxCommand
{
public:

	PCESkeletonInfoCmd(){}
	virtual ~PCESkeletonInfoCmd(){}

	virtual MStatus doIt(const MArgList&);

	static void* creator();
	static MSyntax newSyntax();

	static const MString kSkeletonInfoCmd;
private:
	MStatus parseArgs( const MArgList& args);

	int		_mode;
	int		_frame;
};

#endif