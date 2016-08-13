#ifndef _PCEGrabberCmd
#define _PCEGrabberCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEGrabberCmd.h
//
// Author: Tingzhu Zhou
//
#include <maya/MPxCommand.h>

class PCEGrabberCmd : public MPxCommand
{
public:

	PCEGrabberCmd(){}
	virtual ~PCEGrabberCmd(){}

	virtual MStatus doIt(const MArgList&);

	static void* creator();
	static MSyntax newSyntax();

	static const MString kGrabberCmd;
private:
	MStatus parseArgs( const MArgList& args);

	int		_mode;
	int		_frame;
	MString _path_name;
};

#endif