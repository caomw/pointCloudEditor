#ifndef _PCEPcdWriteCmd
#define _PCEPcdWriteCmd
//
// Copyright (C) Tingzhu Zhou
// 
// File: PCEPcdFileIOCmd.h
//
// Dependency Graph Node:  PCEPcdFileIOCmd
//
// Author: Tingzhu Zhou
//
#include <maya/MPxCommand.h>

//////////////////////////////////////////////////////////////////////////////////////////
// PCEPcdWriter thread class
class PCEPcdFileIOCmd : public MPxCommand
{
public:

	PCEPcdFileIOCmd(){}
	virtual ~PCEPcdFileIOCmd(){}

	virtual MStatus doIt(const MArgList&);

	static void* creator();
	static MSyntax newSyntax();

	static const MString kPCDFileIOCmd;
private:
	MStatus parseArgs( const MArgList& args);

	int		_mode;
	int		_frame;
	MString _path_name;
};

#endif