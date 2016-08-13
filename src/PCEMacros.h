//-
// ==========================================================================
// Copyright 1995,2006,2008 Autodesk, Inc. All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

////////////////////////////////////////////////////////////////////////////////
//
// PCEMacros.h
//
// Description:
//    Convenience macros for error checking and attribute creation,
//
////////////////////////////////////////////////////////////////////////////////

#ifndef NOMINMAX
#define NOMINMAX
#endif


#ifndef NOMINMAX
#define min(x,y) ((x) < (y) ? (x) : (y))
#define max(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef _Matrix4_
#define _Matrix4_
typedef struct _Matrix4
{
	float M11;
	float M12;
	float M13;
	float M14;
	float M21;
	float M22;
	float M23;
	float M24;
	float M31;
	float M32;
	float M33;
	float M34;
	float M41;
	float M42;
	float M43;
	float M44;
} 	Matrix4;
#endif _Matrix4_

#ifndef _Vector3_
#define _Vector3_
typedef struct _Vector3
{
	float x;
	float y;
	float z;
} 	Vector3;
#endif _Vector3_
#ifndef _Vector4_
#define _Vector4_
typedef struct _Vector4
{
	float x;
	float y;
	float z;
	float w;
} 	Vector4;
#endif _Vector4_


#ifndef _Triangle_
#define _Triangle_
typedef struct _Triangle
{
	int i;
	int j;
	int k;
} 	Triangle;
#endif _Triangle_


typedef unsigned char       BYTE;