#ifndef _PCEPointCloudToMesh
#define _PCEPointCloudToMesh

///////////////////////////////////////////////////////////////////////////////
//
// PCEPointCloudToMesh.h
//
// A DG node that takes a point cloud data as input and outputs maya mesh.
// If there is no input then the node creates a cube or sphere
// depending on what the shapeType attribute is set to.
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>

class PCEPointCloudToMesh : public MPxNode
{
public:
	PCEPointCloudToMesh();
	virtual ~PCEPointCloudToMesh(); 

	//////////////////////////////////////////////////////////
	//
	// Overrides
	//
	//////////////////////////////////////////////////////////

	virtual MStatus   		compute( const MPlug& plug, MDataBlock& data );
	virtual MStatus			setDependentsDirty( const MPlug& plug, MPlugArray& plugArray);
	virtual MStatus			connectionMade( const MPlug& plug,
											const MPlug& otherPlug,
											bool asSrc );

	//////////////////////////////////////////////////////////
	//
	// Helper methods
	//
	//////////////////////////////////////////////////////////

	static  void *          creator();
	static  MStatus         initialize();

private:
	MStatus					computeOutputMesh( const MPlug& plug,
		MDataBlock& datablock,
		MObject& meshData );
	MStatus					assignMeshUV( MObject&	meshData, const MIntArray& polygonCounts, const MIntArray& uvIds );
	void					createEmptyMesh( MObject& out_empytMesh );

public:
	//////////////////////////////////////////////////////////
	//
	// Attributes
	//
	//////////////////////////////////////////////////////////
	static	MObject			aInputCloudData;
	static  MObject         aOutputMesh;
	static  MObject         aAssignUV;

public: 
	static	MTypeId		id;

private:
	bool fNeedDirty;
};

#endif /* _PCEPointCloudToMesh */