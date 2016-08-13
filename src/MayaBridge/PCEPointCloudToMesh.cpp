///////////////////////////////////////////////////////////////////////////////
//
// apiMeshCreator.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "PCEPointCloudToMesh.h"
#include "PointCloudShape/PCEPointCloudData.h"
#include "api_macros.h"

#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MGlobal.h>

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

using namespace pcl;

////////////////////////////////////////////////////////////////////////////////
//
// Shape implementation
//
////////////////////////////////////////////////////////////////////////////////

MObject PCEPointCloudToMesh::aInputCloudData;
MObject PCEPointCloudToMesh::aOutputMesh;
MObject PCEPointCloudToMesh::aAssignUV;

MTypeId PCEPointCloudToMesh::id( 0x80190 );

PCEPointCloudToMesh::PCEPointCloudToMesh()
	: fNeedDirty(true)
{}
PCEPointCloudToMesh::~PCEPointCloudToMesh() {}

///////////////////////////////////////////////////////////////////////////////
//
// Overrides
//
///////////////////////////////////////////////////////////////////////////////

/* override */
MStatus PCEPointCloudToMesh::compute( const MPlug& plug, MDataBlock& datablock )
//
// Description
//
//    When input attributes are dirty this method will be called to
//    recompute the output attributes.
//
{ 
	MStatus stat;
	if ( plug == aOutputMesh && fNeedDirty) {
		// Create some mesh data and access the
		// geometry so we can set it
		//
		MFnMeshData dataCreator;
		MObject newOutputData = dataCreator.create(&stat);
		MCHECKERROR(stat, "ERROR creating outputData");

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		stat = computeOutputMesh( plug, datablock, newOutputData);
		if(stat != MS::kSuccess)
		{
			createEmptyMesh( newOutputData );
		}
		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( aOutputMesh );
		outHandle.set( newOutputData );
		datablock.setClean( plug );
		// Clear the flag.
		fNeedDirty = false;
		return MS::kSuccess;
	}
	else {
		return MS::kUnknownParameter;
	}
}

void PCEPointCloudToMesh::createEmptyMesh( MObject& out_empytMesh )
{
	MStatus status;
	MFnMeshData meshData;
	out_empytMesh = meshData.create( &status );
	CHECK_MSTATUS( status );

	MFloatPointArray  	vertexArray;
	MIntArray  	        polygonCounts;
	MIntArray  	        polygonConnects;

	MFnMesh meshCreator;
	MObject newMeshObject = meshCreator.create( 
		0, // nb vertices
		0, // nb triangles 
		vertexArray, 
		polygonCounts, 
		polygonConnects, 
		out_empytMesh );
}

MStatus PCEPointCloudToMesh::assignMeshUV( MObject&	meshData, const MIntArray& polygonCounts, const MIntArray& uvIds )
{

	MStatus stat = MS::kSuccess;
	MString uvSetName("uvset1");
	MFnMesh meshFn(meshData);
	stat = meshFn.getCurrentUVSetName(uvSetName);
	if ( stat != MS::kSuccess )
	{
		uvSetName = MString ("uvset1");
		stat = meshFn.createUVSet(uvSetName);
		stat = meshFn.setCurrentUVSetName(uvSetName);
	}

	//stat = meshFn.clearUVs();
	//stat = meshFn.setUVs(uArray,vArray,&uvSetName);
	stat = meshFn.assignUVs(polygonCounts, uvIds, &uvSetName);
	if(stat != MS::kSuccess)
		MGlobal::displayError( " Failed to assign UVs." );
	return stat;
}

MStatus PCEPointCloudToMesh::computeOutputMesh( 
	const MPlug&		plug,
	MDataBlock&			datablock,
	MObject&			meshData)
//
// Description
//
//     This function takes an input surface of type kMeshData and converts
//     the geometry into this nodes attributes.
//     Returns kFailure if nothing is connected.
//
{
	MStatus stat;

	MDataHandle inputHandle = datablock.inputValue( aInputCloudData, &stat );
	MCHECKERROR( stat, "computeInputSurface error getting inputSurface")

	// Check if anything is connected
	//
	MObject thisObj = thisMObject();
	MPlug surfPlug( thisObj, aInputCloudData );
	if ( !surfPlug.isConnected() ) {
		stat = datablock.setClean( plug );
		MCHECKERROR( stat, "compute setClean" )
		return MS::kFailure;
	}

	PCEPointCloudData* surf = (PCEPointCloudData*) inputHandle.asPluginData();
	if ( NULL == surf ) {
		cerr << "NULL inputCloudData data found\n";
		return stat;
	}

	PCEPointCloud* geomPtr = surf->pCloudData.get();
	if ( NULL == surf ) {
		cerr << "NULL pointCloudGeom found\n";
		return stat;
	}

	PointCloud<PCEPointT>::Ptr cloud (new pcl::PointCloud<PCEPointT>);
	copyPointCloud(geomPtr->pntCloud, *cloud);

	/*PointCloud<PointNormal>::Ptr cloudWithNormal (new PointCloud<PointNormal> ());
	copyPointCloud(*cloud, *cloudWithNormal);
	for(int i = 0;i < normals.size(); i++)
	{
		cloudWithNormal->points[i].normal_x = normals.at(i).normal_x;
		cloudWithNormal->points[i].normal_y = normals.at(i).normal_y;
		cloudWithNormal->points[i].normal_z = normals.at(i).normal_z;
	}*/

	float iso_level = 0.0f;
	int hoppe_or_rbf = 0;
	float extend_percentage = 0.0f;
	int grid_res = 30;
	float off_surface_displacement = 0.01f;

	MarchingCubes<PCEPointT> *mc;
	if (hoppe_or_rbf == 0)
		mc = new MarchingCubesHoppe<PCEPointT> ();
	else
	{
		mc = new MarchingCubesRBF<PCEPointT> ();
		(reinterpret_cast<MarchingCubesRBF<PCEPointT>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
	}


	mc->setIsoLevel (iso_level);
	mc->setGridResolution (grid_res, grid_res, grid_res);
	mc->setPercentageExtendGrid (extend_percentage);
	mc->setInputCloud (cloud);

	PolygonMesh output;
	mc->reconstruct (output);
	delete mc;

	MFloatPointArray vertexArray;
	MIntArray polygonCounts;
	MIntArray polygonConnects;
	MFloatArray uArray;
	MFloatArray vArray;
	MIntArray uvIds;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(output.cloud, *cloud_tmp_ptr);
	// number of points
	size_t nr_points  = cloud_tmp_ptr->width * cloud_tmp_ptr->height;
	// Write down vertices
	for (size_t i = 0; i < nr_points; ++i)
	{
		pcl::PointXYZ pt = cloud_tmp_ptr->at(i);
		MPoint pnt(pt.x, pt.y, pt.z);
		vertexArray.append(pnt);

		//Set UV
		double minDistance = std::numeric_limits<double>::max ();
		float u,v;
//#ifdef _OPENMP
//#pragma omp parallel for shared (minDistance) private (nn_indices, nn_dists) num_threads(10)
//#endif
		for(int j = 0; j < geomPtr->pntCloud.size(); ++j)
		{
			const PCEPointT& ptIn = geomPtr->pntCloud.at(j);
			MPoint pntIn(ptIn.x, ptIn.y, ptIn.z);
			double distance = pnt.distanceTo(pntIn);
			if( minDistance > distance )
			{
				minDistance = distance;
				u = ptIn.u;
				v = ptIn.v;
			}
		}
		uArray.append(u);
		vArray.append(v);
		uvIds.append((int)i);
	}
	// number of faces
	size_t nr_faces = output.polygons.size ();
	// Write down faces
	for (size_t i = 0; i < nr_faces; i++)
	{
		int vertCountOnFace = static_cast<int>(output.polygons[i].vertices.size ());
		polygonCounts.append(vertCountOnFace);
		size_t j = 0;
		for (j = 0; j < vertCountOnFace; ++j)
		{
			int connectIndex = output.polygons[i].vertices[j];
			polygonConnects.append(connectIndex);
		}
	}

	MFnMesh meshFn;
	meshFn.create(vertexArray.length(), polygonCounts.length(), vertexArray, polygonCounts, polygonConnects, uArray, vArray, meshData, &stat);

	bool needAssignUV = false;
	inputHandle = datablock.inputValue( aAssignUV, &stat );
	if(stat == MS::kSuccess)
		needAssignUV = inputHandle.asBool();
	if(needAssignUV)
		assignMeshUV(meshData, polygonCounts, uvIds);
	return stat;
}

/* override */
//
// Description
//
//	Horribly abuse the purpose of this method to notify the Viewport 2.0
//  renderer that something about this shape has changed and that it should
//  be retranslated.
//
MStatus PCEPointCloudToMesh::setDependentsDirty( const MPlug& plug, MPlugArray& plugArray)
{
	// if the dirty attribute is the output mesh then we need to signal the
	// the renderer that it needs to update the object

	/*if ( plug == aInputCloudData)
	{
		fNeedDirty = true;
	}*/
	return MS::kSuccess;
}

MStatus PCEPointCloudToMesh::connectionMade( const MPlug& plug,
	const MPlug& otherPlug,
	bool asSrc )
//
// Description
//
//    Whenever a connection to this node is broken, this method
//    will get called.
//
{
	if ( plug == aInputCloudData ) {
		fNeedDirty = true;
	}

	return MPxNode::connectionBroken( plug, otherPlug, asSrc );
}

void* PCEPointCloudToMesh::creator()
//
// Description
//    Called internally to create a new instance of the users MPx node.
//
{
	return new PCEPointCloudToMesh();
}

MStatus PCEPointCloudToMesh::initialize()
	//
	// Description
	//
	//    Attribute (static) initialization. See api_macros.h.
	//
{ 
	MStatus				stat;
	MFnTypedAttribute	typedAttr;
	MFnNumericAttribute nAttr;

	// ----------------------- INPUTS -------------------------
	aInputCloudData = typedAttr.create( "inputCloudData", "icd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create inputCloudData attribute" )
	typedAttr.setStorable( false );
	typedAttr.setHidden( true );
	ADD_ATTRIBUTE( aInputCloudData );

	aAssignUV = nAttr.create( "assignUV", "uv", MFnNumericData::kBoolean, true );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aAssignUV );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	// ----------------------- OUTPUTS -------------------------
	aOutputMesh = typedAttr.create( "outputMesh", "out",
		MFnData::kMesh,
		MObject::kNullObj, &stat );
	MCHECKERROR( stat, "create outputSurface attribute" )
	typedAttr.setWritable( false );
	ADD_ATTRIBUTE( aOutputMesh );

	// ---------- Specify what inputs affect the outputs ----------
	//
	ATTRIBUTE_AFFECTS( aInputCloudData, aOutputMesh );

	return MS::kSuccess;
}
