// File: PCEPointCloudFilter.cpp
//
// Dependency Graph Node: PCEPointCloudFilter
//
// Author: Tingzhu Zhou
//

#include "PCEPointCloudFilter.h"
#include "PointCloudShape/PCEPointCloudData.h"
#include "api_macros.h"

#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MGlobal.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <boost/make_shared.hpp>

using namespace pcl;
using namespace pcl::console;

MTypeId     PCEPointCloudFilter::id( 0x87089 );

// Attributes
// 
MObject     PCEPointCloudFilter::aMethod;
MObject     PCEPointCloudFilter::aLeafSize;
MObject     PCEPointCloudFilter::aField;
MObject     PCEPointCloudFilter::aMin;
MObject     PCEPointCloudFilter::aMax;
MObject     PCEPointCloudFilter::inputCloudData;
MObject     PCEPointCloudFilter::outputCloudData;

PCEPointCloudFilter::PCEPointCloudFilter() {}
PCEPointCloudFilter::~PCEPointCloudFilter() {}

MStatus PCEPointCloudFilter::compute( const MPlug& plug, MDataBlock& datablock )
//
//	Description:
//		This method computes the value of the given output plug based
//		on the values of the input attributes.
//
//	Arguments:
//		plug - the plug to compute
//		data - object that provides access to the attributes for this node
//
{
	MStatus returnStatus;

	// Check which output attribute we have been asked to compute.  If this 
	// node doesn't know how to compute it, we must return 
	// MS::kUnknownParameter.
	// 
	if ( plug == outputCloudData ) {
		// Create some user defined geometry data and access the
		// geometry so we can set it
		//
		MFnPluginData fnDataCreator;
		MTypeId tmpid( PCEPointCloudData::id );

		fnDataCreator.create( tmpid, &returnStatus );
		MCHECKERROR( returnStatus, "compute : error creating pointCloudData")

		PCEPointCloudData * newData = (PCEPointCloudData*)fnDataCreator.data( &returnStatus );
		MCHECKERROR( returnStatus, "compute : error gettin at proxy pointCloudData object")

		PCEPointCloud* geomPtr = newData->pCloudData.get();

		// If there is an input mesh then copy it's values
		// and construct some apiMeshGeom for it.
		//
		computeFilteredPointCloud( plug, datablock, geomPtr);

		// Assign the new data to the outputSurface handle
		//
		MDataHandle outHandle = datablock.outputValue( outputCloudData );
		outHandle.set( newData );
		datablock.setClean( plug );
		return MS::kSuccess;
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

MStatus PCEPointCloudFilter::computeFilteredPointCloud( 
														const MPlug&		plug,
														MDataBlock&			datablock,
														PCEPointCloud*		geomPtr
	)
//
// Description
//
//     This function takes an input surface of type kMeshData and converts
//     the geometry into this nodes attributes.
//     Returns kFailure if nothing is connected.
//
{
	MStatus stat;

	// Get the cluster index.
	short method = 0;
	MDataHandle inputHandle = datablock.inputValue( aMethod, &stat );
	if(stat == MS::kSuccess)
		method = inputHandle.asShort();

	double leafSize = 0.01;
	inputHandle = datablock.inputValue( aLeafSize, &stat );
	if(stat == MS::kSuccess)
		leafSize = inputHandle.asDouble();

	short eField = 0;
	inputHandle = datablock.inputValue( aField, &stat );
	if(stat == MS::kSuccess)
		eField = inputHandle.asShort();

	double min = -std::numeric_limits<double>::max ();
	inputHandle = datablock.inputValue( aMin, &stat );
	if(stat == MS::kSuccess)
		min = inputHandle.asDouble();

	double max = std::numeric_limits<double>::max ();
	inputHandle = datablock.inputValue( aMax, &stat );
	if(stat == MS::kSuccess)
		max = inputHandle.asDouble();

	// Get the input cloud point data.
	inputHandle = datablock.inputValue( inputCloudData, &stat );
	MCHECKERROR( stat, "computeInputSurface error getting inputSurface")

	// Check if anything is connected
	//
	MObject thisObj = thisMObject();
	MPlug surfPlug( thisObj, inputCloudData );
	if ( !surfPlug.isConnected() ) {
		stat = datablock.setClean( plug );
		MCHECKERROR( stat, "compute setClean" )
			return MS::kFailure;
	}

	PCEPointCloudData* surf = (PCEPointCloudData*) inputHandle.asPluginData();
	if ( NULL == surf ) {

		cerr << "NULL input point cloud data found\n";
		return stat;
	}
	// Extract the input data.
	TicToc tt;
	tt.tic ();

	if(1 == method)
	{

	}
	else
	{
		std::string field("z");
		if(1 == eField)
			field = "x";
		else if(2 == eField)
			field = "y";

		VoxelGrid<PCEPointT> grid;
		grid.setInputCloud (boost::make_shared<const PointCloudT> (surf->pCloudData->pntCloud));
		grid.setFilterFieldName (field);
		grid.setFilterLimits (min, max);
		grid.setLeafSize (leafSize, leafSize, leafSize);
		grid.filter (geomPtr->pntCloud);
	}

	MGlobal::displayInfo( MString(" Voxel Grid Filter: ") + tt.toc () + " ms for " + geomPtr->pntCloud.width * geomPtr->pntCloud.height + " points.");

	return MS::kSuccess;
}

void* PCEPointCloudFilter::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new PCEPointCloudFilter();
}

MStatus PCEPointCloudFilter::initialize()
//
//	Description:
//		This method is called to create and initialize all of the attributes
//      and attribute dependencies for this node type.  This is only called 
//		once when the node type is registered with Maya.
//
//	Return Values:
//		MS::kSuccess
//		MS::kFailure
//
{
	// This sample creates a single input float attribute and a single
	// output float attribute.
	//
	MFnNumericAttribute nAttr;
	MFnTypedAttribute	typedAttr;
	MFnEnumAttribute	enumAttr;
	MStatus				stat;

	// ----------------------- INPUTS -------------------------
	inputCloudData = typedAttr.create( "inputCloudData", "icd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create input point cloud data attribute"); return stat;}
	typedAttr.setStorable( false );
	typedAttr.setWritable( true );
	typedAttr.setReadable( false );
	stat = addAttribute( inputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aMethod = enumAttr.create( "method", "mt", 0, &stat );
	if (!stat) { stat.perror("create filter attribute"); return stat;}
	stat = enumAttr.addField( "voxelGrid", 0 );
	if (!stat) { stat.perror("add enum type voxelGrid"); return stat;}
	// Attribute will be written to files when this type of node is stored
	enumAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	enumAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aMethod );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aLeafSize = nAttr.create( "leafSize", "ls", MFnNumericData::kDouble, 0.01 );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aLeafSize );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aField = enumAttr.create( "field", "fi", 0, &stat );
	if (!stat) { stat.perror("create filter attribute"); return stat;}
	stat = enumAttr.addField( "z", 0 );
	if (!stat) { stat.perror("add enum z"); return stat;}
	stat = enumAttr.addField( "x", 1 );
	if (!stat) { stat.perror("add enum x"); return stat;}
	stat = enumAttr.addField( "y", 2 );
	if (!stat) { stat.perror("add enum y"); return stat;}
	// Attribute will be written to files when this type of node is stored
	enumAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	enumAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aField );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aMin = nAttr.create( "min", "mi", MFnNumericData::kDouble, -std::numeric_limits<double>::max () );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aMin );
	if (!stat) { stat.perror("addAttribute"); return stat;}

	aMax = nAttr.create( "max", "ma", MFnNumericData::kDouble, std::numeric_limits<double>::max () );
	// Attribute will be written to files when this type of node is stored
	nAttr.setStorable(true);
	// Attribute is keyable and will show up in the channel box
	nAttr.setKeyable(true);
	// Add the attributes we have created to the node
	//
	stat = addAttribute( aMax );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// ----------------------- OUTPUTS -------------------------
	outputCloudData = typedAttr.create( "outputCloudData", "ocd",
		PCEPointCloudData::id,
		MObject::kNullObj, &stat );
	if (!stat) { stat.perror("create output point cloud data attribute"); return stat;}
	typedAttr.setWritable( false );
	typedAttr.setStorable( false );
	typedAttr.setReadable( true );
	stat = addAttribute( outputCloudData );
	if (!stat) { stat.perror("addAttribute"); return stat;}


	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	//
	stat = attributeAffects( aMethod, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aLeafSize, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aField, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aMin, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( aMax, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}
	stat = attributeAffects( inputCloudData, outputCloudData );
	if (!stat) { stat.perror("attributeAffects"); return stat;}

	return MS::kSuccess;

}
