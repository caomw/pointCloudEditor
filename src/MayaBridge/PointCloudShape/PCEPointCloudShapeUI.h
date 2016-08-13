#ifndef _PCEPointCloudShapeUI
#define _PCEPointCloudShapeUI
///////////////////////////////////////////////////////////////////////////////
//
// pointCloudShapeUI
//
// There's no need to draw and select this node in vp1.0, so this class
// doesn't override draw(), select(), etc. But the creator() is needed for
// plugin registration and avoid crash in cases (e.g., RB pop up menu of this node).
//
////////////////////////////////////////////////////////////////////////////////

#include <maya/MPxSurfaceShapeUI.h>

class PCEPointCloudShapeUI : public MPxSurfaceShapeUI
{
public:

	static void* creator();

	PCEPointCloudShapeUI();
	virtual ~PCEPointCloudShapeUI();

	/////////////////////////////////////////////////////////////////////
	//
	// Overrides
	//
	/////////////////////////////////////////////////////////////////////

	// Puts draw request on the draw queue
	//
	virtual void	getDrawRequests( const MDrawInfo & info,
		bool objectAndActiveOnly,
		MDrawRequestQueue & requests ){}

	// Main draw routine. Gets called by maya with draw requests.
	//
	virtual void	draw( const MDrawRequest & request,
		M3dView & view ) const{}

	// Main draw routine for UV editor. This is called by maya when the 
	// shape is selected and the UV texture window is visible. 
	// 
	virtual void	drawUV( M3dView &view, const MTextureEditorDrawInfo & ) const{}
	virtual bool	canDrawUV() const{return false;}

	// Main selection routine
	//
	virtual bool	select( MSelectInfo &selectInfo,
		MSelectionList &selectionList,
		MPointArray &worldSpaceSelectPts ) const;

	/////////////////////////////////////////////////////////////////////
	//
	// Helper routines
	//
	/////////////////////////////////////////////////////////////////////
	bool 			selectVertices( MSelectInfo &selectInfo,
		MSelectionList &selectionList,
		MPointArray &worldSpaceSelectPts ) const;

private:
	// Prohibited and not implemented.
	PCEPointCloudShapeUI(const PCEPointCloudShapeUI& obj);
	const PCEPointCloudShapeUI& operator=(const PCEPointCloudShapeUI& obj);
};


#endif
