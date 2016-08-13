#ifndef _PCEGrabberRenderOverride
#define _PCEGrabberRenderOverride

#include <stdio.h>

#include <opencv\cv.h>

#include <maya/MString.h>
#include <maya/MColor.h>
#include <maya/MViewport2Renderer.h>

//Forwards
class PCEDeviceCacheImp;
class PCESkeletonPosition;

struct PCEPreviewHUD
{
	int fDepthFPS;
	int fColorFPS;
	bool bHasSkeleton;
	bool bHasFaceTracking;
};

struct ImagePoint2D
{
	int ptX;
	int ptY;
};

class PCEGrabberRenderOverride : public MHWRender::MRenderOverride
{
private:
	static const int		sMaxPreviewRender = 2;

	enum
	{
		kPreview = 0,		// Preview target
		kHUDBlit,			// Draw HUD on top
		kPresentOp,			// Present target
		kOperationCount
	};

	enum {
		kDepthTexture = 0,
		kColorTexture,
		//kFaceTexture,
		kFusionTexture,
		kAudioTexture,
		kTextureCount
	};

	enum {
		kOnePreviewShader = 0,	// To preview targets
		kTwoPreviewShader,	// To preview targets
		kShaderCount
	};
public:
	
	enum PreviewModeFlag
	{
		EPreviewNone = 0,
		EDepthMode = (0x1 << 0),
		EColorMode = (0x1 << 1),
		EFusionMode = (0x1 << 2),
		EAudioMode = (0x1 << 3),
		Expand_LAST_BIT = (0x1 << 4)
	};

	static const MString kPCEGrabberPreviewerName;

	PCEGrabberRenderOverride(const MString& name);
	virtual ~PCEGrabberRenderOverride();

	virtual MHWRender::DrawAPI supportedDrawAPIs() const;
	virtual bool startOperationIterator();
	virtual MHWRender::MRenderOperation * renderOperation();
	virtual bool nextRenderOperation();
	virtual MStatus setup(const MString& destination);
	virtual MStatus cleanup();
	virtual MString uiName() const
	{
		return mUIName;
	}
	MHWRender::MRenderOperation * getOperation( unsigned int i)
	{
		if (i < kOperationCount)
			return mRenderOperations[i];
		return NULL;
	}

	// For interface
	void releaseRender();
	void updatePreviewerMode(PreviewModeFlag flags);
	bool updatePreviewerTexture(PCEDeviceCacheImp* pCache);
	bool refreshView();
	const PCEPreviewHUD& getPreviewHUD() const { return mHUD; }

protected:
	MStatus updateRenderOperations();
	MStatus updateShaders(const MHWRender::MRenderer *theRenderer);
	MStatus updateOnePreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture);
	MStatus updateTwoPreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture1, MHWRender::MTexture* texture2);

	static void drawCross(void* pTextureData, unsigned int width, unsigned int height, unsigned int bytePerPixel, const ImagePoint2D& pt);
	static void drawLine(void* pTextureData, unsigned int width, unsigned int height, unsigned int bytePerPixel, const ImagePoint2D& pt1, const ImagePoint2D& pt2);
	static void drawSkeletonInImage(PCESkeletonPosition* pSkeleton, IplImage* image);
	bool updateDepthTexture(PCEDeviceCacheImp* pCache);
	bool updateColorTexture(PCEDeviceCacheImp* pCache);
	bool updateFusionTexture(PCEDeviceCacheImp* pCache);

	MString mUIName;
	MColor mClearColor;

	MHWRender::MRenderOperation * mRenderOperations[kOperationCount];
	MString mRenderOperationNames[kOperationCount];
	bool mRenderOperationEnabled[kOperationCount];
	int mCurrentOperation;

	MHWRender::MShaderInstance * mShaderInstances[kShaderCount];

private:
	IplImage* mImages[kTextureCount];
	MHWRender::MTexture* mTextures[kTextureCount];
	bool mTextureEnabled[kTextureCount];

	MString mModelPanel;
	PCEPreviewHUD mHUD;
};

////////////////////////////////////////////////////////////////////////////
// Target preview render
////////////////////////////////////////////////////////////////////////////
class PreviewTargetsOperation : public MHWRender::MQuadRender
{
public:
	PreviewTargetsOperation(const MString &name, PCEGrabberRenderOverride* theOverride);
	~PreviewTargetsOperation();

	virtual const MHWRender::MShaderInstance * shader();
	virtual const MFloatPoint * viewportRectangleOverride();
	virtual MHWRender::MClearOperation & clearOperation();

	void setShader( MHWRender::MShaderInstance *shader)
	{
		mShaderInstance = shader;
	}
	void setTexture( MHWRender::MTexture *texture )
	{
		mTexture = texture;
	}
	void setRectangle( const MFloatPoint& rec)
	{
		mViewRectangle = rec;
	}

protected:
	PCEGrabberRenderOverride *mOverride;

	// Shader and texture used for quad render
	MHWRender::MShaderInstance *mShaderInstance;
	MHWRender::MTexture *mTexture;
	MFloatPoint mViewRectangle;
};

////////////////////////////////////////////////////////////////////////////
// Custom hud operation
////////////////////////////////////////////////////////////////////////////
class previewHUDOperation : public MHWRender::MHUDRender
{
public:
	previewHUDOperation(PCEGrabberRenderOverride *override);
	virtual ~previewHUDOperation();

	virtual bool hasUIDrawables() const
	{
		return true;
	}

	virtual void addUIDrawables( MHWRender::MUIDrawManager& drawManager2D, const MHWRender::MFrameContext& frameContext );
protected:
	PCEGrabberRenderOverride *mOverride;
};

#endif