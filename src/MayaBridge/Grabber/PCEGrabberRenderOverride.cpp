#include <maya/MShaderManager.h>
#include <maya/MDrawContext.h>
#include <maya/MAnimControl.h>
#include <maya/M3dView.h>
#include <maya/MTextureManager.h>
#include <maya/MGlobal.h>

#include "DeviceManager/PCEDeviceCacheImp.h"
#include "DeviceManager/PCECompoundFrame.h"
#include "Shape/PCEFaceTrackingResult.h"

#include "PCEGrabberRenderOverride.h"

using namespace cv;

const MString PCEGrabberRenderOverride::kPCEGrabberPreviewerName = "PCEGrabberPreviewer";

PCEGrabberRenderOverride::PCEGrabberRenderOverride(const MString& name)
	: MRenderOverride(name)
	, mUIName("Point Cloud Grabber Previewer")
{
	unsigned int i = 0;
	for (i=0; i<kOperationCount; i++)
	{
		mRenderOperations[i] = NULL;
	}
	mCurrentOperation = -1;

	for (i=0; i<kShaderCount; i++)
	{
		mShaderInstances[i] = NULL;
	}

	for (i=0; i<kTextureCount; i++)
	{
		mTextures[i] = NULL;
		mImages[i] = NULL;
		mTextureEnabled[i] = false;
	}
}

PCEGrabberRenderOverride::~PCEGrabberRenderOverride()
{
	releaseRender();
}

void PCEGrabberRenderOverride::releaseRender()
{
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		delete mRenderOperations[i];
		mRenderOperations[i] = NULL;
	}

	MHWRender::MRenderer* theRenderer = MHWRender::MRenderer::theRenderer();
	if (theRenderer)
	{
		// Release shaders
		const MHWRender::MShaderManager* shaderMgr = theRenderer->getShaderManager();
		for (unsigned int i=0; i<kShaderCount; i++)
		{
			if (mShaderInstances[i])
			{
				if (shaderMgr)
					shaderMgr->releaseShader(mShaderInstances[i]);
				mShaderInstances[i] = NULL;
			}
		}

		// Release textures
		const MHWRender::MTextureManager* textureManager = theRenderer->getTextureManager();
		for (unsigned int i=0; i<kTextureCount; i++)
		{
			if (mTextures[i])
			{
				if (textureManager)
				{
					textureManager->releaseTexture(mTextures[i]);
					mTextures[i] = NULL;
				}
			}
			if (mImages[i])
			{
				cvReleaseImage(&mImages[i]);
				mImages[i] = NULL;
			}
		}
	}
}

MHWRender::DrawAPI PCEGrabberRenderOverride::supportedDrawAPIs() const
{
	return MHWRender::kAllDevices;
}

bool PCEGrabberRenderOverride::startOperationIterator()
{
	mCurrentOperation = 0;
	return true;
}

MHWRender::MRenderOperation* PCEGrabberRenderOverride::renderOperation()
{
	if (mCurrentOperation >= 0 && mCurrentOperation < kOperationCount)
	{
		// Skip empty and disabled operations
		//
		while(!mRenderOperations[mCurrentOperation] || !mRenderOperationEnabled[mCurrentOperation])
		{
			mCurrentOperation++;
			if (mCurrentOperation >= kOperationCount)
			{
				return NULL;
			}
		}

		if (mRenderOperations[mCurrentOperation])
		{
			return mRenderOperations[mCurrentOperation];
		}
	}
	return NULL;
}

bool PCEGrabberRenderOverride::nextRenderOperation()
{
	mCurrentOperation++;
	/*if(mCurrentOperation == mPreviewTargetCount)
	{
		unsigned int disableOperationCount = 4 - mPreviewTargetCount;
		mCurrentOperation += disableOperationCount;
	}*/
	return (mCurrentOperation < kOperationCount);
}

//
// Update list of operations to perform:
//
//
//		1. Play the cached target.
//		2. Display HUD
//		3. Blit on-screen
//
MStatus PCEGrabberRenderOverride::updateRenderOperations()
{
	bool initOperations = true;
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		if (mRenderOperations[i])
			initOperations = false;
	}

	if (initOperations)
	{
		// There ops are for player
		// A quad blit to preview a target
		mRenderOperationNames[kPreview] = "_PCEGrabberPreviewer_TargetPreview";
		PreviewTargetsOperation * previewOp = new PreviewTargetsOperation (mRenderOperationNames[kPreview], this);
		mRenderOperations[kPreview] = previewOp;
		mRenderOperationEnabled[kPreview] = true;

		// There ops are for HUD
		// A quad blit to display the information of player
		mRenderOperationNames[kHUDBlit] = "_PCEGrabberPreviewer_HUDBlit";
		mRenderOperations[kHUDBlit] = new previewHUDOperation (this);
		mRenderOperationEnabled[kHUDBlit] = true;

		// Generic screen blit - always want to do this
		mRenderOperationNames[kPresentOp] = "_PCEGrabberPreviewer_PresentTarget";
		mRenderOperations[kPresentOp] = new MHWRender::MPresentTarget(mRenderOperationNames[kPresentOp]);
		mRenderOperationEnabled[kPresentOp] = true;
	}
	mCurrentOperation = -1;

	MStatus haveOperations = MStatus::kFailure;
	for (unsigned int i=0; i<kOperationCount; i++)
	{
		if (mRenderOperations[i])
		{
			haveOperations = MStatus::kSuccess;
		}
	}

	return haveOperations;
}

void PCEGrabberRenderOverride::updatePreviewerMode(PreviewModeFlag flags)
{
	int previewerCount = 0;
	if(flags & EDepthMode)
	{
		mTextureEnabled[kDepthTexture] = true;
		previewerCount ++;
	}
	if(flags & EColorMode)
	{
		mTextureEnabled[kColorTexture] = true;
		previewerCount ++;
	}
	/*if((flags & EFaceMode) && (previewerCount < sMaxPreviewRender))
	{
		mTextureEnabled[kFaceTexture] = true;
		previewerCount ++;
	}*/
	if((flags & EFusionMode) && (previewerCount < sMaxPreviewRender))
	{
		mTextureEnabled[kFusionTexture] = true;
		previewerCount ++;
	}
	if((flags & EAudioMode) && (previewerCount < sMaxPreviewRender))
	{
		mTextureEnabled[kAudioTexture] = true;
		previewerCount ++;
	}
}

//
// Update all targets used for rendering
//
MStatus PCEGrabberRenderOverride::updateShaders(const MHWRender::MRenderer *theRenderer)
{
	if (!theRenderer)
		return MStatus::kFailure;

	const MHWRender::MShaderManager* shaderMgr = theRenderer->getShaderManager();
	if (!shaderMgr) return MStatus::kFailure;

	MHWRender::MTexture* previewTextures[kTextureCount];
	int previewerCount = 0;
	for (int i=0; i<kTextureCount; i++)
	{
		if (mTextureEnabled[i])
		{
			previewTextures[previewerCount] = mTextures[i];
			previewerCount ++;
		}
	}
	assert(previewerCount <= sMaxPreviewRender);
	
	/*int disableOperationCount = 4 - mPreviewTargetCount;
	for (int i = 0; i < disableOperationCount; i++)
	{
		mRenderOperationEnabled[3-i] = false;
	}*/
	if(0 == previewerCount)
	{
		return MStatus::kFailure;
	}
	
	// We NULL the output targets here so that the result will go into the
	// internal target and be presented from the internal targets
	/*quadRenderMRT * quadOp2 = (quadRenderMRT * )mRenderOperations[kTargetPreview];
	if (quadOp2)
		quadOp2->setRenderTargets( NULL, 0 );
	MHWRender::MPresentTarget *presentOp = (MHWRender::MPresentTarget *)mRenderOperations[kPresentOp];
	if (presentOp)
		presentOp->setRenderTargets( NULL );*/

	switch (previewerCount)
	{
	case 1:
		return updateOnePreviewShaders(shaderMgr, previewTextures[0]);
	case 2:
		return updateTwoPreviewShaders(shaderMgr, previewTextures[0], previewTextures[1]);
	default:
		return MStatus::kFailure;
	}

	return MStatus::kSuccess;
}

//
// Update all shaders used for rendering
//
MStatus PCEGrabberRenderOverride::updateOnePreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture)
{
	if (!shaderMgr) return MStatus::kFailure;
	// Set up a preview target shader (Targets as input)
	//
	MHWRender::MShaderInstance *shaderInstance = mShaderInstances[kOnePreviewShader];
	if (!shaderInstance)
	{
		shaderInstance = shaderMgr->getEffectsFileShader( "Copy", "" );
		mShaderInstances[kOnePreviewShader] = shaderInstance;

		// Set constant parameters
		if (shaderInstance)
		{
			// We want to make sure to reblit back alpha as well as RGB
			shaderInstance->setParameter("gDisableAlpha", false );
			shaderInstance->setParameter("gVerticalFlip", false );
		}
	}
	MFloatPoint scaleSize( 0.0f, 0.0f, 1.0f,  1.0f );
	// Make sure to update the texture to use
	MHWRender::MTextureAssignment texAssignment;
	texAssignment.texture = texture;
	// Update the parameter for the texture assignment.
	shaderInstance->setParameter("gInputTex", texAssignment );
	// Update the resize of texture on the viewport
	if(texAssignment.texture)
	{
		MHWRender::MTextureDescription textureDesc;
		texAssignment.texture->textureDescription(textureDesc);
		unsigned int targetWidth = 0;
		unsigned int targetHeight = 0;
		MHWRender::MRenderer *theRenderer = MHWRender::MRenderer::theRenderer();
		if( theRenderer )
		{
			// Update the scale of width and height in the shader.
			// The width and height of the preview texture should be in keep with the current viewport.
			theRenderer->outputTargetSize( targetWidth, targetHeight );
			float scaleY = (0 == targetHeight) ? 1.0f : (float)textureDesc.fHeight*targetWidth/textureDesc.fWidth/targetHeight;
			if(scaleY < 1.0f)
			{
				scaleSize[1] = (1.0f-scaleY)/2;
				scaleSize[3] = scaleY;
			}
		}
	}

	// Update shader and texture on quad operation
	PreviewTargetsOperation * quadOp = (PreviewTargetsOperation * )mRenderOperations[kPreview];
	if (quadOp)
	{
		quadOp->setShader( mShaderInstances[kOnePreviewShader] );
		quadOp->setRectangle( scaleSize );
	}

	if (quadOp && shaderInstance)
		return MStatus::kSuccess;
	return MStatus::kFailure;
}

MStatus PCEGrabberRenderOverride::updateTwoPreviewShaders(const MHWRender::MShaderManager* shaderMgr, MHWRender::MTexture* texture1, MHWRender::MTexture* texture2)
{
	MHWRender::MShaderInstance *shaderInstance = mShaderInstances[kTwoPreviewShader];
	if (!shaderInstance)
	{
		shaderInstance = shaderMgr->getEffectsFileShader( "FreeView", "" );
		mShaderInstances[kTwoPreviewShader] = shaderInstance;

		// Set constant parmaeters
		if (shaderInstance)
		{
			const float borderClr[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
			const float backGroundClr[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
			shaderInstance->setParameter("gBorderColor", borderClr );
			shaderInstance->setParameter("gBackgroundColor", backGroundClr );
		}
	}
	// Update shader's per frame parameters
	if (shaderInstance)
	{
		unsigned int targetWidth = 0;
		unsigned int targetHeight = 0;
		MHWRender::MRenderer *theRenderer = MHWRender::MRenderer::theRenderer();
		if( theRenderer )
			theRenderer->outputTargetSize( targetWidth, targetHeight );

		float vpSize[2] = { (float)targetWidth,  (float)targetHeight };
		shaderInstance->setParameter("gViewportSizePixels", vpSize );

		float sourceSize[2] = { (float)targetWidth,  (float)targetHeight };
		shaderInstance->setParameter("gSourceSizePixels", sourceSize );

		/// Could use 0.0125 * width / 2
		shaderInstance->setParameter("gBorderSizePixels", 0.00625f * targetWidth );

		// Bind two input targets
		if (texture1)
		{
			MHWRender::MTextureAssignment texAssignment;
			texAssignment.texture = texture1;
			shaderInstance->setParameter("gSourceTex", texAssignment);
		}
		if (texture2)
		{
			MHWRender::MTextureAssignment assignment2;
			assignment2.texture = texture2;
			shaderInstance->setParameter("gSourceTex2", assignment2);
		}
	}
	// Update shader on quad operation
	PreviewTargetsOperation * quadOp = (PreviewTargetsOperation * )mRenderOperations[kPreview];
	if (quadOp)
		quadOp->setShader( mShaderInstances[kTwoPreviewShader] );

	if (quadOp && shaderInstance)
		return MStatus::kSuccess;
	return MStatus::kFailure;
}

//
// Update override for the current frame
//
MStatus PCEGrabberRenderOverride::setup(const MString& destination)
{
	// Firewall checks
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (!renderer) return MStatus::kFailure;

	// Update render operations
	MStatus status = updateRenderOperations();
	if (status != MStatus::kSuccess)
		return status;

	mModelPanel = destination;
	//M3dView view;
	//status = M3dView::getM3dViewFromModelPanel(destination, view);
	//if (status != MStatus::kSuccess)
	//	return status;
	// Update the player params
	//mPlayer.setView(view);
	//mPlayer.updateTimeRange(MAnimControl::minTime(), MAnimControl::maxTime());
	//// Set the current time and search the nearest time that caches the image.
	//// When the player is on, the current time is handled by the player itself.
	//if( !mPlayer.isPlayOn() )
	//{
	//	MTime keyTime = MAnimControl::currentTime();
	//	// Search & return the nearest key time that cached the image ahead of the current time.
	//	do
	//	{
	//		mPlayer.setCurrentTime(keyTime);
	//		if( mPlayer.getCurrentImage() )
	//			break;
	//		keyTime--;
	//	} while (keyTime >= MAnimControl::minTime());
	//}

	// Update shaders
	status = updateShaders( renderer );

	return status;
}

MStatus PCEGrabberRenderOverride::cleanup()
{
	mCurrentOperation = -1;
	return MStatus::kSuccess;
}

bool PCEGrabberRenderOverride::updatePreviewerTexture(PCEDeviceCacheImp* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	bool hasUpdated = false;

	if(mTextureEnabled[kDepthTexture])
	{
		hasUpdated |= updateDepthTexture(pCache);
	}
	if(mTextureEnabled[kColorTexture])
	{
		hasUpdated |= updateColorTexture(pCache);
	}
	
	if(mTextureEnabled[kFusionTexture])
	{
		hasUpdated |= updateFusionTexture(pCache);
	}
	if(mTextureEnabled[kAudioTexture])
	{
		
	}

	return hasUpdated;
}

bool PCEGrabberRenderOverride::updateDepthTexture(PCEDeviceCacheImp* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	if(!mTextureEnabled[kDepthTexture])
		return false;

	PCECompoundFrame* pFrame = pCache->getLatestCompoundFrame();
	if (!pFrame)
		return false;

	mHUD.fDepthFPS = pFrame->getDepthFPS();
	
	const unsigned int width = pFrame->getDepthWidth();
	const unsigned int height = pFrame->getDepthHeight();
	if(mImages[kDepthTexture])
	{
		if(width != mImages[kDepthTexture]->width || height != mImages[kDepthTexture]->height)
		{
			cvReleaseImage(&mImages[kDepthTexture]);
			mImages[kDepthTexture] = NULL;
		}
	}
	if(!mImages[kDepthTexture])
	{
		mImages[kDepthTexture] = cvCreateImage(cvSize(width,height),IPL_DEPTH_16U,1);
	}

	const unsigned int cBytePerPixel = 2;
	if(mImages[kDepthTexture]->imageData && pFrame->getDepthBuffer())
	{
		pFrame->readImageLock();
		errno_t err = memcpy_s(mImages[kDepthTexture]->imageData, mImages[kDepthTexture]->imageSize * cBytePerPixel, pFrame->getDepthBuffer(), pFrame->getDepthBufferSize());
		pFrame->readImageUnlock();
		if(0 != err)
		{
			printf_s( "Failed to read cached images in previewer.\n" );
			return false;
		}
	}
	else
	{
		printf_s( "Failed to read cached images in previewer.\n" );
		return false;
	}

	PCESkeletonPosition skeleton;
	bool bHasSkeleton = pFrame->readSkeletonPosition(0,&skeleton);
	if(bHasSkeleton)
	{
		mHUD.bHasSkeleton = pFrame->hasSkeleton();
		drawSkeletonInImage(&skeleton, mImages[kDepthTexture]);
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if(!renderer)
		return false;
	MHWRender::MTextureManager* textureManager = renderer->getTextureManager();
	if(!textureManager)
	{
		//MGlobal::displayWarning( hwApiTextureTestStrings::getString( hwApiTextureTestStrings::kErrorTextureManager ) );
		return false;
	}

	
	MString texName = MString("grabberDepthImage");
	mTextures[kDepthTexture] = textureManager->findTexture(texName);
	if(mTextures[kDepthTexture])
	{
		mTextures[kDepthTexture]->update( mImages[kDepthTexture]->imageData, false, cBytePerPixel*width );
	}
	else
	{
		MHWRender::MTextureDescription textureDesc;
		textureDesc.fWidth = width;
		textureDesc.fHeight = height;
		textureDesc.fDepth = 1;
		textureDesc.fBytesPerRow = cBytePerPixel*width;
		textureDesc.fBytesPerSlice = cBytePerPixel*width*height;
		textureDesc.fMipmaps = 1;
		textureDesc.fArraySlices = 1;
		textureDesc.fFormat = MHWRender::kR16_UNORM;
		textureDesc.fTextureType = MHWRender::kImage2D;
		textureDesc.fEnvMapType = MHWRender::kEnvNone;

		// Construct the texture with the screen pixels
		mTextures[kDepthTexture] = textureManager->acquireTexture( texName, textureDesc, mImages[kDepthTexture]->imageData, false );
	}
	if(!mTextures[kDepthTexture])
	{
		printf_s("Preview texture is NULL!\n");
		return false;
	}
	
	return true;
}

bool PCEGrabberRenderOverride::updateColorTexture(PCEDeviceCacheImp* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	if(!mTextureEnabled[kColorTexture])
		return false;

	PCECompoundFrame* pFrame = pCache->getLatestCompoundFrame();
	if (!pFrame)
		return false;

	mHUD.fColorFPS = pFrame->getColorFPS();
	
	const unsigned int width = pFrame->getColorWidth();
	const unsigned int height = pFrame->getColorHeight();
	if(mImages[kColorTexture])
	{
		if(width != mImages[kColorTexture]->width || height != mImages[kColorTexture]->height)
		{
			cvReleaseImage(&mImages[kColorTexture]);
			mImages[kColorTexture] = NULL;
		}
	}
	if(!mImages[kColorTexture])
	{
		mImages[kColorTexture] = cvCreateImage(cvSize(width,height),IPL_DEPTH_32S,1);
	}

	const unsigned int cBytePerPixel = 4;
	if(mImages[kColorTexture]->imageData)
	{
		pFrame->readImageLock();
		errno_t err = memcpy_s(mImages[kColorTexture]->imageData, mImages[kColorTexture]->imageSize * cBytePerPixel, pFrame->getAlignedColorBuffer(), pFrame->getColorBufferSize());
		pFrame->readImageUnlock();
		if(0 != err)
		{
			printf_s( "Failed to read cached images in previewer.\n" );
			return false;
		}
	}
	else
	{
		printf_s( "Failed to read cached images in previewer.\n" );
		return false;
	}

	PCESkeletonPosition skeleton;
	mHUD.bHasSkeleton = pFrame->readSkeletonPosition(0,&skeleton);
	if(mHUD.bHasSkeleton)
	{
		mHUD.bHasSkeleton = pFrame->hasSkeleton();
		drawSkeletonInImage(&skeleton, mImages[kColorTexture]);
	}

	PCEFaceTrackingResult faceResult;
	mHUD.bHasFaceTracking = pCache->readFaceShape(pCache->getFaceTrackingFrame(), &faceResult);
	if(mHUD.bHasFaceTracking)
	{
		if( !faceResult.isMeshEmpty() )
		{
			CvScalar ftColor = CV_RGB(255,255,255);
			unsigned int ptSize = faceResult.sizeOfPts3D();
			for (UINT trId = 0; trId < faceResult.sizeOfTriangles(); trId++)
			{
				Triangle triangle = faceResult.getTriangle(trId);
				CvPoint pt1, pt2, pt3;
				int pt1Index = triangle.i;
				int pt2Index = triangle.j;
				int pt3Index = triangle.k;
				if(pt1Index < ptSize && pt2Index < ptSize)
				{
					Vector3 pt3D = faceResult.getPts3D(pt1Index);
					pt1.x = pt3D.x;
					pt1.y = pt3D.y;
					pt3D = faceResult.getPts3D(pt2Index);
					pt2.x = pt3D.x;
					pt2.y = pt3D.y;
					cvLine(mImages[kColorTexture], pt1, pt2, ftColor);
				}
				if(pt1Index < ptSize && pt3Index < ptSize)
				{
					Vector3 pt3D = faceResult.getPts3D(pt1Index);
					pt1.x = pt3D.x;
					pt1.y = pt3D.y;
					pt3D = faceResult.getPts3D(pt3Index);
					pt3.x = pt3D.x;
					pt3.y = pt3D.y;
					cvLine(mImages[kColorTexture], pt1, pt3, ftColor);
				}
				if(pt2Index < ptSize && pt3Index < ptSize)
				{
					Vector3 pt3D = faceResult.getPts3D(pt1Index);
					pt2.x = pt3D.x;
					pt2.y = pt3D.y;
					pt3D = faceResult.getPts3D(pt3Index);
					pt3.x = pt3D.x;
					pt3.y = pt3D.y;
					cvLine(mImages[kColorTexture], pt2, pt3, ftColor);
				}
			}
		}
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if(!renderer)
		return false;
	MHWRender::MTextureManager* textureManager = renderer->getTextureManager();
	if(!textureManager)
	{
		//MGlobal::displayWarning( hwApiTextureTestStrings::getString( hwApiTextureTestStrings::kErrorTextureManager ) );
		return false;
	}
	
	MString texName = MString("grabberColorImage");
	mTextures[kColorTexture] = textureManager->findTexture(texName);
	if(mTextures[kColorTexture])
	{
		mTextures[kColorTexture]->update( mImages[kColorTexture]->imageData, false, cBytePerPixel*width );
	}
	else
	{
		MHWRender::MTextureDescription textureDesc;
		textureDesc.fWidth = width;
		textureDesc.fHeight = height;
		textureDesc.fDepth = 1;
		textureDesc.fBytesPerRow = cBytePerPixel*width;
		textureDesc.fBytesPerSlice = cBytePerPixel*width*height;
		textureDesc.fMipmaps = 1;
		textureDesc.fArraySlices = 1;
		textureDesc.fFormat = MHWRender::kB8G8R8A8;
		textureDesc.fTextureType = MHWRender::kImage2D;
		textureDesc.fEnvMapType = MHWRender::kEnvNone;

		// Construct the texture with the screen pixels
		mTextures[kColorTexture] = textureManager->acquireTexture( texName, textureDesc, mImages[kColorTexture]->imageData, false );
	}
	if(!mTextures[kColorTexture])
	{
		printf_s("Preview texture is NULL!\n");
		return false;
	}
	return true;
}

bool PCEGrabberRenderOverride::updateFusionTexture(PCEDeviceCacheImp* pCache)
{
	assert(pCache);
	if(!pCache)
		return false;

	if(!mTextureEnabled[kFusionTexture])
		return false;

	if( !pCache->readFusionImage(pCache->getFusionFrame(), mImages[kFusionTexture]) )
	{
		printf_s( "Failed to read cached images in previewer.\n" );
		return false;
	}
	assert(mImages[kFusionTexture]);

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if(!renderer)
		return false;
	MHWRender::MTextureManager* textureManager = renderer->getTextureManager();
	if(!textureManager)
	{
		//MGlobal::displayWarning( hwApiTextureTestStrings::getString( hwApiTextureTestStrings::kErrorTextureManager ) );
		return false;
	}

	const unsigned int cBytePerPixel = 4;
	const unsigned int width = mImages[kFusionTexture]->width;
	const unsigned int height = mImages[kFusionTexture]->height;

	MString texName = MString("kinectFusionImage");
	mTextures[kFusionTexture] = textureManager->findTexture(texName);
	if(mTextures[kFusionTexture])
	{
		mTextures[kFusionTexture]->update( mImages[kFusionTexture]->imageData, false, cBytePerPixel*width );
	}
	else
	{
		MHWRender::MTextureDescription textureDesc;
		textureDesc.fWidth = width;
		textureDesc.fHeight = height;
		textureDesc.fDepth = 1;
		textureDesc.fBytesPerRow = cBytePerPixel*width;
		textureDesc.fBytesPerSlice = cBytePerPixel*width*height;
		textureDesc.fMipmaps = 1;
		textureDesc.fArraySlices = 1;
		textureDesc.fFormat = MHWRender::kB8G8R8A8;
		textureDesc.fTextureType = MHWRender::kImage2D;
		textureDesc.fEnvMapType = MHWRender::kEnvNone;

		// Construct the texture with the screen pixels
		mTextures[kFusionTexture] = textureManager->acquireTexture( texName, textureDesc, mImages[kFusionTexture]->imageData, false );
	}
	if(!mTextures[kFusionTexture])
	{
		printf_s("Preview texture is NULL!\n");
		return false;
	}
	return true;
}

bool PCEGrabberRenderOverride::refreshView()
{
	M3dView view;
	if (M3dView::getM3dViewFromModelPanel(mModelPanel, view) != MStatus::kSuccess)
		return false;

	view.scheduleRefresh();
	return true;
}

/*static*/
void PCEGrabberRenderOverride::drawSkeletonInImage(PCESkeletonPosition* pSkeleton, IplImage* image)
{
	if(!pSkeleton || !image)
		return;

	CvScalar color = CV_RGB(255,255,255);
	// Draw crosses
	for(int i = 0; i < SKELETON_POSITION_COUNT; ++i)
	{
		if( !pSkeleton->isInferred((PCEJointType)i) )
		{
			CvPoint pt2D;
			pt2D.x = (int)(pSkeleton->getJointPositionX((PCEJointType)i));
			pt2D.y = (int)(pSkeleton->getJointPositionY((PCEJointType)i));
			cvCircle(image, pt2D, 3, color);
			//drawCross(pTextureData, width, height, bytePerPixel, pt2D);
		}
	}
	
	// Draw lines
	if( !pSkeleton->isInferred(SKELETON_POSITION_HEAD) && !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_CENTER) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HEAD));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HEAD));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER));
		cvLine(image, pt1, pt2, color);
		/*drawLine(
			pTextureData, width, height, bytePerPixel,
			pt1,
			pt2 );*/
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_SPINE) && !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_CENTER) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SPINE));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SPINE));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_HIP_CENTER) && !pSkeleton->isInferred(SKELETON_POSITION_SPINE) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_CENTER));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_CENTER));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SPINE));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SPINE));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_CENTER) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_ELBOW_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ELBOW_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ELBOW_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_WRIST_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_ELBOW_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_WRIST_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_WRIST_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ELBOW_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ELBOW_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_WRIST_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_HAND_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_WRIST_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_WRIST_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HAND_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HAND_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_CENTER) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_CENTER));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_CENTER));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_SHOULDER_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_ELBOW_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_SHOULDER_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_SHOULDER_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ELBOW_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ELBOW_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_WRIST_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_ELBOW_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_WRIST_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_WRIST_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ELBOW_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ELBOW_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_WRIST_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_HAND_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_WRIST_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_WRIST_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HAND_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HAND_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_HIP_CENTER) && !pSkeleton->isInferred(SKELETON_POSITION_HIP_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_CENTER));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_CENTER));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_KNEE_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_HIP_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_KNEE_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_KNEE_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_KNEE_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_ANKLE_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_KNEE_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_KNEE_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ANKLE_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ANKLE_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_FOOT_LEFT) && !pSkeleton->isInferred(SKELETON_POSITION_ANKLE_LEFT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_FOOT_LEFT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_FOOT_LEFT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ANKLE_LEFT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ANKLE_LEFT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_HIP_CENTER) && !pSkeleton->isInferred(SKELETON_POSITION_HIP_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_CENTER));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_CENTER));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_KNEE_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_HIP_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_KNEE_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_KNEE_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_HIP_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_HIP_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_KNEE_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_ANKLE_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_KNEE_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_KNEE_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ANKLE_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ANKLE_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
	if( !pSkeleton->isInferred(SKELETON_POSITION_FOOT_RIGHT) && !pSkeleton->isInferred(SKELETON_POSITION_ANKLE_RIGHT) )
	{
		CvPoint pt1;
		pt1.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_FOOT_RIGHT));
		pt1.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_FOOT_RIGHT));
		CvPoint pt2;
		pt2.x = (int)(pSkeleton->getJointPositionX(SKELETON_POSITION_ANKLE_RIGHT));
		pt2.y = (int)(pSkeleton->getJointPositionY(SKELETON_POSITION_ANKLE_RIGHT));
		cvLine(image, pt1, pt2, color);
	}
}

/*static*/
void PCEGrabberRenderOverride::drawCross(void* pTextureData, unsigned int width, unsigned int height, unsigned int bytePerPixel, const ImagePoint2D& pt)
{
	if(!pTextureData)
		return;
	for(int i = -2; i < 3; ++i)
	{
		int paintX = pt.ptX + i;
		int paintY = pt.ptY;
		if(paintX > 0 && paintX < (int)width && paintY > 0 && paintY < (int)height)
		{
			BYTE* pPaintData = (BYTE*)pTextureData + paintY*bytePerPixel*width + paintX*bytePerPixel;
			for(unsigned int j = 0; j < bytePerPixel; j++)
				pPaintData[j] = 0;
		}
		
		paintX = pt.ptX;
		paintY = pt.ptY + i;
		if(paintX > 0 && paintX < (int)width && paintY > 0 && paintY < (int)height)
		{
			BYTE* pPaintData = (BYTE*)pTextureData + paintY*bytePerPixel*width + paintX*bytePerPixel;
			for(unsigned int j = 0; j < bytePerPixel; j++)
				pPaintData[j] = 0;
		}
	}
}
/*static*/
void PCEGrabberRenderOverride::drawLine(void* pTextureData, unsigned int width, unsigned int height, unsigned int bytePerPixel, const ImagePoint2D& pt1, const ImagePoint2D& pt2)
{
	if(!pTextureData)
		return;
	int offsetX = pt1.ptX - pt2.ptX;
	int offsetY = pt1.ptY - pt2.ptY;
	int rangeX = std::abs(offsetX);
	int rangeY = std::abs(offsetY);

	if(rangeX > rangeY)
	{
		int step = offsetX / rangeX;
		float slope = (float)rangeY / (float)rangeX;
		for(int i = 0; i <= rangeX; ++i)
		{
			int paintX = pt1.ptX + i;
			int paintY = (int)(pt1.ptY + i * slope);
			if(paintX > 0 && paintX < (int)width && paintY > 0 && paintY < (int)height)
			{
				BYTE* pPaintData = (BYTE*)pTextureData + paintY*bytePerPixel*width + paintX*bytePerPixel;
				for(unsigned int j = 0; j < bytePerPixel; j++)
					pPaintData[j] = 0;
			}
		}
	}
	else
	{
		int step = offsetY / rangeY;
		float slope = (float)rangeX / (float)rangeY;
		for(int i = 0; i <= rangeY; ++i)
		{
			int paintX = (int)(pt1.ptX + i * slope);
			int paintY = pt1.ptY + i;
			if(paintX > 0 && paintX < (int)width && paintY > 0 && paintY < (int)height)
			{
				BYTE* pPaintData = (BYTE*)pTextureData + paintY*bytePerPixel*width + paintX*bytePerPixel;
				for(unsigned int j = 0; j < bytePerPixel; j++)
					pPaintData[j] = 0;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////

PreviewTargetsOperation::PreviewTargetsOperation(const MString &name, PCEGrabberRenderOverride *theOverride)
	: MQuadRender( name )
	, mShaderInstance(NULL)
	, mTexture(NULL)
	, mOverride(theOverride)
{
	// 100 % of target size
	mViewRectangle[0] = 0.0f;
	mViewRectangle[1] = 0.0f;
	mViewRectangle[2] = 1.0f;
	mViewRectangle[3] = 1.0f;
}

PreviewTargetsOperation::~PreviewTargetsOperation()
{
	mShaderInstance = NULL;
	mTexture = NULL;
	mOverride = NULL;
}

const MHWRender::MShaderInstance* PreviewTargetsOperation::shader()
{
	return mShaderInstance;
}

const MFloatPoint * PreviewTargetsOperation::viewportRectangleOverride()
{
	return &mViewRectangle;
}

MHWRender::MClearOperation &
PreviewTargetsOperation::clearOperation()
{
	mClearOperation.setMask( (unsigned int) MHWRender::MClearOperation::kClearAll );
	return mClearOperation;
}

//------------------------------------------------------------------------
// Custom HUD operation
//
previewHUDOperation::previewHUDOperation(PCEGrabberRenderOverride *override)
	: MHWRender::MHUDRender()
	, mOverride(override)
{
}

previewHUDOperation::~previewHUDOperation()
{
	mOverride = NULL;
}

void previewHUDOperation::addUIDrawables( MHWRender::MUIDrawManager& drawManager2D, const MHWRender::MFrameContext& frameContext )
{
	if(!mOverride)
		return;

	// Start draw UI
	drawManager2D.beginDrawable();
	// Set font color
	drawManager2D.setColor( MColor( 0.455f, 0.212f, 0.596f ) );
	// Set font size
	drawManager2D.setFontSize( MHWRender::MUIDrawManager::kSmallFontSize );

	// Draw renderer name
	int x=0, y=0, w=0, h=0;
	frameContext.getViewportDimensions( x, y, w, h );
	drawManager2D.text( MPoint(w*0.5f, y+h*0.075f), MString("Point Cloud Previewer"), MHWRender::MUIDrawManager::kCenter );

	// Draw viewport information
	MString viewportInfoText( "Viewport information: x= " );
	viewportInfoText += x;
	viewportInfoText += ", y= ";
	viewportInfoText += y;
	viewportInfoText += ", w= ";
	viewportInfoText += w;
	viewportInfoText += ", h= ";
	viewportInfoText += h;

	viewportInfoText += ". Depth FPS:";
	viewportInfoText += mOverride->getPreviewHUD().fDepthFPS;
	viewportInfoText += ". Color FPS:";
	viewportInfoText += mOverride->getPreviewHUD().fColorFPS;
	if(mOverride->getPreviewHUD().bHasSkeleton)
		viewportInfoText += ". Has Skeleton";
	if(mOverride->getPreviewHUD().bHasFaceTracking)
		viewportInfoText += ". Has Face Tracking";

	/*{
		viewportInfoText += ". Cache index:";
		viewportInfoText += mOverride->getCachedTextureIndex();
	}*/
	drawManager2D.text( MPoint(w*0.5f, y+h*0.05f), viewportInfoText, MHWRender::MUIDrawManager::kCenter );

	// End draw UI
	drawManager2D.endDrawable();
}

///////////////////////////////////////////////////////////////////
