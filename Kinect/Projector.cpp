/***********************************************************************
Projector - Class to project a depth frame captured from a Kinect camera
back into calibrated 3D camera space, and texture-map it with a matching
color frame.
Copyright (c) 2010-2017 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#include <Kinect/Projector.h>

#include <Misc/FunctionCalls.h>
#include <GL/gl.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/GLTransformationWrappers.h>

namespace Kinect {

/************************************
Methods of class Projector::DataItem:
************************************/

Projector::DataItem::DataItem(void)
	:vertexBufferId(0),
	 indexBufferId(0),
	 meshVersion(0),
	 textureId(0),
	 colorFrameVersion(0)
	{
	/* Check for vertex buffer object extension: */
	if(GLARBVertexBufferObject::isSupported())
		{
		/* Initialize the extension: */
		GLARBVertexBufferObject::initExtension();
		
		/* Allocate vertex and index buffers: */
		glGenBuffersARB(1,&vertexBufferId);
		glGenBuffersARB(1,&indexBufferId);
		}
	
	/* Allocate texture object: */
	glGenTextures(1,&textureId);
	}

Projector::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&vertexBufferId);
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&indexBufferId);
	
	/* Destroy texture object: */
	glDeleteTextures(1,&textureId);
	}

/**********************************
Static elements of class Projector:
**********************************/

const unsigned int Projector::quadCaseNumTriangles[16]={0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,2};

/**************************
Methods of class Projector:
**************************/

void* Projector::depthFrameProcessingThreadMethod(void)
	{
	unsigned int rawDepthFrameVersion=0;
	FrameBuffer rawDepthFrame;
	while(true)
		{
		/* Get the next incoming raw depth frame: */
		{
		Threads::MutexCond::Lock inDepthFrameLock(inDepthFrameCond);
		
		/* Wait until a new raw depth frame arrives: */
		while(rawDepthFrameVersion==inDepthFrameVersion)
			inDepthFrameCond.wait(inDepthFrameLock);
		
		/* Grab the new raw depth frame: */
		rawDepthFrameVersion=inDepthFrameVersion;
		rawDepthFrame=inDepthFrame;
		}
		
		/* Process the depth frame into a new slot in the mesh triple buffer: */
		MeshBuffer& newMesh=meshes.startNewValue();
		processDepthFrame(rawDepthFrame,newMesh);
		meshes.postNewValue();
		
		/* Call the mesh streaming callback: */
		if(streamingCallback!=0)
			(*streamingCallback)(newMesh);
		}
	
	return 0;
	}

Projector::Projector(void)
	:depthCorrection(0),
	 inDepthFrameVersion(0),
	 filterDepthFrames(false),lowpassDepthFrames(false),filteredDepthFrame(0),spatialFilterBuffer(0),
	 triangleDepthRange(5),
	 meshVersion(0),streamingCallback(0),colorFrameVersion(0)
	{
	/* Initialize the depth frame size: */
	for(int i=0;i<2;++i)
		depthSize[i]=0;
	}

Projector::Projector(FrameSource& frameSource)
	:GLObject(false),
	 depthCorrection(0),
	 inDepthFrameVersion(0),
	 filterDepthFrames(false),lowpassDepthFrames(false),filteredDepthFrame(0),spatialFilterBuffer(0),
	 triangleDepthRange(5),
	 meshVersion(0),streamingCallback(0),colorFrameVersion(0)
	{
	/* Set the depth frame size: */
	setDepthFrameSize(frameSource.getActualFrameSize(FrameSource::DEPTH));
	
	/* Query the source's depth correction parameters and calculate the depth correction buffer: */
	FrameSource::DepthCorrection* dc=frameSource.getDepthCorrectionParameters();
	setDepthCorrection(dc);
	delete dc;
	
	/* Query the source's intrinsic and extrinsic parameters: */
	FrameSource::IntrinsicParameters ips=frameSource.getIntrinsicParameters();
	depthLensDistortion=ips.depthLensDistortion;
	depthProjection=ips.depthProjection;
	colorProjection=ips.colorProjection;
	projectorTransform=frameSource.getExtrinsicParameters();
	worldDepthProjection=projectorTransform;
	worldDepthProjection*=depthProjection;
	
	GLObject::init();
	}

Projector::~Projector(void)
	{
	/* Stop background processing, just in case: */
	stopStreaming();
	
	/* Delete the frame filtering buffers: */
	delete[] filteredDepthFrame;
	delete[] spatialFilterBuffer;
	
	/* Delete the depth correction buffer: */
	delete[] depthCorrection;
	}

void Projector::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	if(dataItem->vertexBufferId!=0)
		{
		/* Initialize the vertex and index buffers: */
		glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
		glBufferDataARB(GL_ARRAY_BUFFER_ARB,size_t(depthSize[1])*size_t(depthSize[0])*sizeof(MeshBuffer::Vertex),0,GL_DYNAMIC_DRAW_ARB);
		glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
		glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(depthSize[1]-1)*size_t(depthSize[0]-1)*2*3*sizeof(MeshBuffer::Index),0,GL_DYNAMIC_DRAW_ARB); // Worst-case index buffer size
		glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
		}
	}

void Projector::setDepthFrameSize(const unsigned int newDepthFrameSize[2])
	{
	/* Copy the depth frame size: */
	for(int i=0;i<2;++i)
		depthSize[i]=newDepthFrameSize[i];
	
	/*********************************************************************
	Initialize the quad case vertex offset table:
	*********************************************************************/
	
	/* Case 0x7 - triangle in lower-left corner of quad: */
	quadCaseVertexOffsets[0x7][0]=0;
	quadCaseVertexOffsets[0x7][1]=1;
	quadCaseVertexOffsets[0x7][2]=depthSize[0];
	
	/* Case 0xb - triangle in lower-right corner of quad: */
	quadCaseVertexOffsets[0xb][0]=0;
	quadCaseVertexOffsets[0xb][1]=1;
	quadCaseVertexOffsets[0xb][2]=depthSize[0]+1;
	
	/* Case 0xd - triangle in upper-left corner of quad: */
	quadCaseVertexOffsets[0xd][0]=0;
	quadCaseVertexOffsets[0xd][1]=depthSize[0];
	quadCaseVertexOffsets[0xd][2]=depthSize[0]+1;
	
	/* Case 0xe - triangle in upper-right corner of quad: */
	quadCaseVertexOffsets[0xe][0]=1;
	quadCaseVertexOffsets[0xe][1]=depthSize[0];
	quadCaseVertexOffsets[0xe][2]=depthSize[0]+1;
	
	/* Case 0xf - two triangles in quad, split into lower-left and upper-right: */
	quadCaseVertexOffsets[0xf][0]=0;
	quadCaseVertexOffsets[0xf][1]=1;
	quadCaseVertexOffsets[0xf][2]=depthSize[0];
	quadCaseVertexOffsets[0xf][3]=depthSize[0];
	quadCaseVertexOffsets[0xf][4]=1;
	quadCaseVertexOffsets[0xf][5]=depthSize[0]+1;
	}

void Projector::setDepthCorrection(const FrameSource::DepthCorrection* dc)
	{
	if(dc!=0)
		{
		/* Evaluate the depth correction parameters to create a per-pixel depth value offset buffer: */
		depthCorrection=dc->getPixelCorrection(depthSize);
		}
	}

void Projector::setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips)
	{
	/* Get the depth camera's lens distortion correction parameters: */
	depthLensDistortion=ips.depthLensDistortion;
	
	/* Set the color and depth projection matrices: */
	depthProjection=ips.depthProjection;
	colorProjection=ips.colorProjection;
	
	/* Calculate the combined world-space depth projection matrix: */
	worldDepthProjection=projectorTransform;
	worldDepthProjection*=depthProjection;
	}

void Projector::setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps)
	{
	projectorTransform=eps;
	
	worldDepthProjection=projectorTransform;
	worldDepthProjection*=depthProjection;
	}

Projector::Point Projector::projectPoint(const Projector::Point& p) const
	{
	/* Transform the point from world space to depth image space: */
	Point dip=worldDepthProjection.inverseTransform(p);
	
	/* Apply inverse lens distortion correction if necessary: */
	if(!depthLensDistortion.isIdentity())
		{
		/* Calculate the undistorted pixel position in pixel space: */
		LensDistortion::Point dp=LensDistortion::Point(LensDistortion::Scalar(dip[0]),LensDistortion::Scalar(dip[1]));
		LensDistortion::Point up=depthLensDistortion.undistortPixel(dp);
		dip[0]=up[0];
		dip[1]=up[1];
		}
	
	if(depthCorrection!=0)
		{
		/* Apply per-pixel depth correction to the depth-image point: */
		int dipx=int(Math::floor(dip[0]));
		int dipy=int(Math::floor(dip[1]));
		if(dipx>=0&&(unsigned int)dipx<depthSize[0]&&dipy>=0&&(unsigned int)dipy<depthSize[1])
			{
			const PixelCorrection* dcPtr=depthCorrection+(dipy*depthSize[0]+dipx);
			dip[2]=(dip[2]-dcPtr->offset)/dcPtr->scale;
			}
		}
	
	return dip;
	}

void Projector::setFilterDepthFrames(bool newFilterDepthFrames,bool newLowpassDepthFrames)
	{
	/* Just set the flag; the depth frame processing thread will take care of the rest: */
	filterDepthFrames=newFilterDepthFrames;
	lowpassDepthFrames=newLowpassDepthFrames;
	}

void Projector::setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange)
	{
	/* Set the triangle depth range immediately; it won't kill the depth frame processing thread if changed in mid-process: */
	triangleDepthRange=newTriangleDepthRange;
	}

void Projector::processDepthFrame(const FrameBuffer& depthFrame,MeshBuffer& meshBuffer) const
	{
	/* Check if the buffer is invalid, or is still referenced by someone else: */
	if(!meshBuffer.isValid()||!meshBuffer.isPrivate())
		{
		/* Create a new mesh buffer of the largest possible size: */
		meshBuffer=MeshBuffer(depthSize[1]*depthSize[0],(depthSize[1]-1)*(depthSize[0]-1)*2);
		
		/* Initialize the x and y positions of all vertices: */
		MeshBuffer::Vertex* vPtr=meshBuffer.getVertices();
		
		/* Check if the depth camera requires lens distortion correction: */
		if(!depthLensDistortion.isIdentity())
			{
			/* Create a grid of undistorted pixel positions: */
			for(unsigned int y=0;y<depthSize[1];++y)
				for(unsigned int x=0;x<depthSize[0];++x,++vPtr)
					{
					/* Undistort the grid point in pixel space: */
					LensDistortion::Point up=depthLensDistortion.undistortPixel(x,y);
					vPtr->position[0]=GLfloat(up[0]);
					vPtr->position[1]=GLfloat(up[1]);
					}
			}
		else
			{
			/* Create a regular grid of pixel positions: */
			for(unsigned int y=0;y<depthSize[1];++y)
				for(unsigned int x=0;x<depthSize[0];++x,++vPtr)
					{
					vPtr->position[0]=GLfloat(x)+0.5f;
					vPtr->position[1]=GLfloat(y)+0.5f;
					}
			}
		}
	
	if(filterDepthFrames)
		{
		/*******************************************************************
		Temporally filter the incoming depth frame using a stupid-man's
		Kalman filter.
		*******************************************************************/
		
		if(filteredDepthFrame!=0)
			{
			/* Update the filtered frame buffer with the new raw frame and update the vertex array: */
			GLfloat* fdfPtr=filteredDepthFrame;
			const FrameSource::DepthPixel* dfPtr=depthFrame.getData<FrameSource::DepthPixel>();
			const PixelCorrection* dcPtr=depthCorrection;
			if(dcPtr!=0)
				{
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++fdfPtr,++dfPtr,++dcPtr)
						{
						GLfloat newDepth=dcPtr->correct(*dfPtr);
						
						/* If the new depth value is dissimilar, replace the old; otherwise, filter the old: */
						if(Math::abs(newDepth-*fdfPtr)>=3.0f)
							{
							/* Replace the old value: */
							*fdfPtr=newDepth;
							}
						else
							{
							/* Merge the old and new values: */
							*fdfPtr=(*fdfPtr*15.0f+newDepth*1.0f)/16.0f;
							}
						}
				}
			else
				{
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++fdfPtr,++dfPtr,++dcPtr)
						{
						GLfloat newDepth=*dfPtr;
						
						/* If the new depth value is dissimilar, replace the old; otherwise, filter the old: */
						if(Math::abs(newDepth-*fdfPtr)>=3.0f)
							{
							/* Replace the old value: */
							*fdfPtr=newDepth;
							}
						else
							{
							/* Merge the old and new values: */
							*fdfPtr=(*fdfPtr*15.0f+newDepth*1.0f)/16.0f;
							}
						}
				}
			}
		else
			{
			/* Initialize the filtered frame buffer with the new raw frame and update the vertex array: */
			filteredDepthFrame=new GLfloat[depthSize[1]*depthSize[0]];
			GLfloat* fdfPtr=filteredDepthFrame;
			const FrameSource::DepthPixel* dfPtr=depthFrame.getData<FrameSource::DepthPixel>();
			const PixelCorrection* dcPtr=depthCorrection;
			if(dcPtr!=0)
				{
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++fdfPtr,++dfPtr,++dcPtr)
						*fdfPtr=dcPtr->correct(*dfPtr);
				}
			else
				{
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++fdfPtr,++dfPtr,++dcPtr)
						*fdfPtr=*dfPtr;
				}
			}
		
		if(lowpassDepthFrames)
			{
			/* Filter the temporally-filtered frame with a spatial low-pass filter: */
			if(spatialFilterBuffer==0)
				spatialFilterBuffer=new GLfloat[depthSize[1]*depthSize[0]];
			GLfloat invalidDepth=GLfloat(FrameSource::invalidDepth);
			
			/***********************************
			First pass: filter frame vertically:
			***********************************/
			
			int stride=depthSize[0];
			for(unsigned int x=0;x<depthSize[0];++x)
				{
				// const FrameSource::DepthPixel* sCol=depthFrame.getData<FrameSource::DepthPixel>()+x;
				GLfloat* sCol=filteredDepthFrame+x;
				GLfloat* dCol=spatialFilterBuffer+x;
				// unsigned int sum=0;
				GLfloat sum=0.0f;
				GLfloat weight=0.0f;
				if(sCol[0]!=invalidDepth)
					{
					sum+=sCol[0]*2.0f;
					weight+=2.0f;
					}
				if(sCol[stride]!=invalidDepth)
					{
					sum+=sCol[stride];
					weight+=1.0f;
					}
				*dCol=weight!=0.0f?sum/weight:invalidDepth;
				sCol+=depthSize[0];
				dCol+=depthSize[0];
				for(unsigned int y=1;y<depthSize[1]-1;++y,sCol+=depthSize[0],dCol+=depthSize[0])
					{
					sum=0.0f;
					weight=0.0f;
					if(sCol[-stride]!=invalidDepth)
						{
						sum+=sCol[-stride];
						weight+=1.0f;
						}
					if(sCol[0]!=invalidDepth)
						{
						sum+=sCol[0]*2.0f;
						weight+=2.0f;
						}
					if(sCol[stride]!=invalidDepth)
						{
						sum+=sCol[stride];
						weight+=1.0f;
						}
					*dCol=weight!=0.0f?sum/weight:invalidDepth;
					}
				sum=0.0f;
				weight=0.0f;
				if(sCol[-stride]!=invalidDepth)
					{
					sum+=sCol[-stride];
					weight+=1.0f;
					}
				if(sCol[0]!=invalidDepth)
					{
					sum+=sCol[0]*2.0f;
					weight+=2.0f;
					}
				*dCol=weight!=0.0f?sum/weight:invalidDepth;
				}
			
			/**************************************
			Second pass: filter frame horizontally:
			**************************************/
			
			GLfloat* sPtr=spatialFilterBuffer;
			MeshBuffer::Vertex* vPtr=meshBuffer.getVertices();
			for(unsigned int y=0;y<depthSize[1];++y)
				{
				GLfloat sum=0.0f;
				GLfloat weight=0.0f;
				if(sPtr[0]!=invalidDepth)
					{
					sum+=sPtr[0]*2.0f;
					weight+=2.0f;
					}
				if(sPtr[1]!=invalidDepth)
					{
					sum+=sPtr[1];
					weight+=1.0f;
					}
				vPtr->position[2]=weight!=0.0f?sum/weight:invalidDepth;
				++sPtr;
				++vPtr;
				for(unsigned int x=1;x<depthSize[0]-1;++x,++sPtr,++vPtr)
					{
					sum=0.0f;
					weight=0.0f;
					if(sPtr[-1]!=invalidDepth)
						{
						sum+=sPtr[-1];
						weight+=1.0f;
						}
					if(sPtr[0]!=invalidDepth)
						{
						sum+=sPtr[0]*2.0f;
						weight+=2.0f;
						}
					if(sPtr[1]!=invalidDepth)
						{
						sum+=sPtr[1];
						weight+=1.0f;
						}
					vPtr->position[2]=weight!=0.0f?sum/weight:invalidDepth;
					}
				sum=0.0f;
				weight=0.0f;
				if(sPtr[-1]!=invalidDepth)
					{
					sum+=sPtr[-1];
					weight+=1.0f;
					}
				if(sPtr[0]!=invalidDepth)
					{
					sum+=sPtr[0]*2.0f;
					weight+=2.0f;
					}
				vPtr->position[2]=weight!=0.0f?sum/weight:invalidDepth;
				++sPtr;
				++vPtr;
				}
			}
		else
			{
			if(spatialFilterBuffer!=0)
				{
				delete[] spatialFilterBuffer;
				spatialFilterBuffer=0;
				}
			
			/* Copy the filtered depth frame into the mesh vertex buffer: */
			const float* fdfPtr=filteredDepthFrame;
			MeshBuffer::Vertex* vPtr=meshBuffer.getVertices();
			for(unsigned int y=0;y<depthSize[1];++y)
				for(unsigned int x=0;x<depthSize[0];++x,++fdfPtr,++vPtr)
					vPtr->position[2]=*fdfPtr;
			}
		}
	else
		{
		/* Delete the filtered frame buffers: */
		if(filteredDepthFrame!=0)
			{
			delete[] filteredDepthFrame;
			filteredDepthFrame=0;
			}
		if(spatialFilterBuffer!=0)
			{
			delete[] spatialFilterBuffer;
			spatialFilterBuffer=0;
			}
		
		/* Update the vertex array: */
		const FrameSource::DepthPixel* dfPtr=depthFrame.getData<FrameSource::DepthPixel>();
		MeshBuffer::Vertex* vPtr=meshBuffer.getVertices();
		const PixelCorrection* dcPtr=depthCorrection;
		if(dcPtr!=0)
			{
			for(unsigned int y=0;y<depthSize[1];++y)
				for(unsigned int x=0;x<depthSize[0];++x,++dfPtr,++dcPtr,++vPtr)
					vPtr->position[2]=dcPtr->correct(*dfPtr);
			}
		else
			{
			for(unsigned int y=0;y<depthSize[1];++y)
				for(unsigned int x=0;x<depthSize[0];++x,++dfPtr,++dcPtr,++vPtr)
					vPtr->position[2]=*dfPtr;
			}
		}
	
	/* Store the number of generated vertices: */
	meshBuffer.numVertices=depthSize[1]*depthSize[0];
	
	/*******************************************************************
	Create triangle indices for all valid pixels that don't exceed the
	valid depth range.
	*******************************************************************/
	
	/* Iterate through all quads and generate triangles: */
	FrameSource::DepthPixel tdr=triangleDepthRange; // Get the currently set triangle depth range
	meshBuffer.numTriangles=0;
	MeshBuffer::Index* tiPtr=meshBuffer.getTriangleIndices();
	const FrameSource::DepthPixel* dfRowPtr=depthFrame.getData<FrameSource::DepthPixel>();
	GLuint rowIndex=0;
	for(unsigned int y=1;y<depthSize[1];++y,dfRowPtr+=depthSize[0],rowIndex+=depthSize[0])
		{
		const FrameSource::DepthPixel* dfPtr=dfRowPtr;
		GLuint index=rowIndex;
		for(unsigned int x=1;x<depthSize[0];++x,++dfPtr,++index)
			{
			/* Calculate the quad's validity case index: */
			unsigned int caseIndex=0x0U;
			if(dfPtr[0]<FrameSource::invalidDepth-1)
				caseIndex|=0x1U;
			if(dfPtr[1]<FrameSource::invalidDepth-1)
				caseIndex|=0x2U;
			if(dfPtr[depthSize[0]]<FrameSource::invalidDepth-1)
				caseIndex|=0x4U;
			if(dfPtr[depthSize[0]+1]<FrameSource::invalidDepth-1)
				caseIndex|=0x8U;
			
			/* Generate candidate triangles according to the quad's case index: */
			const int* cvo=quadCaseVertexOffsets[caseIndex];
			for(unsigned int i=0;i<quadCaseNumTriangles[caseIndex];++i,cvo+=3)
				{
				/* Calculate the depth range of the candidate triangle: */
				FrameSource::DepthPixel minDepth,maxDepth;
				minDepth=maxDepth=dfPtr[cvo[0]];
				for(int j=1;j<3;++j)
					{
					if(minDepth>dfPtr[cvo[j]])
						minDepth=dfPtr[cvo[j]];
					if(maxDepth<dfPtr[cvo[j]])
						maxDepth=dfPtr[cvo[j]];
					}
				
				/* Generate the triangle if it doesn't exceed the maximum depth range: */
				if(maxDepth-minDepth<=tdr)
					{
					/* Generate the triangle: */
					for(int j=0;j<3;++j)
						*(tiPtr++)=index+cvo[j];
					++meshBuffer.numTriangles;
					}
				}
			}
		}
	
	/* Copy the depth buffer's time stamp: */
	meshBuffer.timeStamp=depthFrame.timeStamp;
	}

void Projector::startStreaming(Projector::StreamingCallback* newStreamingCallback)
	{
	/* Delete the old streaming callback and install the new one: */
	delete streamingCallback;
	streamingCallback=newStreamingCallback;
	
	/* Start the depth frame processing thread: */
	depthFrameProcessingThread.start(this,&Projector::depthFrameProcessingThreadMethod);
	}

void Projector::setDepthFrame(const FrameBuffer& newDepthFrame)
	{
	/* Put the new depth frame into the input slot and wake up the depth frame processing thread: */
	Threads::MutexCond::Lock inDepthFrameLock(inDepthFrameCond);
	++inDepthFrameVersion;
	inDepthFrame=newDepthFrame;
	inDepthFrameCond.signal();
	}

void Projector::setMesh(const MeshBuffer& newMesh)
	{
	/* Post the new mesh into the triple buffer: */
	meshes.postNewValue(newMesh);
	}

void Projector::setColorFrame(const FrameBuffer& newColorFrame)
	{
	/* Post the new color frame into the triple buffer: */
	colorFrames.postNewValue(newColorFrame);
	}

void Projector::stopStreaming(void)
	{
	if(!depthFrameProcessingThread.isJoined())
		{
		/* Shut down the depth processing thread: */
		depthFrameProcessingThread.cancel();
		depthFrameProcessingThread.join();
		}
	
	/* Delete the streaming callback: */
	delete streamingCallback;
	streamingCallback=0;
	}

void Projector::updateFrames(void)
	{
	/* Lock the most recent mesh: */
	if(meshes.lockNewValue())
		++meshVersion;
	
	/* Lock the most recent color frame: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	}

void Projector::glRenderAction(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_POLYGON_BIT|GL_TEXTURE_BIT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	
	/* Load the transformation projecting depth images into 3D world space: */
	glPushMatrix();
	glMultMatrix(worldDepthProjection);
	
	/* Get the currently locked mesh: */
	const MeshBuffer& mesh=meshes.getLockedValue();
	
	/* Bind the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(dataItem->meshVersion!=meshVersion)
		{
		/* Load the mesh's vertices into the vertex buffer object: */
		glBufferSubDataARB(GL_ARRAY_BUFFER_ARB,0,mesh.numVertices*sizeof(MeshBuffer::Vertex),mesh.getVertices());
		
		/* Load the mesh's triangle indices into the index buffer object: */
		glBufferSubDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0,mesh.numTriangles*3*sizeof(MeshBuffer::Index),mesh.getTriangleIndices());
		
		/* Mark the cached mesh as valid: */
		dataItem->meshVersion=meshVersion;
		}
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->textureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Get the currently locked color frame: */
		const FrameBuffer& colorFrame=colorFrames.getLockedValue();
		
		/* Upload the color frame into the texture object: */
		unsigned int width=colorFrame.getSize(0);
		unsigned int height=colorFrame.getSize(1);
		const GLubyte* framePtr=colorFrame.getData<GLubyte>();
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
		
		/* Upload the color texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	
	/* Enable texture mapping: */
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	
	/* Enable projective texture mapping: */
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glEnable(GL_TEXTURE_GEN_Q);
	glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	glTexGeni(GL_Q,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	
	/* Upload the projective texture matrix: */
	glTexGendv(GL_S,GL_OBJECT_PLANE,colorProjection.getMatrix().getEntries());
	glTexGendv(GL_T,GL_OBJECT_PLANE,colorProjection.getMatrix().getEntries()+4);
	glTexGendv(GL_Q,GL_OBJECT_PLANE,colorProjection.getMatrix().getEntries()+12);
	
	/* Draw the cached indexed triangle set: */
	GLVertexArrayParts::enable(MeshBuffer::Vertex::getPartsMask());
	glVertexPointer(static_cast<const MeshBuffer::Vertex*>(0));
	glDrawElements(GL_TRIANGLES,mesh.numTriangles*3,GL_UNSIGNED_INT,static_cast<const MeshBuffer::Index*>(0));
	GLVertexArrayParts::disable(MeshBuffer::Vertex::getPartsMask());
	
	/* Protect the color texture object: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	/* Protect the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	/* Restore the modelview matrix: */
	glPopMatrix();
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

}
