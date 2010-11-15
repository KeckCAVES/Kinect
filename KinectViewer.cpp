/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
Copyright (c) 2010 Oliver Kreylos

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

#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <GL/gl.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/GLVertex.h>
#include <GL/GLObject.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/Application.h>

#include "USBContext.h"
#include "FrameBuffer.h"
#include "KinectCamera.h"
#include "MD5MeshAnimator.h"

class KinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	typedef Geometry::ProjectiveTransformation<double,3> PTransform; // Type for projective transformations
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of vertex buffer object holding the current depth frame
		GLuint indexBufferId; // ID of index buffer object holding the current depth frame
		size_t numIndices; // Number of used vertex indices in the current index buffer
		unsigned int depthFrameVersion; // Version number of frame currently in vertex / index buffer
		GLuint textureId; // ID of texture object holding the current color frame
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	private:
	USBContext usbContext; // USB device context
	KinectCamera* kinectCamera; // Pointer to camera aspect of Kinect device
	PTransform textureMatrix; // The color camera calibration texture matrix
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of frame in currently locked color frame triple buffer slot
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of frame in currently locked depth frame triple buffer slot
	unsigned short* backgroundFrame; // Buffer for background removal
	int numBackgroundFrames; // Number of captured background frames
	MD5MeshAnimator anim; // An animator
	
	/* Private methods: */
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback called when a new color frame was received
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback called when a new depth frame was received
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int& argc,char**& argv,char**& appDefaults);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

/***************************************
Methods of class KinectViewer::DataItem:
***************************************/

KinectViewer::DataItem::DataItem(void)
	:vertexBufferId(0),indexBufferId(0),
	 numIndices(0),
	 depthFrameVersion(0),
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

KinectViewer::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&vertexBufferId);
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&indexBufferId);
	
	/* Destroy texture object: */
	glDeleteTextures(1,&textureId);
	}

/*****************************
Methods of class KinectViewer:
*****************************/

void KinectViewer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the color frame triple buffer: */
	colorFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void KinectViewer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the depth frame triple buffer: */
	depthFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

KinectViewer::KinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 kinectCamera(0),
	 depthFrameVersion(0),colorFrameVersion(0),
	 backgroundFrame(0),numBackgroundFrames(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to first Kinect camera device on the host: */
	kinectCamera=new KinectCamera(usbContext);
	
	/* Start streaming: */
	kinectCamera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer>(this,&KinectViewer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer>(this,&KinectViewer::depthStreamingCallback));
	
	/* Load the texture matrix: */
	Misc::File matrixFile("CameraCalibrationMatrix.dat","rb",Misc::File::LittleEndian);
	double mat[12];
	matrixFile.read<double>(mat,12);
	textureMatrix=PTransform::identity;
	for(int i=0;i<2;++i)
		for(int j=0;j<4;++j)
			textureMatrix.getMatrix()(i,j)=mat[i*4+j];
	for(int j=0;j<4;++j)
		textureMatrix.getMatrix()(3,j)=mat[2*4+j];
	
	/* Initialize navigation transformation: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	Vrui::setNavigationTransformation(nav);
	// Vrui::setNavigationTransformation(Vrui::Point(0,0,0),Vrui::Scalar(400),Vrui::Vector(0,1,0));
	// Vrui::setNavigationTransformation(Vrui::Point(320,240,-400),Vrui::Scalar(500),Vrui::Vector(0,1,0));
	}

KinectViewer::~KinectViewer(void)
	{
	delete[] backgroundFrame;
	
	/* Stop streaming: */
	kinectCamera->stopStreaming();
	
	/* Disconnect from the Kinect camera device: */
	delete kinectCamera;
	}

void KinectViewer::frame(void)
	{
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		++depthFrameVersion;
		
		/* Check whether to capture background removal frames: */
		if(numBackgroundFrames<30)
			{
			const FrameBuffer& frame=depthFrames.getLockedValue();
			int width=frame.getSize(0);
			int height=frame.getSize(1);
			const unsigned short* fPtr=static_cast<const unsigned short*>(frame.getBuffer());
			
			if(numBackgroundFrames==0)
				{
				/* Allocate and initialize the background removal buffer: */
				backgroundFrame=new unsigned short[height*width*2];
				unsigned short* bfPtr=backgroundFrame;
				for(int y=0;y<height;++y)
					for(int x=0;x<width;++x,++fPtr,bfPtr+=2)
						{
						if(*fPtr!=0x0U)
							{
							bfPtr[0]=*fPtr;
							bfPtr[1]=*fPtr;
							}
						else
							{
							bfPtr[0]=0xffffU;
							bfPtr[1]=0x0U;
							}
						}
				}
			else
				{
				/* Add the current frame to the background buffer: */
				unsigned short* bfPtr=backgroundFrame;
				for(int y=0;y<height;++y)
					for(int x=0;x<width;++x,++fPtr,bfPtr+=2)
						{
						if(*fPtr!=0x0U)
							{
							if(bfPtr[0]>*fPtr)
								bfPtr[0]=*fPtr;
							if(bfPtr[1]<*fPtr)
								bfPtr[1]=*fPtr;
							}
						}
				}
			
			++numBackgroundFrames;
			
			if(numBackgroundFrames==30)
				{
				/* Extend the background depth ranges: */
				unsigned short* bfPtr=backgroundFrame;
				for(int y=0;y<height;++y)
					for(int x=0;x<width;++x,bfPtr+=2)
						{
						if(bfPtr[0]>=5U)
							bfPtr[0]-=5;
						if(bfPtr[1]<=0xfffaU)
							bfPtr[1]+=5;
						}
				}
			}
		}
	
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	
	anim.frame();
	}

namespace {

/****************************************
Helper functions to create triangle sets:
****************************************/

inline unsigned short depthRange(unsigned short d0,unsigned short d1,unsigned short d2)
	{
	unsigned short min=d0;
	unsigned short max=d0;
	if(min>d1)
		min=d1;
	if(max<d1)
		max=d1;
	if(min>d2)
		min=d2;
	if(max<d2)
		max=d2;
	return max-min;
	}

}

void KinectViewer::display(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	glPushAttrib(GL_ENABLE_BIT|GL_POLYGON_BIT|GL_TEXTURE_BIT);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);
	
	/* Use a projective modelview matrix to re-project the depth image into world space: */
	glPushMatrix();
	PTransform pt=PTransform::identity;
	float scale=560.0f;
	pt.getMatrix()(0,0)=1.0f/scale;
	pt.getMatrix()(0,3)=-320.0f/scale;
	pt.getMatrix()(1,1)=1.0f/scale;
	pt.getMatrix()(1,3)=-240.0f/scale;
	pt.getMatrix()(2,2)=0.0f;
	pt.getMatrix()(2,3)=-1.0f;
	pt.getMatrix()(3,2)=-1.0f/34400.0f;
	pt.getMatrix()(3,3)=1090.0f/34400.0f;
	glMultMatrix(pt);
	
	typedef GLVertex<GLfloat,2,void,0,GLfloat,GLfloat,3> Vertex; // Type for vertices stored in vertex buffer object
	
	/* Bind the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	
	/* Check if the cached frame needs to be updated: */
	if(dataItem->depthFrameVersion!=depthFrameVersion)
		{
		/* Generate all vertices of the new frame: */
		const FrameBuffer& frame=depthFrames.getLockedValue();
		int width=frame.getSize(0);
		int height=frame.getSize(1);
		const unsigned short* rowPtr=static_cast<const unsigned short*>(frame.getBuffer());
		
		glBufferDataARB(GL_ARRAY_BUFFER_ARB,size_t(width)*size_t(height)*sizeof(Vertex),0,GL_DYNAMIC_DRAW_ARB);
		Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
		{
		const unsigned short* p1=rowPtr;
		const unsigned short* p2=p1+width;
		vPtr->texCoord=Vertex::TexCoord((float(0)+0.5f)/float(width),(float(0)+0.5f)/float(height));
		vPtr->normal=Vertex::Normal(p1[0]-p1[1],p1[0]-p2[0],-1.0f);
		vPtr->position=Vertex::Position(float(0)+0.5f,float(0)+0.5f,p1[0]);
		++vPtr;
		++p1;
		++p2;
		for(int x=1;x<width-1;++x,++vPtr,++p1,++p2)
			{
			vPtr->texCoord=Vertex::TexCoord((float(x)+0.5f)/float(width),(float(0)+0.5f)/float(height));
			vPtr->normal=Vertex::Normal((p1[-1]-p1[1])*0.5f,p1[0]-p2[0],-1.0f);
			vPtr->position=Vertex::Position(float(x)+0.5f,float(0)+0.5f,p1[0]);
			}
		vPtr->texCoord=Vertex::TexCoord((float(width-1)+0.5f)/float(width),(float(0)+0.5f)/float(height));
		vPtr->normal=Vertex::Normal(p1[-1]-p1[0],p1[0]-p2[0],-1.0f);
		vPtr->position=Vertex::Position(float(width-1)+0.5f,float(0)+0.5f,p1[0]);
		++vPtr;
		++p1;
		++p2;
		}
		rowPtr+=width;
		for(int y=1;y<height-1;++y,rowPtr+=width)
			{
			const unsigned short* p0=rowPtr-width;
			const unsigned short* p1=p0+width;
			const unsigned short* p2=p1+width;
			vPtr->texCoord=Vertex::TexCoord((float(0)+0.5f)/float(width),(float(y)+0.5f)/float(height));
			vPtr->normal=Vertex::Normal(p1[0]-p1[1],(p0[0]-p2[0])*0.5f,-1.0f);
			vPtr->position=Vertex::Position(float(0)+0.5f,float(y)+0.5f,p1[0]);
			++vPtr;
			++p0;
			++p1;
			++p2;
			for(int x=1;x<width-1;++x,++vPtr,++p0,++p1,++p2)
				{
				vPtr->texCoord=Vertex::TexCoord((float(x)+0.5f)/float(width),(float(y)+0.5f)/float(height));
				vPtr->normal=Vertex::Normal((p1[-1]-p1[1])*0.5f,(p0[0]-p2[0])*0.5f,-1.0f);
				vPtr->position=Vertex::Position(float(x)+0.5f,float(y)+0.5f,p1[0]);
				}
			vPtr->texCoord=Vertex::TexCoord((float(width-1)+0.5f)/float(width),(float(y)+0.5f)/float(height));
			vPtr->normal=Vertex::Normal(p1[-1]-p1[0],(p0[0]-p2[0])*0.5f,-1.0f);
			vPtr->position=Vertex::Position(float(width-1)+0.5f,float(y)+0.5f,p1[0]);
			++vPtr;
			++p0;
			++p1;
			++p2;
			}
		{
		const unsigned short* p0=rowPtr-width;
		const unsigned short* p1=p0+width;
		vPtr->texCoord=Vertex::TexCoord((float(0)+0.5f)/float(width),(float(height-1)+0.5f)/float(height));
		vPtr->normal=Vertex::Normal(p1[0]-p1[1],p0[0]-p1[0],-1.0f);
		vPtr->position=Vertex::Position(float(0)+0.5f,float(height-1)+0.5f,p1[0]);
		++vPtr;
		++p0;
		++p1;
		for(int x=1;x<width-1;++x,++vPtr,++p0,++p1)
			{
			vPtr->texCoord=Vertex::TexCoord((float(x)+0.5f)/float(width),(float(height-1)+0.5f)/float(height));
			vPtr->normal=Vertex::Normal((p1[-1]-p1[1])*0.5f,p0[0]-p1[0],-1.0f);
			vPtr->position=Vertex::Position(float(x)+0.5f,float(height-1)+0.5f,p1[0]);
			}
		vPtr->texCoord=Vertex::TexCoord((float(width-1)+0.5f)/float(width),(float(height-1)+0.5f)/float(height));
		vPtr->normal=Vertex::Normal(p1[-1]-p1[0],p0[0]-p1[0],-1.0f);
		vPtr->position=Vertex::Position(float(width-1)+0.5f,float(height-1)+0.5f,p1[0]);
		++vPtr;
		++p0;
		++p1;
		}
		glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
		
		/* Generate all triangles of the current frame: */
		unsigned short maxDepthRange=5U;
		
		rowPtr=static_cast<const unsigned short*>(frame.getBuffer());
		glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(width-1)*size_t(height-1)*2*3*sizeof(GLuint),0,GL_DYNAMIC_DRAW_ARB);
		GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
		GLuint rowI=0;
		dataItem->numIndices=0;
		const unsigned short* bfRowPtr=backgroundFrame;
		for(int y=1;y<height;++y,rowPtr+=width,rowI+=width,bfRowPtr+=width*2)
			{
			const unsigned short* p1=rowPtr;
			const unsigned short* p2=p1+width;
			GLuint i1=rowI;
			GLuint i2=i1+width;
			const unsigned short* bf1=bfRowPtr;
			const unsigned short* bf2=bf1+width*2;
			for(int x=1;x<width;++x,++p1,++p2,++i1,++i2,bf1+=2,bf2+=2)
				{
				/* Calculate the quad's validity case index: */
				unsigned int caseIndex=0x0;
				if(p1[0]!=0x0) // &&(p1[0]<bf1[0]||p1[0]>bf1[1]))
					caseIndex|=0x1U;
				if(p1[1]!=0x0) // &&(p1[1]<bf1[2]||p1[1]>bf1[3]))
					caseIndex|=0x2U;
				if(p2[0]!=0x0) // &&(p2[0]<bf2[0]||p2[0]>bf2[1]))
					caseIndex|=0x4U;
				if(p2[1]!=0x0) // &&(p2[1]<bf2[2]||p2[1]>bf2[3]))
					caseIndex|=0x8U;
				
				/* Create triangles according to the case index: */
				switch(caseIndex)
					{
					case 0x7U:
						if(depthRange(p1[0],p1[1],p2[0])<=maxDepthRange)
							{
							iPtr[0]=i1;
							iPtr[1]=i1+1;
							iPtr[2]=i2;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						break;
					
					case 0xbU:
						if(depthRange(p1[0],p1[1],p2[1])<=maxDepthRange)
							{
							iPtr[0]=i1;
							iPtr[1]=i1+1;
							iPtr[2]=i2+1;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						break;
					
					case 0xdU:
						if(depthRange(p1[0],p2[0],p2[1])<=maxDepthRange)
							{
							iPtr[0]=i1;
							iPtr[1]=i2;
							iPtr[2]=i2+1;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						break;
					
					case 0xeU:
						if(depthRange(p1[1],p2[0],p2[1])<=maxDepthRange)
							{
							iPtr[0]=i1+1;
							iPtr[1]=i2;
							iPtr[2]=i2+1;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						break;
					
					case 0xfU:
						if(depthRange(p1[0],p1[1],p2[0])<=maxDepthRange)
							{
							iPtr[0]=i1;
							iPtr[1]=i1+1;
							iPtr[2]=i2;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						if(depthRange(p2[0],p1[1],p2[1])<=maxDepthRange)
							{
							iPtr[0]=i2;
							iPtr[1]=i1+1;
							iPtr[2]=i2+1;
							iPtr+=3;
							dataItem->numIndices+=3;
							}
						break;
					}
				}
			}
		glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
		
		/* Mark the cache as up-to-date: */
		dataItem->depthFrameVersion=depthFrameVersion;
		}
	
	/* Bind the texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->textureId);
	
	/* Check if the texture needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		const FrameBuffer& frame=colorFrames.getLockedValue();
		int width=frame.getSize(0);
		int height=frame.getSize(1);
		const GLubyte* framePtr=static_cast<const GLubyte*>(frame.getBuffer());
		
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cache as up-to-date: */
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
	
	PTransform tm=textureMatrix;
	tm*=pt;
	glTexGendv(GL_S,GL_OBJECT_PLANE,&tm.getMatrix()(0,0));
	glTexGendv(GL_T,GL_OBJECT_PLANE,&tm.getMatrix()(1,0));
	glTexGendv(GL_Q,GL_OBJECT_PLANE,&tm.getMatrix()(3,0));
	
	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();
	glScaled(1.0/640.0,1.0/480.0,1.0);
	
	/* Draw the cached indexed triangle set: */
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	glDrawElements(GL_TRIANGLES,dataItem->numIndices,GL_UNSIGNED_INT,static_cast<const GLuint*>(0));
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	
	/* Protect the texture: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	/* Protect the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	/* Return to navigation space: */
	glPopMatrix();
	
	/* Restore OpenGL state: */
	glPopAttrib();
	
	glPushMatrix();
	glTranslated(7.71,-29.37,-94.57);
	glScaled(0.2,0.2,0.2);
	glRotated(120.0,0.0,1.0,0.0);
	glRotated(-90.0,1.0,0.0,0.0);
	anim.glRenderAction(contextData);
	glPopMatrix();
	}

void KinectViewer::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}

int main(int argc,char* argv[])
	{
	try
		{
		char** appDefaults=0;
		KinectViewer app(argc,argv,appDefaults);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
