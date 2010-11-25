/***********************************************************************
KinectProjector - Class to project a depth frame captured from a Kinect
camera back into calibrated 3D camera space, and texture-map it with a
matching color frame.
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

#include "KinectProjector.h"

#include <Misc/File.h>
#include <GL/gl.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/GLVertex.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/GLTransformationWrappers.h>

#include "KinectCamera.h"

/******************************************
Methods of class KinectProjector::DataItem:
******************************************/

KinectProjector::DataItem::DataItem(void)
	:vertexBufferId(0),
	 indexBufferId(0),numIndices(0),
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

KinectProjector::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&vertexBufferId);
	if(vertexBufferId!=0)
		glDeleteBuffersARB(1,&indexBufferId);
	
	/* Destroy texture object: */
	glDeleteTextures(1,&textureId);
	}

/********************************
Methods of class KinectProjector:
********************************/

KinectProjector::KinectProjector(const char* calibrationFileName)
	:colorFrameVersion(0),depthFrameVersion(0)
	{
	/* Open the calibration file: */
	Misc::File calibrationFile(calibrationFileName,"rb",Misc::File::LittleEndian);
	
	/* Read the depth projection matrix: */
	calibrationFile.read(depthProjection.getMatrix().getEntries(),4*4);
	
	/* Read the color projection matrix: */
	calibrationFile.read(colorProjection.getMatrix().getEntries(),4*4);
	}

void KinectProjector::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}

void KinectProjector::setDepthFrame(const FrameBuffer& newDepthFrame)
	{
	/* Copy the depth frame: */
	depthFrame=newDepthFrame;
	
	/* Increment the frame version number to invalidate all per-context depth frame caches: */
	++depthFrameVersion;
	}

void KinectProjector::setColorFrame(const FrameBuffer& newColorFrame)
	{
	/* Copy the color frame: */
	colorFrame=newColorFrame;
	
	/* Increment the frame version number to invalidate all per-context color frame caches: */
	++colorFrameVersion;
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

void KinectProjector::draw(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_POLYGON_BIT|GL_TEXTURE_BIT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	
	/* Load the transformation projecting depth images into 3D camera space: */
	glPushMatrix();
	glMultMatrix(depthProjection);
	
	/* Bind the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(dataItem->depthFrameVersion!=depthFrameVersion)
		{
		/* Generate all vertices of the new frame: */
		unsigned int width=depthFrame.getSize(0);
		unsigned int height=depthFrame.getSize(1);
		
		/* Initialize and get a pointer to the vertex buffer object: */
		glBufferDataARB(GL_ARRAY_BUFFER_ARB,size_t(width)*size_t(height)*sizeof(Vertex),0,GL_DYNAMIC_DRAW_ARB);
		Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
		
		/* Upload vertices: */
		const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrame.getBuffer());
		for(unsigned int y=0;y<height;++y)
			for(unsigned int x=0;x<width;++x,++vPtr,++dfPtr)
				{
				vPtr->position[0]=GLfloat(x)+0.5f;
				vPtr->position[1]=GLfloat(y)+0.5f;
				vPtr->position[2]=GLfloat(*dfPtr);
				}
		
		/* Release the vertex buffer object: */
		glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
		
		/* Generate all triangles of the current frame: */
		unsigned short maxDepthRange=5U;
		
		/* Initialize and get a pointer to the index buffer object: */
		glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(width-1)*size_t(height-1)*2*3*sizeof(GLuint),0,GL_DYNAMIC_DRAW_ARB); // Worst-case index buffer size
		GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
		
		/* Generate a case table of triangle vertices for each possible quad validity case: */
		static const unsigned int caseNumTriangles[16]={0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,2};
		GLuint caseVertexOffsets[16][6];
		
		/* Case 0x7: */
		caseVertexOffsets[7][0]=0;
		caseVertexOffsets[7][1]=1;
		caseVertexOffsets[7][2]=width;
		
		/* Case 0xb: */
		caseVertexOffsets[11][0]=0;
		caseVertexOffsets[11][1]=1;
		caseVertexOffsets[11][2]=width+1;
		
		/* Case 0xd: */
		caseVertexOffsets[13][0]=0;
		caseVertexOffsets[13][1]=width;
		caseVertexOffsets[13][2]=width+1;
		
		/* Case 0xe: */
		caseVertexOffsets[14][0]=1;
		caseVertexOffsets[14][1]=width;
		caseVertexOffsets[14][2]=width+1;
		
		/* Case 0xf: */
		caseVertexOffsets[15][0]=0;
		caseVertexOffsets[15][1]=1;
		caseVertexOffsets[15][2]=width;
		caseVertexOffsets[15][3]=width;
		caseVertexOffsets[15][4]=1;
		caseVertexOffsets[15][5]=width+1;
		
		/* Generate triangle vertex indices: */
		dataItem->numIndices=0;
		const unsigned short* dfRowPtr=static_cast<const unsigned short*>(depthFrame.getBuffer());
		GLuint rowIndex=0;
		for(unsigned int y=1;y<height;++y,dfRowPtr+=width,rowIndex+=width)
			{
			/* Process a horizontal strip of triangles one quad at a time: */
			const unsigned short* dfPtr=dfRowPtr;
			GLuint index=rowIndex;
			for(unsigned int x=1;x<width;++x,++dfPtr,++index)
				{
				/* Calculate the quad's validity case index: */
				unsigned int caseIndex=0x0;
				if(dfPtr[0]!=KinectCamera::invalidDepth)
					caseIndex|=0x1U;
				if(dfPtr[1]!=KinectCamera::invalidDepth)
					caseIndex|=0x2U;
				if(dfPtr[width]!=KinectCamera::invalidDepth)
					caseIndex|=0x4U;
				if(dfPtr[width+1]!=KinectCamera::invalidDepth)
					caseIndex|=0x8U;
				
				/* Generate candidate triangles according to the quad's case index: */
				const GLuint* cvo=caseVertexOffsets[caseIndex];
				for(unsigned int i=0;i<caseNumTriangles[caseIndex];++i,cvo+=3)
					{
					/* Ensure that the candidate triangle is not a fringe triangle: */
					if(depthRange(dfPtr[cvo[0]],dfPtr[cvo[1]],dfPtr[cvo[2]])<=maxDepthRange)
						{
						/* Generate the triangle: */
						for(int j=0;j<3;++j)
							*(iPtr++)=index+cvo[j];
						dataItem->numIndices+=3;
						}
					}
				}
			}
		
		/* Release the index buffer object: */
		glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
		
		/* Mark the cached depth frame as valid: */
		dataItem->depthFrameVersion=depthFrameVersion;
		}
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->textureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		unsigned int width=colorFrame.getSize(0);
		unsigned int height=colorFrame.getSize(1);
		const GLubyte* framePtr=static_cast<const GLubyte*>(colorFrame.getBuffer());
		
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
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	glDrawElements(GL_TRIANGLES,dataItem->numIndices,GL_UNSIGNED_INT,static_cast<const GLuint*>(0));
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
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
