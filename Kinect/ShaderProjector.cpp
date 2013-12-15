/***********************************************************************
ShaderProjector - Class to project a depth frame captured from a Kinect
camera back into calibrated 3D camera space, and texture-map it with a
matching color frame using a custom shader to perform most processing on
the GPU. Copyright (c) 2013 Oliver Kreylos

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

#include <Kinect/ShaderProjector.h>

#include <Misc/ThrowStdErr.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <GL/gl.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/Extensions/GLARBGeometryShader4.h>
#include <GL/Extensions/GLARBMultitexture.h>
#include <GL/Extensions/GLARBTextureFloat.h>
#include <GL/Extensions/GLARBTextureNonPowerOfTwo.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/Extensions/GLARBTextureRg.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/GLGeometryVertex.h>
#include <GL/GLTransformationWrappers.h>

#define KINECT_SHADERPROJECTOR_USE_ZIGZAGSTRIP 0

namespace Kinect {

/******************************************
Methods of class ShaderProjector::DataItem:
******************************************/

ShaderProjector::DataItem::DataItem(void)
	:vertexBufferId(0),
	 indexBufferId(0),
	 depthCorrectionTextureId(0),
	 vertexShaderId(0),geometryShaderId(0),fragmentShaderId(0),
	 shaderProgramId(0),
	 depthTextureId(0),depthFrameVersion(0),
	 colorTextureId(0),colorFrameVersion(0)
	{
	/* Initialize the required OpenGL extensions: */
	GLARBFragmentShader::initExtension();
	GLARBGeometryShader4::initExtension();
	GLARBMultitexture::initExtension();
	GLARBShaderObjects::initExtension();
	GLARBTextureFloat::initExtension();
	GLARBTextureNonPowerOfTwo::initExtension();
	GLARBTextureRectangle::initExtension();
	GLARBTextureRg::initExtension();
	GLARBVertexBufferObject::initExtension();
	GLARBVertexShader::initExtension();
	
	/* Allocate static vertex and index buffers: */
	GLuint buffers[2];
	glGenBuffersARB(2,buffers);
	vertexBufferId=buffers[0];
	indexBufferId=buffers[1];
	
	/* Allocate the static depth correction texture and depth and color texture objects: */
	GLuint textures[3];
	glGenTextures(3,textures);
	depthCorrectionTextureId=textures[0];
	depthTextureId=textures[1];
	colorTextureId=textures[2];
	
	/* Create shader objects: */
	vertexShaderId=glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
	geometryShaderId=glCreateShaderObjectARB(GL_GEOMETRY_SHADER_ARB);
	fragmentShaderId=glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);
	
	/* Create the program object and attach the shaders: */
	shaderProgramId=glCreateProgramObjectARB();
	glAttachObjectARB(shaderProgramId,vertexShaderId);
	glAttachObjectARB(shaderProgramId,geometryShaderId);
	glAttachObjectARB(shaderProgramId,fragmentShaderId);
	}

ShaderProjector::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */
	GLuint buffers[2];
	buffers[0]=vertexBufferId;
	buffers[1]=indexBufferId;
	glDeleteBuffersARB(2,buffers);
	
	/* Destroy texture objects: */
	GLuint textures[3];
	textures[0]=depthCorrectionTextureId;
	textures[1]=depthTextureId;
	textures[2]=colorTextureId;
	glDeleteTextures(3,textures);
	
	/* Delete the shaders and shader program: */
	glDeleteObjectARB(vertexShaderId);
	glDeleteObjectARB(geometryShaderId);
	glDeleteObjectARB(fragmentShaderId);
	glDeleteObjectARB(shaderProgramId);
	}

void ShaderProjector::DataItem::buildShader(GLContextData& contextData)
	{
	/* Compile the vertex, geometry, and fragment shaders: */
	std::string shaderDir=KINECT_SHADERPROJECTOR_SHADERDIR;
	glCompileShaderFromFile(vertexShaderId,(shaderDir+"/RenderFacade.vs").c_str());
	#if KINECT_SHADERPROJECTOR_USE_ZIGZAGSTRIP
	glCompileShaderFromFile(geometryShaderId,(shaderDir+"/RenderFacadeZigZag.gs").c_str());
	#else
	glCompileShaderFromFile(geometryShaderId,(shaderDir+"/RenderFacade.gs").c_str());
	#endif
	glCompileShaderFromFile(fragmentShaderId,(shaderDir+"/RenderFacade.fs").c_str());
	
	/* Set the geometry shader's parameters: */
	#if KINECT_SHADERPROJECTOR_USE_ZIGZAGSTRIP
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_INPUT_TYPE_ARB,GL_TRIANGLES);
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_OUTPUT_TYPE_ARB,GL_TRIANGLE_STRIP);
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_VERTICES_OUT_ARB,3);
	#else
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_INPUT_TYPE_ARB,GL_LINES_ADJACENCY_ARB);
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_OUTPUT_TYPE_ARB,GL_TRIANGLE_STRIP);
	glProgramParameteriARB(shaderProgramId,GL_GEOMETRY_VERTICES_OUT_ARB,4);
	#endif
	
	/* Link the shader program: */
	glLinkProgramARB(shaderProgramId);
	
	/* Check if the program linked successfully: */
	GLint linkStatus;
	glGetObjectParameterivARB(shaderProgramId,GL_OBJECT_LINK_STATUS_ARB,&linkStatus);
	if(!linkStatus)
		{
		/* Get some more detailed information: */
		GLcharARB linkLogBuffer[2048];
		GLsizei linkLogSize;
		glGetInfoLogARB(shaderProgramId,sizeof(linkLogBuffer),&linkLogSize,linkLogBuffer);
		
		/* Signal an error: */
		Misc::throwStdErr("Error \"%s\" while linking shader program",linkLogBuffer);
		}
	
	/* Get the shader program's uniform variable locations: */
	shaderUniforms[0]=glGetUniformLocationARB(shaderProgramId,"depthSampler");
	shaderUniforms[1]=glGetUniformLocationARB(shaderProgramId,"depthCorrectionSampler");
	shaderUniforms[2]=glGetUniformLocationARB(shaderProgramId,"depthProjection");
	shaderUniforms[3]=glGetUniformLocationARB(shaderProgramId,"colorProjection");
	shaderUniforms[4]=glGetUniformLocationARB(shaderProgramId,"triangleDepthRange");
	shaderUniforms[5]=glGetUniformLocationARB(shaderProgramId,"colorSampler");
	}

/********************************
Methods of class ShaderProjector:
********************************/

ShaderProjector::ShaderProjector(void)
	:depthCorrection(0),
	 triangleDepthRange(5),
	 depthFrameVersion(0),colorFrameVersion(0)
	{
	/* Initialize the depth frame size: */
	for(int i=0;i<2;++i)
		depthSize[i]=0;
	}

ShaderProjector::ShaderProjector(FrameSource& frameSource)
	:GLObject(false),
	 depthCorrection(0),
	 triangleDepthRange(5),
	 depthFrameVersion(0),colorFrameVersion(0)
	{
	/* Set the depth frame size: */
	setDepthFrameSize(frameSource.getActualFrameSize(FrameSource::DEPTH));
	
	/* Query the source's depth correction parameters and calculate the depth correction buffer: */
	FrameSource::DepthCorrection* dc=frameSource.getDepthCorrectionParameters();
	setDepthCorrection(dc);
	delete dc;
	
	/* Query the source's intrinsic and extrinsic parameters: */
	setParameters(frameSource.getIntrinsicParameters(),frameSource.getExtrinsicParameters());
	
	/* Register this object with the current OpenGL context: */
	GLObject::init();
	}

ShaderProjector::~ShaderProjector(void)
	{
	/* Delete the depth correction buffer: */
	delete[] depthCorrection;
	}

void ShaderProjector::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Create the static (template) vertex buffer: */
	typedef GLGeometry::Vertex<void,0,void,0,void,GLfloat,3> Vertex;
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,size_t(depthSize[1])*size_t(depthSize[0])*sizeof(Vertex),0,GL_STATIC_DRAW_ARB);
	Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(unsigned int y=0;y<depthSize[1];++y)
		for(unsigned int x=0;x<depthSize[0];++x,++vPtr)
			{
			/* Intrinsic calibration matrices expect depth space vertices at integer pixel-center positions: */
			vPtr->position[0]=GLfloat(x)+0.5f;
			vPtr->position[1]=GLfloat(y)+0.5f;
			vPtr->position[2]=0.0f;
			}
	glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	
	#if KINECT_SHADERPROJECTOR_USE_ZIGZAGSTRIP
	
	/* Create static (template) zig-zag triangle strip index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(depthSize[1]-1)*size_t(depthSize[0]*2+1)*2*sizeof(GLuint),0,GL_STATIC_DRAW_ARB);
	GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	unsigned int baseIndex=0;
	for(unsigned int y=1;y<depthSize[1];++y,baseIndex+=depthSize[0])
		{
		/* Go from left to right creating bottom-left and top-right triangles: */
		iPtr[0]=baseIndex;
		++iPtr;
		for(unsigned int x=0;x<depthSize[0];++x,iPtr+=2)
			{
			iPtr[0]=baseIndex+x;
			iPtr[1]=baseIndex+depthSize[0]+x;
			}
		
		/* Go from right to left creating bottom-right and top-left triangles: */
		iPtr[0]=baseIndex+depthSize[0]+depthSize[0]-1;
		++iPtr;
		for(unsigned int x=0;x<depthSize[0];++x,iPtr+=2)
			{
			iPtr[0]=baseIndex+depthSize[0]-1-x;
			iPtr[1]=baseIndex+depthSize[0]+depthSize[0]-1-x;
			}
		}
	glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	#else
	
	/* Create static (template) line-with-adjacency index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(depthSize[1]-1)*size_t(depthSize[0]-1)*4*sizeof(GLuint),0,GL_STATIC_DRAW_ARB);
	GLuint* iPtr=static_cast<GLuint*>(glMapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	for(unsigned int y=1;y<depthSize[1];++y)
		for(unsigned int x=1;x<depthSize[0];++x,iPtr+=4)
			{
			iPtr[0]=y*depthSize[0]+(x-1);
			iPtr[1]=(y-1)*depthSize[0]+(x-1);
			iPtr[2]=(y-1)*depthSize[0]+x;
			iPtr[3]=y*depthSize[0]+x;
			}
	glUnmapBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	#endif
	
	/* Upload per-pixel depth correction coefficients as a 2-component float texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthCorrectionTextureId);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RG32F,depthSize[0],depthSize[1],0,GL_RG,GL_FLOAT,depthCorrection);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Prepare the depth frame texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthTextureId);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Prepare the color frame texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D,0);
	
	/* Build the initial version of the facade rendering shader: */
	dataItem->buildShader(contextData);
	}

void ShaderProjector::setDepthFrameSize(const unsigned int newDepthFrameSize[2])
	{
	/* Copy the depth frame size: */
	for(int i=0;i<2;++i)
		depthSize[i]=newDepthFrameSize[i];
	}

void ShaderProjector::setDepthCorrection(const FrameSource::DepthCorrection* dc)
	{
	/* Evaluate the depth correction parameters to create a per-pixel depth value offset buffer: */
	depthCorrection=dc->getPixelCorrection(depthSize);
	}

void ShaderProjector::setParameters(const FrameSource::IntrinsicParameters& ips,const FrameSource::ExtrinsicParameters& eps)
	{
	projectorTransform=eps;
	
	/* Calculate the combined depth projection matrix: */
	depthProjection=eps;
	depthProjection*=ips.depthProjection;
	
	/* Calculate the combined color projection matrix: */
	colorProjection=ips.colorProjection;
	// colorProjection*=ips.depthProjection;
	}

void ShaderProjector::setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange)
	{
	/* Set the triangle depth range immediately; it won't kill the depth frame processing thread if changed in mid-process: */
	triangleDepthRange=newTriangleDepthRange;
	}

void ShaderProjector::setDepthFrame(const FrameBuffer& newDepthFrame)
	{
	/* Post the new depth frame into the triple buffer: */
	depthFrames.postNewValue(newDepthFrame);
	}

void ShaderProjector::setColorFrame(const FrameBuffer& newColorFrame)
	{
	/* Post the new color frame into the triple buffer: */
	colorFrames.postNewValue(newColorFrame);
	}

void ShaderProjector::updateFrames(void)
	{
	/* Lock the most recent depth frame: */
	if(depthFrames.lockNewValue())
		++depthFrameVersion;
	
	/* Lock the most recent color frame: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	}

void ShaderProjector::glRenderAction(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_CULL_FACE);
	
	/* Activate the facade rendering shader: */
	glUseProgramObjectARB(dataItem->shaderProgramId);
	
	/* Bind the current depth frame texture: */
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthTextureId);
	if(dataItem->depthFrameVersion!=depthFrameVersion)
		{
		/* Get the currently locked depth frame: */
		const FrameBuffer& depthFrame=depthFrames.getLockedValue();
		
		/* Upload the depth frame into the texture object: */
		const GLushort* dfPtr=static_cast<const GLushort*>(depthFrame.getBuffer());
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_R16UI,depthSize[0],depthSize[1],0,GL_RED_INTEGER_EXT,GL_UNSIGNED_SHORT,dfPtr);
		
		/* Mark the cached depth frame as up-to-date: */
		dataItem->depthFrameVersion=depthFrameVersion;
		}
	glUniformARB(dataItem->shaderUniforms[0],0);
	
	/* Bind the depth correction texture: */
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthCorrectionTextureId);
	glUniformARB(dataItem->shaderUniforms[1],1);
	
	/* Calculate and upload the full depth projection matrix: */
	PTransform fullDP=glGetProjectionMatrix<GLfloat>();
	fullDP*=glGetModelviewMatrix<GLfloat>();
	fullDP*=depthProjection;
	glUniformMatrix4fvARB(dataItem->shaderUniforms[2],1,GL_TRUE,fullDP.getMatrix().getEntries());
	
	/* Upload the color projection matrix: */
	glUniformMatrix4fvARB(dataItem->shaderUniforms[3],1,GL_TRUE,colorProjection.getMatrix().getEntries());
	
	/* Upload the triangle depth range: */
	glUniformARB(dataItem->shaderUniforms[4],GLfloat(triangleDepthRange));
	
	/* Bind the current color frame texture: */
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Get the currently locked color frame: */
		const FrameBuffer& colorFrame=colorFrames.getLockedValue();
		
		/* Upload the color frame into the texture object: */
		unsigned int width=colorFrame.getSize(0);
		unsigned int height=colorFrame.getSize(1);
		const GLubyte* cfPtr=static_cast<const GLubyte*>(colorFrame.getBuffer());
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,cfPtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	glUniformARB(dataItem->shaderUniforms[5],2);
	
	/* Bind the vertex and index buffers: */
	typedef GLGeometry::Vertex<void,0,void,0,void,GLfloat,3> Vertex;
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	
	#if KINECT_SHADERPROJECTOR_USE_ZIGZAGSTRIP
	
	/* Draw the template zig-zag triangle strip: */
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	glDrawElements(GL_TRIANGLE_STRIP,(depthSize[1]-1)*(depthSize[0]*2+1)*2,GL_UNSIGNED_INT,static_cast<const GLuint*>(0));
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
	#else
	
	/* Draw the template lines-with-adjacency set: */
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	glDrawElements(GL_LINES_ADJACENCY_ARB,(depthSize[1]-1)*(depthSize[0]-1)*4,GL_UNSIGNED_INT,static_cast<const GLuint*>(0));
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
	#endif
	
	/* Protect the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	/* Protect the texture objects: */
	glBindTexture(GL_TEXTURE_2D,0);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Disable the facade rendering shader: */
	glUseProgramObjectARB(0);
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

}
