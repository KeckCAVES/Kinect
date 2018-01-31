/***********************************************************************
Projector2 - Class to project a depth frame captured from a 3D camera
back into calibrated 3D camera space, and texture-map it with a matching
color frame, using a combination of shader and CPU work.
Copyright (c) 2016-2017 Oliver Kreylos

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

#include <Kinect/Projector2.h>

#include <string>
#include <Misc/PrintInteger.h>
#include <Misc/FunctionCalls.h>
#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLVertexArrayParts.h>
#include <GL/GLVertex.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/Extensions/GLARBMultitexture.h>
#include <GL/Extensions/GLARBTextureFloat.h>
#include <GL/Extensions/GLARBTextureNonPowerOfTwo.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/Extensions/GLARBTextureRg.h>
#include <GL/Extensions/GLEXTGpuShader4.h>
#include <GL/GLLightTracker.h>
#include <GL/GLTransformationWrappers.h>
#include <Kinect/Internal/Config.h>

// DEBUGGING
#include <iostream>

namespace Kinect {

/*************************************
Methods of class Projector2::DataItem:
*************************************/

Projector2::DataItem::DataItem(void)
	:vertexBufferId(0),depthCorrectionTextureId(0),
	 depthTextureId(0),indexBufferId(0),meshVersion(0),
	 colorTextureId(0),colorFrameVersion(0),
	 renderingShaderSettingsVersion(0),lightStateVersion(0)
	{
	/* Initialize the required OpenGL extensions: */
	GLARBMultitexture::initExtension();
	GLARBTextureFloat::initExtension();
	GLARBTextureNonPowerOfTwo::initExtension();
	GLARBTextureRectangle::initExtension();
	GLARBTextureRg::initExtension();
	GLARBVertexBufferObject::initExtension();
	GLEXTGpuShader4::initExtension();
	
	/* Allocate buffer objects: */
	GLuint buffers[2];
	glGenBuffersARB(2,buffers);
	vertexBufferId=buffers[0];
	indexBufferId=buffers[1];
	
	/* Allocate texture objects: */
	GLuint textures[3];
	glGenTextures(3,textures);
	depthCorrectionTextureId=textures[0];
	depthTextureId=textures[1];
	colorTextureId=textures[2];
	}

Projector2::DataItem::~DataItem(void)
	{
	/* Destroy buffer objects: */
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
	}

/***********************************
Static elements of class Projector2:
***********************************/

const unsigned int Projector2::quadCaseNumTriangles[16]={0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,2};

/***************************
Methods of class Projector2:
***************************/

void* Projector2::depthFrameProcessingThreadMethod(void)
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
		std::pair<FrameBuffer,MeshBuffer>& newMesh=meshes.startNewValue();
		
		if(filterDepthFrames)
			{
			const FrameSource::DepthPixel* dfPtr=rawDepthFrame.getData<FrameSource::DepthPixel>();
			newMesh.first=FrameBuffer(depthSize[0],depthSize[1],depthSize[1]*depthSize[0]*sizeof(FrameSource::DepthPixel));
			newMesh.first.timeStamp=rawDepthFrame.timeStamp;
			FrameSource::DepthPixel* mPtr=newMesh.first.getData<FrameSource::DepthPixel>();
			
			/* Check if the filtered depth frame buffer already exists: */
			if(filteredDepthFrame!=0)
				{
				/* Update the filtered depth frame with the current raw depth frame: */
				FrameSource::DepthPixel* fdfPtr=filteredDepthFrame;
				
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++dfPtr,++fdfPtr,++mPtr)
						{
						if(*dfPtr!=FrameSource::invalidDepth&&*fdfPtr!=FrameSource::invalidDepth)
							*mPtr=*fdfPtr=FrameSource::DepthPixel((((unsigned int)*fdfPtr*15U+(unsigned int)*dfPtr)+8U)>>4);
						else
							*mPtr=*fdfPtr=*dfPtr;
						}
				}
			else // filteredDepthFrame==0
				{
				/* Initialize the filtered depth frame with the current raw depth frame: */
				filteredDepthFrame=new FrameSource::DepthPixel[depthSize[1]*depthSize[0]];
				FrameSource::DepthPixel* fdfPtr=filteredDepthFrame;
				
				for(unsigned int y=0;y<depthSize[1];++y)
					for(unsigned int x=0;x<depthSize[0];++x,++dfPtr,++fdfPtr,++mPtr)
						*mPtr=*fdfPtr=*dfPtr;
				}
			}
		else // !filterDepthFrames
			{
			/* Release the filtered depth frame buffer if it still exists: */
			if(filteredDepthFrame!=0)
				{
				delete[] filteredDepthFrame;
				filteredDepthFrame=0;
				}
			
			/* Use the raw depth frame to create the mesh: */
			newMesh.first=rawDepthFrame;
			}
		processDepthFrame(newMesh.first,newMesh.second);
		meshes.postNewValue();
		
		/* Call the mesh streaming callback: */
		if(streamingCallback!=0)
			(*streamingCallback)(newMesh.second);
		}
	
	return 0;
	}

void Projector2::buildRenderingShader(DataItem* dataItem,GLLightTracker* lightTracker) const
	{
	/* Rebuild the rendering shader: */
	dataItem->renderingShader.reset();
	
	/* Start vertex shader's declarations: */
	std::string vertexShaderDeclarations="\
		#extension GL_ARB_texture_rectangle : enable\n\
		#extension GL_EXT_gpu_shader4 : enable\n\
		\n\
		uniform usampler2DRect depthSampler; // Sampler for depth image texture\n";
	
	/* Start vertex shader's main function: */
	std::string vertexShaderMain="\
		void main()\n\
			{\n\
			/* Get the pixel's depth value: */\n\
			float depth=float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy).r);\n";
	
	/* Check if per-pixel depth correction is required: */
	if(depthCorrection!=0)
		{
		/* Add to vertex shader's declarations: */
		vertexShaderDeclarations+="\
			uniform sampler2DRect depthCorrectionSampler; // Sampler for per-pixel depth correction texture\n";
		
		/* Add to vertex shader's main function: */
		vertexShaderMain+="\
				\n\
				/* Get the pixel's depth correction coefficients: */\n\
				vec2 depthCorrection=texture2DRect(depthCorrectionSampler,gl_MultiTexCoord0.xy).rg;\n\
				\n\
				/* Correct the pixel's depth: */\n\
				depth=depth*depthCorrection.x+depthCorrection.y;\n";
		}
	
	/* Continue vertex shader's main function: */
	vertexShaderMain+="\
			\n\
			/* Create the pixel's position in depth image space: */\n\
			vec4 diPixel=vec4(gl_Vertex.xy,depth,1.0);\n";
	
	/* Check if color texture mapping was requested: */
	if(mapTexture)
		{
		/* Add to vertex shader's declarations: */
		vertexShaderDeclarations+="\
			uniform mat4 colorProjection; // Projection from depth image space to color image space\n";
		
		/* Add to vertex shader's main function: */
		vertexShaderMain+="\
				\n\
				/* Project the pixel from depth image space to color image space: */\n\
				gl_TexCoord[0]=colorProjection*diPixel;\n";
		}
	
	/* Check if illumination was requested: */
	std::string vertexShaderFunctions;
	if(illuminate)
		{
		/* Add to vertex shader's declarations: */
		vertexShaderDeclarations+="\
			uniform mat4 depthProjection; // Projection from depth image space to eye space\n\
			uniform mat4 inverseTransposedDepthProjection; // Inverse transposed projection from depth image space into eye space for illumination\n\
			varying vec4 frontColor;\n";
		
		/* Add to vertex shader's main function: */
		vertexShaderMain+="\
				\n\
				/* Calculate the pixel's normal vector in depth image space, ignoring per-pixel depth correction: */\n\
				vec3 diNormal=vec3(float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy+vec2(1.0,0.0)).r)-float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy-vec2(1.0,0.0)).r),\n\
				                   float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy+vec2(0.0,1.0)).r)-float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy-vec2(0.0,1.0)).r),\n\
				                   -2.0);\n\
				\n\
				/* Calculate the pixel's normal plane in depth image space: */\n\
				vec4 diPlane=vec4(diNormal,-dot(diPixel.xyz,diNormal));\n\
				\n\
				/* Project the pixel to eye space: */\n\
				vec4 eyePixel=depthProjection*diPixel;\n\
				\n\
				/* Transform the normal plane to eye space: */\n\
				vec3 eyeNormal=normalize((inverseTransposedDepthProjection*diPlane).xyz);\n\
				\n\
				/* Initialize the color accumulators: */\n\
				vec4 ambientDiffuseAccumulator=gl_LightModel.ambient*gl_FrontMaterial.ambient;\n\
				vec4 specularAccumulator=vec4(0.0,0.0,0.0,0.0);\n\
				\n\
				/* Call light accumulation functions for all enabled light sources: */\n";
		
		/* Call the appropriate light accumulation function for every enabled light source: */
		for(int lightIndex=0;lightIndex<lightTracker->getMaxNumLights();++lightIndex)
			if(lightTracker->getLightState(lightIndex).isEnabled())
				{
				/* Create the light accumulation function: */
				vertexShaderFunctions+=lightTracker->createAccumulateLightFunction(lightIndex);
				
				/* Call the light accumulation function from the vertex shader's main function: */
				vertexShaderMain+="\
						accumulateLight";
				char liBuffer[12];
				vertexShaderMain.append(Misc::print(lightIndex,liBuffer+11));
				vertexShaderMain+="(eyePixel,eyeNormal,gl_FrontMaterial.ambient,gl_FrontMaterial.diffuse,gl_FrontMaterial.specular,gl_FrontMaterial.shininess,ambientDiffuseAccumulator,specularAccumulator);\n";
				}
		
		/* Finish vertex shader's main function: */
		vertexShaderMain+="\
				\n\
				/* Assign the final accumulated vertex color: */\n\
				frontColor=ambientDiffuseAccumulator+specularAccumulator;\n\
				\n\
				/* Project the pixel from eye space to clip space: */\n\
				gl_Position=gl_ProjectionMatrix*eyePixel;\n\
				}\n";
		}
	else
		{
		/* Add to vertex shader's declarations: */
		vertexShaderDeclarations+="\
			uniform mat4 depthProjection; // Projection from depth image space to clip space, bypassing eye space\n";
		
		if(!mapTexture)
			{
			/* Add to vertex shader's declarations: */
			vertexShaderDeclarations+="\
				varying vec4 frontColor;\n";
			
			/* Add to vertex shader's main function: */
			vertexShaderMain+="\
					\n\
					/* Assign uniform fragment color: */\n\
					frontColor=gl_Color;\n";
			}
		
		/* Finish vertex shader's main function: */
		vertexShaderMain+="\
				\n\
				/* Project the pixel from depth image space to clip space: */\n\
				gl_Position=depthProjection*diPixel;\n\
				}\n";
		}
	
	/* Compile the vertex shader: */
	dataItem->renderingShader.compileVertexShaderFromString((vertexShaderDeclarations+vertexShaderFunctions+vertexShaderMain).c_str());
	
	/* Start fragment shader's main function: */
	std::string fragmentShaderMain="\
		void main()\n\
			{\n";
	
	std::string fragmentShaderDeclarations;
	if(mapTexture)
		{
		/* Add to fragment shader's declarations: */
		fragmentShaderDeclarations+="\
			uniform sampler2D colorSampler; // Sampler for color image texture\n";
		
		if(illuminate)
			{
			/* Add to fragment shader's declarations: */
			fragmentShaderDeclarations+="\
				varying vec4 frontColor;\n";
			
			/* Finish fragment shader's main function: */
			fragmentShaderMain+="\
					gl_FragColor=texture2DProj(colorSampler,gl_TexCoord[0])*frontColor;\n\
					}\n";
			}
		else
			{
			/* Finish fragment shader's main function: */
			fragmentShaderMain+="\
					gl_FragColor=texture2DProj(colorSampler,gl_TexCoord[0]);\n\
					}\n";
			}
		}
	else
		{
		/* Add to fragment shader's declarations: */
		fragmentShaderDeclarations+="\
			varying vec4 frontColor;\n";
		
		/* Finish fragment shader's main function: */
		fragmentShaderMain+="\
				gl_FragColor=frontColor;\n\
				}\n";
		}
	
	/* Compile the fragment shader: */
	dataItem->renderingShader.compileFragmentShaderFromString((fragmentShaderDeclarations+fragmentShaderMain).c_str());
	
	/* Link the shader: */
	dataItem->renderingShader.linkShader();
	
	/* Query the rendering shader's uniform variable locations: */
	int* rsuPtr=dataItem->renderingShaderUniforms;
	*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("depthSampler");
	if(depthCorrection!=0)
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("depthCorrectionSampler");
	if(mapTexture)
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("colorProjection");
	if(illuminate)
		{
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("depthProjection");
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("inverseTransposedDepthProjection");
		}
	else
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("depthProjection");
	if(mapTexture)
		*(rsuPtr++)=dataItem->renderingShader.getUniformLocation("colorSampler");
	
	/* Mark the rendering shader as up-to-date: */
	dataItem->renderingShaderSettingsVersion=renderingShaderSettingsVersion;
	dataItem->lightStateVersion=lightTracker->getVersion();
	}

Projector2::Projector2(void)
	:depthCorrection(0),
	 inDepthFrameVersion(0),
	 filterDepthFrames(false),lowpassDepthFrames(false),filteredDepthFrame(0),spatialFilterBuffer(0),
	 mapTexture(true),illuminate(false),renderingShaderSettingsVersion(1),
	 triangleDepthRange(5),
	 meshVersion(0),streamingCallback(0),colorFrameVersion(0)
	{
	/* Initialize the depth frame size: */
	for(int i=0;i<2;++i)
		depthSize[i]=0;
	}

Projector2::Projector2(FrameSource& frameSource)
	:GLObject(false),
	 depthCorrection(0),
	 inDepthFrameVersion(0),
	 filterDepthFrames(false),lowpassDepthFrames(false),filteredDepthFrame(0),spatialFilterBuffer(0),
	 mapTexture(true),illuminate(false),renderingShaderSettingsVersion(1),
	 triangleDepthRange(5),
	 meshVersion(0),streamingCallback(0),colorFrameVersion(0)
	{
	/* Set the depth frame size: */
	setDepthFrameSize(frameSource.getActualFrameSize(FrameSource::DEPTH));
	
	/* Query the source's depth correction parameters and calculate the depth correction buffer: */
	FrameSource::DepthCorrection* dc=frameSource.getDepthCorrectionParameters();
	if(dc!=0)
		{
		/* Evaluate the depth correction parameters to create a per-pixel depth value offset buffer: */
		depthCorrection=dc->getPixelCorrection(depthSize);
		
		/* Delete the temporary depth correction object: */
		delete dc;
		}
	
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

Projector2::~Projector2(void)
	{
	/* Stop background processing, just in case: */
	stopStreaming();
	
	/* Delete the frame filtering buffers: */
	delete[] filteredDepthFrame;
	delete[] spatialFilterBuffer;
	
	/* Delete the depth correction buffer: */
	delete[] depthCorrection;
	}

void Projector2::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Create the template vertex buffer: */
	typedef GLVertex<GLfloat,2,void,0,void,GLfloat,3> Vertex; // Type for vertices
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,size_t(depthSize[1])*size_t(depthSize[0])*sizeof(Vertex),0,GL_STATIC_DRAW_ARB);
	Vertex* vPtr=static_cast<Vertex*>(glMapBufferARB(GL_ARRAY_BUFFER_ARB,GL_WRITE_ONLY_ARB));
	
	/* Check if the depth camera requires lens distortion correction: */
	if(!depthLensDistortion.isIdentity())
		{
		/* Create a grid of undistorted pixel positions: */
		for(unsigned int y=0;y<depthSize[1];++y)
			for(unsigned int x=0;x<depthSize[0];++x,++vPtr)
				{
				/* Store the undistorted pixel position for depth texture look-up: */
				vPtr->texCoord[0]=GLfloat(x)+0.5f;
				vPtr->texCoord[1]=GLfloat(y)+0.5f;
				
				/* Calculate the undistorted pixel position in pixel space: */
				LensDistortion::Point up=depthLensDistortion.undistortPixel(x,y);
				vPtr->position[0]=GLfloat(up[0]);
				vPtr->position[1]=GLfloat(up[1]);
				vPtr->position[2]=0.0f;
				}
		}
	else
		{
		for(unsigned int y=0;y<depthSize[1];++y)
			for(unsigned int x=0;x<depthSize[0];++x,++vPtr)
				{
				/* Intrinsic calibration matrices expect depth space vertices at integer pixel-center positions: */
				vPtr->texCoord[0]=GLfloat(x)+0.5f;
				vPtr->texCoord[1]=GLfloat(y)+0.5f;
				vPtr->position[0]=GLfloat(x)+0.5f;
				vPtr->position[1]=GLfloat(y)+0.5f;
				vPtr->position[2]=0.0f;
				}
		}
	
	glUnmapBufferARB(GL_ARRAY_BUFFER_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	
	/* Initialize the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,size_t(depthSize[1]-1)*size_t(depthSize[0]-1)*2*3*sizeof(MeshBuffer::Index),0,GL_DYNAMIC_DRAW_ARB); // Worst-case index buffer size
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	if(depthCorrection!=0)
		{
		/* Upload per-pixel depth correction coefficients as a 2-component float texture: */
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthCorrectionTextureId);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RG32F,depthSize[0],depthSize[1],0,GL_RG,GL_FLOAT,depthCorrection);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
		}
	
	/* Prepare the depth frame texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthTextureId);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_BASE_LEVEL,0);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Prepare the color frame texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D,0);
	}

void Projector2::setDepthFrameSize(const unsigned int newDepthFrameSize[2])
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

void Projector2::setDepthCorrection(const FrameSource::DepthCorrection* dc)
	{
	/* Delete the current per-pixel depth correction buffer: */
	delete depthCorrection;
	depthCorrection=0;
	
	if(dc!=0)
		{
		/* Evaluate the depth correction parameters to create a per-pixel depth correction buffer: */
		depthCorrection=dc->getPixelCorrection(depthSize);
		}
	}

void Projector2::setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips)
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

void Projector2::setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps)
	{
	projectorTransform=eps;
	
	worldDepthProjection=projectorTransform;
	worldDepthProjection*=depthProjection;
	}

void Projector2::setFilterDepthFrames(bool newFilterDepthFrames,bool newLowpassDepthFrames)
	{
	/* Just set the flag; the depth frame processing thread will take care of the rest: */
	filterDepthFrames=newFilterDepthFrames;
	lowpassDepthFrames=newLowpassDepthFrames;
	}

void Projector2::setMapTexture(bool newMapTexture)
	{
	/* Set the texture mapping flag: */
	mapTexture=newMapTexture;
	
	/* Invalidate the rendering shader: */
	++renderingShaderSettingsVersion;
	}

void Projector2::setIlluminate(bool newIlluminate)
	{
	/* Set the illumination flag: */
	illuminate=newIlluminate;
	
	/* Invalidate the rendering shader: */
	++renderingShaderSettingsVersion;
	}

void Projector2::setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange)
	{
	/* Set the triangle depth range immediately; it won't kill the depth frame processing thread if changed in mid-process: */
	triangleDepthRange=newTriangleDepthRange;
	}

void Projector2::processDepthFrame(const FrameBuffer& depthFrame,MeshBuffer& meshBuffer) const
	{
	/* Check if the buffer is invalid, or is still referenced by someone else: */
	if(!meshBuffer.isValid()||!meshBuffer.isPrivate())
		{
		/* Create a new mesh buffer of the largest possible size: */
		meshBuffer=MeshBuffer(0,(depthSize[1]-1)*(depthSize[0]-1)*2);
		meshBuffer.numVertices=0;
		}
	
	#if 0
	
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
		}
	
	#endif
	
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

void Projector2::startStreaming(Projector2::StreamingCallback* newStreamingCallback)
	{
	/* Delete the old streaming callback and install the new one: */
	delete streamingCallback;
	streamingCallback=newStreamingCallback;
	
	/* Start the depth frame processing thread: */
	depthFrameProcessingThread.start(this,&Projector2::depthFrameProcessingThreadMethod);
	}

void Projector2::setDepthFrame(const FrameBuffer& newDepthFrame)
	{
	/* Put the new depth frame into the input slot and wake up the depth frame processing thread: */
	Threads::MutexCond::Lock inDepthFrameLock(inDepthFrameCond);
	++inDepthFrameVersion;
	inDepthFrame=newDepthFrame;
	inDepthFrameCond.signal();
	}

void Projector2::setMesh(const FrameBuffer& newDepthFrame,const MeshBuffer& newMesh)
	{
	/* Post the new depth frame and mesh into the triple buffer: */
	std::pair<FrameBuffer,MeshBuffer>& newMeshBuffer=meshes.startNewValue();
	newMeshBuffer.first=newDepthFrame;
	newMeshBuffer.second=newMesh;
	meshes.postNewValue();
	}

void Projector2::setColorFrame(const FrameBuffer& newColorFrame)
	{
	/* Post the new color frame into the triple buffer: */
	colorFrames.postNewValue(newColorFrame);
	}

void Projector2::stopStreaming(void)
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

void Projector2::updateFrames(void)
	{
	/* Lock the most recent mesh: */
	if(meshes.lockNewValue())
		++meshVersion;
	
	/* Lock the most recent color frame: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	}

void Projector2::glRenderAction(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_CULL_FACE);
	
	/* Check if the facade rendering shader is outdated: */
	GLLightTracker* lightTracker=contextData.getLightTracker();
	if(dataItem->renderingShaderSettingsVersion!=renderingShaderSettingsVersion||dataItem->lightStateVersion!=lightTracker->getVersion())
		{
		/* Rebuild the facade rendering shader: */
		buildRenderingShader(dataItem,lightTracker);
		}
	
	/* Activate the facade rendering shader: */
	dataItem->renderingShader.useProgram();
	int* rsuPtr=dataItem->renderingShaderUniforms;
	
	/* Bind the vertex and index buffers: */
	typedef GLVertex<GLfloat,2,void,0,void,GLfloat,3> Vertex;
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->vertexBufferId);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId);
	
	/* Bind the current depth frame texture: */
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthTextureId);
	const MeshBuffer& mesh=meshes.getLockedValue().second;
	if(dataItem->meshVersion!=meshVersion)
		{
		/* Get the currently locked depth frame: */
		const FrameBuffer& depthFrame=meshes.getLockedValue().first;
		
		/* Upload the depth frame into the texture object: */
		const GLushort* dfPtr=depthFrame.getData<GLushort>();
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_R16UI,depthSize[0],depthSize[1],0,GL_RED_INTEGER_EXT,GL_UNSIGNED_SHORT,dfPtr);
		
		/* Load the mesh's triangle indices into the index buffer object: */
		glBufferSubDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0,mesh.numTriangles*3*sizeof(MeshBuffer::Index),mesh.getTriangleIndices());
		
		/* Mark the cached mesh as valid: */
		dataItem->meshVersion=meshVersion;
		}
	glUniformARB(*(rsuPtr++),0);
	
	if(depthCorrection!=0)
		{
		/* Bind the depth correction texture: */
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->depthCorrectionTextureId);
		glUniformARB(*(rsuPtr++),1);
		}
	
	if(mapTexture)
		{
		/* Upload the color projection matrix: */
		glUniformARB(*(rsuPtr++),colorProjection);
		}
	
	if(illuminate)
		{
		/* Set surface material properties (this should be done by caller, ideally): */
		if(mapTexture)
			{
			glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT_AND_BACK,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
			glMaterialSpecular(GLMaterialEnums::FRONT_AND_BACK,GLColor<GLfloat,4>(0.0f,0.0f,0.0f));
			glMaterialShininess(GLMaterialEnums::FRONT_AND_BACK,0.0f);
			}
		else
			{
			glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT_AND_BACK,GLColor<GLfloat,4>(0.3f,0.5f,1.0f));
			glMaterialSpecular(GLMaterialEnums::FRONT_AND_BACK,GLColor<GLfloat,4>(1.0f,1.0f,1.0f));
			glMaterialShininess(GLMaterialEnums::FRONT_AND_BACK,128.0f);
			}
		
		/* Calculate and upload the depth projection matrix from depth image space to eye space: */
		PTransform fullDP=glGetModelviewMatrix<GLfloat>();
		fullDP*=worldDepthProjection;
		glUniformARB(*(rsuPtr++),fullDP);
		
		/* Calculate and upload the transposed inverse depth projection matrix from depth image space to eye space: */
		PTransform invFullDP=Geometry::invert(fullDP);
		PTransform::Matrix& ifdpm=invFullDP.getMatrix();
		for(int i=0;i<4;++i)
			for(int j=i+1;j<4;++j)
				std::swap(ifdpm(i,j),ifdpm(j,i));
		glUniformARB(*(rsuPtr++),invFullDP);
		}
	else
		{
		/* Calculate and upload the full depth projection matrix: */
		PTransform fullDP=glGetProjectionMatrix<GLfloat>();
		fullDP*=glGetModelviewMatrix<GLfloat>();
		fullDP*=worldDepthProjection;
		glUniformARB(*(rsuPtr++),fullDP);
		}
	
	if(mapTexture)
		{
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
			const GLubyte* cfPtr=colorFrame.getData<GLubyte>();
			glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,cfPtr);
			
			/* Mark the cached color frame as up-to-date: */
			dataItem->colorFrameVersion=colorFrameVersion;
			}
		glUniformARB(*(rsuPtr++),2);
		}
	
	/* Draw the cached indexed triangle set: */
	GLVertexArrayParts::enable(Vertex::getPartsMask());
	glVertexPointer(static_cast<const Vertex*>(0));
	glDrawElements(GL_TRIANGLES,mesh.numTriangles*3,GL_UNSIGNED_INT,static_cast<const MeshBuffer::Index*>(0));
	GLVertexArrayParts::disable(Vertex::getPartsMask());
	
	/* Protect the texture objects: */
	if(mapTexture)
		glBindTexture(GL_TEXTURE_2D,0);
	if(depthCorrection!=0)
		{
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
		}
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Protect the vertex and index buffers: */
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	/* Disable the facade rendering shader: */
	GLShader::disablePrograms();
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

}
