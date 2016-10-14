/***********************************************************************
ShaderProjector - Class to project a depth frame captured from a Kinect
camera back into calibrated 3D camera space, and texture-map it with a
matching color frame using a custom shader to perform most processing on
the GPU. Copyright (c) 2013-2016 Oliver Kreylos

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

#ifndef KINECT_SHADERPROJECTOR_INCLUDED
#define KINECT_SHADERPROJECTOR_INCLUDED

#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/LensDistortion.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

class ShaderProjector:public GLObject
	{
	/* Embedded classes: */
	private:
	typedef FrameSource::DepthCorrection::PixelCorrection PixelCorrection; // Type for per-pixel depth correction factors
	typedef Geometry::ProjectiveTransformation<GLfloat,3> PTransform; // Type for projective transformations
	typedef FrameSource::ExtrinsicParameters ProjectorTransform; // Type for transformations from 3D camera space to 3D world space
	
	struct DataItem:public GLObject::DataItem // Structure containing per-context state
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of vertex buffer object holding the static vertices used for all depth frames
		GLuint indexBufferId; // ID of index buffer object holding the static triangle strips used for all depth frames
		GLuint depthCorrectionTextureId; // ID of texture object holding the per-pixel depth correction parameters
		GLhandleARB vertexShaderId,geometryShaderId,fragmentShaderId; // Handle of shader objects
		GLhandleARB shaderProgramId; // Handle of linked shader program
		GLuint shaderUniforms[6]; // Locations of shader program's uniform variables
		GLuint depthTextureId; // ID of texture object holding the current depth frame
		unsigned int depthFrameVersion; // Version number of mesh currently in vertex / index buffer
		GLuint colorTextureId; // ID of texture object holding the current color frame
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		
		/* Methods: */
		void buildShader(bool depthCorrection,GLContextData& contextData); // Builds the shader program based on current OpenGL state
		};
	
	/* Elements: */
	unsigned int depthSize[2]; // Width and height of all incoming depth frames
	PixelCorrection* depthCorrection; // Buffer of per-pixel depth correction parameters
	ProjectorTransform projectorTransform; // Transformation from 3D camera space to 3D world space
	LensDistortion depthLensDistortion; // Lens distortion correction parameters for the depth camera
	PTransform depthProjection; // Projection transformation from depth image space into 3D camera space
	PTransform worldDepthProjection; // Projection transformation from depth image space into 3D world space
	PTransform colorProjection; // Projection transformation from 3D camera space into color image space
	FrameSource::DepthPixel triangleDepthRange; // Maximum depth distance between a triangle's vertices
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames ready for rendering
	unsigned int depthFrameVersion; // Version number of current depth frame
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames ready for rendering
	unsigned int colorFrameVersion; // Version number of current color frame
	
	/* Constructors and destructors: */
	public:
	ShaderProjector(void); // Creates a facade projector with uninitialized camera parameters
	ShaderProjector(FrameSource& frameSource); // Creates a facade projector for the given frame source
	~ShaderProjector(void);
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	const unsigned int* getDepthFrameSize(void) const // Returns the current depth frame size
		{
		return depthSize;
		}
	unsigned int getDepthFrameSize(int index) const // Ditto
		{
		return depthSize[index];
		}
	const PixelCorrection* getDepthCorrection(void) const // Returns the array of per-pixel depth correction factors
		{
		return depthCorrection;
		}
	const LensDistortion& getDepthLensDistortion(void) const // Returns the lens distortion correction parameters for the depth camera
		{
		return depthLensDistortion;
		}
	const PTransform& getDepthProjection(void) const // Returns the depth unprojection transformation from depth image space into 3D camera space
		{
		return depthProjection;
		}
	const PTransform& getColorProjection(void) const // Returns the color unprojection transformation from color image space into 3D camera space
		{
		return colorProjection;
		}
	void setDepthFrameSize(const unsigned int newDepthFrameSize[2]); // Sets the size of all future incoming depth frames
	void setDepthCorrection(const FrameSource::DepthCorrection* dc); // Enables per-pixel depth correction using the given depth correction parameters
	void setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips); // Sets the projector's intrinsic camera parameters
	void setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps); // Sets the projector's extrinsic camera parameters
	const ProjectorTransform& getProjectorTransform(void) const // Returns the transformation from camera to world space
		{
		return projectorTransform;
		}
	FrameSource::DepthPixel getTriangleDepthRange(void) const // Returns the maximum depth range for generated triangles
		{
		return triangleDepthRange;
		}
	void setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange); // Sets the maximum depth range for valid triangles
	void setDepthFrame(const FrameBuffer& newDepthFrame); // Updates the projector's current depth frame in streaming mode; can be called from any thread
	void setColorFrame(const FrameBuffer& newColorFrame); // Updates the projector's current color frame in streaming mode; can be called from any thread
	void updateFrames(void); // Selects the most recent depth and color frames for rendering; must be called from foreground thread
	double getColorTimeStamp(void) const // Returns the time stamp of the color frame currently locked for rendering
		{
		return colorFrames.getLockedValue().timeStamp;
		}
	double getMeshTimeStamp(void) const // Returns the time stamp of the triangle mesh currently locked for rendering
		{
		return depthFrames.getLockedValue().timeStamp;
		}
	void glRenderAction(GLContextData& contextData) const; // Draws the current depth and color frames in the current model coordinate system
	};

}

#endif
