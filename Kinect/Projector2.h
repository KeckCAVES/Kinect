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

#ifndef KINECT_PROJECTOR2_INCLUDED
#define KINECT_PROJECTOR2_INCLUDED

#include <utility>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/GLShader.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/LensDistortion.h>
#include <Kinect/FrameSource.h>
#include <Kinect/MeshBuffer.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
class GLLightTracker;

namespace Kinect {

class Projector2:public GLObject
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<const MeshBuffer&> StreamingCallback; // Function call type for streaming callbacks
	private:
	typedef FrameSource::DepthCorrection::PixelCorrection PixelCorrection; // Type for per-pixel depth correction factors
	typedef FrameSource::IntrinsicParameters::PTransform PTransform; // Type for projective transformations
	typedef FrameSource::ExtrinsicParameters ProjectorTransform; // Type for transformations from 3D camera space to 3D world space
	
	struct DataItem:public GLObject::DataItem // Structure containing per-context state
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of static vertex buffer object holding the template vertices for the connected frame source
		GLuint depthCorrectionTextureId; // ID of static texture object holding the per-pixel depth correction parameters
		GLuint depthTextureId; // ID of texture object holding the current depth frame
		GLuint indexBufferId; // ID of index buffer object holding the triangles of the current depth frame
		unsigned int meshVersion; // Version number of mesh currently in depth texture / index buffer
		GLuint colorTextureId; // ID of texture object holding the current color frame
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		GLShader renderingShader; // The facade rendering shader
		unsigned int renderingShaderSettingsVersion; // Version number of settings built into the current rendering shader
		unsigned int lightStateVersion; // Version number of OpenGL lighting state
		int renderingShaderUniforms[6]; // The uniform variable locations of the facade rendering shader
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	static const unsigned int quadCaseNumTriangles[16]; // Number of triangles to be generated for each quad corner validity case
	unsigned int depthSize[2]; // Width and height of all incoming depth frames
	LensDistortion depthLensDistortion; // Lens distortion correction parameters for the depth camera
	PTransform depthProjection; // Projection transformation from depth image space into 3D camera space
	PTransform colorProjection; // Projection transformation from 3D camera space into color image space
	ProjectorTransform projectorTransform; // Transformation from 3D camera space into 3D world space
	PTransform worldDepthProjection; // Projection transformation from depth image space into 3D world space
	PixelCorrection* depthCorrection; // Buffer of per-pixel depth correction parameters
	Threads::MutexCond inDepthFrameCond; // Condition variable to signal arrival of a new depth frame
	unsigned int inDepthFrameVersion; // Version number of most-recently arrived raw depth frame
	FrameBuffer inDepthFrame; // Most-recently arrived raw depth frame
	bool filterDepthFrames; // Flag if temporal depth frame filtering is enabled
	bool lowpassDepthFrames; // Flag it spatial depth frame filtering is enabled
	mutable FrameSource::DepthPixel* filteredDepthFrame; // Temporally filtered depth frame, same version number as current depth frame
	mutable GLfloat* spatialFilterBuffer; // Intermediate buffer to filter depth frames spatially
	bool mapTexture; // Flag whether to map the color texture onto the 3D geometry, or render as raw lit surfaces
	bool illuminate; // Flag whether to illuminate the 3D geometry from all active light sources
	unsigned int renderingShaderSettingsVersion; // Version number of rendering shader settings
	int quadCaseVertexOffsets[16][6]; // Offsets of triangle vertices to be used for each quad corner validity case
	FrameSource::DepthPixel triangleDepthRange; // Maximum depth distance between a triangle's vertices
	Threads::Thread depthFrameProcessingThread; // Background thread to process incoming depth frames for rendering
	Threads::TripleBuffer<std::pair<FrameBuffer,MeshBuffer> > meshes; // Triple buffer of meshes ready for rendering
	unsigned int meshVersion; // Version number of current mesh
	StreamingCallback* streamingCallback; // Function to be called when a new mesh has been produced
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames ready for rendering
	unsigned int colorFrameVersion; // Version number of current color frame
	
	/* Private methods: */
	void* depthFrameProcessingThreadMethod(void); // Thread method for background depth frame processing
	void buildRenderingShader(DataItem* dataItem,GLLightTracker* lightTracker) const; // Builds the rendering shader based on current settings or OpenGL state
	
	/* Constructors and destructors: */
	public:
	Projector2(void); // Creates a facade projector with uninitialized camera parameters
	Projector2(FrameSource& frameSource); // Creates a facade projector for the given frame source
	~Projector2(void);
	
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
	bool getFilterDepthFrames(void) const // Returns true if depth frame filtering is enabled
		{
		return filterDepthFrames;
		}
	void setFilterDepthFrames(bool newFilterDepthFrames,bool newLowpassDepthFrames); // Enables or disables temporal and spatial depth frame filtering
	void setMapTexture(bool newMapTexture); // Sets the texture mapping flag
	void setIlluminate(bool newIlluminate); // Sets the illumination flag
	FrameSource::DepthPixel getTriangleDepthRange(void) const // Returns the maximum depth range for generated triangles
		{
		return triangleDepthRange;
		}
	void setTriangleDepthRange(FrameSource::DepthPixel newTriangleDepthRange); // Sets the maximum depth range for valid triangles
	void processDepthFrame(const FrameBuffer& depthFrame,MeshBuffer& meshBuffer) const; // Processes the given depth frame into the given mesh buffer immediately and returns the resuling mesh
	void startStreaming(StreamingCallback* newStreamingCallback); // Starts processing depth frames in the background; calls the provided callback function every time a new mesh is produced
	void setDepthFrame(const FrameBuffer& newDepthFrame); // Updates the projector's current depth frame in streaming mode; can be called from any thread
	void setMesh(const FrameBuffer& newDepthFrame,const MeshBuffer& newMesh); // Updates the projector's current mesh in streaming mode; can be called from any thread
	void setColorFrame(const FrameBuffer& newColorFrame); // Updates the projector's current color frame in streaming mode; can be called from any thread
	void stopStreaming(void); // Stops background processing of depth frames
	void updateFrames(void); // Selects the most recent depth and color frames for rendering; must be called from foreground thread
	double getColorTimeStamp(void) const // Returns the time stamp of the color frame currently locked for rendering
		{
		return colorFrames.getLockedValue().timeStamp;
		}
	double getMeshTimeStamp(void) const // Returns the time stamp of the triangle mesh currently locked for rendering
		{
		return meshes.getLockedValue().first.timeStamp;
		}
	void glRenderAction(GLContextData& contextData) const; // Draws the current depth and color frames in the current model coordinate system
	};

}

#endif
