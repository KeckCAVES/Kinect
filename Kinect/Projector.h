/***********************************************************************
Projector - Class to project a depth frame captured from a Kinect camera
back into calibrated 3D camera space, and texture-map it with a matching
color frame.
Copyright (c) 2010-2011 Oliver Kreylos

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

#ifndef KINECT_PROJECTOR_INCLUDED
#define KINECT_PROJECTOR_INCLUDED

#define KINECT_PROJECTOR_FILTERING 1

#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <GL/gl.h>
#include <GL/GLVertex.h>
#include <GL/GLObject.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace IO {
class File;
}

namespace Kinect {

class Projector:public GLObject
	{
	/* Embedded classes: */
	private:
	typedef FrameSource::IntrinsicParameters::PTransform PTransform; // Type for projective transformations
	typedef FrameSource::ExtrinsicParameters OGTransform; // Type for orthogonal transformations
	typedef GLVertex<void,0,void,0,void,GLfloat,3> Vertex; // Type for vertices to render depth images
	
	struct DataItem:public GLObject::DataItem // Structure containing per-context state
		{
		/* Elements: */
		public:
		GLuint vertexBufferId; // ID of vertex buffer object holding the vertices of the current depth frame
		GLuint indexBufferId; // ID of index buffer object holding the triangles of the current depth frame
		size_t numIndices; // Number of used vertex indices in the current index buffer
		unsigned int depthFrameVersion; // Version number of frame currently in vertex / index buffer
		GLuint textureId; // ID of texture object holding the current color frame
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	PTransform colorProjection; // Projection transformation from color image space into 3D camera space
	PTransform depthProjection; // Projection transformation from depth image space into 3D camera space
	OGTransform projectorTransform; // Transformation from 3D camera space into 3D world space
	FrameBuffer colorFrame; // Current color frame
	unsigned int colorFrameVersion; // Version number of current color frame
	FrameBuffer depthFrame; // Current depth frame
	unsigned int depthFrameVersion; // Version number of current depth frame
	#if KINECT_PROJECTOR_FILTERING
	float* filteredDepthFrame; // Filtered depth frame, same version number as current depth frame
	#endif
	
	/* Constructors and destructors: */
	public:
	Projector(void); // Creates a facade projector with uninitialized camera parameters
	Projector(const FrameSource& frameSource); // Creates a facade projector for the given frame source
	~Projector(void);
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	
	/* New methods: */
	void setIntrinsicParameters(const FrameSource::IntrinsicParameters& ips); // Sets the projectors intrinsic camera parameters
	void setExtrinsicParameters(const FrameSource::ExtrinsicParameters& eps); // Sets the projectors extrinsic camera parameters
	const OGTransform& getProjectorTransform(void) const // Returns the transformation from camera to world space
		{
		return projectorTransform;
		}
	const FrameBuffer& getColorFrame(void) const // Returns the current color frame
		{
		return colorFrame;
		}
	const FrameBuffer& getDepthFrame(void) const // Returns the current depth frame
		{
		return depthFrame;
		}
	void setColorFrame(const FrameBuffer& newColorFrame); // Updates the projector's current color frame
	void setDepthFrame(const FrameBuffer& newDepthFrame); // Updates the projector's current depth frame
	void draw(GLContextData& contextData) const; // Draws the current depth and color frames in the current model coordinate system
	};

}

#endif
