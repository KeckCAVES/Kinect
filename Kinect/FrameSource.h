/***********************************************************************
FrameSource - Base class for objects that create streams of depth and
color frames.
Copyright (c) 2011 Oliver Kreylos

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

#ifndef KINECT_FRAMESOURCE_INCLUDED
#define KINECT_FRAMESOURCE_INCLUDED

#include <Geometry/ProjectiveTransformation.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace Geometry {
template <class ScalarParam,int dimensionParam>
class OrthogonalTransformation;
}
namespace Kinect {
class FrameBuffer;
}

namespace Kinect {

class FrameSource
	{
	/* Embedded classes: */
	public:
	enum Sensor // Enumerated type to select one of frame streams
		{
		COLOR=0,DEPTH
		};
	
	struct IntrinsicParameters // Structure defining the intrinsic parameters of a depth and color frame source
		{
		/* Embedded classes: */
		public:
		typedef Geometry::ProjectiveTransformation<double,3> PTransform; // Type for projective transformations
		
		/* Elements: */
		PTransform depthProjection; // The projection transformation from depth image space into 3D camera space
		PTransform colorProjection; // The projection transformation from color image space into 3D camera space
		};
	
	typedef Geometry::OrthogonalTransformation<double,3> ExtrinsicParameters; // Type for extrinsic camera parameters
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	static const unsigned short invalidDepth=0x07ffU; // The depth value indicating an invalid (or removed) pixel
	
	/* Constructors and destructors: */
	public:
	FrameSource(void);
	virtual ~FrameSource(void);
	
	/* Methods: */
	virtual IntrinsicParameters getIntrinsicParameters(void) const =0; // Returns the intrinsic camera parameters, i.e., the virtual camera's projection matrix in camera space
	virtual ExtrinsicParameters getExtrinsicParameters(void) const =0; // Returns the extrinsic camera parameters, i.e., the position and orientation of the virtual camera in 3D world space
	virtual const unsigned int* getActualFrameSize(int sensor) const =0; // Returns the selected frame size of the color or depth stream as an array of (width, height) in pixels
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback) =0; // Installs the given streaming callback and starts receiving color and depth frames
	virtual void stopStreaming(void) =0; // Stops streaming; blocks until all pending frame transfers have either completed or been cancelled
	};

}

#endif
