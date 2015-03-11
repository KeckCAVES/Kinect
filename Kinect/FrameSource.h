/***********************************************************************
FrameSource - Base class for objects that create streams of depth and
color frames.
Copyright (c) 2011-2013 Oliver Kreylos

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

#include <Misc/SizedTypes.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Kinect/Config.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace IO {
class File;
}
namespace Geometry {
#if KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
template <class ScalarParam,int dimensionParam>
class ProjectiveTransformation;
#else
template <class ScalarParam,int dimensionParam>
class OrthogonalTransformation;
#endif
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
	
	typedef Misc::UInt16 DepthPixel; // Type for raw depth pixels
	typedef Misc::UInt8 ColorComponent; // Type for color pixel components
	
	struct ColorPixel // Type for color pixels
		{
		/* Embedded classes: */
		public:
		typedef ColorComponent Component; // Type for color components
		
		/* Elements: */
		public:
		Component rgb[3]; // RGB color components
		};
	
	class DepthCorrection // Class defining the depth correction parameters of a depth frame source
		{
		/* Embedded classes: */
		public:
		struct PixelCorrection // Structure describing a per-pixel depth correction factor
			{
			/* Elements: */
			public:
			float scale,offset; // Scale and offset, parameters of the formula depth' = depth*scale + offset
			
			/* Methods: */
			float correct(float depth) const // Corrects the given raw depth value
				{
				return float(depth)*scale+offset;
				}
			};
		
		/* Elements: */
		private:
		int degree; // Degree of bivariate B-spline approximating the per-pixel depth correction offsets
		int numSegments[2]; // Number of B-spline segments horizontally and vertically
		PixelCorrection* controlPoints; // Array of control points defining the depth correction B-spline
		
		/* Constructors and destructors: */
		public:
		DepthCorrection(int sDegree,const int sNumSegments[2]); // Creates a depth correction object with the given frame size, degree, and number of segments
		DepthCorrection(IO::File& file); // Reads a depth correction object from a binary file or pipe
		DepthCorrection(const DepthCorrection& source); // Copy constructor
		private:
		DepthCorrection& operator=(const DepthCorrection& source); // Prohibit assignment operator
		public:
		~DepthCorrection(void); // Destroys a depth correction object
		
		/* Methods: */
		void write(IO::File& file) const; // Writes a depth correction object to a binary file or pipe
		PixelCorrection getPixelCorrection(unsigned int x,unsigned int y,const unsigned int frameSize[2]) const; // Returns the depth correction factor for the depth image pixel at the given position
		PixelCorrection* getPixelCorrection(const unsigned int frameSize[2]) const; // Returns pointer to a new-allocated array containing per-pixel depth correction parameters
		};
	
	struct IntrinsicParameters // Structure defining the intrinsic parameters of a depth and color frame source
		{
		/* Embedded classes: */
		public:
		typedef Geometry::ProjectiveTransformation<double,3> PTransform; // Type for projective transformations
		
		/* Elements: */
		PTransform depthProjection; // The projection transformation from depth image space into 3D camera space
		PTransform colorProjection; // The projection transformation from 3D camera space into color image space
		};
	#if KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
	typedef Geometry::ProjectiveTransformation<double,3> ExtrinsicParameters; // Type for extrinsic camera parameters
	#else
	typedef Geometry::OrthogonalTransformation<double,3> ExtrinsicParameters; // Type for extrinsic camera parameters
	#endif
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	static const DepthPixel invalidDepth=0x07ffU; // The depth value indicating an invalid (or removed) pixel
	
	/* Constructors and destructors: */
	public:
	FrameSource(void);
	virtual ~FrameSource(void);
	
	/* Methods: */
	virtual DepthCorrection* getDepthCorrectionParameters(void); // Returns the camera depth correction object, i.e., per-pixel depth value offsets
	virtual IntrinsicParameters getIntrinsicParameters(void) =0; // Returns the intrinsic camera parameters, i.e., the virtual camera's projection matrix in camera space
	virtual ExtrinsicParameters getExtrinsicParameters(void) =0; // Returns the extrinsic camera parameters, i.e., the position and orientation of the virtual camera in 3D world space
	virtual const unsigned int* getActualFrameSize(int sensor) const =0; // Returns the selected frame size of the color or depth stream as an array of (width, height) in pixels
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback) =0; // Installs the given streaming callback and starts receiving color and depth frames
	virtual void stopStreaming(void) =0; // Stops streaming; blocks until all pending frame transfers have either completed or been cancelled
	};

}

#endif
