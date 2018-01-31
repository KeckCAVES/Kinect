/***********************************************************************
SphereExtractor - Helper class to identify and extract spheres of known
radii in depth images.
Copyright (c) 2014-2017 Oliver Kreylos

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

#ifndef SPHEREEXTRACTOR_INCLUDED
#define SPHEREEXTRACTOR_INCLUDED

#include <vector>
#include <Threads/Mutex.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/Point.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/Sphere.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

class SphereExtractor
	{
	/* Embedded classes: */
	private:
	typedef Kinect::FrameSource::DepthPixel DepthPixel;
	typedef Kinect::FrameSource::ColorPixel ColorPixel;
	typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection;
	typedef Kinect::FrameSource::IntrinsicParameters::PTransform PTransform;
	public:
	typedef PTransform::Scalar Scalar;
	typedef Geometry::Point<Scalar,2> PixelPos; // Type for lens distortion-corrected depth frame pixel center positions
	typedef Geometry::Sphere<Scalar,3> Sphere;
	typedef std::vector<Sphere> SphereList;
	typedef Misc::FunctionCall<const SphereList&> StreamingCallback; // Function call type for streaming callbacks
	
	/* Elements: */
	private:
	unsigned int depthFrameSize[2]; // Width and height of source's depth frames
	PixelPos* depthPixels; // 2D array of (lens distortion-corrected) depth frame pixel center positions
	const PixelDepthCorrection* dcBuffer; // Pointer to frame source's per-pixel depth correction buffer
	PTransform depthProjection; // Frame source's depth unprojection matrix
	unsigned int colorFrameSize[2]; // Width and height of source's color frames
	PTransform colorProjection; // Frame source's color projection matrix
	int maxBlobMergeDist; // Maximum depth distance between adjacent pixels to merge their respective blobs
	Scalar sphereRadius; // Radius of sphere in 3D camera space's measurement unit
	ColorPixel::Component minWhite; // Minimum color component value to classify a pixel as white
	ColorPixel::Component maxSpread; // Maximum spread between color component values to classify a pixel as white
	size_t minBlobSize; // Minimum number of pixels in a blob to be considered for sphere extraction
	Scalar radiusTolerance; // Relative tolerance in sphere radius
	Scalar maxResidual; // Maximum approximation residual
	Threads::Thread frameProcessingThread; // Background thread to extract spheres from incoming depth and color frames
	Threads::MutexCond inDepthFrameCond; // Condition variable to signal arrival of a new depth frame
	unsigned int inDepthFrameVersion; // Version number of most-recently arrived raw depth frame
	Kinect::FrameBuffer inDepthFrame; // Most-recently arrived raw depth frame
	Threads::Mutex inColorFrameMutex; // Mutex protecting incoming color frames
	Kinect::FrameBuffer inColorFrame; // Most-recently arrived raw color frame
	Threads::TripleBuffer<SphereList> sphereLists; // Triple buffer of recently extracted lists of spheres
	StreamingCallback* streamingCallback; // Function to be called when spheres have been extracted from a depth frame
	
	/* Private methods: */
	void* frameProcessingThreadMethod(void); // Thread method for the background sphere extraction thread
	
	/* Constructors and destructors: */
	public:
	SphereExtractor(Kinect::FrameSource& frameSource,const PixelDepthCorrection* sDcBuffer);
	~SphereExtractor(void);
	
	/* Methods: */
	void setMaxBlobMergeDist(int newMaxBlobMergeDist); // Sets a new maximum pixel depth distance to merge adjacent blobs
	void setSphereRadius(Scalar newSphereRadius); // Sets a new target radius for sphere extraction
	void setMatchLimits(unsigned int newMinWhite,unsigned int newMaxSpread,size_t newMinBlobSize,Scalar newRadiusTolerance,Scalar newMaxResidual); // Sets new sphere matching parameters
	void startStreaming(StreamingCallback* newStreamingCallback); // Starts processing depth frames in the background; calls the provided callback function every time a depth frame has been processed
	void setDepthFrame(const Kinect::FrameBuffer& newDepthFrame); // Submits a new depth frame for sphere extraction
	void setColorFrame(const Kinect::FrameBuffer& newColorFrame); // Submits a new color frame for sphere extraction
	void stopStreaming(void); // Stops background processing of depth frames
	bool lockSpheres(void) // Locks the most recent list of extracted spheres; returns true if there is a new list
		{
		return sphereLists.lockNewValue();
		}
	const SphereList& getSpheres(void) const // Returns the currently locked list of extracted spheres
		{
		return sphereLists.getLockedValue();
		}
	};

#endif
