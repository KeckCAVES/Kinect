/***********************************************************************
KinectPlayback - Class implementing a fake Kinect device by playing back
previously recorded time-stamped depth and color frames.
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

#ifndef KINECTPLAYBACK_INCLUDED
#define KINECTPLAYBACK_INCLUDED

#include <Misc/Timer.h>
#include <IO/File.h>
#include <Threads/Thread.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
class DepthFrameReader;
class ColorFrameReader;

class KinectPlayback
	{
	/* Embedded classes: */
	public:
	typedef Geometry::OrthogonalTransformation<double,3> Transform; // Type for projector transformations
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	/* Elements: */
	private:
	Misc::Timer frameTimer; // Free-running timer to synchronize playback of depth and color frames
	IO::FilePtr depthFrameFile; // File containing depth frames
	double depthMatrix[16]; // Depth calibration matrix read from depth frame file
	Transform projectorTransform; // Projector transformation read from depth frame file
	DepthFrameReader* depthFrameReader; // Reader for depth frames
	IO::FilePtr colorFrameFile; // File containing color frames
	double colorMatrix[16]; // Color calibration matrix read from color frame file
	ColorFrameReader* colorFrameReader; // Reader for color frames
	unsigned int depthSize[2]; // Frame size of depth frames
	Threads::Thread playbackThread; // Thread playing back depth and color frames
	StreamingCallback* depthStreamingCallback; // Callback to be called when a new depth frame has been loaded
	StreamingCallback* colorStreamingCallback; // Callback to be called when a new color frame has been loaded
	unsigned int numBackgroundFrames; // Number of background frames left to capture
	unsigned short* backgroundFrame; // Frame containing minimal depth values for a captured background
	bool removeBackground; // Flag whether to remove background information during frame processing
	
	/* Private methods: */
	void* playbackThreadMethod(void); // Thread method playing back depth and color frames
	
	/* Constructors and destructors: */
	public:
	KinectPlayback(const char* depthFrameFileName,const char* colorFrameFileName); // Creates playback device for given depth and color frame files
	~KinectPlayback(void);
	
	/* Methods: */
	const double* getDepthMatrix(void) const // Returns the depth calibration matrix
		{
		return depthMatrix;
		}
	const Transform& getProjectorTransform(void) const // Returns the projector transformation
		{
		return projectorTransform;
		}
	const double* getColorMatrix(void) const // Returns the color calibration matrix
		{
		return colorMatrix;
		}
	void resetFrameTimer(void); // Resets the internal frame timer
	void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback); // Installs the given streaming callback and starts loading color and depth frames from the files
	void captureBackground(unsigned int newNumBackgroundFrames); // Captures the given number of frames to create a background removal buffer
	void setRemoveBackground(bool newRemoveBackground); // Enables or disables background removal
	bool getRemoveBackground(void) const // Returns the current background removal flag
		{
		return removeBackground;
		}
	void stopStreaming(void); // Stops streaming
	};

#endif
