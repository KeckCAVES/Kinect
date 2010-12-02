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
#include <Misc/File.h>
#include <Threads/Thread.h>

#include "FrameBuffer.h"

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

class KinectPlayback
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	private:
	typedef std::pair<double,FrameBuffer> TimedFrameBuffer; // Frame buffer with time stamp
	
	/* Elements: */
	Misc::Timer frameTimer; // Free-running timer to synchronize playback of depth and color frames
	Misc::File depthFrameFile; // File containing depth frames
	Misc::File colorFrameFile; // File containing color frames
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
