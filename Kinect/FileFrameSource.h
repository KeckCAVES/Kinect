/***********************************************************************
FileFrameSource - Class to stream depth and color frames from a pair of
time-stamped depth and color stream files.
Copyright (c) 2010-2013 Oliver Kreylos

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

#ifndef KINECT_FILEFRAMESOURCE_INCLUDED
#define KINECT_FILEFRAMESOURCE_INCLUDED

#include <Misc/Timer.h>
#include <IO/File.h>
#include <Threads/Thread.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Kinect {
class FrameReader;
}

namespace Kinect {

class FileFrameSource:public FrameSource
	{
	/* Elements: */
	private:
	Misc::Timer frameTimer; // Free-running timer to synchronize playback of depth and color frames
	IO::FilePtr colorFrameFile; // File containing color frames
	IO::FilePtr depthFrameFile; // File containing depth frames
	unsigned int fileFormatVersions[2]; // Format version numbers of the color and depth files, respectively
	FrameReader* colorFrameReader; // Reader for color frames
	FrameReader* depthFrameReader; // Reader for depth frames
	unsigned int depthSize[2]; // Size of depth frames in pixels
	DepthCorrection* depthCorrection; // Depth correction parameters read from the depth file
	IntrinsicParameters intrinsicParameters; // Intrinsic parameters read from the color and depth files
	ExtrinsicParameters extrinsicParameters; // Extrinsic parameters read from the color and depth files
	StreamingCallback* colorStreamingCallback; // Callback to be called when a new color frame has been loaded
	StreamingCallback* depthStreamingCallback; // Callback to be called when a new depth frame has been loaded
	Threads::Thread playbackThread; // Thread playing back depth and color frames
	unsigned int numBackgroundFrames; // Number of background frames left to capture
	DepthPixel* backgroundFrame; // Frame containing minimal depth values for a captured background
	bool removeBackground; // Flag whether to remove background information during frame processing
	
	/* Private methods: */
	void initialize(void);
	void* playbackThreadMethod(void); // Thread method playing back depth and color frames
	
	/* Constructors and destructors: */
	public:
	FileFrameSource(const char* colorFrameFileName,const char* depthFrameFileName); // Creates frame source for given color and depth frame files
	FileFrameSource(IO::FilePtr sColorFrameFile,IO::FilePtr sDepthFrameFile); // Ditto, for the two already opened files
	~FileFrameSource(void);
	
	/* Methods from FrameSource: */
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual ExtrinsicParameters getExtrinsicParameters(void);
	virtual const unsigned int* getActualFrameSize(int sensor) const;
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
	virtual void stopStreaming(void);
	
	/* New methods: */
	FrameBuffer readNextColorFrame(void); // Immediately reads, decompresses, and returns the next frame from the color file
	FrameBuffer readNextDepthFrame(void); // Immediately reads, decompresses, and returns the next frame from the depth file
	void resetFrameTimer(void); // Resets the internal frame timer
	void captureBackground(unsigned int newNumBackgroundFrames); // Captures the given number of frames to create a background removal buffer
	void setRemoveBackground(bool newRemoveBackground); // Enables or disables background removal
	bool getRemoveBackground(void) const // Returns the current background removal flag
		{
		return removeBackground;
		}
	};

}

#endif
