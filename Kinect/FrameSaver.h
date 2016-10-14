/***********************************************************************
FrameSaver - Helper class to save raw color and video frames from a
Kinect frame source to a set of time-stamped files for playback and
further processing.
Copyright (c) 2010-2015 Oliver Kreylos

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

#ifndef KINECT_FRAMESAVER_INCLUDED
#define KINECT_FRAMESAVER_INCLUDED

#include <deque>
#include <Misc/Timer.h>
#include <IO/File.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
namespace Kinect {
class FrameSource;
class FrameWriter;
}

namespace Kinect {

class FrameSaver
	{
	/* Elements: */
	private:
	double timeStampOffset; // Offset value subtracted from the time stamps of all incoming color and depth frames
	volatile bool done; // Flag set when all frames have been queued for saving
	Threads::MutexCond colorFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<FrameBuffer> colorFrames; // Queue of color frames still to be saved
	IO::FilePtr colorFrameFile; // File receiving color frames
	FrameWriter* colorFrameWriter; // Helper object to compress and write color frames
	Threads::Thread colorFrameWritingThread; // Thread saving color frames
	Threads::MutexCond depthFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<FrameBuffer> depthFrames; // Queue of depth frames still to be saved
	IO::FilePtr depthFrameFile; // File receiving depth frames
	FrameWriter* depthFrameWriter; // Helper object to compress and write depth frames
	Threads::Thread depthFrameWritingThread; // Thread saving depth frames
	
	/* Private methods: */
	void initialize(FrameSource& frameSource); // Initializes the frame files and writers
	void* colorFrameWritingThreadMethod(void); // Thread method saving color frames
	void* depthFrameWritingThreadMethod(void); // Thread method saving depth frames
	
	/* Constructors and destructors: */
	public:
	FrameSaver(FrameSource& frameSource,const char* colorFrameFileName,const char* depthFrameFileName); // Creates frame saver for the given frame source, writing to two files of the given names
	FrameSaver(FrameSource& frameSource,IO::FilePtr sColorFrameFile,IO::FilePtr sDepthFrameFile); // Ditto, to the two already opened files
	~FrameSaver(void);
	
	/* Methods: */
	void setTimeStampOffset(double newTimeStampOffset); // Sets the time stamp offset for all subsequent frames
	void saveColorFrame(const FrameBuffer& newFrame); // Queues a new color frame for writing
	void saveDepthFrame(const FrameBuffer& newFrame); // Queues a new depth frame for writing
	};

}

#endif
