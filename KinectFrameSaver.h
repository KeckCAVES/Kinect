/***********************************************************************
KinectFrameSaver - Helper class to save raw color and video frames from
a Kinect camera to a time-stamped file on disk for playback and further
processing.
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

#ifndef KINECTFRAMESAVER_INCLUDED
#define KINECTFRAMESAVER_INCLUDED

#include <deque>
#include <Misc/Timer.h>
#include <Misc/File.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>

#include "FrameBuffer.h"

class KinectFrameSaver
	{
	/* Embedded classes: */
	private:
	typedef std::pair<double,FrameBuffer> TimedFrameBuffer; // Frame buffer with time stamp
	
	/* Elements: */
	Misc::Timer frameTimer; // Free-running timer to stamp incoming depth and color frames
	volatile bool done; // Flag set when all frames have been queued for saving
	Threads::MutexCond depthFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<TimedFrameBuffer> depthFrames; // Queue of depth frames still to be saved
	Misc::File depthFrameFile; // File receiving depth frames
	Threads::Thread depthFrameWritingThread; // Thread saving depth frames
	Threads::MutexCond colorFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<TimedFrameBuffer> colorFrames; // Queue of color frames still to be saved
	Misc::File colorFrameFile; // File receiving color frames
	Threads::Thread colorFrameWritingThread; // Thread saving color frames
	
	/* Private methods: */
	void* depthFrameWritingThreadMethod(void); // Thread method saving depth frames
	void* colorFrameWritingThreadMethod(void); // Thread method saving color frames
	
	/* Constructors and destructors: */
	public:
	KinectFrameSaver(const char* depthFrameFileName,const char* colorFrameFileName); // Creates frame saver writing to the two given files
	~KinectFrameSaver(void);
	
	/* Methods: */
	void saveDepthFrame(const FrameBuffer& newFrame); // Queues a new depth frame for writing
	void saveColorFrame(const FrameBuffer& newFrame); // Queues a new color frame for writing
	};

#endif
