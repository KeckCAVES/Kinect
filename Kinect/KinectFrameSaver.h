/***********************************************************************
KinectFrameSaver - Helper class to save raw color and video frames from
a Kinect camera to a time-stamped file on disk for playback and further
processing.
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

#ifndef KINECTFRAMESAVER_INCLUDED
#define KINECTFRAMESAVER_INCLUDED

#include <deque>
#include <Misc/Timer.h>
#include <IO/File.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
class KinectCamera;
class DepthFrameWriter;
class ColorFrameWriter;

class KinectFrameSaver
	{
	/* Embedded classes: */
	public:
	typedef Geometry::OrthogonalTransformation<double,3> Transform; // Type for projector transformations
	
	/* Elements: */
	private:
	volatile bool done; // Flag set when all frames have been queued for saving
	Threads::MutexCond depthFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<FrameBuffer> depthFrames; // Queue of depth frames still to be saved
	IO::FilePtr depthFrameFile; // File receiving depth frames
	DepthFrameWriter* depthFrameWriter; // Helper object to compress and write depth frames
	Threads::Thread depthFrameWritingThread; // Thread saving depth frames
	Threads::MutexCond colorFramesCond; // Condition variable to signal new frames in the depth queue
	std::deque<FrameBuffer> colorFrames; // Queue of color frames still to be saved
	IO::FilePtr colorFrameFile; // File receiving color frames
	ColorFrameWriter* colorFrameWriter; // Helper object to compress and write color frames
	Threads::Thread colorFrameWritingThread; // Thread saving color frames
	
	/* Private methods: */
	void* depthFrameWritingThreadMethod(void); // Thread method saving depth frames
	void* colorFrameWritingThreadMethod(void); // Thread method saving color frames
	
	/* Constructors and destructors: */
	public:
	KinectFrameSaver(const KinectCamera& camera,const char* calibrationFileName,const Transform& projectorTransform,const char* depthFrameFileName,const char* colorFrameFileName); // Creates frame saver writing to the two given files
	~KinectFrameSaver(void);
	
	/* Methods: */
	void saveDepthFrame(const FrameBuffer& newFrame); // Queues a new depth frame for writing
	void saveColorFrame(const FrameBuffer& newFrame); // Queues a new color frame for writing
	};

#endif
