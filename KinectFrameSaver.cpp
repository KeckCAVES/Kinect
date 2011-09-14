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

#include "KinectFrameSaver.h"

/*********************************
Methods of class KinectFrameSaver:
*********************************/

void* KinectFrameSaver::depthFrameWritingThreadMethod(void)
	{
	while(true)
		{
		TimedFrameBuffer tfb;
		{
		/* Wait until there is an unsaved frame in the queue: */
		Threads::MutexCond::Lock depthFramesLock(depthFramesCond);
		while(!done&&depthFrames.empty())
			depthFramesCond.wait(depthFramesLock);
		
		/* Bail out if there are no more frames: */
		if(depthFrames.empty())
			break;
		
		/* Grab the next frame: */
		tfb=depthFrames.front();
		depthFrames.pop_front();
		}
		
		/* Write the next frame to the depth frame file: */
		depthFrameFile.write<double>(tfb.first);
		depthFrameFile.write<unsigned short>(static_cast<const unsigned short*>(tfb.second.getBuffer()),tfb.second.getSize(1)*tfb.second.getSize(0));
		}
	
	return 0;
	}

void* KinectFrameSaver::colorFrameWritingThreadMethod(void)
	{
	while(true)
		{
		TimedFrameBuffer tfb;
		{
		/* Wait until there is an unsaved frame in the queue: */
		Threads::MutexCond::Lock colorFramesLock(colorFramesCond);
		while(!done&&colorFrames.empty())
			colorFramesCond.wait(colorFramesLock);
		
		/* Bail out if there are no more frames: */
		if(colorFrames.empty())
			break;
		
		/* Grab the next frame: */
		tfb=colorFrames.front();
		colorFrames.pop_front();
		}
		
		/* Write the next frame to the color frame file: */
		colorFrameFile.write<double>(tfb.first);
		colorFrameFile.write<unsigned char>(static_cast<const unsigned char*>(tfb.second.getBuffer()),tfb.second.getSize(1)*tfb.second.getSize(0)*3);
		}
	
	return 0;
	}

KinectFrameSaver::KinectFrameSaver(const char* depthFrameFileName,const char* colorFrameFileName)
	:done(false),
	 depthFrameFile(depthFrameFileName,"wb",Misc::File::LittleEndian),
	 colorFrameFile(colorFrameFileName,"wb",Misc::File::LittleEndian)
	{
	/* Write the frame sizes to the frame files: */
	depthFrameFile.write<unsigned int>(640);
	depthFrameFile.write<unsigned int>(480);
	colorFrameFile.write<unsigned int>(640);
	colorFrameFile.write<unsigned int>(480);
	
	/* Start the frame writing threads: */
	depthFrameWritingThread.start(this,&KinectFrameSaver::depthFrameWritingThreadMethod);
	colorFrameWritingThread.start(this,&KinectFrameSaver::colorFrameWritingThreadMethod);
	}

KinectFrameSaver::~KinectFrameSaver(void)
	{
	/* Tell the frame writing threads to shut down once their queues are empty: */
	done=true;
	depthFramesCond.signal();
	colorFramesCond.signal();
	
	/* Wait for the frame writing threads to finish: */
	depthFrameWritingThread.join();
	colorFrameWritingThread.join();
	}

void KinectFrameSaver::saveDepthFrame(const FrameBuffer& newFrame)
	{
	/* Time-stamp the depth frame: */
	TimedFrameBuffer tfb;
	tfb.first=frameTimer.peekTime();
	tfb.second=newFrame;
	
	/* Enqueue the depth frame: */
	{
	Threads::MutexCond::Lock depthFramesLock(depthFramesCond);
	depthFrames.push_back(tfb);
	depthFramesCond.signal(depthFramesLock);
	}
	}

void KinectFrameSaver::saveColorFrame(const FrameBuffer& newFrame)
	{
	/* Time-stamp the color frame: */
	TimedFrameBuffer tfb;
	tfb.first=frameTimer.peekTime();
	tfb.second=newFrame;
	
	/* Enqueue the color frame: */
	{
	Threads::MutexCond::Lock colorFramesLock(colorFramesCond);
	colorFrames.push_back(tfb);
	colorFramesCond.signal(colorFramesLock);
	}
	}
