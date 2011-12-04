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

#include <Kinect/KinectFrameSaver.h>

#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Geometry/GeometryMarshallers.h>
#include <Kinect/KinectCamera.h>
#include <Kinect/DepthFrameWriter.h>
#include <Kinect/ColorFrameWriter.h>

/*********************************
Methods of class KinectFrameSaver:
*********************************/

void* KinectFrameSaver::depthFrameWritingThreadMethod(void)
	{
	while(true)
		{
		FrameBuffer fb;
		{
		/* Wait until there is an unsaved frame in the queue: */
		Threads::MutexCond::Lock depthFramesLock(depthFramesCond);
		while(!done&&depthFrames.empty())
			depthFramesCond.wait(depthFramesLock);
		
		/* Bail out if there are no more frames: */
		if(depthFrames.empty())
			break;
		
		/* Grab the next frame: */
		fb=depthFrames.front();
		depthFrames.pop_front();
		}
		
		/* Write the next frame to the depth frame file: */
		depthFrameWriter->writeFrame(fb);
		}
	
	return 0;
	}

void* KinectFrameSaver::colorFrameWritingThreadMethod(void)
	{
	while(true)
		{
		FrameBuffer fb;
		{
		/* Wait until there is an unsaved frame in the queue: */
		Threads::MutexCond::Lock colorFramesLock(colorFramesCond);
		while(!done&&colorFrames.empty())
			colorFramesCond.wait(colorFramesLock);
		
		/* Bail out if there are no more frames: */
		if(colorFrames.empty())
			break;
		
		/* Grab the next frame: */
		fb=colorFrames.front();
		colorFrames.pop_front();
		}
		
		/* Write the next frame to the color frame file: */
		colorFrameWriter->writeFrame(fb);
		}
	
	return 0;
	}

KinectFrameSaver::KinectFrameSaver(const KinectCamera& camera,const char* calibrationFileName,const KinectFrameSaver::Transform& projectorTransform,const char* depthFrameFileName,const char* colorFrameFileName)
	:done(false),
	 depthFrameFile(IO::openFile(depthFrameFileName,IO::File::WriteOnly)),
	 depthFrameWriter(0),
	 colorFrameFile(IO::openFile(colorFrameFileName,IO::File::WriteOnly)),
	 colorFrameWriter(0)
	{
	/* Open the calibration file: */
	IO::FilePtr calibrationFile(IO::openFile(calibrationFileName));
	calibrationFile->setEndianness(Misc::LittleEndian);
	
	/* Read the depth projection matrix: */
	double depthMatrix[16];
	calibrationFile->read(depthMatrix,4*4);
	
	/* Read the color projection matrix: */
	double colorMatrix[16];
	calibrationFile->read(colorMatrix,4*4);
	
	/* Write the depth frame file's header: */
	depthFrameFile->setEndianness(Misc::LittleEndian);
	depthFrameFile->write<double>(depthMatrix,4*4);
	Misc::Marshaller<Transform>::write(projectorTransform,*depthFrameFile);
	
	/* Create the depth frame writer: */
	depthFrameWriter=new DepthFrameWriter(*depthFrameFile,camera.getActualFrameSize(KinectCamera::DEPTH));
	
	/* Write the color frame file's header: */
	colorFrameFile->setEndianness(Misc::LittleEndian);
	colorFrameFile->write<double>(colorMatrix,4*4);
	
	/* Create the depth frame writer: */
	colorFrameWriter=new ColorFrameWriter(*colorFrameFile,camera.getActualFrameSize(KinectCamera::COLOR));
	
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
	
	/* Delete the frame writers: */
	delete depthFrameWriter;
	delete colorFrameWriter;
	}

void KinectFrameSaver::saveDepthFrame(const FrameBuffer& newFrame)
	{
	/* Enqueue the depth frame: */
	{
	Threads::MutexCond::Lock depthFramesLock(depthFramesCond);
	depthFrames.push_back(newFrame);
	depthFramesCond.signal();
	}
	}

void KinectFrameSaver::saveColorFrame(const FrameBuffer& newFrame)
	{
	/* Enqueue the color frame: */
	{
	Threads::MutexCond::Lock colorFramesLock(colorFramesCond);
	colorFrames.push_back(newFrame);
	colorFramesCond.signal();
	}
	}
