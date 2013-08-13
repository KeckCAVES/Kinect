/***********************************************************************
FrameSaver - Helper class to save raw color and video frames from a
Kinect camera to a time-stamped file on disk for playback and further
processing.
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

#include <Kinect/FrameSaver.h>

#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Geometry/GeometryMarshallers.h>
#include <Video/Config.h>
#include <Kinect/FrameSource.h>
#include <Kinect/DepthFrameWriter.h>
#include <Kinect/LossyDepthFrameWriter.h>
#include <Kinect/ColorFrameWriter.h>

#if VIDEO_CONFIG_HAVE_THEORA
#define KINECT_FRAMESAVER_LOSSY 0 // Disabled until I figure out a codec that works
#else
#define KINECT_FRAMESAVER_LOSSY 0
#endif

namespace Kinect {

/***************************
Methods of class FrameSaver:
***************************/

void FrameSaver::initialize(FrameSource& frameSource)
	{
	/* Write the file formats' version numbers to the depth and color files: */
	colorFrameFile->write<Misc::UInt32>(1);
	depthFrameFile->write<Misc::UInt32>(4);
	
	/* Write the frame source's depth correction parameters: */
	FrameSource::DepthCorrection* dc=frameSource.getDepthCorrectionParameters();
	dc->write(*depthFrameFile);
	delete dc;
	
	#if KINECT_FRAMESAVER_LOSSY
	
	/* Signal whether the depth stream will contain losslessly compressed frames (lossy compression disabled for now): */
	depthFrameFile->write<Misc::UInt8>(1);
	
	#else
	
	/* Signal that the depth stream will contain losslessly compressed frames: */
	depthFrameFile->write<Misc::UInt8>(0);
	
	#endif
	
	/* Get the frame source's intrinsic calibration parameters: */
	FrameSource::IntrinsicParameters ips=frameSource.getIntrinsicParameters();
	
	/* Write the color and depth projections to their respective files: */
	Misc::Marshaller<FrameSource::IntrinsicParameters::PTransform>::write(ips.colorProjection,*colorFrameFile);
	Misc::Marshaller<FrameSource::IntrinsicParameters::PTransform>::write(ips.depthProjection,*depthFrameFile);
	
	/* Get the frame source's extrinsic calibration parameters: */
	FrameSource::ExtrinsicParameters eps=frameSource.getExtrinsicParameters();
	
	/* Write the camera transformation to the depth file: */
	Misc::Marshaller<FrameSource::ExtrinsicParameters>::write(eps,*depthFrameFile);
	
	/* Create the color and depth frame writers: */
	colorFrameWriter=new ColorFrameWriter(*colorFrameFile,frameSource.getActualFrameSize(FrameSource::COLOR));
	#if KINECT_FRAMESAVER_LOSSY
	depthFrameWriter=new LossyDepthFrameWriter(*depthFrameFile,frameSource.getActualFrameSize(FrameSource::DEPTH));
	#else
	depthFrameWriter=new DepthFrameWriter(*depthFrameFile,frameSource.getActualFrameSize(FrameSource::DEPTH));
	#endif
	
	/* Start the frame writing threads: */
	colorFrameWritingThread.start(this,&FrameSaver::colorFrameWritingThreadMethod);
	depthFrameWritingThread.start(this,&FrameSaver::depthFrameWritingThreadMethod);
	}

void* FrameSaver::colorFrameWritingThreadMethod(void)
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

void* FrameSaver::depthFrameWritingThreadMethod(void)
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

FrameSaver::FrameSaver(FrameSource& frameSource,const char* colorFrameFileName,const char* depthFrameFileName)
	:timeStampOffset(0.0),
	 done(false),
	 colorFrameFile(IO::openFile(colorFrameFileName,IO::File::WriteOnly)),
	 colorFrameWriter(0),
	 depthFrameFile(IO::openFile(depthFrameFileName,IO::File::WriteOnly)),
	 depthFrameWriter(0)
	{
	/* Initialize the frame files: */
	colorFrameFile->setEndianness(Misc::LittleEndian);
	depthFrameFile->setEndianness(Misc::LittleEndian);
	
	/* Initialize the frame saver: */
	initialize(frameSource);
	}

FrameSaver::FrameSaver(FrameSource& frameSource,IO::FilePtr sColorFrameFile,IO::FilePtr sDepthFrameFile)
	:timeStampOffset(0.0),
	 done(false),
	 colorFrameFile(sColorFrameFile),
	 colorFrameWriter(0),
	 depthFrameFile(sDepthFrameFile),
	 depthFrameWriter(0)
	{
	/* Initialize the frame saver: */
	initialize(frameSource);
	}

FrameSaver::~FrameSaver(void)
	{
	/* Tell the frame writing threads to shut down once their queues are empty: */
	done=true;
	colorFramesCond.signal();
	depthFramesCond.signal();
	
	/* Wait for the frame writing threads to finish: */
	colorFrameWritingThread.join();
	depthFrameWritingThread.join();
	
	/* Delete the frame writers: */
	delete colorFrameWriter;
	delete depthFrameWriter;
	}

void FrameSaver::setTimeStampOffset(double newTimeStampOffset)
	{
	/* Copy the new time stamp offset: */
	timeStampOffset=newTimeStampOffset;
	}

void FrameSaver::saveColorFrame(const FrameBuffer& newFrame)
	{
	/* Enqueue the color frame: */
	Threads::MutexCond::Lock colorFramesLock(colorFramesCond);
	colorFrames.push_back(newFrame);
	
	/* Offset the new frame's time stamp: */
	colorFrames.back().timeStamp-=timeStampOffset;
	
	/* Wake up the color frame saver: */
	colorFramesCond.signal();
	}

void FrameSaver::saveDepthFrame(const FrameBuffer& newFrame)
	{
	/* Enqueue the depth frame: */
	Threads::MutexCond::Lock depthFramesLock(depthFramesCond);
	depthFrames.push_back(newFrame);
	
	/* Offset the new frame's time stamp: */
	depthFrames.back().timeStamp-=timeStampOffset;
	
	/* Wake up the depth frame saver: */
	depthFramesCond.signal();
	}

}
