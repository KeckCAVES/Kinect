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

#include <Kinect/KinectPlayback.h>

#include <Misc/FunctionCalls.h>
#include <Misc/Time.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Constants.h>
#include <Geometry/GeometryMarshallers.h>
#include <Kinect/DepthFrameReader.h>
#include <Kinect/ColorFrameReader.h>

/*******************************
Methods of class KinectPlayback:
*******************************/

void* KinectPlayback::playbackThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Create a set of three depth frame buffers to do on-the-fly median spot noise filtering: */
	FrameBuffer depthFrames[3];
	int currentDepthFrame=2;
	unsigned int numDepthFrames=0;
	
	/* Load the first depth frame: */
	currentDepthFrame=(currentDepthFrame+1)%3;
	depthFrames[currentDepthFrame]=depthFrameReader->readNextFrame();
	++numDepthFrames;
	
	/* Load the first color frame: */
	FrameBuffer colorFrame=colorFrameReader->readNextFrame();
	
	while(true)
		{
		/* Wait until the next frame is due: */
		double dueTime=depthFrames[currentDepthFrame].timeStamp;
		if(dueTime>colorFrame.timeStamp)
			dueTime=colorFrame.timeStamp;
		double currentTime=frameTimer.peekTime();
		if(currentTime<dueTime)
			{
			Misc::sleep(dueTime-currentTime);
			currentTime=dueTime;
			}
		
		/* Check which frame has become active: */
		if(currentTime>=depthFrames[currentDepthFrame].timeStamp)
			{
			if(depthStreamingCallback!=0&&numDepthFrames>=3)
				{
				#if 0
				
				/* Create a median-filtered depth frame: */
				FrameBuffer median(depthSize[0],depthSize[1],depthSize[1]*depthSize[0]*sizeof(unsigned short));
				unsigned short* mPtr=static_cast<unsigned short*>(median.getBuffer());
				const unsigned short* d0Ptr=static_cast<unsigned short*>(depthFrames[0].getBuffer());
				const unsigned short* d1Ptr=static_cast<unsigned short*>(depthFrames[1].getBuffer());
				const unsigned short* d2Ptr=static_cast<unsigned short*>(depthFrames[2].getBuffer());
				for(int y=0;y<depthSize[1];++y)
					for(int x=0;x<depthSize[0];++x,++mPtr,++d0Ptr,++d1Ptr,++d2Ptr)
						{
						/* Find the median: */
						if(*d0Ptr<=*d1Ptr)
							{
							if(*d1Ptr<=*d2Ptr)
								*mPtr=*d1Ptr;
							else if(*d0Ptr<=*d2Ptr)
								*mPtr=*d2Ptr;
							else
								*mPtr=*d0Ptr;
							}
						else
							{
							if(*d0Ptr<=*d2Ptr)
								*mPtr=*d0Ptr;
							else if(*d1Ptr<=*d2Ptr)
								*mPtr=*d2Ptr;
							else
								*mPtr=*d1Ptr;
							}
						}
				
				#else
				
				FrameBuffer median=depthFrames[currentDepthFrame];
				
				#endif
				
				if(numBackgroundFrames>0)
					{
					/* Add the median-filtered depth frame to the background frame: */
					unsigned short* bfPtr=backgroundFrame;
					const unsigned short* mPtr=static_cast<const unsigned short*>(median.getBuffer());
					for(unsigned int y=0;y<depthSize[1];++y)
						for(unsigned int x=0;x<depthSize[0];++x,++bfPtr,++mPtr)
							if(*bfPtr>*mPtr-1)
								*bfPtr=*mPtr-1;
					
					--numBackgroundFrames;
					}
				
				if(removeBackground&&backgroundFrame!=0&&numBackgroundFrames==0)
					{
					/* Remove background pixels from the median-filtered depth frame: */
					unsigned short* mPtr=static_cast<unsigned short*>(median.getBuffer());
					const unsigned short* bfPtr=backgroundFrame;
					for(unsigned int y=0;y<depthSize[1];++y)
						for(unsigned int x=0;x<depthSize[0];++x,++mPtr,++bfPtr)
							if(*mPtr>=*bfPtr)
								*mPtr=0x07ffU;
					}
				
				/* Post the median-filtered depth frame to the consumer: */
				(*depthStreamingCallback)(median);
				}
			
			/* Read the next depth frame: */
			currentDepthFrame=(currentDepthFrame+1)%3;
			depthFrames[currentDepthFrame]=depthFrameReader->readNextFrame();
			++numDepthFrames;
			}
		if(currentTime>=colorFrame.timeStamp)
			{
			if(colorStreamingCallback!=0)
				{
				/* Post the next color frame to the consumer: */
				(*colorStreamingCallback)(colorFrame);
				}
			
			/* Read the next color frame: */
			colorFrame=colorFrameReader->readNextFrame();
			}
		}
	
	return 0;
	}

KinectPlayback::KinectPlayback(const char* depthFrameFileName,const char* colorFrameFileName)
	:depthFrameFile(IO::openFile(depthFrameFileName)),
	 depthFrameReader(0),
	 colorFrameFile(IO::openFile(colorFrameFileName)),
	 colorFrameReader(0),
	 depthStreamingCallback(0),colorStreamingCallback(0),
	 numBackgroundFrames(0),backgroundFrame(0),removeBackground(false)
	{
	/* Read the depth frame file's header: */
	depthFrameFile->setEndianness(Misc::LittleEndian);
	depthFrameFile->read<double>(depthMatrix,4*4);
	projectorTransform=Misc::Marshaller<Transform>::read(*depthFrameFile);
	
	/* Create the depth frame reader: */
	depthFrameReader=new DepthFrameReader(*depthFrameFile);
	for(int i=0;i<2;++i)
		depthSize[i]=depthFrameReader->getSize()[i];
	
	/* Read the color frame file's header: */
	colorFrameFile->setEndianness(Misc::LittleEndian);
	colorFrameFile->read<double>(colorMatrix,4*4);
	
	/* Create the color frame reader: */
	colorFrameReader=new ColorFrameReader(*colorFrameFile);
	}

KinectPlayback::~KinectPlayback(void)
	{
	/* Stop the playback thread: */
	if(!playbackThread.isJoined())
		{
		playbackThread.cancel();
		playbackThread.join();
		}
	
	/* Delete the callbacks: */
	delete depthStreamingCallback;
	delete colorStreamingCallback;
	
	/* Delete the frame readers: */
	delete depthFrameReader;
	delete colorFrameReader;
	
	delete[] backgroundFrame;
	}

void KinectPlayback::resetFrameTimer(void)
	{
	/* Reset the frame timer: */
	frameTimer.elapse();
	}

void KinectPlayback::startStreaming(KinectPlayback::StreamingCallback* newColorStreamingCallback,KinectPlayback::StreamingCallback* newDepthStreamingCallback)
	{
	/* Set the streaming callbacks: */
	delete depthStreamingCallback;
	depthStreamingCallback=newDepthStreamingCallback;
	delete colorStreamingCallback;
	colorStreamingCallback=newColorStreamingCallback;
	
	/* Start the playback thread: */
	playbackThread.start(this,&KinectPlayback::playbackThreadMethod);
	}

void KinectPlayback::captureBackground(unsigned int newNumBackgroundFrames)
	{
	/* Initialize the background frame buffer: */
	if(backgroundFrame==0)
		backgroundFrame=new unsigned short[depthSize[1]*depthSize[0]];
	
	/* Initialize the background frame to "empty:" */
	unsigned short* bfPtr=backgroundFrame;
	for(unsigned int y=0;y<depthSize[1];++y)
		for(unsigned int x=0;x<depthSize[0];++x,++bfPtr)
			*bfPtr=0x07ffU;
	
	/* Start capturing background frames: */
	numBackgroundFrames=newNumBackgroundFrames;
	}

void KinectPlayback::setRemoveBackground(bool newRemoveBackground)
	{
	/* Set the background removal flag: */
	removeBackground=newRemoveBackground;
	}

void KinectPlayback::stopStreaming(void)
	{
	/* Stop the playback thread: */
	if(!playbackThread.isJoined())
		{
		playbackThread.cancel();
		playbackThread.join();
		}
	
	/* Delete the callbacks: */
	delete depthStreamingCallback;
	depthStreamingCallback=0;
	delete colorStreamingCallback;
	colorStreamingCallback=0;
	}
