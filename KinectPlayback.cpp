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

#include "KinectPlayback.h"

#include <Misc/FunctionCalls.h>
#include <Misc/Time.h>
#include <Math/Constants.h>

/*******************************
Methods of class KinectPlayback:
*******************************/

void* KinectPlayback::playbackThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Create a set of three depth frame buffers to do on-the-fly median spot noise filtering: */
	FrameBuffer depthFrames[3];
	for(int i=0;i<3;++i)
		depthFrames[i]=FrameBuffer(640,480,640*480*sizeof(unsigned short));
	int currentDepthFrame=2;
	unsigned int numDepthFrames=0;
	
	/* Load the first depth frame: */
	double depthTimeStamp=depthFrameFile.read<double>();
	currentDepthFrame=(currentDepthFrame+1)%3;
	depthFrameFile.read<unsigned short>(static_cast<unsigned short*>(depthFrames[currentDepthFrame].getBuffer()),640*480);
	++numDepthFrames;
	
	/* Load the first color frame: */
	double colorTimeStamp=colorFrameFile.read<double>();
	FrameBuffer colorFrame(640,480,640*480*3*sizeof(unsigned char));
	colorFrameFile.read<unsigned char>(static_cast<unsigned char*>(colorFrame.getBuffer()),640*480*3);
	
	while(true)
		{
		/* Wait until the next frame is due: */
		double dueTime=depthTimeStamp;
		if(dueTime>colorTimeStamp)
			dueTime=colorTimeStamp;
		double currentTime=frameTimer.peekTime();
		if(currentTime<dueTime)
			{
			Misc::sleep(dueTime-currentTime);
			currentTime=dueTime;
			}
		
		/* Check which frame has become active: */
		if(currentTime>=depthTimeStamp)
			{
			if(depthStreamingCallback!=0&&numDepthFrames>=3)
				{
				/* Create a median-filtered depth frame: */
				FrameBuffer median(640,480,640*480*sizeof(unsigned short));
				unsigned short* mPtr=static_cast<unsigned short*>(median.getBuffer());
				const unsigned short* d0Ptr=static_cast<unsigned short*>(depthFrames[0].getBuffer());
				const unsigned short* d1Ptr=static_cast<unsigned short*>(depthFrames[1].getBuffer());
				const unsigned short* d2Ptr=static_cast<unsigned short*>(depthFrames[2].getBuffer());
				for(int y=0;y<480;++y)
					for(int x=0;x<640;++x,++mPtr,++d0Ptr,++d1Ptr,++d2Ptr)
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
				
					if(numBackgroundFrames>0)
						{
						/* Add the median-filtered depth frame to the background frame: */
						unsigned short* bfPtr=backgroundFrame;
						const unsigned short* mPtr=static_cast<const unsigned short*>(median.getBuffer());
						for(int y=0;y<480;++y)
							for(int x=0;x<640;++x,++bfPtr,++mPtr)
								if(*bfPtr>*mPtr-1)
									*bfPtr=*mPtr-1;
						
						--numBackgroundFrames;
						}
				
				if(removeBackground&&numBackgroundFrames==0)
					{
					/* Remove background pixels from the median-filtered depth frame: */
					unsigned short* mPtr=static_cast<unsigned short*>(median.getBuffer());
					const unsigned short* bfPtr=backgroundFrame;
					for(int y=0;y<480;++y)
						for(int x=0;x<640;++x,++mPtr,++bfPtr)
							if(*mPtr>=*bfPtr)
								*mPtr=0x07ffU;
					}
				
				/* Post the median-filtered depth frame to the consumer: */
				(*depthStreamingCallback)(median);
				}
			
			/* Read the next depth frame: */
			if(!depthFrameFile.eof())
				{
				depthTimeStamp=depthFrameFile.read<double>();
				currentDepthFrame=(currentDepthFrame+1)%3;
				depthFrameFile.read<unsigned short>(static_cast<unsigned short*>(depthFrames[currentDepthFrame].getBuffer()),640*480);
				++numDepthFrames;
				}
			else
				depthTimeStamp=Math::Constants<double>::max;
			}
		if(currentTime>=colorTimeStamp)
			{
			if(colorStreamingCallback!=0)
				{
				/* Post the next color frame to the consumer: */
				(*colorStreamingCallback)(colorFrame);
				}
			
			/* Read the next color frame: */
			if(!colorFrameFile.eof())
				{
				colorTimeStamp=colorFrameFile.read<double>();
				colorFrame=FrameBuffer(640,480,640*480*3*sizeof(unsigned char));
				colorFrameFile.read<unsigned char>(static_cast<unsigned char*>(colorFrame.getBuffer()),640*480*3);
				}
			else
				colorTimeStamp=Math::Constants<double>::max;
			}
		}
	
	return 0;
	}

KinectPlayback::KinectPlayback(const char* depthFrameFileName,const char* colorFrameFileName)
	:depthFrameFile(depthFrameFileName,"rb",Misc::File::LittleEndian),
	 colorFrameFile(colorFrameFileName,"rb",Misc::File::LittleEndian),
	 depthStreamingCallback(0),colorStreamingCallback(0),
	 numBackgroundFrames(0),backgroundFrame(0),removeBackground(false)
	{
	/* Read and ignore the frame sizes from the depth and color frame files: */
	unsigned int frameSize[2];
	depthFrameFile.read<unsigned int>(frameSize,2);
	colorFrameFile.read<unsigned int>(frameSize,2);
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
	
	delete[] backgroundFrame;
	}

void KinectPlayback::startStreaming(KinectPlayback::StreamingCallback* newColorStreamingCallback,KinectPlayback::StreamingCallback* newDepthStreamingCallback)
	{
	/* Set the streaming callbacks: */
	delete depthStreamingCallback;
	depthStreamingCallback=newDepthStreamingCallback;
	delete colorStreamingCallback;
	colorStreamingCallback=newColorStreamingCallback;
	
	/* Reset the frame timer: */
	frameTimer.elapse();
	
	/* Start the playback thread: */
	playbackThread.start(this,&KinectPlayback::playbackThreadMethod);
	}

void KinectPlayback::captureBackground(unsigned int newNumBackgroundFrames)
	{
	/* Initialize the background frame buffer: */
	if(backgroundFrame==0)
		backgroundFrame=new unsigned short[640*480];
	
	/* Initialize the background frame to "empty:" */
	unsigned short* bfPtr=backgroundFrame;
	for(unsigned int y=0;y<480;++y)
		for(unsigned int x=0;x<640;++x,++bfPtr)
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
