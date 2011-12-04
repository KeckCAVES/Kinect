/***********************************************************************
KinectClient - Client to receive 3D video data of one or more Kinect
cameras from a remote server for tele-immersion.
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

#include "KinectClient.h"

#if defined(VERBOSE)||defined(VVERBOSE)
#include <iostream>
#endif
#include <Misc/ThrowStdErr.h>
#include <Comm/TCPPipe.h>
#include <Cluster/Multiplexer.h>
#include <Cluster/OpenPipe.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Kinect/FrameBuffer.h>

/******************************************
Methods of class KinectClient::CameraState:
******************************************/

KinectClient::CameraState::CameraState(IO::File& source)
	:depthDecompressor(source),
	 colorDecompressor(source),
	 projector(source),
	 facadeTransform(Misc::Marshaller<OGTransform>::read(source))
	{
	}

KinectClient::CameraState::~CameraState(void)
	{
	}

/*****************************
Methods of class KinectClient:
*****************************/

void* KinectClient::receivingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Get the first metaframe: */
	#if KINECTCLIENT_DELAY
	FrameBuffer* frameBuffers=new FrameBuffer[numCameras*2];
	#else
	FrameBuffer* frameBuffers=metaFrames.startNewValue();
	#endif
	
	while(keepReceiving)
		{
		/* Receive the next frame's identifier: */
		unsigned int metaFrameIndex=source->read<unsigned int>();
		unsigned int frameId=source->read<unsigned int>();
		
		#ifdef VVERBOSE
		std::cout<<metaFrameIndex<<", "<<frameId<<", "<<std::flush;
		#endif
		
		/* Check for the beginning of a new meta frame: */
		if(currentMetaFrameIndex!=metaFrameIndex)
			{
			/* Reset the frame counters: */
			numMissingDepthFrames=numCameras;
			numMissingColorFrames=numCameras;
			
			/* Start a new meta frame: */
			currentMetaFrameIndex=metaFrameIndex;
			}
		
		/* Process the frame: */
		unsigned int cameraIndex=frameId>>1;
		if(frameId&0x1U)
			{
			/* Receive a color frame: */
			frameBuffers[cameraIndex*2+1]=cameraStates[cameraIndex]->colorDecompressor.readNextFrame();
			--numMissingColorFrames;
			
			#ifdef VVERBOSE
			std::cout<<frameBuffers[cameraIndex*2+1].timeStamp<<std::endl;
			#endif
			}
		else
			{
			/* Receive a depth frame: */
			frameBuffers[cameraIndex*2+0]=cameraStates[cameraIndex]->depthDecompressor.readNextFrame();
			--numMissingDepthFrames;
			
			#ifdef VVERBOSE
			std::cout<<frameBuffers[cameraIndex*2+0].timeStamp<<std::endl;
			#endif
			}
		
		/* Check if the current meta frame is complete: */
		if(keepReceiving&&numMissingDepthFrames==0&&numMissingColorFrames==0)
			{
			/* Push the completed metaframe to the projectors: */
			#if KINECTCLIENT_DELAY
			for(unsigned int i=0;i<numCameras*2;++i)
				frameBuffers[i].timeStamp=Vrui::getApplicationTime()+5.0;
			
			{
			Threads::Spinlock::Lock metaFrameQueueLock(metaFrameQueueMutex);
			metaFrameQueue.push_back(frameBuffers);
			}
			
			/* Create the next frame buffer: */
			frameBuffers=new FrameBuffer[numCameras*2];
			#else
			metaFrames.postNewValue();
			frameBuffers=metaFrames.startNewValue();
			#endif
			
			/* Tell the main thread that the current meta frame is complete: */
			Vrui::requestUpdate();
			}
		}
	
	return 0;
	}

KinectClient::KinectClient(const char* kinectServerHostName,int kinectServerPortId,Cluster::Multiplexer* multiplexer)
	:numCameras(0),cameraStates(0)
	{
	/* Connect to the Kinect server: */
	source=Cluster::openTCPPipe(multiplexer,kinectServerHostName,kinectServerPortId);
	
	/* Determine server's endianness: */
	unsigned int endiannessFlag=source->read<unsigned int>();
	if(endiannessFlag==0x78563412U)
		source->setSwapOnRead(true);
	else if(endiannessFlag!=0x12345678U)
		Misc::throwStdErr("KinectClient::KinectClient: Server has unrecognized endianness");
	
	/* Read the number of served cameras: */
	numCameras=source->read<unsigned int>();
	
	/* Initialize all camera states: */
	#ifdef VERBOSE
	if(multiplexer==0||multiplexer->isMaster())
		std::cout<<"KinectClient: Initializing "<<numCameras<<" cameras"<<std::endl;
	#endif
	cameraStates=new CameraState*[numCameras];
	for(unsigned int i=0;i<numCameras;++i)
		cameraStates[i]=new CameraState(*source);
	
	#if !KINECTCLIENT_DELAY
	/* Create the metaframe triple buffer: */
	for(int i=0;i<3;++i)
		metaFrames.getBuffer(i)=new FrameBuffer[numCameras*2]; // One depth and color frame per camera
	#endif
	
	/* Initialize streaming state: */
	currentMetaFrameIndex=0;
	numMissingDepthFrames=numCameras;
	numMissingColorFrames=numCameras;
	
	/* Start the receiving thread: */
	keepReceiving=true;
	receivingThread.start(this,&KinectClient::receivingThreadMethod);
	}

KinectClient::~KinectClient(void)
	{
	/* Shut down the receiving thread: */
	receivingThread.cancel();
	receivingThread.join();
	
	/* Delete all camera states: */
	for(unsigned int i=0;i<numCameras;++i)
		delete cameraStates[i];
	delete[] cameraStates;
	
	#if KINECTCLIENT_DELAY
	for(std::deque<FrameBuffer*>::iterator mfqIt=metaFrameQueue.begin();mfqIt!=metaFrameQueue.end();++mfqIt)
		delete[] *mfqIt;
	#else
	/* Clear the metaframe triple buffer: */
	for(int i=0;i<3;++i)
		delete[] metaFrames.getBuffer(i);
	#endif
	
	/* Send the disconnect request and shut down the server pipe: */
	source->write<unsigned int>(0);
	source->flush();
	}

void KinectClient::frame(void)
	{
	/* Check if a new metaframe has been completed: */
	#if KINECTCLIENT_DELAY
	{
	Threads::Spinlock::Lock metaFrameQueueLock(metaFrameQueueMutex);
	if(!metaFrameQueue.empty()&&metaFrameQueue.front()[0].timeStamp<=Vrui::getApplicationTime())
		{
		FrameBuffer* fbs=metaFrameQueue.front();
		/* Update all Kinect projectors: */
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->projector.setDepthFrame(fbs[i*2+0]);
			cameraStates[i]->projector.setColorFrame(fbs[i*2+1]);
			}
		
		/* Remove the metaframe: */
		delete[] fbs;
		metaFrameQueue.pop_front();
		}
	}
	#else
	if(metaFrames.lockNewValue())
		{
		/* Update all Kinect projectors: */
		FrameBuffer* fbs=metaFrames.getLockedValue();
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->projector.setDepthFrame(fbs[i*2+0]);
			cameraStates[i]->projector.setColorFrame(fbs[i*2+1]);
			}
		}
	#endif
	}

void KinectClient::glRenderAction(GLContextData& contextData) const
	{
	for(unsigned int i=0;i<numCameras;++i)
		{
		/* Go to the camera's facade's coordinate system: */
		glPushMatrix();
		glMultMatrix(cameraStates[i]->facadeTransform);
		
		/* Draw the camera's facade: */
		cameraStates[i]->projector.draw(contextData);
		
		/* Go back to previous coordinate system: */
		glPopMatrix();
		}
	}
