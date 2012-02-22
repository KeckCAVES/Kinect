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
	:colorDecompressor(source),
	 depthDecompressor(source)
	{
	/* Read the intrinsic and extrinsic camera parameters from the source: */
	Kinect::FrameSource::IntrinsicParameters ips;
	ips.colorProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(source);
	ips.depthProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(source);
	Kinect::FrameSource::ExtrinsicParameters eps=Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::read(source);
	
	/* Set the projector's camera parameters: */
	projector.setIntrinsicParameters(ips);
	projector.setExtrinsicParameters(eps);
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
	Kinect::FrameBuffer* frameBuffers=new Kinect::FrameBuffer[numCameras*2];
	#else
	Kinect::FrameBuffer* frameBuffers=metaFrames.startNewValue();
	#endif
	
	try
		{
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
				numMissingColorFrames=numCameras;
				numMissingDepthFrames=numCameras;
				
				/* Start a new meta frame: */
				currentMetaFrameIndex=metaFrameIndex;
				}
			
			/* Process the frame: */
			unsigned int cameraIndex=frameId>>1;
			if(frameId&0x1U)
				{
				/* Receive a depth frame: */
				frameBuffers[frameId]=cameraStates[cameraIndex]->depthDecompressor.readNextFrame();
				--numMissingDepthFrames;
				
				#ifdef VVERBOSE
				std::cout<<frameBuffers[frameId].timeStamp<<std::endl;
				#endif
				}
			else
				{
				/* Receive a color frame: */
				frameBuffers[frameId]=cameraStates[cameraIndex]->colorDecompressor.readNextFrame();
				--numMissingColorFrames;
				
				#ifdef VVERBOSE
				std::cout<<frameBuffers[frameId].timeStamp<<std::endl;
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
		}
	catch(std::runtime_error err)
		{
		/* Ignore the error and terminate the thread */
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
		metaFrames.getBuffer(i)=new Kinect::FrameBuffer[numCameras*2]; // One depth and color frame per camera
	#endif
	
	/* Initialize streaming state: */
	currentMetaFrameIndex=0;
	numMissingColorFrames=numCameras;
	numMissingDepthFrames=numCameras;
	
	/* Start the receiving thread: */
	keepReceiving=true;
	receivingThread.start(this,&KinectClient::receivingThreadMethod);
	}

KinectClient::~KinectClient(void)
	{
	/* Shut down the receiving thread: */
	// receivingThread.cancel();
	keepReceiving=false;
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
	
	try
		{
		/* Send the disconnect request and shut down the server pipe: */
		source->write<unsigned int>(0);
		source->flush();
		}
	catch(...)
		{
		/* Ignore the error; we were just being polite anyway */
		}
	}

void KinectClient::frame(void)
	{
	/* Check if a new metaframe has been completed: */
	#if KINECTCLIENT_DELAY
	{
	Threads::Spinlock::Lock metaFrameQueueLock(metaFrameQueueMutex);
	if(!metaFrameQueue.empty()&&metaFrameQueue.front()[0].timeStamp<=Vrui::getApplicationTime())
		{
		Kinect::FrameBuffer* fbs=metaFrameQueue.front();
		
		/* Update all Kinect projectors: */
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->projector.setColorFrame(fbs[i*2+0]);
			cameraStates[i]->projector.setDepthFrame(fbs[i*2+1]);
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
		Kinect::FrameBuffer* fbs=metaFrames.getLockedValue();
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->projector.setColorFrame(fbs[i*2+0]);
			cameraStates[i]->projector.setDepthFrame(fbs[i*2+1]);
			}
		}
	#endif
	}

void KinectClient::glRenderAction(GLContextData& contextData) const
	{
	for(unsigned int i=0;i<numCameras;++i)
		{
		/* Draw the camera's facade: */
		cameraStates[i]->projector.draw(contextData);
		}
	}
