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
#include <Comm/BufferedTCPSocket.h>
#include <Comm/MulticastPipeMultiplexer.h>
#include <Comm/BufferedMulticastPipe.h>
#include <Comm/ClusterBufferedTCPSocket.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Kinect/FrameBuffer.h>

/******************************************
Methods of class KinectClient::CameraState:
******************************************/

KinectClient::CameraState::CameraState(IO::File* source)
	:depthDecompressor(*source),
	 colorDecompressor(*source),
	 projector(*source)
	{
	/* Read the facade transform: */
	OGTransform::Vector translation;
	source->read(translation.getComponents(),3);
	OGTransform::Scalar quaternion[4];
	source->read(quaternion,4);
	OGTransform::Scalar scaling=source->read<OGTransform::Scalar>();
	facadeTransform=OGTransform(translation,OGTransform::Rotation::fromQuaternion(quaternion),scaling);
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
	Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
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
			cameraStates[cameraIndex]->nextColorFrame=cameraStates[cameraIndex]->colorDecompressor.readNextFrame();
			--numMissingColorFrames;
			
			#ifdef VVERBOSE
			std::cout<<cameraStates[cameraIndex]->nextColorFrame.timeStamp<<std::endl;
			#endif
			}
		else
			{
			/* Receive a depth frame: */
			cameraStates[cameraIndex]->nextDepthFrame=cameraStates[cameraIndex]->depthDecompressor.readNextFrame();
			--numMissingDepthFrames;
			
			#ifdef VVERBOSE
			std::cout<<cameraStates[cameraIndex]->nextDepthFrame.timeStamp<<std::endl;
			#endif
			}
		
		/* Check if the current meta frame is complete: */
		if(keepReceiving&&numMissingDepthFrames==0&&numMissingColorFrames==0)
			{
			/* Update all camera's frames for the just-completed meta frame: */
			for(unsigned int i=0;i<numCameras;++i)
				{
				cameraStates[i]->depthFrame=cameraStates[i]->nextDepthFrame;
				cameraStates[i]->colorFrame=cameraStates[i]->nextColorFrame;
				}
			
			/* Tell the main thread that the current meta frame is complete: */
			metaFrameComplete=true;
			Vrui::requestUpdate();
			}
		}
	
	return 0;
	}

KinectClient::KinectClient(const char* kinectServerHostName,int kinectServerPortId,Comm::MulticastPipeMultiplexer* multiplexer)
	:master(multiplexer==0||multiplexer->isMaster()),
	 source(0),
	 numCameras(0),cameraStates(0)
	{
	/* Connect to the Kinect server: */
	if(multiplexer==0)
		{
		/* Create a regular TCP socket: */
		source=new Comm::BufferedTCPSocket(kinectServerHostName,kinectServerPortId);
		}
	else if(multiplexer->isMaster())
		{
		/* Create a cluster-forwarding TCP socket: */
		source=new Comm::ClusterBufferedTCPSocket(kinectServerHostName,kinectServerPortId,multiplexer);
		}
	else
		{
		/* Create a slave-side buffered multicast pipe: */
		source=new Comm::BufferedMulticastPipe(multiplexer);
		}
	
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
	if(master)
		std::cout<<"KinectClient: Initializing "<<numCameras<<" cameras"<<std::endl;
	#endif
	cameraStates=new CameraState*[numCameras];
	for(unsigned int i=0;i<numCameras;++i)
		cameraStates[i]=new CameraState(source);
	
	/* Initialize streaming state: */
	currentMetaFrameIndex=0;
	numMissingDepthFrames=numCameras;
	numMissingColorFrames=numCameras;
	metaFrameComplete=false;
	
	/* Start the receiving thread: */
	keepReceiving=true;
	receivingThread.start(this,&KinectClient::receivingThreadMethod);
	}

KinectClient::~KinectClient(void)
	{
	/* Shut down the receiving thread: */
	if(master)
		{
		/* Shut the thread down gently: */
		keepReceiving=false;
		}
	else
		{
		/* Kill the thread brutally: */
		receivingThread.cancel();
		}
	receivingThread.join();
	
	/* Delete all camera states: */
	for(unsigned int i=0;i<numCameras;++i)
		delete cameraStates[i];
	delete[] cameraStates;
	
	if(master)
		{
		/* Send the disconnect request and shut down the server pipe: */
		source->write<unsigned int>(0);
		source->flush();
		}
	
	/* Close the connection to the server: */
	delete source;
	}

void KinectClient::frame(void)
	{
	if(metaFrameComplete)
		{
		/* Update all Kinect projectors: */
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->projector.setDepthFrame(cameraStates[i]->depthFrame);
			cameraStates[i]->projector.setColorFrame(cameraStates[i]->colorFrame);
			}
		
		metaFrameComplete=false;
		}
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
