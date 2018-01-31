/***********************************************************************
KinectServer - Server to stream 3D video data from one or more Kinect
cameras to remote clients for tele-immersion.
Copyright (c) 2010-2017 Oliver Kreylos

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

#include "KinectServer.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/PrintInteger.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <Misc/Time.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/CompoundValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <USB/DeviceList.h>
#include <IO/File.h>
#include <Geometry/GeometryMarshallers.h>
#include <Video/Config.h>
#include <Kinect/Internal/Config.h>
#include <Kinect/DirectFrameSource.h>
#include <Kinect/OpenDirectFrameSource.h>
#include <Kinect/ColorFrameWriter.h>
#include <Kinect/DepthFrameWriter.h>
#include <Kinect/LossyDepthFrameWriter.h>

/******************************************
Methods of class KinectServer::CameraState:
******************************************/

void KinectServer::CameraState::colorStreamingCallback(const Kinect::FrameBuffer& frame)
	{
	/* Pass the frame to the color compressor: */
	colorCompressor->writeFrame(frame);
	
	/* Store the compressed frame data in the color frame triple buffer: */
	CompressedFrame& compressedFrame=colorFrames.startNewValue();
	compressedFrame.index=colorFrameIndex;
	compressedFrame.timeStamp=frame.timeStamp;
	colorFile.storeBuffers(compressedFrame.data);
	colorFrames.postNewValue();
	++colorFrameIndex;
	
	/* Notify the run loop: */
	Misc::UInt32 frameIndex=cameraIndex*2U;
	write(framePipeFd,&frameIndex,sizeof(frameIndex));
	}

void KinectServer::CameraState::depthStreamingCallback(const Kinect::FrameBuffer& frame)
	{
	/* Pass the frame to the depth compressor: */
	depthCompressor->writeFrame(frame);
	
	/* Store the compressed frame data in the depth frame triple buffer: */
	CompressedFrame& compressedFrame=depthFrames.startNewValue();
	compressedFrame.index=depthFrameIndex;
	compressedFrame.timeStamp=frame.timeStamp;
	depthFile.storeBuffers(compressedFrame.data);
	depthFrames.postNewValue();
	++depthFrameIndex;
	
	/* Notify the run loop: */
	Misc::UInt32 frameIndex=cameraIndex*2U+1U;
	write(framePipeFd,&frameIndex,sizeof(frameIndex));
	}

KinectServer::CameraState::CameraState(const char* serialNumber,bool sLossyDepthCompression)
	:camera(Kinect::openDirectFrameSource(serialNumber)),cameraIndex(0U),
	 depthCorrection(0),framePipeFd(-1),
	 colorFile(16384),colorCompressor(0),
	 colorFrameIndex(0),hasSentColorFrame(false),
	 depthFile(16384),lossyDepthCompression(sLossyDepthCompression),depthCompressor(0),
	 depthFrameIndex(0),hasSentDepthFrame(false)
	{
	/* Retrieve the camera's depth correction parameters: */
	depthCorrection=camera->getDepthCorrectionParameters();
	
	/* Retrieve the camera's intrinsic and extrinsic parameters: */
	ips=camera->getIntrinsicParameters();
	eps=camera->getExtrinsicParameters();
	
	/* Create the color and depth frame compressors: */
	colorCompressor=new Kinect::ColorFrameWriter(colorFile,camera->getActualFrameSize(Kinect::FrameSource::COLOR));
	#if VIDEO_CONFIG_HAVE_THEORA
	if(lossyDepthCompression)
		depthCompressor=new Kinect::LossyDepthFrameWriter(depthFile,camera->getActualFrameSize(Kinect::FrameSource::DEPTH));
	else
		depthCompressor=new Kinect::DepthFrameWriter(depthFile,camera->getActualFrameSize(Kinect::FrameSource::DEPTH));
	#else
	depthCompressor=new Kinect::DepthFrameWriter(depthFile,camera->getActualFrameSize(Kinect::FrameSource::DEPTH));
	#endif
	
	/* Extract the color and depth compressors' stream header data: */
	colorFile.storeBuffers(colorHeaders);
	depthFile.storeBuffers(depthHeaders);
	}

KinectServer::CameraState::~CameraState(void)
	{
	/* Stop streaming: */
	camera->stopStreaming();
	
	/* Destroy the color and depth compressors: */
	delete colorCompressor;
	delete depthCompressor;
	
	/* Destroy the depth correction parameters: */
	delete depthCorrection;
	
	/* Destroy the camera: */
	delete camera;
	}

void KinectServer::CameraState::startStreaming(const Kinect::FrameSource::Time& timeBase)
	{
	/* Start streaming: */
	camera->setTimeBase(timeBase);
	camera->startStreaming(Misc::createFunctionCall(this,&KinectServer::CameraState::colorStreamingCallback),Misc::createFunctionCall(this,&KinectServer::CameraState::depthStreamingCallback));
	}

void KinectServer::CameraState::writeHeaders(IO::File& sink) const
	{
	/* Write the stream format versions: */
	sink.write<Misc::UInt32>(1);
	sink.write<Misc::UInt32>(5);
	
	/* Write the camera's depth correction parameters: */
	if(depthCorrection!=0)
		depthCorrection->write(sink);
	else
		{
		/* Write a dummy depth correction object: */
		int dcSize[2]={1,1};
		Kinect::FrameSource::DepthCorrection dc(0,dcSize);
		dc.write(sink);
		}
	
	/* Check whether the depth stream uses lossy compression: */
	#if VIDEO_CONFIG_HAVE_THEORA
	sink.write<Misc::UInt8>(lossyDepthCompression?1:0);
	#else
	sink.write<Misc::UInt8>(0);
	#endif
	
	/* Write the depth camera's lens distortion correction parameters to the sink: */
	ips.depthLensDistortion.write(sink);
	
	/* Write the camera's intrinsic and extrinsic parameters to the sink: */
	Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::write(ips.colorProjection,sink);
	Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::write(ips.depthProjection,sink);
	Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::write(eps,sink);
	
	/* Write the color and depth compression headers: */
	colorHeaders.writeToSink(sink);
	depthHeaders.writeToSink(sink);
	}

namespace {

/**************
Helper classes:
**************/

enum State
	{
	START,STREAMING
	};

}

/******************************************
Methods of class KinectServer::ClientState:
******************************************/

KinectServer::ClientState::ClientState(KinectServer* sServer,Comm::ListeningTCPSocket& listenSocket)
	:server(sServer),
	 pipe(listenSocket),
	 state(START),
	 protocolVersion(0),
	 streaming(false)
	{
	#ifdef VERBOSE
	/* Assemble the client name: */
	clientName=pipe.getPeerHostName();
	clientName.push_back(':');
	char portId[10];
	clientName.append(Misc::print(pipe.getPeerPortId(),portId+sizeof(portId)-1));
	#endif
	}

/*****************************
Methods of class KinectServer:
*****************************/

void KinectServer::newFrameCallback(void)
	{
	/* Read the camera index and frame type: */
	Misc::UInt32 frameIndex;
	read(framePipeFds[0],&frameIndex,sizeof(frameIndex));
	unsigned int cameraIndex=frameIndex>>1;
	
	/* Check if the frame is a color or depth frame: */
	if(frameIndex&0x01U) // New frame is a depth frame
		{
		/* Check if the camera has not yet sent a depth frame in the current meta frame: */
		if(!cameraStates[cameraIndex]->hasSentDepthFrame&&cameraStates[cameraIndex]->depthFrames.lockNewValue())
			{
			#ifdef VERBOSE2
			std::cout<<" depth "<<cameraIndex<<", "<<cameraStates[cameraIndex]->depthFrames.getLockedValue().index<<", "<<cameraStates[cameraIndex]->depthFrames.getLockedValue().timeStamp<<';';
			#endif
			
			/* Send the camera's new depth frame to all connected clients: */
			for(ClientStateList::iterator csIt=clients.begin();csIt!=clients.end();++csIt)
				if((*csIt)->streaming)
					{
					try
						{
						/* Write the meta frame index and frame identifier: */
						(*csIt)->pipe.write<Misc::UInt32>(metaFrameIndex);
						(*csIt)->pipe.write<Misc::UInt32>(frameIndex);
						
						/* Write the compressed depth frame: */
						cameraStates[cameraIndex]->depthFrames.getLockedValue().data.writeToSink((*csIt)->pipe);
						(*csIt)->pipe.flush();
						}
					catch(std::runtime_error err)
						{
						#ifdef VERBOSE
						std::cout<<"KinectServer: Disconnecting client "<<(*csIt)->clientName<<" due to exception "<<err.what()<<std::endl;
						#endif
						disconnectClient(*csIt,true,false);
						
						/* Remove the client from the list by moving the last element forward: */
						*csIt=clients.back();
						--csIt;
						clients.pop_back();
						}
					}
			
			/* Reduce the number of outstanding depth frames in the current meta frame: */
			cameraStates[cameraIndex]->hasSentDepthFrame=true;
			--numMissingDepthFrames;
			}
		}
	else // New frame is a color frame
		{
		/* Check if the camera has not yet sent a color frame in the current meta frame: */
		if(!cameraStates[cameraIndex]->hasSentColorFrame&&cameraStates[cameraIndex]->colorFrames.lockNewValue())
			{
			#ifdef VERBOSE2
			std::cout<<" color "<<cameraIndex<<", "<<cameraStates[cameraIndex]->colorFrames.getLockedValue().index<<", "<<cameraStates[cameraIndex]->colorFrames.getLockedValue().timeStamp<<';';
			#endif
			
			/* Send the camera's new color frame to all connected clients: */
			for(ClientStateList::iterator csIt=clients.begin();csIt!=clients.end();++csIt)
				if((*csIt)->streaming)
					{
					try
						{
						/* Write the meta frame index and frame identifier: */
						(*csIt)->pipe.write<Misc::UInt32>(metaFrameIndex);
						(*csIt)->pipe.write<Misc::UInt32>(frameIndex);
						
						/* Write the compressed color frame: */
						cameraStates[cameraIndex]->colorFrames.getLockedValue().data.writeToSink((*csIt)->pipe);
						(*csIt)->pipe.flush();
						}
					catch(std::runtime_error err)
						{
						#ifdef VERBOSE
						std::cout<<"KinectServer: Disconnecting client "<<(*csIt)->clientName<<" due to exception "<<err.what()<<std::endl;
						#endif
						disconnectClient(*csIt,true,false);
						
						/* Remove the client from the list by moving the last element forward: */
						*csIt=clients.back();
						--csIt;
						clients.pop_back();
						}
					}
			
			/* Reduce the number of outstanding color frames in the current meta frame: */
			cameraStates[cameraIndex]->hasSentColorFrame=true;
			--numMissingColorFrames;
			}
		}
	
	/* Check if the current meta frame is complete: */
	if(numMissingDepthFrames==0U&&numMissingColorFrames==0U)
		{
		/* Start the next meta frame: */
		++metaFrameIndex;
		for(unsigned int i=0;i<numCameras;++i)
			{
			cameraStates[i]->hasSentColorFrame=false;
			cameraStates[i]->hasSentDepthFrame=false;
			}
		numMissingColorFrames=numCameras;
		numMissingDepthFrames=numCameras;
		
		#ifdef VERBOSE2
		std::cout<<std::endl;
		std::cout<<"Meta frame "<<metaFrameIndex;
		#endif
		}
	}

bool KinectServer::newConnectionCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData)
	{
	KinectServer* thisPtr=static_cast<KinectServer*>(userData);
	
	/* Create a new client state object and add it to the list: */
	ClientState* newClient=new ClientState(thisPtr,thisPtr->listeningSocket);
	
	#ifdef VERBOSE
	std::cout<<"KinectServer: Connecting new client "<<newClient->clientName<<std::endl;
	#endif
	
	thisPtr->clients.push_back(newClient);
	
	/* Add an event listener for incoming messages from the client: */
	newClient->listenerKey=thisPtr->dispatcher.addIOEventListener(newClient->pipe.getFd(),Threads::EventDispatcher::Read,thisPtr->clientMessageCallback,newClient);
	
	return false;
	}

void KinectServer::disconnectClient(KinectServer::ClientState* client,bool removeListener,bool removeFromList)
	{
	if(removeListener)
		{
		/* Stop listening on the client's pipe: */
		dispatcher.removeIOEventListener(client->listenerKey);
		}
	
	/* Check if the client is still streaming: */
	if(client->streaming)
		--numStreamingClients;
	
	/* Disconnect the client: */
	delete client;
	
	if(removeFromList)
		{
		/* Remove the dead client from the list: */
		for(ClientStateList::iterator csIt=clients.begin();csIt!=clients.end();++csIt)
			if(*csIt==client)
				{
				/* Remove it and stop searching: */
				*csIt=clients.back();
				clients.pop_back();
				break;
				}
		}
	}

bool KinectServer::clientMessageCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData)
	{
	ClientState* client=static_cast<ClientState*>(userData);
	KinectServer* thisPtr=client->server;
	
	bool result=false;
	
	try
		{
		/* Read some data from the socket into the socket's read buffer and check if client hung up: */
		if(client->pipe.readSomeData()==0)
			throw std::runtime_error("Client terminated connection");
		
		/* Process messages as long as there is data in the read buffer: */
		while(!result&&client->pipe.canReadImmediately())
			{
			switch(client->state)
				{
				case START:
					{
					/* Read endianness flag and protocol version from new client: */
					Misc::UInt32 endiannessFlag=client->pipe.read<Misc::UInt32>();
					if(endiannessFlag==0x78563412U)
						client->pipe.setSwapOnRead(true);
					else if(endiannessFlag!=0x12345678U)
						throw std::runtime_error("Client has unrecognized endianness");
					client->protocolVersion=client->pipe.read<Misc::UInt32>();
					if(client->protocolVersion>1U)
						client->protocolVersion=1U;
					
					/* Send stream initialization states to the new client: */
					#ifdef VERBOSE
					std::cout<<"KinectServer: Sending stream headers to client "<<client->clientName<<std::endl;
					#endif
					client->pipe.write<Misc::UInt32>(0x12345678U);
					client->pipe.write<Misc::UInt32>(client->protocolVersion);
					Kinect::FrameSource::Time now;
					client->pipe.write<Misc::Float64>(double(now-thisPtr->timeBase));
					client->pipe.write<Misc::UInt32>(thisPtr->numCameras);
					for(unsigned i=0;i<thisPtr->numCameras;++i)
						thisPtr->cameraStates[i]->writeHeaders(client->pipe);
					
					/* Finish the reply message: */
					client->pipe.flush();
					
					/* Increase the number of streaming clients: */
					++thisPtr->numStreamingClients;
					
					/* Go to streaming state: */
					client->state=STREAMING;
					client->streaming=true;
					#ifdef VERBOSE
					std::cout<<"KinectServer: Client "<<client->clientName<<" entered streaming mode"<<std::endl;
					#endif
					
					break;
					}
				
				case STREAMING:
					{
					/* Read a message from the client: */
					Misc::UInt32 message=client->pipe.read<Misc::UInt32>();
					
					if(message==0U) // Disconnect request
						{
						/* Cleanly disconnect this client: */
						#ifdef VERBOSE
						std::cout<<"KinectServer: Disconnecting client "<<client->clientName<<std::endl;
						#endif
						thisPtr->disconnectClient(client,false,true);
						result=true;
						}
					else
						throw std::runtime_error("Protocol error in STREAMING state");
						
					break;
					}
				}
			}
		}
	catch(std::runtime_error err)
		{
		#ifdef VERBOSE
		std::cout<<"KinectServer: Disconnecting client "<<client->clientName<<" due to exception "<<err.what()<<std::endl;
		#endif
		thisPtr->disconnectClient(client,false,true);
		result=true;
		}
	
	return result;
	}

KinectServer::KinectServer(Misc::ConfigurationFileSection& configFileSection)
	:numCameras(0),cameraStates(0),
	 listeningSocket(configFileSection.retrieveValue<int>("./listenPortId",26000),5),
	 numStreamingClients(0)
	{
	/* Create a pipe to signal arrival of new frames to the run loop: */
	if(pipe(framePipeFds)<0)
		{
		int error=errno;
		Misc::throwStdErr("KinectServer: Unable to open frame notification pipe due to error %d (%s)",error,strerror(error));
		}
	
	/* Read the list of cameras: */
	std::vector<std::string> cameraNames=configFileSection.retrieveValue<std::vector<std::string> >("./cameras",std::vector<std::string>());
	numCameras=cameraNames.size();
	cameraStates=new CameraState*[numCameras];
	
	/* Connect to all requested Kinect devices: */
	unsigned int numFoundCameras=0;
	for(unsigned int i=0;i<numCameras;++i)
		{
		/* Read the camera's serial number: */
		Misc::ConfigurationFileSection cameraSection=configFileSection.getSection(cameraNames[i].c_str());
		std::string serialNumber=cameraSection.retrieveString("./serialNumber");
		
		try
			{
			/* Create a streamer for the Kinect device of the requested serial number: */
			#ifdef VERBOSE
			std::cout<<"KinectServer: Creating streamer for camera with serial number "<<serialNumber<<std::endl;
			#endif
			cameraStates[numFoundCameras]=new CameraState(serialNumber.c_str(),cameraSection.retrieveValue<bool>("./lossyDepthCompression",false));
			
			/* Check if camera is to remove background: */
			if(cameraSection.retrieveValue<bool>("./removeBackground",true))
				{
				Kinect::DirectFrameSource* camera=cameraStates[numFoundCameras]->camera;
				
				/* Check whether to load a previously saved background file: */
				std::string backgroundFile=cameraSection.retrieveValue<std::string>("./backgroundFile",std::string());
				if(!backgroundFile.empty())
					{
					/* Load the background file: */
					std::string fullBackgroundFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
					fullBackgroundFileName.push_back('/');
					fullBackgroundFileName.append(backgroundFile);
					#ifdef VERBOSE
					std::cout<<"KinectServer: Loading background depth image file "<<fullBackgroundFileName<<'-'<<serialNumber<<".background"<<std::endl;
					#endif
					camera->loadBackground(fullBackgroundFileName.c_str());
					}
				
				/* Check whether to capture background: */
				unsigned int captureBackgroundFrames=cameraSection.retrieveValue<unsigned int>("./captureBackgroundFrames",0);
				if(captureBackgroundFrames>0)
					{
					/* Request background capture: */
					#ifdef VERBOSE
					std::cout<<"KinectServer: Capturing "<<captureBackgroundFrames<<" background depth frames"<<std::endl;
					#endif
					camera->captureBackground(captureBackgroundFrames,false);
					}
				
				/* Check whether to set a maximum depth value: */
				unsigned int maxDepth=cameraSection.retrieveValue<unsigned int>("./maxDepth",0);
				if(maxDepth>0)
					{
					/* Set the maximum depth: */
					#ifdef VERBOSE
					std::cout<<"KinectServer: Setting maximum depth value to "<<maxDepth<<std::endl;
					#endif
					camera->setMaxDepth(maxDepth,false);
					}
				
				/* Set the background removal fuzz value: */
				int backgroundFuzz=cameraSection.retrieveValue<int>("./backgroundFuzz",camera->getBackgroundRemovalFuzz());
				#ifdef VERBOSE
				std::cout<<"KinectServer: Setting background depth fuzz value to "<<backgroundFuzz<<std::endl;
				#endif
				camera->setBackgroundRemovalFuzz(backgroundFuzz);
				
				/* Enable background removal: */
				camera->setRemoveBackground(true);
				}
			
			++numFoundCameras;
			}
		catch(std::runtime_error err)
			{
			std::cerr<<"Could not open Kinect camera with serial number "<<serialNumber<<" due to exception "<<err.what()<<std::endl;
			}
		}
	
	/* Initialize streaming state: */
	#ifdef VERBOSE
	std::cout<<"KinectServer: "<<numFoundCameras<<" cameras initialized"<<std::endl;
	#endif
	numCameras=numFoundCameras;
	metaFrameIndex=0;
	numMissingColorFrames=numCameras;
	numMissingDepthFrames=numCameras;
	for(unsigned int i=0;i<numCameras;++i)
		{
		cameraStates[i]->cameraIndex=i;
		cameraStates[i]->framePipeFd=framePipeFds[1];
		}
	
	/* Add an event listener for frame arrival messages: */
	dispatcher.addIOEventListener(framePipeFds[0],Threads::EventDispatcher::Read,newFrameCallbackWrapper,this);
	
	/* Add an event listener for incoming connections on the listening socket: */
	#ifdef VERBOSE
	std::cout<<"KinectServer: Listening for incoming connections on TCP port "<<listeningSocket.getPortId()<<std::endl;
	#endif
	dispatcher.addIOEventListener(listeningSocket.getFd(),Threads::EventDispatcher::Read,newConnectionCallback,this);
	}

KinectServer::~KinectServer(void)
	{
	/* Forcefully disconnect all clients: */
	#ifdef VERBOSE
	std::cout<<"KinectServer: Disconnecting all clients"<<std::endl;
	#endif
	for(ClientStateList::iterator csIt=clients.begin();csIt!=clients.end();++csIt)
		delete *csIt;
	
	/* Delete all camera states: */
	#ifdef VERBOSE
	std::cout<<"KinectServer: Disconnecting from all cameras"<<std::endl;
	#endif
	for(unsigned int i=0;i<numCameras;++i)
		delete cameraStates[i];
	delete[] cameraStates;
	
	/* Close the frame notification pipe: */
	for(int i=0;i<2;++i)
		close(framePipeFds[i]);
	}

void KinectServer::run(void)
	{
	/* Start streaming on all connected cameras: */
	#ifdef VERBOSE
	std::cout<<"KinectServer: Starting streaming on "<<numCameras<<" cameras"<<std::endl;
	#endif
	timeBase.set();
	for(unsigned int i=0;i<numCameras;++i)
		cameraStates[i]->startStreaming(timeBase);
	
	#ifdef VERBOSE2
	std::cout<<"Meta frame "<<metaFrameIndex;
	#endif
	
	/* Run the main loop and dispatch events until stopped: */
	dispatcher.dispatchEvents();
	}
