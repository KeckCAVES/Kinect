/***********************************************************************
KinectClient - Client object to implement the Kinect 3D video tele-
immersion protocol for the Vrui collaboration infrastructure.
Copyright (c) 2010-2012 Oliver Kreylos

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

#include "Plugins/KinectClient.h"

#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/StandardMarshallers.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Comm/NetPipe.h>
#include <Cluster/OpenPipe.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLClipPlaneTracker.h>
#include <GL/GLContextData.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Collaboration/CollaborationClient.h>
#include <Kinect/FunctionCalls.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/Renderer.h>

/************************************************
Methods of class KinectClient::RemoteClientState:
************************************************/

void KinectClient::RemoteClientState::connect(void)
	{
	unsigned int newNumRenderers=0;
	Kinect::Renderer** newRenderers=0;
	try
		{
		/* Connect to the remote Kinect server: */
		#ifdef VERBOSE
		const Threads::Thread::ID& threadId=Threads::Thread::getThreadObject()->getId();
		std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Connecting to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId;
		std::cout<<" from thread ";
		if(threadId.getNumParts()==0)
			std::cout<<"root";
		else
			{
			std::cout<<threadId.getPart(0);
			for(unsigned int i=1;i<threadId.getNumParts();++i)
				std::cout<<'.'<<threadId.getPart(i);
			}
		std::cout<<std::endl;
		#endif
		Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),kinectServerHostName.c_str(),kinectServerPortId));
		
		/* Create one renderer object for each 3D video stream sent by the server: */
		newNumRenderers=source->getNumStreams();
		newRenderers=new Kinect::Renderer*[newNumRenderers];
		for(unsigned int i=0;i<newNumRenderers;++i)
			newRenderers[i]=0;
		for(unsigned int i=0;i<newNumRenderers;++i)
			newRenderers[i]=new Kinect::Renderer(source->getStream(i));
		
		/* Start streaming on all renderers: */
		for(unsigned int i=0;i<newNumRenderers;++i)
			newRenderers[i]->startStreaming(Misc::createFunctionCall(clientPlugin,&KinectClient::updateCallback));
		
		#ifdef VERBOSE
		std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Connection established"<<std::endl;
		#endif
		
		/* Success; now hand the Kinect client to the rest of the plugin: */
		renderers=newRenderers;
		numRenderers=newNumRenderers;
		clientReady=true;
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Could not connect to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<" due to exception "<<err.what()<<std::endl;
		for(unsigned int i=0;i<newNumRenderers;++i)
			delete newRenderers[i];
		delete[] newRenderers;
		}
	catch(...)
		{
		std::cerr<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Connection to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<" cancelled"<<std::endl;
		for(unsigned int i=0;i<newNumRenderers;++i)
			delete newRenderers[i];
		delete[] newRenderers;
		
		/* Re-throw the exception: */
		throw;
		}
	}

void* KinectClient::RemoteClientState::initializationThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Connect to the remote Kinect server: */
	connect();
	
	return 0;
	}

KinectClient::RemoteClientState::RemoteClientState(KinectClient* sClientPlugin,std::string sKinectServerHostName,int sKinectServerPortId)
	:clientPlugin(sClientPlugin),
	 kinectServerHostName(sKinectServerHostName),kinectServerPortId(sKinectServerPortId),
	 hasServer(!kinectServerHostName.empty()&&kinectServerPortId>=0),
	 numRenderers(0),renderers(0),clientReady(false)
	{
	if(hasServer)
		{
		#if 0
		
		/* Start the initialization thread: */
		initializationThread.start(this,&KinectClient::RemoteClientState::initializationThreadMethod);
		
		#else
		
		/* Connect to the remote Kinect server: */
		connect();
		
		#endif
		}
	}

KinectClient::RemoteClientState::~RemoteClientState(void)
	{
	if(hasServer)
		{
		#if 0
		
		/* Shut down the initialization thread: */
		initializationThread.cancel();
		initializationThread.join();
		
		#endif
		}
	
	/* Disconnect from the remote Kinect server: */
	#ifdef VERBOSE
	if(numRenderers>0)
		std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Disconnecting from remote Kinect server"<<std::endl;
	#endif
	for(unsigned int i=0;i<numRenderers;++i)
		delete renderers[i];
	delete[] renderers;
	}

/*****************************
Methods of class KinectClient:
*****************************/

void KinectClient::updateCallback(void)
	{
	Vrui::requestUpdate();
	}

KinectClient::KinectClient(void)
	:kinectServerHostName(""),kinectServerPortId(-1),haveServer(false)
	{
	}

KinectClient::~KinectClient(void)
	{
	}

const char* KinectClient::getName(void) const
	{
	return protocolName;
	}

void KinectClient::initialize(Collaboration::CollaborationClient* sClient,Misc::ConfigurationFileSection& configFileSection)
	{
	/* Call the base class method: */
	ProtocolClient::initialize(sClient,configFileSection);
	
	/* Read the Kinect server's host name and port number: */
	kinectServerHostName=configFileSection.retrieveString("./kinectServerHostName",kinectServerHostName);
	kinectServerPortId=configFileSection.retrieveValue<int>("./kinectServerPort",kinectServerPortId);
	
	/* Determine if client has a local Kinect server: */
	haveServer=!kinectServerHostName.empty()&&kinectServerPortId>=0;
	
	#ifdef VERBOSE
	if(haveServer)
		std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Local Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<std::endl;
	else
		std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Do not have local Kinect server"<<std::endl;
	#endif
	}

void KinectClient::sendConnectRequest(Comm::NetPipe& pipe)
	{
	#ifdef VERBOSE
	std::cout<<"Node "<<Vrui::getNodeIndex()<<": "<<"KinectClient: Sending connect request to server"<<std::endl;
	#endif
	
	/* Send the length of the following message: */
	unsigned int messageLength=sizeof(Card)+Misc::Marshaller<std::string>::getSize(kinectServerHostName)+sizeof(Misc::SInt32);
	pipe.write<Card>(messageLength);
	
	/* Write the protocol version number: */
	pipe.write<Card>(protocolVersion);
	
	/* Write the Kinect server's host name and port number: */
	write(kinectServerHostName,pipe);
	pipe.write<Misc::SInt32>(kinectServerPortId);
	}

KinectClient::RemoteClientState* KinectClient::receiveClientConnect(Comm::NetPipe& pipe)
	{
	/* Read the remote client's Kinect server's host name and port number: */
	std::string clientKinectServerHostName=read<std::string>(pipe);
	int clientKinectServerPortId=pipe.read<Misc::SInt32>();
	
	/* Create the remote client state object: */
	RemoteClientState* result=new RemoteClientState(this,clientKinectServerHostName,clientKinectServerPortId);
	
	return result;
	}

void KinectClient::frame(Collaboration::ProtocolClient::RemoteClientState* rcs)
	{
	RemoteClientState* myRcs=dynamic_cast<RemoteClientState*>(rcs);
	if(myRcs==0)
		Misc::throwStdErr("KinectClient::frame: Mismatching remote client state object type");
	
	/* Call the Kinect renderers's frame methods: */
	if(myRcs->clientReady)
		{
		/* Update the client's renderers: */
		for(unsigned int i=0;i<myRcs->numRenderers;++i)
			myRcs->renderers[i]->frame();
		
		/* Get the remote client's current client state: */
		const Collaboration::CollaborationProtocol::ClientState& cs=client->getClientState(rcs).getLockedValue();
		
		/* Calculate the client's transformation: */
		myRcs->clientTransform=cs.navTransform;
		myRcs->clientTransform.doInvert();
		}
	}

void KinectClient::glRenderAction(const Collaboration::ProtocolClient::RemoteClientState* rcs,GLContextData& contextData) const
	{
	const RemoteClientState* myRcs=dynamic_cast<const RemoteClientState*>(rcs);
	if(myRcs==0)
		Misc::throwStdErr("KinectClient::glRenderAction: Mismatching remote client state object type");
	
	if(myRcs->clientReady)
		{
		/* Go to the remote client's navigational space: */
		glPushMatrix();
		glMultMatrix(myRcs->clientTransform);
		
		/* Temporarily disable all clipping planes: */
		contextData.getClipPlaneTracker()->pause();
		
		/* Call the client's Kinect renderers' glRenderAction methods: */
		for(unsigned int i=0;i<myRcs->numRenderers;++i)
			myRcs->renderers[i]->glRenderAction(contextData);
		
		/* Re-enable clipping: */
		contextData.getClipPlaneTracker()->resume();
		
		glPopMatrix();
		}
	}

/****************
DSO entry points:
****************/

extern "C" {

Collaboration::ProtocolClient* createObject(Collaboration::ProtocolClientLoader& objectLoader)
	{
	return new KinectClient;
	}

void destroyObject(Collaboration::ProtocolClient* object)
	{
	delete object;
	}

}
