/***********************************************************************
KinectClientPlugin - Client object to implement the Kinect 3D video
tele-immersion protocol for the Vrui collaboration infrastructure.
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

#include "KinectClientPlugin.h"

#include <stdexcept>
#include <iostream>
#include <Misc/ThrowStdErr.h>
#include <Misc/StandardMarshallers.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Comm/NetPipe.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>

#include "KinectClient.h"

/******************************************************
Methods of class KinectClientPlugin::RemoteClientState:
******************************************************/

void* KinectClientPlugin::RemoteClientState::initializationThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	KinectClient* newClient=0;
	try
		{
		/* Connect to the remote Kinect server: */
		#ifdef VERBOSE
		std::cout<<"KinectClient: Connecting to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<std::endl;
		#endif
		newClient=new KinectClient(kinectServerHostName.c_str(),kinectServerPortId,Vrui::getClusterMultiplexer());
		
		/* Success; now hand the Kinect client to the rest of the plugin: */
		client=newClient;
		clientReady=true;
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"KinectClient: Could not connect to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<" due to exception "<<err.what()<<std::endl;
		delete newClient;
		}
	catch(...)
		{
		std::cerr<<"KinectClient: Could not connect to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<" due to spurious exception"<<std::endl;
		delete newClient;
		}
	
	return 0;
	}

KinectClientPlugin::RemoteClientState::RemoteClientState(std::string sKinectServerHostName,int sKinectServerPortId)
	:kinectServerHostName(sKinectServerHostName),kinectServerPortId(sKinectServerPortId),
	 hasServer(!kinectServerHostName.empty()&&kinectServerPortId>=0),
	 client(0),clientReady(false)
	{
	if(hasServer)
		{
		#if 0
		
		/* Start the initialization thread: */
		initializationThread.start(this,&KinectClientPlugin::RemoteClientState::initializationThreadMethod);
		
		#else
		
		/* Connect to the remote Kinect server: */
		#ifdef VERBOSE
		std::cout<<"KinectClient: Connecting to remote Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<std::endl;
		#endif
		client=new KinectClient(kinectServerHostName.c_str(),kinectServerPortId,Vrui::getClusterMultiplexer());
		clientReady=true;
		
		#endif
		}
	}

KinectClientPlugin::RemoteClientState::~RemoteClientState(void)
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
	delete client;
	}

/***********************************
Methods of class KinectClientPlugin:
***********************************/

KinectClientPlugin::KinectClientPlugin(void)
	:kinectServerHostName(""),kinectServerPortId(-1),haveServer(false)
	{
	}

KinectClientPlugin::~KinectClientPlugin(void)
	{
	}

const char* KinectClientPlugin::getName(void) const
	{
	return protocolName;
	}

void KinectClientPlugin::initialize(Collaboration::CollaborationClient* sClient,Misc::ConfigurationFileSection& configFileSection)
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
		std::cout<<"KinectClient: Local Kinect server on host "<<kinectServerHostName<<", port "<<kinectServerPortId<<std::endl;
	else
		std::cout<<"KinectClient: Do not have local Kinect server"<<std::endl;
	#endif
	}

void KinectClientPlugin::sendConnectRequest(Comm::NetPipe& pipe)
	{
	#ifdef VERBOSE
	std::cout<<"KinectClient: Sending connect request to server"<<std::endl;
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

KinectClientPlugin::RemoteClientState* KinectClientPlugin::receiveClientConnect(Comm::NetPipe& pipe)
	{
	/* Read the remote client's Kinect server's host name and port number: */
	std::string clientKinectServerHostName=read<std::string>(pipe);
	int clientKinectServerPortId=pipe.read<Misc::SInt32>();
	
	/* Create the remote client state object: */
	RemoteClientState* result=new RemoteClientState(clientKinectServerHostName,clientKinectServerPortId);
	
	return result;
	}

bool KinectClientPlugin::receiveServerUpdate(Collaboration::ProtocolClient::RemoteClientState* rcs,Comm::NetPipe& pipe)
	{
	bool result=false;
	
	RemoteClientState* myRcs=dynamic_cast<RemoteClientState*>(rcs);
	if(myRcs==0)
		Misc::throwStdErr("KinectClientPlugin::receiveServerUpdate: Mismatching remote client state object type");
	
	if(myRcs->hasServer)
		{
		/* Read a new inverse navigation transformation from the server: */
		OGTransform& invNav=myRcs->inverseNavigationTransform.startNewValue();
		read(invNav,pipe);
		myRcs->inverseNavigationTransform.postNewValue();
		
		result=true;
		}
	
	return result;
	}

void KinectClientPlugin::sendClientUpdate(Comm::NetPipe& pipe)
	{
	if(haveServer)
		{
		/* Send the current inverse navigation transformation: */
		write(OGTransform(Vrui::getInverseNavigationTransformation()),pipe);
		}
	}

void KinectClientPlugin::frame(Collaboration::ProtocolClient::RemoteClientState* rcs)
	{
	RemoteClientState* myRcs=dynamic_cast<RemoteClientState*>(rcs);
	if(myRcs==0)
		Misc::throwStdErr("KinectClientPlugin::frame: Mismatching remote client state object type");
	
	/* Call the Kinect client's frame method: */
	if(myRcs->clientReady)
		myRcs->client->frame();
	
	/* Lock the most recent inverse navigation transformation: */
	myRcs->inverseNavigationTransform.lockNewValue();
	}

void KinectClientPlugin::glRenderAction(const Collaboration::ProtocolClient::RemoteClientState* rcs,GLContextData& contextData) const
	{
	const RemoteClientState* myRcs=dynamic_cast<const RemoteClientState*>(rcs);
	if(myRcs==0)
		Misc::throwStdErr("KinectClientPlugin::glRenderAction: Mismatching remote client state object type");
	
	if(myRcs->clientReady)
		{
		/* Go to the client's navigational space: */
		glPushMatrix();
		glMultMatrix(myRcs->inverseNavigationTransform.getLockedValue());
	
		/* Call the Kinect client's glRenderAction method: */
		myRcs->client->glRenderAction(contextData);
		
		glPopMatrix();
		}
	}

/****************
DSO entry points:
****************/

extern "C" {

Collaboration::ProtocolClient* createObject(Collaboration::ProtocolClientLoader& objectLoader)
	{
	return new KinectClientPlugin;
	}

void destroyObject(Collaboration::ProtocolClient* object)
	{
	delete object;
	}

}
