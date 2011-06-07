/***********************************************************************
KinectServerPlugin - Server object to implement the Kinect 3D video
tele-immersion protocol for the Vrui collaboration infrastructure.
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

#include "KinectServerPlugin.h"

#include <Misc/ThrowStdErr.h>
#include <Misc/StandardMarshallers.h>

/************************************************
Methods of class KinectServerPlugin::ClientState:
************************************************/

KinectServerPlugin::ClientState::ClientState(void)
	:kinectServerHostName(""),kinectServerPortId(-1)
	{
	}

KinectServerPlugin::ClientState::~ClientState(void)
	{
	}

/***********************************
Methods of class KinectServerPlugin:
***********************************/

const char* KinectServerPlugin::getName(void) const
	{
	return protocolName;
	}

unsigned int KinectServerPlugin::getNumMessages(void) const
	{
	return numProtocolMessages;
	}

KinectServerPlugin::ClientState* KinectServerPlugin::receiveConnectRequest(unsigned int protocolMessageLength,Collaboration::CollaborationPipe& pipe)
	{
	/* Read the remote client's Kinect server's host name and port number: */
	std::string clientKinectServerHostName=Misc::Marshaller<std::string>::read(pipe);
	int clientKinectServerPortId=pipe.read<int>();
	
	/* Check for correctness: */
	if(protocolMessageLength!=Misc::Marshaller<std::string>::getSize(clientKinectServerHostName)+sizeof(unsigned int))
		{
		/* Must be a protocol error; signal failure: */
		return 0;
		}
	
	/* Create a client object: */
	ClientState* result=new ClientState;
	result->kinectServerHostName=clientKinectServerHostName;
	result->kinectServerPortId=clientKinectServerPortId;
	result->hasServer=!result->kinectServerHostName.empty()&&result->kinectServerPortId>=0;
	
	return result;
	}

void KinectServerPlugin::receiveClientUpdate(Collaboration::ProtocolServer::ClientState* cs,Collaboration::CollaborationPipe& pipe)
	{
	/* Get a handle on the source client's state object: */
	ClientState* myCs=dynamic_cast<ClientState*>(cs);
	if(myCs==0)
		Misc::throwStdErr("KinectServerPlugin::receiveClientUpdate: Client state object has mismatching type");
	
	if(myCs->hasServer)
		{
		/* Read the client's new inverse navigation transformation: */
		myCs->inverseNavigationTransform=pipe.readTrackerState();
		}
	}

void KinectServerPlugin::sendClientConnect(Collaboration::ProtocolServer::ClientState* sourceCs,Collaboration::ProtocolServer::ClientState* destCs,Collaboration::CollaborationPipe& pipe)
	{
	/* Get a handle on the source client's state object: */
	ClientState* mySourceCs=dynamic_cast<ClientState*>(sourceCs);
	if(mySourceCs==0)
		Misc::throwStdErr("KinectServerPlugin::sendClientConnect: Client state object has mismatching type");
	
	/* Write the source client's Kinect server's host name and port number: */
	Misc::Marshaller<std::string>::write(mySourceCs->kinectServerHostName,pipe);
	pipe.write<int>(mySourceCs->kinectServerPortId);
	}

void KinectServerPlugin::sendServerUpdate(Collaboration::ProtocolServer::ClientState* sourceCs,Collaboration::ProtocolServer::ClientState* destCs,Collaboration::CollaborationPipe& pipe)
	{
	/* Get a handle on the source client's state object: */
	ClientState* mySourceCs=dynamic_cast<ClientState*>(sourceCs);
	if(mySourceCs==0)
		Misc::throwStdErr("KinectServerPlugin::sendServerUpdate: Client state object has mismatching type");
	
	if(mySourceCs->hasServer)
		{
		/* Write the client's current inverse navigation transformation: */
		pipe.writeTrackerState(mySourceCs->inverseNavigationTransform);
		}
	}

/****************
DSO entry points:
****************/

extern "C" {

Collaboration::ProtocolServer* createObject(Collaboration::ProtocolServerLoader& objectLoader)
	{
	return new KinectServerPlugin;
	}

void destroyObject(Collaboration::ProtocolServer* object)
	{
	delete object;
	}

}
