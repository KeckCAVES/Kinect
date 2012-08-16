/***********************************************************************
KinectServer - Server object to implement the Kinect 3D video tele-
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

#include "Plugins/KinectServer.h"

#include <Misc/ThrowStdErr.h>
#include <Misc/StandardMarshallers.h>
#include <Comm/NetPipe.h>

/******************************************
Methods of class KinectServer::ClientState:
******************************************/

KinectServer::ClientState::ClientState(void)
	:kinectServerHostName(""),kinectServerPortId(-1)
	{
	}

KinectServer::ClientState::~ClientState(void)
	{
	}

/*****************************
Methods of class KinectServer:
*****************************/

const char* KinectServer::getName(void) const
	{
	return protocolName;
	}

KinectServer::ClientState* KinectServer::receiveConnectRequest(unsigned int protocolMessageLength,Comm::NetPipe& pipe)
	{
	/* Read the client's Kinect protocol version: */
	unsigned int clientProtocolVersion=pipe.read<Card>();
	
	/* Check for the correct version number: */
	if(clientProtocolVersion!=protocolVersion)
		return 0;
	
	/* Read the remote client's Kinect server's host name and port number: */
	std::string clientKinectServerHostName=read<std::string>(pipe);
	int clientKinectServerPortId=pipe.read<Misc::SInt32>();
	
	/* Check for correctness: */
	if(protocolMessageLength!=sizeof(Card)+Misc::Marshaller<std::string>::getSize(clientKinectServerHostName)+sizeof(Misc::SInt32))
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

void KinectServer::sendClientConnect(Collaboration::ProtocolServer::ClientState* sourceCs,Collaboration::ProtocolServer::ClientState* destCs,Comm::NetPipe& pipe)
	{
	/* Get a handle on the source client's state object: */
	ClientState* mySourceCs=dynamic_cast<ClientState*>(sourceCs);
	if(mySourceCs==0)
		Misc::throwStdErr("KinectServer::sendClientConnect: Client state object has mismatching type");
	
	/* Write the source client's Kinect server's host name and port number: */
	write(mySourceCs->kinectServerHostName,pipe);
	pipe.write<Misc::SInt32>(mySourceCs->kinectServerPortId);
	}

/****************
DSO entry points:
****************/

extern "C" {

Collaboration::ProtocolServer* createObject(Collaboration::ProtocolServerLoader& objectLoader)
	{
	return new KinectServer;
	}

void destroyObject(Collaboration::ProtocolServer* object)
	{
	delete object;
	}

}
