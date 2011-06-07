/***********************************************************************
KinectClientPlugin - Client object to implement the Kinect 3D video
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

#ifndef KINECTCLIENTPLUGIN_INCLUDED
#define KINECTCLIENTPLUGIN_INCLUDED

#include <string>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Collaboration/ProtocolClient.h>

#include "KinectPipe.h"

/* Forward declarations: */
class KinectClient;

class KinectClientPlugin:public Collaboration::ProtocolClient,public KinectPipe
	{
	/* Embedded classes: */
	private:
	class RemoteClientState:public Collaboration::ProtocolClient::RemoteClientState
		{
		/* Elements: */
		public:
		std::string kinectServerHostName; // Host name of remote client's local Kinect server
		int kinectServerPortId; // Port number of remote client's local Kinect server
		bool hasServer; // Flag if the remote client has a Kinect server, even if the connection itself failed
		KinectClient* client; // Client object to receive 3D video from a stand-alone Kinect server
		volatile bool clientReady; // Flag whether the client has been initialized by the background thread
		Threads::Thread initializationThread; // Thread to establish a connection to the remote client's local Kinect server in the background
		Threads::TripleBuffer<Collaboration::CollaborationPipe::OGTransform> inverseNavigationTransform; // The remote client's current inverse navigation transformation
		
		/* Private methods: */
		void* initializationThreadMethod(void); // Thread method to establish a connection to the remote client's local Kinect server in the background
		
		/* Constructors and destructors: */
		public:
		RemoteClientState(std::string sKinectServerHostName,int sKinectServerPortId);
		virtual ~RemoteClientState(void);
		};
	
	/* Elements: */
	private:
	std::string kinectServerHostName; // Host name of this client's own Kinect server
	int kinectServerPortId; // Port number of this client's own Kinect server
	bool haveServer; // Flag if the client has its own Kinect server
	
	/* Constructors and destructors: */
	public:
	KinectClientPlugin(void); // Creates a Kinect client
	virtual ~KinectClientPlugin(void);
	
	/* Methods from Collaboration::ProtocolClient: */
	virtual const char* getName(void) const;
	virtual unsigned int getNumMessages(void) const;
	virtual void initialize(Collaboration::CollaborationClient& collaborationClient,Misc::ConfigurationFileSection& configFileSection);
	virtual void sendConnectRequest(Collaboration::CollaborationPipe& pipe);
	virtual void sendClientUpdate(Collaboration::CollaborationPipe& pipe);
	virtual RemoteClientState* receiveClientConnect(Collaboration::CollaborationPipe& pipe);
	virtual void receiveServerUpdate(Collaboration::ProtocolClient::RemoteClientState* rcs,Collaboration::CollaborationPipe& pipe);
	virtual void frame(Collaboration::ProtocolClient::RemoteClientState* rcs);
	virtual void glRenderAction(const Collaboration::ProtocolClient::RemoteClientState* rcs,GLContextData& contextData) const;
	};

#endif
