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

#ifndef KINECTCLIENT_INCLUDED
#define KINECTCLIENT_INCLUDED

#include <string>
#include <Threads/Thread.h>
#include <Collaboration/ProtocolClient.h>

#include "Plugins/KinectProtocol.h"

/* Forward declarations: */
namespace Kinect {
class Renderer;
}

class KinectClient:public Collaboration::ProtocolClient,private KinectProtocol
	{
	/* Embedded classes: */
	private:
	class RemoteClientState:public Collaboration::ProtocolClient::RemoteClientState
		{
		/* Elements: */
		public:
		KinectClient* clientPlugin; // Pointer back to the client plugin owning this client state
		std::string kinectServerHostName; // Host name of remote client's local Kinect server
		int kinectServerPortId; // Port number of remote client's local Kinect server
		bool hasServer; // Flag if the remote client has a Kinect server, even if the connection itself failed
		unsigned int numRenderers; // Number of renderers used by the remote client
		Kinect::Renderer** renderers; // Array of streaming renderer objects to receive 3D video from a stand-alone Kinect server
		volatile bool clientReady; // Flag whether the client has been initialized by the background thread
		Threads::Thread initializationThread; // Thread to establish a connection to the remote client's local Kinect server in the background
		OGTransform clientTransform; // Transformation from remote client's navigational space into local client's navigational space
		
		/* Private methods: */
		void connect(void); // Method to connect the client to a remote Kinect server
		void* initializationThreadMethod(void); // Thread method to establish a connection to the remote client's local Kinect server in the background
		
		/* Constructors and destructors: */
		public:
		RemoteClientState(KinectClient* sClientPlugin,std::string sKinectServerHostName,int sKinectServerPortId);
		virtual ~RemoteClientState(void);
		};
	
	friend class RemoteClientState;
	
	/* Elements: */
	private:
	std::string kinectServerHostName; // Host name of this client's own Kinect server
	int kinectServerPortId; // Port number of this client's own Kinect server
	bool haveServer; // Flag if this client has its own Kinect server
	
	/* Private methods: */
	void updateCallback(void); // Called when any remote client has new 3D video data
	
	/* Constructors and destructors: */
	public:
	KinectClient(void); // Creates a Kinect client
	virtual ~KinectClient(void);
	
	/* Methods from Collaboration::ProtocolClient: */
	virtual const char* getName(void) const;
	virtual void initialize(Collaboration::CollaborationClient* sClient,Misc::ConfigurationFileSection& configFileSection);
	virtual void sendConnectRequest(Comm::NetPipe& pipe);
	virtual RemoteClientState* receiveClientConnect(Comm::NetPipe& pipe);
	virtual void frame(Collaboration::ProtocolClient::RemoteClientState* rcs);
	virtual void glRenderAction(const Collaboration::ProtocolClient::RemoteClientState* rcs,GLContextData& contextData) const;
	};

#endif
