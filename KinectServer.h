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

#ifndef KINECTSERVER_INCLUDED
#define KINECTSERVER_INCLUDED

#ifdef VERBOSE
#include <string>
#endif
#include <vector>
#include <IO/VariableMemoryFile.h>
#include <Threads/TripleBuffer.h>
#include <Threads/EventDispatcher.h>
#include <Comm/ListeningTCPSocket.h>
#include <Comm/TCPPipe.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
class libusb_device;
namespace Misc {
class ConfigurationFileSection;
}
namespace Kinect {
class DirectFrameSource;
class FrameWriter;
}

class KinectServer
	{
	/* Embedded classes: */
	private:
	struct CameraState // Structure to hold state related to capturing and compressing a color and depth stream from a Kinect camera
		{
		/* Embedded classes: */
		public:
		struct CompressedFrame // Structure to hold a compressed depth or color frame
			{
			/* Elements: */
			public:
			unsigned int index; // Frame's sequence number as delivered from the camera
			double timeStamp; // Frame's time stamp
			IO::VariableMemoryFile::BufferChain data; // Frame's compressed data
			
			/* Constructors and destructors: */
			CompressedFrame(void) // Dummy constructor
				:index(0),timeStamp(0.0)
				{
				}
			};
		
		/* Elements: */
		public:
		Kinect::DirectFrameSource* camera; // Camera generating the depth and color streams
		unsigned int cameraIndex; // Camera index to identify depth and color frames
		Kinect::FrameSource::DepthCorrection* depthCorrection; // Camera's depth correction parameters
		Kinect::FrameSource::IntrinsicParameters ips; // Camera's intrinsic parameters
		Kinect::FrameSource::ExtrinsicParameters eps; // Camera's extrinsic parameters
		int framePipeFd; // Pipe to signal arrival of new depth or color frames to the run loop
		
		IO::VariableMemoryFile colorFile; // In-memory file to receive compressed color frame data
		Kinect::FrameWriter* colorCompressor; // Compressor for color frames
		IO::VariableMemoryFile::BufferChain colorHeaders; // Write buffer containing the color compressor's header data
		unsigned int colorFrameIndex; // Sequential frame index for color frames
		Threads::TripleBuffer<CompressedFrame> colorFrames; // Triple buffer of compressed color frames
		bool hasSentColorFrame; // Flag whether the camera has sent a color frame as part of the current meta-frame
		
		IO::VariableMemoryFile depthFile; // In-memory file to receive compressed depth frame data
		bool lossyDepthCompression; // Flag whether this camera streams lossy-compressed depth frames
		Kinect::FrameWriter* depthCompressor; // Compressor for depth frames
		IO::VariableMemoryFile::BufferChain depthHeaders; // Write buffer containing the depth compressor's header data
		unsigned int depthFrameIndex; // Sequential frame index for depth frames
		Threads::TripleBuffer<CompressedFrame> depthFrames; // Triple buffer of compressed depth frames
		bool hasSentDepthFrame; // Flag whether the camera has sent a depth frame as part of the current meta-frame
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frame);
		void depthStreamingCallback(const Kinect::FrameBuffer& frame);
		
		/* Constructors and destructors: */
		CameraState(const char* serialNumber,bool sLossyDepthCompression); // Creates a capture and compression state for the given Kinect camera device
		~CameraState(void);
		
		/* Methods: */
		void startStreaming(const Kinect::FrameSource::Time& timeBase); // Starts streaming from the Kinect camera
		void writeHeaders(IO::File& sink) const; // Writes the camera's streaming headers to the given sink
		};
	
	struct ClientState // Class containing state of connected client
		{
		/* Elements: */
		public:
		KinectServer* server; // Pointer to server object handling this client, to simplify event handling
		Comm::TCPPipe pipe; // Pipe connected to the client
		#ifdef VERBOSE
		std::string clientName; // Name of the client, to keep track of connections in verbose mode
		#endif
		Threads::EventDispatcher::ListenerKey listenerKey; // Key with which this client is listening for I/O events
		int state; // Client's current position in the KinectServer protocol state machine
		unsigned int protocolVersion; // Version of the KinectServer protocol to use with this client
		bool streaming; // Flag whether client is currently in streaming mode
		
		/* Constructors and destructors: */
		ClientState(KinectServer* sServer,Comm::ListeningTCPSocket& listenSocket); // Accepts next incoming connection on given listening socket and establishes 3D video streaming connection
		};
	
	typedef std::vector<ClientState*> ClientStateList; // Type for list of connected clients
	
	/* Elements: */
	private:
	Kinect::FrameSource::Time timeBase; // Time point at which server started streaming
	unsigned int numCameras; // Number of Kinect cameras served by the server
	CameraState** cameraStates; // Array of pointers to camera state objects
	int framePipeFds[2]; // Pipe to signal arrivals of new depth or color frames to the run loop
	Threads::EventDispatcher dispatcher; // Event dispatcher to handle communication with multiple clients in parallel
	Comm::ListeningTCPSocket listeningSocket; // Socket listening for incoming client connections
	ClientStateList clients; // List of currently connected clients
	int numStreamingClients; // Number of clients that are currently streaming
	unsigned int metaFrameIndex; // Index of the current meta-frame
	unsigned int numMissingDepthFrames; // Number of outstanding depth frames for this meta-frame
	unsigned int numMissingColorFrames; // Number of outstanding color frames for this meta-frame
	
	/* Private methods: */
	void newFrameCallback(void); // Callback called when a new depth or color frame arrives from one of the cameras
	static bool newFrameCallbackWrapper(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData) // Wrapper function for above
		{
		static_cast<KinectServer*>(userData)->newFrameCallback();
		return false;
		}
	static bool newConnectionCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData); // Callback called when a connection attempt is made at the listening socket
	void disconnectClient(ClientState* client,bool removeListener,bool removeFromList); // Disconnects the given client due to a communication error; removes listener and/or dead client from list if respective flags are true
	static bool clientMessageCallback(Threads::EventDispatcher::ListenerKey eventKey,int eventType,void* userData); // Callback called when a message from a client arrives
	
	/* Constructors and destructors: */
	public:
	KinectServer(Misc::ConfigurationFileSection& configFileSection);
	~KinectServer(void);
	
	/* Methods: */
	void run(void); // Runs the server state machine
	void stop(void) // Stops the server state machine; can be called asynchronously
		{
		/* Stop the dispatcher's event handling: */
		dispatcher.stop();
		}
	};

#endif
