/***********************************************************************
KinectServer - Server to stream 3D video data from one or more Kinect
cameras to remote clients for tele-immersion.
Copyright (c) 2010-2013 Oliver Kreylos

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

#include <vector>
#include <IO/VariableMemoryFile.h>
#include <Threads/Mutex.h>
#include <Threads/MutexCond.h>
#include <Threads/TripleBuffer.h>
#include <Comm/ListeningTCPSocket.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>

/* Forward declarations: */
class libusb_device;
namespace Misc {
class ConfigurationFileSection;
}
namespace USB {
class USBContext;
}
namespace Comm {
class TCPPipe;
}
namespace Kinect {
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
		Kinect::Camera camera; // Camera generating the depth and color streams
		Kinect::FrameSource::DepthCorrection* depthCorrection; // Camera's depth correction parameters
		Kinect::FrameSource::IntrinsicParameters ips; // Camera's intrinsic parameters
		Kinect::FrameSource::ExtrinsicParameters eps; // Camera's extrinsic parameters
		IO::VariableMemoryFile colorFile; // In-memory file to receive compressed color frame data
		Kinect::FrameWriter* colorCompressor; // Compressor for color frames
		IO::VariableMemoryFile::BufferChain colorHeaders; // Write buffer containing the color compressor's header data
		unsigned int colorFrameIndex; // Sequential frame index for color frames
		Threads::TripleBuffer<CompressedFrame> colorFrames; // Triple buffer of compressed color frames
		Threads::MutexCond& newColorFrameCond; // Condition variable to signal a new depth frame
		bool hasSentColorFrame; // Flag whether the camera has sent a color frame as part of the current meta-frame
		
		IO::VariableMemoryFile depthFile; // In-memory file to receive compressed depth frame data
		bool lossyDepthCompression; // Flag whether this camera streams lossy-compressed depth frames
		Kinect::FrameWriter* depthCompressor; // Compressor for depth frames
		IO::VariableMemoryFile::BufferChain depthHeaders; // Write buffer containing the depth compressor's header data
		unsigned int depthFrameIndex; // Sequential frame index for depth frames
		Threads::TripleBuffer<CompressedFrame> depthFrames; // Triple buffer of compressed depth frames
		Threads::MutexCond& newDepthFrameCond; // Condition variable to signal a new depth frame
		bool hasSentDepthFrame; // Flag whether the camera has sent a depth frame as part of the current meta-frame
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frame);
		void depthStreamingCallback(const Kinect::FrameBuffer& frame);
		
		/* Constructors and destructors: */
		CameraState(USB::Context& usbContext,const char* serialNumber,bool sLossyDepthCompression,Threads::MutexCond& sNewColorFrameCond,Threads::MutexCond& sNewDepthFrameCond); // Creates a capture and compression state for the given Kinect camera device
		~CameraState(void);
		
		/* Methods: */
		void startStreaming(void); // Starts streaming from the Kinect camera
		void writeHeaders(IO::File& sink) const; // Writes the camera's streaming headers to the given sink
		};
	
	/* Elements: */
	private:
	unsigned int numCameras; // Number of Kinect cameras served by the server
	CameraState** cameraStates; // Array of pointers to camera state objects
	Threads::MutexCond newFrameCond; // Condition variable to signal a new depth or color frame
	Comm::ListeningTCPSocket listeningSocket; // Socket listening for incoming client connections
	Threads::Mutex clientListMutex; // Mutex protecting access to the client list
	std::vector<Comm::TCPPipe*> clients; // List of TCP sockets for currently connected clients
	Threads::Thread listeningThread; // Thread to listen for incoming client connections
	unsigned int metaFrameIndex; // Index of the current meta-frame
	unsigned int numMissingDepthFrames; // Number of outstanding depth frames for this meta-frame
	unsigned int numMissingColorFrames; // Number of outstanding color frames for this meta-frame
	Threads::Thread streamingThread; // Thread to stream depth and color frames to connected clients
	
	/* Private methods: */
	void* listeningThreadMethod(void); // Thread method to listen for new client connections
	void* streamingThreadMethod(void); // Thread method delivering depth and color frames to all connected clients
	
	/* Constructors and destructors: */
	public:
	KinectServer(USB::Context& usbContext,Misc::ConfigurationFileSection& configFileSection);
	~KinectServer(void);
	
	/* Methods: */
	};

#endif
