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
**********************************************************************/

#ifndef KINECTCLIENT_INCLUDED
#define KINECTCLIENT_INCLUDED

#define KINECTCLIENT_DELAY 0

#if KINECTCLIENT_DELAY
#include <deque>
#include <Threads/Spinlock.h>
#endif
#include <IO/File.h>
#include <Threads/Thread.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/DepthFrameReader.h>
#include <Kinect/ColorFrameReader.h>
#include <Kinect/Projector.h>

/* Forward declarations: */
namespace Cluster {
class Multiplexer;
}
class GLContextData;

class KinectClient
	{
	/* Embedded classes: */
	private:
	struct CameraState // Structure to hold state related to displaying a color and depth stream from a remote Kinect camera
		{
		/* Elements: */
		public:
		Kinect::ColorFrameReader colorDecompressor; // Decompressor for color frames
		Kinect::DepthFrameReader depthDecompressor; // Decompressor for depth frames
		Kinect::Projector projector; // Projector to reproject and render the 3D video facade
		
		/* Constructors and destructors: */
		CameraState(IO::File& source); // Initializes camera state by reading from given data source
		};
	
	/* Elements: */
	IO::FilePtr source; // Data source receiving depth and color frames from the Kinect server
	unsigned int numCameras; // Number of Kinect cameras served by the server
	CameraState** cameraStates; // Array of pointers to camera state objects
	Threads::Thread receivingThread; // Thread receiving frame data from the server
	#if KINECTCLIENT_DELAY
	Threads::Spinlock metaFrameQueueMutex; // Mutex serializing access to the metaframe queue
	std::deque<FrameBuffer*> metaFrameQueue; // Queue of complete metaframes waiting to be displayed
	#else
	Threads::TripleBuffer<Kinect::FrameBuffer*> metaFrames; // Triple buffer holding complete metaframes (depth and color interleaved)
	#endif
	unsigned int currentMetaFrameIndex; // Index of the meta frame currently being received from the server
	unsigned int numMissingColorFrames; // Number of color frames still missing from the current meta frame
	unsigned int numMissingDepthFrames; // Number of depth frames still missing from the current meta frame
	volatile bool keepReceiving; // Flag to cleanly shut down the receiving thread
	
	/* Private methods: */
	void* receivingThreadMethod(void); // Thread method receiving frame data from the server
	
	/* Constructors and destructors: */
	public:
	KinectClient(const char* kinectServerHostName,int kinectServerPortId,Cluster::Multiplexer* multiplexer =0); // Creates Kinect client connected to the given Kinect server; adds cluster distribution if given multiplexer is not 0
	~KinectClient(void);
	
	/* Methods: */
	void frame(void);
	void glRenderAction(GLContextData& contextData) const; // Draws all facades
	};

#endif
