/***********************************************************************
MultiplexedFrameSource - Class to stream several pairs of color and
depth frames from a single source file or pipe.
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
**********************************************************************/

#ifndef KINECT_MULTIPLEXEDFRAMESOURCE_INCLUDED
#define KINECT_MULTIPLEXEDFRAMESOURCE_INCLUDED

#include <Threads/Mutex.h>
#include <Threads/Spinlock.h>
#include <Threads/Thread.h>
#include <Comm/Pipe.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Kinect {
class FrameReader;
}

namespace Kinect {

class MultiplexedFrameSource
	{
	/* Embedded classes: */
	private:
	class Stream:public FrameSource // Class representing a single corresponding color and depth frame stream inside the multiplexed stream
		{
		friend class MultiplexedFrameSource;
		
		/* Elements: */
		private:
		MultiplexedFrameSource* owner; // Pointer to object owning this stream
		unsigned int index; // Index of this stream in owner's stream array
		unsigned int streamFormatVersions[2]; // Format version numbers of the color and depth streams, respectively
		DepthCorrection* depthCorrection; // Stream's depth correction object
		IntrinsicParameters ips; // Stream's intrinsic camera parameters
		ExtrinsicParameters eps; // Stream's extrinsic camera parameters
		Threads::Spinlock streamingMutex; // Mutex protecing the stream's streaming state
		bool streaming; // Flag whether this stream is currently streaming
		StreamingCallback* colorStreamingCallback; // Callback to be called when a new color frame has been received
		StreamingCallback* depthStreamingCallback; // Callback to be called when a new depth frame has been received
		
		/* Constructors and destructors: */
		Stream(MultiplexedFrameSource* sOwner,unsigned int sIndex,IO::File& source); // Initializes stream by reading from given data source
		virtual ~Stream(void); // Destroys the stream
		
		/* Methods from FrameSource: */
		virtual DepthCorrection* getDepthCorrectionParameters(void);
		virtual IntrinsicParameters getIntrinsicParameters(void);
		virtual ExtrinsicParameters getExtrinsicParameters(void);
		virtual const unsigned int* getActualFrameSize(int sensor) const;
		virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
		virtual void stopStreaming(void);
		};
	
	friend class Stream;
	
	/* Elements: */
	private:
	Comm::PipePtr pipe; // The multiplexed source stream
	unsigned int numStreams; // Number of streams in the multiplexer
	FrameReader** colorFrameReaders; // Array of color stream readers for the component streams
	FrameReader** depthFrameReaders; // Array of depth stream readers for the component streams
	FrameBuffer* frames; // Array of color and depth frames in the current metaframe
	Threads::Mutex streamMutex; // Mutex serializing access to the stream array
	unsigned int numStreamsAlive; // Number of streams that are still receiving frames
	Stream** streams; // Array of pointers to streams
	Threads::Thread receivingThread; // The demultiplexer thread
	
	/* Private methods: */
	void* receivingThreadMethod(void); // Thread method demultiplexing streams from the source
	
	/* Constructors and destructors: */
	private:
	MultiplexedFrameSource(Comm::PipePtr sPipe); // Creates a multiplexed source for the given stream source
	~MultiplexedFrameSource(void); // Shuts down the multiplexed source
	
	/* Methods: */
	public:
	static MultiplexedFrameSource* create(Comm::PipePtr sPipe); // Returns a new multiplexed frame source that will self-destruct after the last stream has been destroyed
	unsigned int getNumStreams(void) const // Returns the number of streams in the multiplexed source
		{
		return numStreams;
		}
	FrameSource* getStream(unsigned int streamIndex) // Returns the stream of the given index
		{
		return streams[streamIndex];
		}
	};

}

#endif
