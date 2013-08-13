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

#include <Kinect/MultiplexedFrameSource.h>

#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <Cluster/ClusterPipe.h>
#include <Geometry/GeometryMarshallers.h>
#include <Kinect/ColorFrameReader.h>
#include <Kinect/DepthFrameReader.h>
#include <Kinect/LossyDepthFrameReader.h>

namespace Kinect {

/***********************************************
Methods of class MultiplexedFrameSource::Stream:
***********************************************/

MultiplexedFrameSource::Stream::Stream(MultiplexedFrameSource* sOwner,unsigned int sIndex,IO::File& source)
	:owner(sOwner),index(sIndex),
	 depthCorrection(0),
	 streaming(false),colorStreamingCallback(0),depthStreamingCallback(0)
	{
	/* Register this source with the stream multiplexer: */
	{
	Threads::Mutex::Lock streamLock(owner->streamMutex);
	++owner->numStreamsAlive;
	}
	
	/* Read the format versions of the color and depth streams: */
	for(int i=0;i<2;++i)
		streamFormatVersions[i]=source.read<Misc::UInt32>();
	
	/* Check if the depth stream has per-pixel depth correction coefficients: */
	if(streamFormatVersions[1]>=4)
		{
		/* Read new B-spline based depth correction parameters: */
		depthCorrection=new DepthCorrection(source);
		}
	else
		{
		if(streamFormatVersions[1]>=2&&source.read<char>()!=0)
			{
			/* Skip the depth correction buffer: */
			Misc::SInt32 size[2];
			source.read<Misc::SInt32>(size,2);
			source.skip<Misc::Float32>(size[1]*size[0]*2);
			}
		
		/* Create a dummy depth correction object: */
		int numSegments[2]={1,1};
		depthCorrection=new DepthCorrection(0,numSegments);
		}
	
	/* Check if the depth stream uses lossy compression: */
	bool depthIsLossy=streamFormatVersions[1]>=3&&source.read<Misc::UInt8>()!=0;
	
	/* Read the intrinsic and extrinsic camera parameters from the source: */
	ips.colorProjection=Misc::Marshaller<IntrinsicParameters::PTransform>::read(source);
	ips.depthProjection=Misc::Marshaller<IntrinsicParameters::PTransform>::read(source);
	eps=Misc::Marshaller<ExtrinsicParameters>::read(source);
	
	/* Create the frame readers: */
	owner->colorFrameReaders[index]=new ColorFrameReader(source);
	if(depthIsLossy)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		owner->depthFrameReaders[index]=new LossyDepthFrameReader(source);
		#else
		Misc::throwStdErr("Kinect::MultiplexedFrameSource::Stream::Stream: Lossy depth compression not supported due to lack of Theora library");
		#endif
		}
	else
		owner->depthFrameReaders[index]=new DepthFrameReader(source);
	}

MultiplexedFrameSource::Stream::~Stream(void)
	{
	{
	Threads::Spinlock::Lock streamingLock(streamingMutex);
	streaming=false;
	
	/* Delete any old streaming callbacks: */
	delete colorStreamingCallback;
	delete depthStreamingCallback;
	}
	
	/* Delete the depth correction object: */
	delete depthCorrection;
	
	/* Remove this stream from the owner's stream array: */
	bool lastOneOut;
	{
	Threads::Mutex::Lock streamLock(owner->streamMutex);
	owner->streams[index]=0;
	--owner->numStreamsAlive;
	lastOneOut=owner->numStreamsAlive==0;
	}
	
	/* Destroy the owner if this was the last stream to die: */
	if(lastOneOut)
		delete owner;
	}

FrameSource::DepthCorrection* MultiplexedFrameSource::Stream::getDepthCorrectionParameters(void)
	{
	/* Clone and return the depth correction object: */
	return new DepthCorrection(*depthCorrection);
	}

FrameSource::IntrinsicParameters MultiplexedFrameSource::Stream::getIntrinsicParameters(void)
	{
	return ips;
	}

FrameSource::ExtrinsicParameters MultiplexedFrameSource::Stream::getExtrinsicParameters(void)
	{
	return eps;
	}

const unsigned int* MultiplexedFrameSource::Stream::getActualFrameSize(int sensor) const
	{
	switch(sensor)
		{
		case COLOR:
			return owner->colorFrameReaders[index]->getSize();
			break;
		
		case DEPTH:
			return owner->depthFrameReaders[index]->getSize();
			break;
		
		default:
			return 0;
		}
	}

void MultiplexedFrameSource::Stream::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	Threads::Spinlock::Lock streamingLock(streamingMutex);
	streaming=true;
	
	/* Delete any old streaming callbacks: */
	delete colorStreamingCallback;
	delete depthStreamingCallback;
	
	/* Install the new streaming callbacks: */
	colorStreamingCallback=newColorStreamingCallback;
	depthStreamingCallback=newDepthStreamingCallback;
	}

void MultiplexedFrameSource::Stream::stopStreaming(void)
	{
	Threads::Spinlock::Lock streamingLock(streamingMutex);
	streaming=false;
	
	/* Delete any old streaming callbacks: */
	delete colorStreamingCallback;
	colorStreamingCallback=0;
	delete depthStreamingCallback;
	depthStreamingCallback=0;
	}

/***************************************
Methods of class MultiplexedFrameSource:
***************************************/

void* MultiplexedFrameSource::receivingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Initialize the demultiplexer state: */
	unsigned int currentMetaFrameIndex=0; // Index of the meta frame currently being received from the server
	unsigned int numMissingColorFrames=numStreams; // Number of color frames still missing from the current meta frame
	unsigned int numMissingDepthFrames=numStreams; // Number of depth frames still missing from the current meta frame
	
	try
		{
		while(true)
			{
			/* Receive the next frame's identifier: */
			unsigned int metaFrameIndex=pipe->read<Misc::UInt32>();
			unsigned int frameId=pipe->read<Misc::UInt32>();
			
			/* Check for the beginning of a new meta frame: */
			if(currentMetaFrameIndex!=metaFrameIndex)
				{
				/* If the previous metaframe was complete, stream all current frames to their respective listeners: */
				if(numMissingColorFrames==0&&numMissingDepthFrames==0)
					{
					Threads::Mutex::Lock streamLock(streamMutex);
					
					for(unsigned int i=0;i<numStreams;++i)
						{
						if(streams[i]!=0)
							{
							Threads::Spinlock::Lock streamingLock(streams[i]->streamingMutex);
							if(streams[i]->streaming)
								{
								/* Push the streamer's frames: */
								(*streams[i]->colorStreamingCallback)(frames[i*2+0]);
								(*streams[i]->depthStreamingCallback)(frames[i*2+1]);
								}
							}
						}
					}
				
				/* Start the next metaframe: */
				currentMetaFrameIndex=metaFrameIndex;
				numMissingColorFrames=numStreams;
				numMissingDepthFrames=numStreams;
				}
			
			/* Read the new frame: */
			unsigned int streamIndex=frameId>>1;
			if(frameId&0x1U)
				{
				/* Receive a depth frame: */
				frames[frameId]=depthFrameReaders[streamIndex]->readNextFrame();
				--numMissingDepthFrames;
				}
			else
				{
				/* Receive a color frame: */
				frames[frameId]=colorFrameReaders[streamIndex]->readNextFrame();
				--numMissingColorFrames;
				}
			}
		}
	catch(std::runtime_error err)
		{
		/* Ignore the error and terminate the thread */
		}
	
	return 0;
	}

MultiplexedFrameSource::MultiplexedFrameSource(Comm::PipePtr sPipe)
	:pipe(sPipe),
	 numStreams(0),
	 colorFrameReaders(0),
	 depthFrameReaders(0),
	 frames(0),
	 numStreamsAlive(0),
	 streams(0)
	{
	/* Check if the pipe is a cluster-forwarded pipe: */
	Cluster::ClusterPipe* cPipe=dynamic_cast<Cluster::ClusterPipe*>(pipe.getPointer());
	if(cPipe!=0)
		{
		/* Decouple the write direction of the pipe: */
		cPipe->couple(true,false);
		}
	
	/* Determine server's endianness: */
	Misc::UInt32 endiannessFlag=pipe->read<Misc::UInt32>();
	if(endiannessFlag==0x78563412U)
		pipe->setSwapOnRead(true);
	else if(endiannessFlag!=0x12345678U)
		Misc::throwStdErr("MultiplexedFrameSource::MultiplexedFrameSource: Server has unrecognized endianness");
	
	/* Initialize all streams: */
	numStreams=pipe->read<Misc::UInt32>();
	colorFrameReaders=new FrameReader*[numStreams];
	depthFrameReaders=new FrameReader*[numStreams];
	streams=new Stream*[numStreams];
	for(unsigned int i=0;i<numStreams;++i)
		{
		colorFrameReaders[i]=0;
		depthFrameReaders[i]=0;
		streams[i]=0;
		}
	bool allStreamsOk=true;
	for(unsigned int i=0;i<numStreams;++i)
		{
		try
			{
			streams[i]=new Stream(this,i,*pipe);
			}
		catch(std::runtime_error err)
			{
			/* Signal an error to clean up later: */
			allStreamsOk=false;
			}
		}
	
	/* Check if all streams were initialized correctly: */
	if(!allStreamsOk)
		{
		/* Close all streams that were initialized OK: */
		for(unsigned int i=0;i<numStreams;++i)
			{
			delete colorFrameReaders[i];
			delete depthFrameReaders[i];
			delete streams[i];
			}
		
		/* Clean up and signal an error: */
		delete[] colorFrameReaders;
		delete[] depthFrameReaders;
		delete[] streams;
		Misc::throwStdErr("MultiplexedFrameSource::MultiplexedFrameSource: Error while initializing component streams");
		}
	
	/* Allocate the frame buffer array: */
	frames=new FrameBuffer[numStreams*2];
	
	/* Start the demultiplexer thread: */
	receivingThread.start(this,&MultiplexedFrameSource::receivingThreadMethod);
	}

MultiplexedFrameSource::~MultiplexedFrameSource(void)
	{
	/* Signal the receiving thread to shut down: */
	receivingThread.cancel();
	receivingThread.join();
	
	/* Delete all streams: */
	for(unsigned int i=0;i<numStreams;++i)
		{
		delete colorFrameReaders[i];
		delete depthFrameReaders[i];
		delete streams[i]; // None of these can actually be !=0, but whatever
		}
	delete[] colorFrameReaders;
	delete[] depthFrameReaders;
	delete[] streams;
	
	/* Delete the frame buffers: */
	delete[] frames;
	
	/* Say goodbye to the server: */
	try
		{
		/* Send the disconnect request and shut down the server pipe: */
		pipe->write<Misc::UInt32>(0);
		pipe->flush();
		}
	catch(...)
		{
		/* Ignore the error; we were just being polite anyway */
		}
	}

MultiplexedFrameSource* MultiplexedFrameSource::create(Comm::PipePtr sPipe)
	{
	return new MultiplexedFrameSource(sPipe);
	}

}
