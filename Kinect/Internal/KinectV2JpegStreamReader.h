/***********************************************************************
KinectV2JpegStreamReader - Class to read JPEG-compressed RGB images
asynchronously from a stream of USB transfer buffers.
Copyright (c) 2014-2015 Oliver Kreylos

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

#ifndef KINECT_INTERNAL_KINECTV2JPEGSTREAMREADER_INCLUDED
#define KINECT_INTERNAL_KINECTV2JPEGSTREAMREADER_INCLUDED

#include <stddef.h>
#include <stdio.h>
#include <jpeglib.h>
#include <Threads/Thread.h>
#include <Threads/MutexCond.h>
#include <USB/TransferPool.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace Kinect {
class FrameBuffer;
class CameraV2;
}

namespace Kinect {

class KinectV2JpegStreamReader
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<const FrameBuffer&> ImageReadyCallback; // Type for functions called when a new color image has been decompressed
	
	/* Elements: */
	private:
	CameraV2& camera; // Kinect v2 device with which this JPEG stream reader is associated
	Threads::MutexCond inQueueCond; // Condition variable to notify the decompression thread of new data
	USB::TransferPool::TransferQueue inQueue; // Queue of incoming USB transfer buffers
	jpeg_error_mgr errorManager; // Manager to handle JPEG decompression errors
	jpeg_source_mgr sourceManager; // Manager to handle streaming compressed data to the JPEG decompressor
	USB::TransferPool* transferPool; // The transfer pool from which transfer buffers are received
	USB::TransferPool::Transfer* currentTransfer; // Transfer buffer currently read by the JPEG decompressor
	jpeg_decompress_struct decompressor; // The JPEG decompressor
	Threads::Thread decompressionThread; // A background thread running the JPEG decompressor
	int imageHeight; // Height of last decompressed image
	FrameSource::ColorPixel** imageRowPointers; // Array of pointers to image rows to flip image during decompression
	size_t frameSize; // Total compressed image size for the current image
	bool error; // Flag to remember errors while decompressing the current image
	ImageReadyCallback* imageReadyCallback; // Function called whenever a new image has been decompressed
	
	/* Private methods: */
	void getNextTransfer(void); // Grabs the next transfer buffer from the input queue and sets up the JPEG decompressor's input buffer
	static void errorExitFunction(j_common_ptr cinfo); // JPEG error handler
	static void initSourceFunction(j_decompress_ptr cinfo);
	static boolean fillInputBufferFunction(j_decompress_ptr cinfo);
	static void skipInputDataFunction(j_decompress_ptr cinfo,long count);
	static void termSourceFunction(j_decompress_ptr cinfo);
	void* decompressionThreadMethod(void); // Method for the JPEG decompression thread
	
	/* Constructors and destructors: */
	public:
	KinectV2JpegStreamReader(CameraV2& sCamera); // Creates a stream reader
	~KinectV2JpegStreamReader(void); // Destroys the stream reader
	
	/* Methods: */
	void postTransfer(USB::TransferPool::Transfer* newTransfer,USB::TransferPool* newTransferPool) // Appends the given transfer buffer to the input queue
		{
		#if 0
		/* Bail out if the new transfer pool doesn't match the established transfer pool: */
		if(newTransferPool!=transferPool)
			return;
		#endif
		
		Threads::MutexCond::Lock inQueueLock(inQueueCond);
		
		/* Remember if the queue is currently empty, and append the new transfer: */
		bool empty=inQueue.empty();
		inQueue.push_back(newTransfer);
		
		/* If the queue was previously empty, the processing thread might be waiting for data: */
		if(empty)
			inQueueCond.signal();
		}
	USB::TransferPool::UserTransferCallback* startStreaming(USB::TransferPool* newTransferPool,ImageReadyCallback* newImageReadyCallback); // Starts the decompression thread and registers the given callback; returns a callback set up to receive USB transfer buffers
	void stopStreaming(void); // Stops background decompression
	};

}

#endif

