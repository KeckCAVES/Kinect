/***********************************************************************
KinectCamera - Wrapper class to represent the color and depth camera
interface aspects of the Kinect sensor.
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
***********************************************************************/

#ifndef KINECTCAMERA_INCLUDED
#define KINECTCAMERA_INCLUDED

#include <Threads/MutexCond.h>
#include <Threads/Thread.h>

#include "USBDevice.h"

/* Forward declarations: */
struct libusb_transfer;
namespace Misc {
class ValueSource;
template <class ParameterParam>
class FunctionCall;
}
class USBContext;
class FrameBuffer;

class KinectCamera:public USBDevice
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<const FrameBuffer&> StreamingCallback; // Function call type for streaming color or depth image capture callback
	
	private:
	struct StreamingState // Structure containing necessary state to stream color or depth frames from the respective camera
		{
		/* Elements: */
		public:
		unsigned int packetFlagBase; // Base value for stream's packet header flags
		int packetSize; // Size of isochronous packets in bytes
		int numPackets; // Number of packets per transfer
		int numTransfers; // Size of transfer ring buffer to handle delays or transfer bursts
		unsigned char** transferBuffers; // Array of transfer buffers
		libusb_transfer** transfers; // Array of transfer structures
		volatile int numActiveTransfers; // Number of currently active transfers to properly handle cancellation
		
		int frameSize[2]; // Size of streamed frames in pixels
		size_t rawFrameSize; // Total size of encoded frames received from the camera
		unsigned char* rawFrameBuffer; // Double buffer to assemble an encoded frame during streaming and hold a previous frame for processing
		int activeBuffer; // Index of buffer half currently receiving frame data from the camera
		unsigned char* writePtr; // Current write position in active buffer half
		size_t bufferSpace; // Number of bytes still to be written into active buffer half
		
		Threads::MutexCond frameReadyCond; // Condition variable to signal completion of a new frame to the decoding thread
		unsigned char* volatile readyFrame; // Pointer to buffer half containing the completed frame
		Threads::Thread decodingThread; // Thread to decode raw frames into user-visible format
		
		StreamingCallback* streamingCallback; // Callback to be called when a new frame has been decoded
		
		/* Constructors and destructors: */
		public:
		StreamingState(libusb_device_handle* handle,unsigned int endpoint,int sPacketFlagBase,int sPacketSize,const int sFrameSize[2],size_t sRawFrameSize,StreamingCallback* sStreamingCallback); // Prepares a streaming state for streaming
		~StreamingState(void); // Cleanly stops streaming and destroys the streaming state
		
		/* Methods: */
		static void transferCallback(libusb_transfer* transfer); // Callback called when a USB transfer completes or is cancelled
		};
	
	/* Elements: */
	public:
	static const unsigned short invalidDepth=0x07ffU; // The depth value indicating an invalid (or removed) pixel
	private:
	unsigned short messageSequenceNumber; // Incrementing sequence number for command messages to the camera
	StreamingState* colorStreamer; // Streaming state for color frames
	StreamingState* depthStreamer; // Streaming state for depth frames
	unsigned int numBackgroundFrames; // Number of background frames left to capture
	unsigned short* backgroundFrame; // Frame containing minimal depth values for a captured background
	bool removeBackground; // Flag whether to remove background information during frame processing
	
	/* Private methods: */
	void chant(Misc::ValueSource& chantSource); // Chants part of an incantation at the device and expects a proper reply
	size_t sendMessage(unsigned short messageType,const unsigned short* messageData,size_t messageSize,void* replyBuffer,size_t replyBufferSize); // Sends a general message to the camera device; returns reply size in bytes
	bool sendCommand(unsigned short command,unsigned short value); // Sends a command message to the camera device; returns true if command was processed properly
	void* colorDecodingThreadMethod(void); // The color decoding thread method
	void* depthDecodingThreadMethod(void); // The depth decoding thread method
	
	/* Constructors and destructors: */
	public:
	KinectCamera(USBContext& usbContext,size_t index =0); // Opens the index-th Kinect camera device on the given USB context
	~KinectCamera(void); // Destroys the camera
	
	/* Methods: */
	void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback); // Installs the given streaming callback and starts receiving color and depth data from the camera
	void captureBackground(unsigned int newNumBackgroundFrames); // Captures the given number of frames to create a background removal buffer
	void setRemoveBackground(bool newRemoveBackground); // Enables or disables background removal
	bool getRemoveBackground(void) const // Returns the current background removal flag
		{
		return removeBackground;
		}
	void stopStreaming(void); // Stops streaming; blocks until all pending transfers have either completed or been cancelled
	};

#endif
