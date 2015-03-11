/***********************************************************************
Camera - Wrapper class to represent the color and depth camera interface
aspects of the Kinect sensor.
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

#ifndef KINECT_CAMERA_INCLUDED
#define KINECT_CAMERA_INCLUDED

/* Set to 1 for frame stream analysis: */
#define KINECT_CAMERA_DUMP_HEADERS 0

#include <string>
#include <Misc/SizedTypes.h>
#include <Misc/Timer.h>
#include <Threads/MutexCond.h>
#include <Threads/Thread.h>
#include <USB/Device.h>
#if KINECT_CAMERA_DUMP_HEADERS
#include <IO/File.h>
#endif
#include <Kinect/FrameSource.h>

/* Forward declarations: */
struct libusb_device;
struct libusb_transfer;
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
namespace USB {
class Context;
class DeviceList;
}
namespace IO {
class File;
}

namespace Kinect {

class Camera:public FrameSource
	{
	/* Embedded classes: */
	public:
	enum FrameSize // Enumerated type to select color and depth frame sizes
		{
		FS_640_480=0, // 640x480 frames
		FS_1280_1024 // 1280x1024 frames, only valid for color camera
		};
	enum FrameRate // Enumerated type to select frame rates for the color and depth cameras
		{
		FR_15_HZ=0, // 15 Hz
		FR_30_HZ // 30 Hz, only possible for 640x480 pixel frames
		};
	typedef Misc::FunctionCall<Camera&> BackgroundCaptureCallback; // Function call type for completion of background capture callback
	
	struct CalibrationParameters // Structure to hold factory calibration parameters read from Kinect camera's non-volatile RAM
		{
		/* Elements: */
		public:
		/* First parameter subsection: registration parameters */
		Misc::SInt32 dxCenter;
		Misc::SInt32 ax,bx,cx,dx;
		Misc::SInt32 dxStart;
		Misc::SInt32 ay,by,cy,dy;
		Misc::SInt32 dyStart;
		Misc::SInt32 dxBetaStart,dyBetaStart;
		Misc::SInt32 rolloutBlank,rolloutSize;
		Misc::SInt32 dxBetaInc,dyBetaInc;
		Misc::SInt32 dxdxStart,dxdyStart,dydxStart,dydyStart;
		Misc::SInt32 dxdxdxStart,dydxdxStart,dxdxdyStart,dydxdyStart;
		Misc::SInt32 backComp1;
		Misc::SInt32 dydydxStart;
		Misc::SInt32 backComp2;
		Misc::SInt32 dydydyStart;
		
		/* Second parameter subsection: padding parameters */
		Misc::UInt16 startLines;
		Misc::UInt16 endLines;
		Misc::UInt16 croppingLines;
		
		/* Third parameter subsection: constant shift */
		Misc::UInt16 constantShift;
		
		/* Fourth parameter subsection: zero-plane parameters */
		Misc::Float32 dcmosEmitterDist;
		Misc::Float32 dcmosRcmosDist;
		Misc::Float32 referenceDistance;
		Misc::Float32 referencePixelSize;
		
		/* Methods: */
		void read(int subsection,IO::File& file); // Reads calibration parameter subsection from file
		void read(IO::File& file); // Reads all calibration parameters from file
		void write(IO::File& file) const; // Writes all calibration parameters to file
		};
	
	struct CameraParameters // Structure containing the imaging parameters of the color camera
		{
		/* Elements: */
		public:
		unsigned short exposure; // Camera exposure time, 654==33ms, 0==500ms
		unsigned short sharpening; // Bits 0-2 are sharpening factor from 0% to 200%; bit 3 enables automatic sharpening reduction
		unsigned short operatingMode; // Bit field defining the camera's operating mode
		};
	
	private:
	typedef Misc::UInt16 USBWord; // Type for words of data exchanged at the USB library API
	
	struct StreamingState // Structure containing necessary state to stream color or depth frames from the respective camera
		{
		/* Elements: */
		public:
		Misc::Timer& frameTimer; // Reference to camera's frame timer
		double& frameTimerOffset; // Time offset to apply to camera's timer
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
		double activeFrameTimeStamp; // Time stamp for the frame currently being received
		unsigned char* writePtr; // Current write position in active buffer half
		size_t bufferSpace; // Number of bytes still to be written into active buffer half
		
		Threads::MutexCond frameReadyCond; // Condition variable to signal completion of a new frame to the decoding thread
		bool readyFrameIntact; // Flag whether the completed frame was received intact
		unsigned char* volatile readyFrame; // Pointer to buffer half containing the completed frame
		double readyFrameTimeStamp; // Time stamp of completed frame
		volatile bool cancelDecoding; // Flag to cancel the deocding thread
		Threads::Thread decodingThread; // Thread to decode raw frames into user-visible format
		
		StreamingCallback* streamingCallback; // Callback to be called when a new frame has been decoded
		
		#if KINECT_CAMERA_DUMP_HEADERS
		IO::FilePtr headerFile;
		#endif
		
		/* Constructors and destructors: */
		public:
		StreamingState(libusb_device_handle* handle,unsigned int endpoint,Misc::Timer& sFrameTimer,double& sFrameTimerOffset,int sPacketFlagBase,int sPacketSize,const unsigned int sFrameSize[2],size_t sRawFrameSize,StreamingCallback* sStreamingCallback); // Prepares a streaming state for streaming
		~StreamingState(void); // Cleanly stops streaming and destroys the streaming state
		
		/* Methods: */
		static void transferCallback(libusb_transfer* transfer); // Callback called when a USB transfer completes or is cancelled
		};
	
	/* Elements: */
	private:
	USB::Device device; // The USB device representing this Kinect camera
	std::string serialNumber; // This Kinect camera's serial number
	size_t calibrationParameterReplySizes[4]; // USB reply sizes when querying the four subsets of factory calibration parameters
	bool needAltInterface; // Flag whether the camera device needs to be switched to an alternate interface
	FrameSize frameSizes[2]; // Selected frame sizes for the color and depth cameras
	FrameRate frameRates[2]; // Selected frame rates for the color and depth cameras
	USBWord messageSequenceNumber; // Incrementing sequence number for command messages to the camera
	Misc::Timer frameTimer; // Free-running timer to time-stamp depth and color frames
	double frameTimerOffset; // Time offset to apply to cameras' timers
	bool compressDepthFrames; // Flag whether to request RLE/differential compressed depth frames from the depth camera
	bool smoothDepthFrames; // Flag whether to smooth depth frames inside the Kinect's processor, whatever that means
	StreamingState* streamers[2]; // Streaming states for color and depth frames
	unsigned int numBackgroundFrames; // Number of background frames left to capture
	DepthPixel* backgroundFrame; // Frame containing minimal depth values for a captured background
	BackgroundCaptureCallback* backgroundCaptureCallback; // Function to call upon completion of background capture
	bool removeBackground; // Flag whether to remove background information during frame processing
	Misc::SInt16 backgroundRemovalFuzz; // Fuzz value for background removal (positive values: more aggressive removal)
	
	#if KINECT_CAMERA_DUMP_HEADERS
	IO::FilePtr headerFile;
	#endif
	
	/* Private methods: */
	size_t sendMessage(USBWord messageType,const USBWord* messageData,size_t messageSize,void* replyBuffer,size_t replyBufferSize); // Sends a general message to the camera device; returns reply size in bytes
	bool sendCommand(USBWord command,USBWord value); // Sends a command message to the camera device; returns true if command was processed properly
	USBWord readRegister(USBWord address); // Reads a camera register and returns its value
	void writeRegister(USBWord address,USBWord value); // Sets a camera register to the given value
	void* colorDecodingThreadMethod(void); // The color decoding thread method
	void* depthDecodingThreadMethod(void); // The depth decoding thread method
	void* compressedDepthDecodingThreadMethod(void); // The depth decoding thread method for RLE/differential-compressed frames
	void initialize(USB::Context& usbContext,USB::DeviceList* deviceList =0); // Initializes the Kinect camera; called from constructors
	
	/* Constructors and destructors: */
	public:
	Camera(USB::Context& usbContext,libusb_device* sDevice); // Creates a Kinect camera wrapper around the given USB device, which is assumed to be a Kinect camera
	Camera(USB::Context& usbContext,size_t index =0); // Opens the index-th Kinect camera device on the given USB context
	Camera(USB::Context& usbContext,const char* serialNumber); // Opens the Kinect camera with the given serial number on the given USB context
	virtual ~Camera(void); // Destroys the camera
	
	/* Methods from FrameSource: */
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual ExtrinsicParameters getExtrinsicParameters(void);
	virtual const unsigned int* getActualFrameSize(int sensor) const;
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
	virtual void stopStreaming(void);
	
	/* New methods: */
	void getCalibrationParameters(CalibrationParameters& calib); // Queries factory calibration parameters from Kinect's non-volatile RAM
	const std::string& getSerialNumber(void) const // Returns the camera's serial number
		{
		return serialNumber;
		}
	void setFrameSize(int camera,FrameSize newFrameSize); // Sets the frame size of the color or depth camera for the next streaming operation
	FrameSize getFrameSize(int camera) // Returns the selected frame size of the color or depth camera
		{
		return frameSizes[camera];
		}
	void setFrameRate(int camera,FrameRate newFrameRate); // Sets the frame rate of the color or depth camera for the next streaming operation
	FrameRate getFrameRate(int camera) const // Returns the selected frame rate of the color or depth camera
		{
		return frameRates[camera];
		}
	unsigned int getActualFrameRate(int camera) const; // Returns the selected frame rate of the color or depth camera in Hz
	void resetFrameTimer(double newFrameTimerOffset =0.0); // Resets the frame timer to zero
	void setCompressDepthFrames(bool newCompressDepthFrames); // Enables or disables depth frame compression for the next streaming operation
	void setSmoothDepthFrames(bool newSmoothDepthFrames); // Enables or disables depth frame smoothing for the next streaming operation
	void captureBackground(unsigned int newNumBackgroundFrames,bool replace,BackgroundCaptureCallback* newBackgroundCaptureCallback =0); // Captures the given number of frames to create a background removal buffer and calls optional callback upon completion
	bool loadDefaultBackground(void); // Loads the default background removal buffer for this camera; returns true if background was loaded
	void loadBackground(const char* fileNamePrefix); // Loads a background removal buffer from a file with the given prefix
	void loadBackground(IO::File& file); // Ditto, from already opened file
	void setMaxDepth(unsigned int newMaxDepth,bool replace =false); // Sets a depth value beyond which all pixels are considered background
	void saveBackground(const char* fileNamePrefix); // Saves the current background frame to a file with the given prefix
	void saveBackground(IO::File& file); // Ditto, into an already opened file
	void setRemoveBackground(bool newRemoveBackground); // Enables or disables background removal
	bool getRemoveBackground(void) const // Returns the current background removal flag
		{
		return removeBackground;
		}
	void setBackgroundRemovalFuzz(int newBackgroundRemovalFuzz); // Sets the fuzz value for background removal
	int getBackgroundRemovalFuzz(void) const // Returns the current background removal fuzz value
		{
		return backgroundRemovalFuzz;
		}
	
	/* Control methods for the color camera: */
	unsigned int getSharpening(void); // Returns the color camera's sharpening value
	void setSharpening(unsigned int newSharpening); // Sets the color camera's sharpening value
	};

}

#endif
