/***********************************************************************
KinectV2DepthStreamReader - Class to extract depth images from raw gated
IR images read from a stream of USB transfer buffers.
Copyright (c) 2015-2016 Oliver Kreylos

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

#ifndef KINECT_INTERNAL_KINECTV2DEPTHSTREAMREADER_INCLUDED
#define KINECT_INTERNAL_KINECTV2DEPTHSTREAMREADER_INCLUDED

#include <Misc/SizedTypes.h>
#include <Threads/Thread.h>
#include <Threads/MutexCond.h>
#include <IO/File.h>
#include <USB/TransferPool.h>
#include <Kinect/Internal/KinectV2CommandDispatcher.h>

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

class KinectV2DepthStreamReader
	{
	/* Embedded classes: */
	public:
	typedef Misc::SInt16 IRPixel; // Type for raw gated IR image pixels
	
	struct RawImage // Structure to pass raw range-gated IR images to an interested party
		{
		/* Elements: */
		public:
		const IRPixel* image; // Pointer to the image's first pixel
		unsigned int imageIndex; // Index of this IR image in [0, 10)
		};
	
	typedef Misc::FunctionCall<const RawImage&> RawImageReadyCallback; // Type for functions called when a new raw range-gated IR image is finished
	typedef Misc::FunctionCall<const FrameBuffer&> ImageReadyCallback; // Type for functions called when a new depth image has been decoded
	
	/* Elements: */
	private:
	CameraV2& camera; // Kinect v2 device with which this depth stream reader is associated
	USB::TransferPool* transferPool; // The transfer pool from which transfer buffers are received
	IRPixel* decompressTable; // Uncompression table to extend 11-bit IR pixels to 16 bit
	IRPixel* inputBufferBlock; // Block of memory to hold the 10 raw gated IR images comprising a depth frame
	IRPixel* inputBuffers[10]; // Pointer to the individual IR images in the memory block
	bool frameStart; // Flag to indicate the first USB transfer packet of a new depth frame
	double nextFrameTimeStamp; // Time stamp of the frame that is currently being received over USB
	unsigned int frameNumber; // Index of currently processed depth frame, as assigned by the camera
	unsigned int currentImage; // Index of the raw gated IR image that is currently being received
	unsigned int nextRow; // Next pixel row in the current frame to be received
	bool frameValid; // Flag to keep track of errors during frame processing
	double frameTimeStamp; // Time stamp for the frame that was just received over USB
	RawImageReadyCallback* rawImageReadyCallback; // Function called whenever a raw range-gated IR image has been decompressed
	float* trigonometryTables[3]; // Three arrays of coefficients to convert a range-gated IR image triple into a 2D phase vector
	float* arctanTable; // Table to calculate phase angles from orthogonal vectors
	float magnitudeFactors[3]; // Multiplication factors for IR pixel intensity for each image triplet
	Threads::Thread phaseThreads[3]; // Three threads to calculate phase vector image for each exposure in parallel
	Threads::MutexCond phaseThreadConds[3]; // Three condition variables to wake up the phase angle calculation threads
	float* phaseImages[3]; // Three phase images, containing (angle, squared magnitude) pairs
	double phaseFrameTimeStamp; // Time stamp of the frame currently processed by the phase calculation threads
	unsigned int phaseFrameNumbers[3]; // Index of phase image currently in the phase image buffers
	Threads::Thread depthThread; // Thread to convert a triplet of phase images into a depth image
	Threads::MutexCond depthThreadCond; // Condition variable to wake up the depth calculation thread
	float magThreshold1,magThreshold2; // Validity thresholds for each exposure's magnitude, and sum of magnitudes
	float confidenceSlope,confidenceOffset; // Slope and offset for dealiasing confidence check
	float minConfidence,maxConfidence; // Dealiasing confidence interval
	float* confidenceTable; // Tabulated confidence function
	float phaseOffset; // Constant offset to dealiased phase values
	float unambiguousDistance;
	float* xTable;
	float* zTable;
	float* depthImage; // Final depth image
	float filterDistanceThreshold; // Threshold value for edge-retaining low-pass filter
	unsigned int depthFrameNumber; // Index of depth image currently in the buffer
	float zMin,zMax; // Z value range for quantization
	unsigned int dMax; // Maximum integer depth value
	float A,B; // Z-to-depth conversion formula coefficients
	ImageReadyCallback* imageReadyCallback; // Function called whenever a new image has been decompressed
	
	/* Private methods: */
	void* phaseThreadMethod(int exposure); // Method implementing the phase vector calculation thread
	void* depthThreadMethod(void); // Method implementing the depth calculation thread
	
	/* Constructors and destructors: */
	public:
	KinectV2DepthStreamReader(CameraV2& sCamera); // Creates an uninitialized stream reader
	~KinectV2DepthStreamReader(void); // Destroys the stream reader
	
	/* Methods: */
	void loadP0Tables(IO::FilePtr file); // Reads per-pixel and per-exposure phase offset tables from file
	void calcXZTables(const KinectV2CommandDispatcher::DepthCameraParams& depthCameraParams); // Calculates the X and Z depth calculation tables based on depth camera parameters
	void setDMax(unsigned int newDMax); // Sets the maximum integer depth value contained in returned depth images; current Kinect package expects 2047; maximum is 65535
	void setZRange(float newZMin,float newZMax); // Sets the range of linear z values for quantization
	float getA(void) const // Returns the first z-to-depth conversion formula coefficient
		{
		return A;
		}
	float getB(void) const // Returns the second z-to-depth conversion formula coefficient
		{
		return B;
		}
	void postTransfer(USB::TransferPool::Transfer* newTransfer,USB::TransferPool* newTransferPool); // Writes the given transfer into the raw input buffer
	void setRawImageReadyCallback(RawImageReadyCallback* newRawImageReadyCallback); // Installs a function to be called when a raw range-gated IR image is decompressed
	USB::TransferPool::UserTransferCallback* startStreaming(USB::TransferPool* newTransferPool,ImageReadyCallback* newImageReadyCallback); // Starts the decoding thread(s) and registers the given callback; returns a callback set up to receive USB transfer buffers
	void stopStreaming(void); // Stops background decoding
	};

}

#endif
