/***********************************************************************
CameraV2 - Class representing a Kinect v2 camera.
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

#ifndef KINECT_CAMERAV2_INCLUDED
#define KINECT_CAMERAV2_INCLUDED

#include <stddef.h>
#include <USB/Device.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
namespace USB {
class TransferPool;
}
namespace Kinect {
class KinectV2CommandDispatcher;
class KinectV2JpegStreamReader;
class KinectV2DepthStreamReader;
}

namespace Kinect {

class CameraV2:public DirectFrameSource
	{
	friend class KinectV2JpegStreamReader;
	friend class KinectV2DepthStreamReader;
	
	/* Elements: */
	private:
	USB::Device device; // The USB device representing this Kinect v2 camera
	std::string serialNumber; // The unique serial number of this camera
	KinectV2CommandDispatcher* commandDispatcher;
	KinectV2JpegStreamReader* colorStreamReader;
	USB::TransferPool* colorTransfers;
	KinectV2DepthStreamReader* depthStreamReader;
	USB::TransferPool* depthTransfers;
	unsigned int frameSizes[2][2]; // Width and height of the color and depth frames, respectively
	
	/* Private methods: */
	void initialize(void); // Initializes the Kinect v2 camera; called from constructors
	
	/* Constructors and destructors: */
	public:
	static size_t getNumDevices(void); // Returns the number of Kinect v2 cameras on the local host's bus
	CameraV2(libusb_device* sDevice); // Creates a Kinect v2 camera wrapper around the given USB device, which is assumed to be a Kinect v2 camera
	CameraV2(size_t index =0); // Opens the index-th Kinect v2 camera device
	CameraV2(const char* serialNumber); // Opens the Kinect v2 camera with the given serial number
	virtual ~CameraV2(void); // Destroys the camera
	
	/* Methods from FrameSource: */
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual const unsigned int* getActualFrameSize(int sensor) const;
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
	virtual void stopStreaming(void);
	
	/* Methods from DirectFrameSource: */
	virtual std::string getSerialNumber(void);
	};

}

#endif
