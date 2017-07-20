/***********************************************************************
KinectV2CommandDispatcher - Class to exchange commands and command
replies with a Kinect v2 device via USB bulk transfers.
Copyright (c) 2014-2017 Oliver Kreylos

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

#ifndef KINECT_INTERNAL_KINECTV2COMMANDDISPATCHER_INCLUDED
#define KINECT_INTERNAL_KINECTV2COMMANDDISPATCHER_INCLUDED

#include <stddef.h>
#include <Misc/SizedTypes.h>

/* Forward declarations: */
namespace USB {
class Device;
}
namespace Kinect {
class KinectV2DepthStreamReader;
}

namespace Kinect {

class KinectV2CommandDispatcher
	{
	/* Embedded classes: */
	private:
	struct Command // Structure for commands sent to the Kinect2
		{
		/* Elements: */
		public:
		static const Misc::UInt32 commandMagic=0x06022009U;
		Misc::UInt32 magic; // Magic number identifying Kinect commands
		Misc::UInt32 seq; // Sequence number to pair commands and responses
		Misc::UInt32 maxResponseSize; // Maximum size of Kinect's response
		Misc::UInt32 command; // Command identifier
		Misc::UInt32 res0; // Reserved data block, always 0
		Misc::UInt32 parameters[4]; // Up to four command parameters
		
		/* Constructors and destructors: */
		Command(unsigned int sSeq,unsigned int sMaxResponseSize,unsigned int sCommand)
			:magic(commandMagic),seq(sSeq),maxResponseSize(sMaxResponseSize),
			 command(sCommand),res0(0)
			{
			}
		};
	
	struct Reply // Structure for replies received from the Kinect2
		{
		/* Elements: */
		public:
		static const Misc::UInt32 replyCompleteMagic=0x0a6fe000U;
		Misc::UInt32 magic; // Magic number identifying Kinect command replies
		Misc::UInt32 seq; // Sequence number matching the command
		Misc::UInt32 res0; // Reserved data block
		Misc::UInt32 res1; // Reserved data block
		};
	
	public:
	struct DepthCameraParams // Structure to retrieve depth camera projection and lens distortion correction parameters
		{
		/* Elements: */
		public:
		float sx,cx; // Projection scale and center in x
		float sy,cy; // Projection scale and center in y
		float k1,k2,k3; // Lens distortion correction kappa parameters
		float p1,p2; // Lens distortion correction rho parameters
		};
	
	struct ColorCameraParams // Structure to retrieve color camera projection and lens distortion correction parameters
		{
		/* Elements: */
		public:
		float sx,cx; // Projection scale and center in x
		float sy,cy; // Projection scale and center in y
		float shiftM,shiftD; // Multiplier and divisor for lateral shift between depth and color cameras
		float pxx3y0,pxx2y1,pxx1y2,pxx0y3,pxx2y0,pxx1y1,pxx0y2,pxx1y0,pxx0y1,pxx0y0; // Coefficients of polynomial mapping depth image pixels to color image pixels in x
		float pyx3y0,pyx2y1,pyx1y2,pyx0y3,pyx2y0,pyx1y1,pyx0y2,pyx1y0,pyx0y1,pyx0y0; // Coefficients of polynomial mapping depth image pixels to color image pixels in y
		};
	
	/* Elements: */
	private:
	USB::Device& device; // USB device representing the Kinect2
	Misc::UInt32 nextSeq; // Sequence number for the next command sent to the Kinect2
	size_t replyBufferSize; // Allocated size of reply buffer
	Misc::UInt8* replyBuffer; // Currently allocated reply buffer
	size_t replySize; // Actual size of reply currently in reply buffer
	DepthCameraParams depthCameraParams; // Depth camera parameters retrieved during initialization
	ColorCameraParams colorCameraParams; // Color camera parameters retrieved during initialization
	
	/* Private methods: */
	size_t execute(const Command& command,unsigned int numParameters); // Executes the given command
	
	/* Constructors and destructors: */
	public:
	KinectV2CommandDispatcher(USB::Device& sDevice); // Creates a command dispatcher for the Kinect2 represented by the given USB device
	private:
	KinectV2CommandDispatcher(const KinectV2CommandDispatcher& source); // Prohibit copy constructor
	KinectV2CommandDispatcher& operator=(const KinectV2CommandDispatcher& source); // Prohibit assignment operator
	public:
	~KinectV2CommandDispatcher(void); // Destroys the dispatcher
	
	/* Low-level communication methods: */
	USB::Device& getDevice(void) // Returns the USB device representing the Kinect2
		{
		return device;
		}
	size_t execute(unsigned int command,unsigned int maxReplySize); // Executes a command with zero parameters and returns size of reply
	size_t execute(unsigned int command,unsigned int parameter0,unsigned int maxReplySize); // Executes a command with one parameter and returns size of reply
	size_t execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int maxReplySize); // Executes a command with two parameters and returns size of reply
	size_t execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int parameter2,unsigned int maxReplySize); // Executes a command with three parameters and returns size of reply
	size_t execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int parameter2,unsigned int parameter3,unsigned int maxReplySize); // Executes a command with four parameters and returns size of reply
	const Misc::UInt8* getReplyBuffer(void) const // Returns the reply buffer
		{
		return replyBuffer;
		}
	size_t getReplySize(void) const // Returns the amount of data in the reply buffer
		{
		return replySize;
		}
	void saveReply(const char* fileName) const; // Saves the current reply to a binary file of the given name
	Misc::UInt8* detachReply(void); // Detaches and returns the current reply buffer; buffer must be delete[]ed by caller
	
	/* Methods implementing the Kinect v2 USB command protocol: */
	void initInterfaces(void);
	void downloadTables(KinectV2DepthStreamReader* depthStreamReader);
	const DepthCameraParams& getDepthCameraParams(void) const
		{
		return depthCameraParams;
		}
	const ColorCameraParams& getColorCameraParams(void) const
		{
		return colorCameraParams;
		}
	void startSensors(void);
	void stopSensors(void);
	};

}

#endif
