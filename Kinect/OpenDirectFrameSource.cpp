/***********************************************************************
OpenDirectFrameSource - Helper functions to open a 3D camera by index or
serial number without having to know its type.
Copyright (c) 2016-2018 Oliver Kreylos

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

#include <Kinect/OpenDirectFrameSource.h>

#include <string.h>
#include <Misc/ThrowStdErr.h>
#include <USB/Context.h>
#include <Kinect/Config.h>
#include <Kinect/Camera.h>
#include <Kinect/CameraV2.h>
#include <Kinect/CameraRealSense.h>

namespace Kinect {

/**************************
Namespace-global functions:
**************************/

DirectFrameSource* openDirectFrameSource(unsigned int index)
	{
	/* Acquire a USB context to prevent it from being created/destroyed repeatedly: */
	USB::ContextPtr usbContext=USB::Context::acquireContext();
	
	/* Search for the given index among all supported 3D camera types: */
	size_t searchIndex=index;
	
	/* Get the number of first-generation Kinect cameras: */
	size_t numKinects=Camera::getNumDevices();
	if(searchIndex<numKinects)
		{
		/* Return the first-generation Kinect camera of the given index: */
		return new Camera(searchIndex);
		}
	searchIndex-=numKinects;
	
	/* Get the number of second-generation Kinect cameras: */
	size_t numKinectV2s=CameraV2::getNumDevices();
	if(searchIndex<numKinectV2s)
		{
		/* Return the second-generation Kinect camera of the given index: */
		return new CameraV2(searchIndex);
		}
	searchIndex-=numKinectV2s;
	
	/* Get the number of Intel RealSense cameras: */
	size_t numRealSenses=CameraRealSense::getNumDevices();
	if(searchIndex<numRealSenses)
		{
		/* Return the Intel RealSense camera of the given index: */
		return new CameraRealSense(searchIndex);
		}
	searchIndex-=numRealSenses;
	
	/* Not enough cameras: */
	Misc::throwStdErr("Kinect::openDirectFrameSource: Fewer than %u 3D cameras connected to local host",index+1);
	return 0; // Never reached
	}

DirectFrameSource* openDirectFrameSource(const char* serialNumber)
	{
	/* Determine the type of camera from the given serial number: */
	const char* snPtr;
	for(snPtr=serialNumber;*snPtr!='\0'&&*snPtr!='-';++snPtr)
		;
	if(*snPtr=='\0')
		{
		/* Look for a first-generation Kinect camera: */
		return new Camera(serialNumber);
		}
	else
		{
		if(snPtr-serialNumber==2&&strncasecmp(serialNumber,"V2",2)==0)
			{
			/* Look for a second-generation Kinect camera: */
			return new CameraV2(snPtr+1);
			}
		else if(snPtr-serialNumber==2&&strncasecmp(serialNumber,"RS",2)==0)
			{
			/* Look for an Intel RealSense camera: */
			return new CameraRealSense(snPtr+1);
			}
		else
			Misc::throwStdErr("Kinect::openDirectFrameSource: Unsupported 3D camera type \"%s\"",std::string(serialNumber,snPtr).c_str());
		}
	
	return 0; // Never reached
	}

}
