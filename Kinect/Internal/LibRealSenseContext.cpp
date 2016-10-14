/***********************************************************************
LibRealSenseContext - Class to share librealsense context objects
between multiple RealSense cameras (as required by the API).
Copyright (c) 2016 Oliver Kreylos

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

#include <Kinect/Internal/LibRealSenseContext.h>

#include <librealsense/rs.h>
#include <stdexcept>
#include <Misc/ThrowStdErr.h>

namespace Kinect {

/********************************************
Static elements of class LibRealSenseContext:
********************************************/

LibRealSenseContext LibRealSenseContext::theContext;

/************************************
Methods of class LibRealSenseContext:
************************************/

void LibRealSenseContext::ref(void)
	{
	/* Lock the reference mutex: */
	Threads::Mutex::Lock refLock(refMutex);
	
	/* Increment the reference counter and check if it was zero before: */
	if((refCount++)==0)
		{
		/* Initialize the librealsense context: */
		rs_error* error=0;
		context=rs_create_context(RS_API_VERSION,&error);
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::printStdErrMsg("Kinect::LibRealSenseContext: Error %s while creating librealsense context",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		}
	}

void LibRealSenseContext::unref(void)
	{
	/* Lock the reference mutex: */
	Threads::Mutex::Lock refLock(refMutex);
	
	/* Decrement the reference counter and check if it reached zero: */
	if((--refCount)==0)
		{
		/* Destroy the librealsense context: */
		rs_error* error=0;
		rs_delete_context(context,&error);
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::printStdErrMsg("Kinect::LibRealSenseContext: Error %s while destroying librealsense context",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		context=0;
		}
	}

LibRealSenseContext::LibRealSenseContext(void)
	:refCount(0),
	 context(0)
	{
	}

LibRealSenseContextPtr LibRealSenseContext::acquireContext(void)
	{
	/* Return an autopointer to the singleton librealsense context object; autopointer's call to ref() will set up context on first call: */
	return LibRealSenseContextPtr(&theContext);
	}

int LibRealSenseContext::getNumDevices(void) const
	{
	rs_error* error=0;
	int result=rs_get_device_count(context,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::LibRealSenseContext: Error %s while querying number of RealSense devices",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	
	return result;
	}

struct rs_device* LibRealSenseContext::getDevice(int index)
	{
	rs_error* error=0;
	struct rs_device* result=rs_get_device(context,index,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::LibRealSenseContext: Error %s while opening RealSense device %d",rs_get_error_message(error),index));
		rs_free_error(error);
		throw exception;
		}
	
	return result;
	}

}
