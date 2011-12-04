/***********************************************************************
USBContext - Class representing USB library contexts.
Copyright (c) 2010 Oliver Kreylos

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

#include <Kinect/USBContext.h>

#include <libusb-1.0/libusb.h>
#include <Misc/ThrowStdErr.h>

/***************************
Methods of class USBContext:
***************************/

void* USBContext::eventHandlingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Block until the next USB event and handle it: */
		libusb_handle_events(context);
		}
	
	return 0;
	}

USBContext::USBContext(void)
	:context(0)
	{
	/* Initialize the context: */
	if(libusb_init(&context)!=0)
		Misc::throwStdErr("USBContext::USBContext: Error initializing USB context");
	}

USBContext::~USBContext(void)
	{
	if(!eventHandlingThread.isJoined())
		{
		/* Stop the event handling thread: */
		eventHandlingThread.cancel();
		eventHandlingThread.join();
		}
	
	/* Destroy the context: */
	if(context!=0)
		libusb_exit(context);
	}

void USBContext::setDebugLevel(int newDebugLevel)
	{
	libusb_set_debug(context,newDebugLevel);
	}

void USBContext::startEventHandling(void)
	{
	if(eventHandlingThread.isJoined())
		{
		/* Start the event handling thread: */
		eventHandlingThread.start(this,&USBContext::eventHandlingThreadMethod);
		}
	}

void USBContext::stopEventHandling(void)
	{
	if(!eventHandlingThread.isJoined())
		{
		/* Stop the event handling thread: */
		eventHandlingThread.cancel();
		eventHandlingThread.join();
		}
	}

void USBContext::processEvents(void)
	{
	/* Check if events are already handled in the background: */
	if(eventHandlingThread.isJoined())
		{
		/* Block until the next USB event and handle it: */
		libusb_handle_events(context);
		}
	}
