/***********************************************************************
USBContext - Class representing USB library contexts.
Copyright (c) 2010 Oliver Kreylos
***********************************************************************/

#include "USBContext.h"

#include <libusb-1.0/libusb.h>
#include <Misc/ThrowStdErr.h>

/***************************
Methods of class USBContext:
***************************/

void* USBContext::eventHandlingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
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
