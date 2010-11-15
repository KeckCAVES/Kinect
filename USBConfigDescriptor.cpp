/***********************************************************************
USBConfigDescriptor - Class representing a USB configuration descriptor.
Copyright (c) 2010 Oliver Kreylos
***********************************************************************/

#include "USBConfigDescriptor.h"

#include <libusb-1.0/libusb.h>

/************************************
Methods of class USBConfigDescriptor:
************************************/

USBConfigDescriptor::~USBConfigDescriptor(void)
	{
	libusb_free_config_descriptor(descriptor);
	}
