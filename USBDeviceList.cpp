/***********************************************************************
USBDeviceList - Class representing lists of USB devices resulting from
device enumeration.
Copyright (c) 2010 Oliver Kreylos
***********************************************************************/

#include "USBDeviceList.h"

#include <libusb-1.0/libusb.h>
#include <Misc/ThrowStdErr.h>

#include "USBContext.h"

/******************************
Methods of class USBDeviceList:
******************************/

USBDeviceList::USBDeviceList(const USBContext& context)
	:numDevices(0),deviceList(0)
	{
	/* Enumerate devices in the given context: */
	ssize_t enumerateResult=libusb_get_device_list(context.getContext(),&deviceList);
	if(enumerateResult>=0)
		numDevices=size_t(enumerateResult);
	else
		{
		deviceList=0;
		Misc::throwStdErr("USBDeviceList::USBDeviceList: Error while enumerating USB devices");
		}
	}

USBDeviceList::~USBDeviceList(void)
	{
	if(numDevices>0)
		{
		/* Unreference all devices and destroy the list: */
		libusb_free_device_list(deviceList,1);
		}
	}

size_t USBDeviceList::getNumDevices(unsigned short idVendor,unsigned short idProduct) const
	{
	size_t result=0;
	for(size_t i=0;i<numDevices;++i)
		{
		/* Get the device's descriptor: */
		libusb_device_descriptor descriptor;
		if(libusb_get_device_descriptor(deviceList[i],&descriptor)==0)
			{
			/* Check the vendor/product ID: */
			if(descriptor.idVendor==idVendor&&descriptor.idProduct==idProduct)
				{
				/* Found one! */
				++result;
				}
			}
		}
	return result;
	}

libusb_device* USBDeviceList::getDevice(unsigned short idVendor,unsigned short idProduct,size_t index) const
	{
	libusb_device* result=0;
	for(size_t i=0;result==0&&i<numDevices;++i)
		{
		/* Get the device's descriptor: */
		libusb_device_descriptor descriptor;
		if(libusb_get_device_descriptor(deviceList[i],&descriptor)==0)
			{
			/* Check the vendor/product ID: */
			if(descriptor.idVendor==idVendor&&descriptor.idProduct==idProduct)
				{
				/* Found a match! */
				if(index>0) // Not the one
					--index;
				else // The one!
					result=deviceList[i];
				}
			}
		}
	return result;
	}
