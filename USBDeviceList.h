/***********************************************************************
USBDeviceList - Class representing lists of USB devices resulting from
device enumeration.
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

#ifndef USBDEVICELIST_INCLUDED
#define USBDEVICELIST_INCLUDED

#include <stddef.h>

/* Forward declarations: */
struct libusb_device;
class USBContext;

class USBDeviceList
	{
	/* Elements: */
	private:
	size_t numDevices; // Number of devices in list
	libusb_device** deviceList; // Array of device structure pointers as returned by device enumeration function
	
	/* Constructors and destructors: */
	public:
	USBDeviceList(void) // Creates empty device list
		:numDevices(0),deviceList(0)
		{
		}
	USBDeviceList(const USBContext& context); // Creates a device list by enumerating all devices in the given context
	private:
	USBDeviceList(const USBDeviceList& source); // Prohibit copy constructor
	USBDeviceList& operator=(const USBDeviceList& source); // Prohibit assignment operator
	public:
	~USBDeviceList(void); // Destroys the device list
	
	/* Methods: */
	size_t getNumDevices(void) const // Returns the number of devices in the list
		{
		return numDevices;
		}
	libusb_device* getDevice(size_t index) const // Returns the device of the given index
		{
		return deviceList[index];
		}
	size_t getNumDevices(unsigned short idVendor,unsigned short idProduct) const; // Returns the number of USB devices with the given vendor / product ID
	libusb_device* getDevice(unsigned short idVendor,unsigned short idProduct,size_t index =0) const; // Returns index-th USB devices with the given vendor / product ID, or NULL
	};

#endif
