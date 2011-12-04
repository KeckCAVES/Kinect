/***********************************************************************
USBDevice - Class representing a USB device and optionally a handle
resulting from opening the device.
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

#ifndef USBDEVICE_INCLUDED
#define USBDEVICE_INCLUDED

#include <string>
#include <vector>

/* Forward declarations: */
struct libusb_device;
struct libusb_device_handle;
struct libusb_device_descriptor;
struct libusb_config_descriptor;

class USBDevice
	{
	/* Embedded classes: */
	private:
	struct ClaimedInterface // Structure to keep track of claimed interfaces
		{
		/* Elements: */
		public:
		int interfaceNumber; // Number of the interface
		bool detachedKernelDriver; // Flag if the interface had a kernel driver attached before it was forcefully claimed
		};
	
	/* Elements: */
	private:
	libusb_device* device; // USB device pointer from the USB library
	libusb_device_handle* handle; // USB device handle for opened device
	std::vector<ClaimedInterface> claimedInterfaces; // List of currently claimed interfaces on the device
	
	/* Constructors and destructors: */
	public:
	USBDevice(void) // Creates an invalid USB device
		:device(0),handle(0)
		{
		}
	USBDevice(libusb_device* sDevice); // Creates wrapper around given USB device pointer
	USBDevice(const USBDevice& source); // Copy constructor; does not automatically open copied device
	USBDevice& operator=(libusb_device* sDevice); // Assignment operator for given USB device pointer
	USBDevice& operator=(const USBDevice& source); // Assignment operator; does not automatically open copied device
	~USBDevice(void); // Closes and destroys the USB device
	
	/* Methods: */
	bool isValid(void) const // Returns true if the device is valid
		{
		return device!=0;
		}
	libusb_device* getDevice(void) const // Returns a pointer to the USB device
		{
		return device;
		}
	unsigned int getBusNumber(void) const; // Returns the number of the USB bus to which the device is connected
	unsigned int getAddress(void) const; // REturns the device's address on the USB bus to which it is connected
	libusb_device_descriptor getDeviceDescriptor(void); // Returns the device's device descriptor
	std::string getSerialNumber(void); // Returns the device's serial number, or the empty string if none is defined
	libusb_config_descriptor* getActiveConfigDescriptor(void); // Returns a descriptor for the device's active configuration
	libusb_config_descriptor* getConfigDescriptorByIndex(unsigned int index); // Returns a descriptor for the device's configuration of the given index
	libusb_config_descriptor* getConfigDescriptorByValue(unsigned int value); // Returns a descriptor for the device's configuration of the given configuration value
	bool isOpen(void) const // Returns true if the device has been opened successfully
		{
		return handle!=0;
		}
	libusb_device_handle* getDeviceHandle(void) const // Returns the USB device's handle
		{
		return handle;
		}
	void open(void); // Opens the device
	int getConfiguration(void) const; // Returns the number of the device's current configuration
	void setConfiguration(int newConfiguration); // Sets the device's current configuration; pass -1 to set device to unconfigured state
	void claimInterface(int interfaceNumber,bool detachKernelDriver =false); // Claims the interface of the given number; tries detaching kernel drivers from interface first if flag is true
	void setAlternateSetting(int interfaceNumber,int alternateSettingNumber); // Sets an alternate setting for the given interface
	void writeControl(unsigned int requestType,unsigned int request,unsigned int value,unsigned int index,const unsigned char* data,size_t dataSize,unsigned int timeOut =0); // Writes a control message to the device
	size_t readControl(unsigned int requestType,unsigned int request,unsigned int value,unsigned int index,unsigned char* data,size_t maxDataSize,unsigned int timeOut =0); // Reads a control message from the device; returns amount of data actually read
	void releaseInterface(int interfaceNumber); // Releases the interface of the given number and re-attaches a kernel driver that was detached when the interface was claimed
	bool reset(void); // Resets device; returns true if the device becomes invalid and needs to be re-discovered
	void close(void); // Closes the device explicitly
	};

#endif
