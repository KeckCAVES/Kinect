/***********************************************************************
KinectUtil - Utility program to detect, list, and configure Kinect
devices.
Copyright (c) 2011 Oliver Kreylos

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

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <Misc/HashTable.h>
#include <USB/Context.h>
#include <USB/Device.h>
#include <USB/DeviceList.h>

USB::Context usbContext;

void list(void)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the number of Kinect camera devices: */
	size_t numKinects=deviceList.getNumDevices(0x045e,0x02ae);
	
	/* Print information about all Kinect camera devices: */
	unsigned int* busNumbers=new unsigned int[numKinects];
	unsigned int maxBusNumber=0;
	for(size_t i=0;i<numKinects;++i)
		{
		/* Get the i-th Kinect device: */
		USB::Device kinect=deviceList.getDevice(0x045e,0x02ae,i);
		kinect.open();
		busNumbers[i]=kinect.getBusNumber();
		if(maxBusNumber<busNumbers[i]+1)
			maxBusNumber=busNumbers[i]+1;
		std::cout<<"Kinect "<<i<<": USB address ";
		std::cout<<std::setfill('0')<<std::setw(3)<<kinect.getBusNumber()<<":"<<std::setfill('0')<<std::setw(3)<<kinect.getAddress();
		std::cout<<", device serial number "<<kinect.getSerialNumber()<<std::endl;
		}
	
	/* Check for bus conflicts: */
	for(unsigned int busNumber=0;busNumber<maxBusNumber;++busNumber)
		{
		/* Count the number of Kinect devices on this USB bus: */
		size_t numKinectsOnBus=0;
		for(size_t i=0;i<numKinects;++i)
			if(busNumbers[i]==busNumber)
				++numKinectsOnBus;
		
		/* Check for conflicts: */
		if(numKinectsOnBus>1)
			{
			size_t i=0;
			while(busNumbers[i]!=busNumber)
				++i;
			std::cout<<"Warning: USB bus "<<busNumber<<" is shared by Kinect devices "<<i;
			for(size_t j=1;j<numKinectsOnBus-1;++j)
				{
				while(busNumbers[i]!=busNumber)
					++i;
				std::cout<<", "<<i;
				}
			while(busNumbers[i]!=busNumber)
				++i;
			std::cout<<" and "<<i<<"."<<std::endl;
			std::cout<<"This will not work unless bus "<<busNumber<<" is a super-speed USB 3 bus."<<std::endl;
			}
		}
	}

void resetAll(void)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the number of Kinect camera devices: */
	size_t numKinects=deviceList.getNumDevices(0x045e,0x02ae);
	
	/* Reset all Kinect devices: */
	for(size_t i=0;i<numKinects;++i)
		{
		std::cout<<"Resetting Kinect "<<i<<"..."<<std::flush;
		USB::Device kinect=deviceList.getDevice(0x045e,0x02ae,i);
		kinect.open();
		kinect.reset();
		std::cout<<" done"<<std::endl;
		}
	}

bool reset(unsigned int index)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the index-th Kinect device: */
	USB::Device kinect=deviceList.getDevice(0x045e,0x02ae,index);
	if(!kinect.isValid())
		return false;
		
	std::cout<<"Resetting Kinect "<<index<<"..."<<std::flush;
	kinect.open();
	kinect.reset();
	std::cout<<" done"<<std::endl;
	
	return true;
	}

int main(int argc,char* argv[])
	{
	/* Initialize the USB context: */
	usbContext.setDebugLevel(3);
	usbContext.startEventHandling();
	
	/* Parse the command line: */
	if(argc<2)
		{
		std::cout<<"Missing command. Usage:"<<std::endl;
		std::cout<<"KinectUtil ( list | ( reset [ all | <index> ] ) )"<<std::endl;
		return 1;
		}
	if(strcasecmp(argv[1],"list")==0)
		{
		/* List all Kinect devices: */
		list();
		}
	else if(strcasecmp(argv[1],"reset")==0)
		{
		if(argc>2&&strcasecmp(argv[2],"all")==0)
			{
			/* Reset all Kinect devices: */
			resetAll();
			}
		else
			{
			/* Reset the given Kinect device: */
			int index=0;
			if(argc>2)
				index=atoi(argv[2]);
			if(!reset(index))
				{
				std::cerr<<"Kinect "<<index<<" does not exist."<<std::endl;
				return 1;
				}
			}
		}
	
	return 0;
	}
