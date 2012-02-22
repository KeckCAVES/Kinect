/***********************************************************************
Motor - Wrapper class to represent the motor and accelerometer interface
aspect of the Kinect sensor.
Copyright (c) 2010-2011 Oliver Kreylos

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

#include <Kinect/Motor.h>

#include <Misc/ThrowStdErr.h>
#include <USB/Context.h>
#include <USB/DeviceList.h>

namespace Kinect {

/**********************
Methods of class Motor:
**********************/

Motor::Motor(USB::Context& usbContext,size_t index)
	{
	/* Get the index-th Kinect motor device from the context: */
	USB::DeviceList deviceList(usbContext);
	USB::Device::operator=(deviceList.getDevice(0x045e,0x02b0,index));
	if(!isValid())
		Misc::throwStdErr("Kinect::Motor::Motor: Less than %d Kinect motor devices detected",int(index));
	
	/* Open and prepare the device: */
	open();
	setConfiguration(1);
	claimInterface(0);
	}

void Motor::setLED(Motor::LEDState newLEDState)
	{
	/* Write an LED control message: */
	writeControl(0x40,0x06,newLEDState,0x0000,0,0);
	}

void Motor::setPitch(int pitch)
	{
	/* Limit the pitch value to valid interval to prevent motor breakage: */
	if(pitch<-35)
		pitch=-35;
	if(pitch>55)
		pitch=55;
	
	/* Convert the pitch value to unsigned int: */
	if(pitch<0)
		pitch+=65536;
	
	/* Write a pitch control message: */
	writeControl(0x40,0x31,pitch,0x0000,0,0);
	}

void Motor::readAccelerometers(float accels[3])
	{
	/* Read the raw accelerometer values: */
	unsigned char data[32];
	size_t dataSize=readControl(0x40,0x32,0x0000,0x0000,data,sizeof(data));
	if(dataSize>=8)
		{
		for(int i=0;i<3;++i)
			{
			int val=(int(data[i*2+2])<<8)|int(data[i*2+3]);
			if(val>=0x8000)
				val-=65536;
			accels[i]=float(val);
			}
		}
	else
		Misc::throwStdErr("Kinect::Motor::readAccelerometers: Short control packet, received %d bytes instead of 8",int(dataSize));
	}

}
