/***********************************************************************
KinectMotor - Wrapper class to represent the motor and accelerometer
interface aspect of the Kinect sensor.
Copyright (c) 2010 Oliver Kreylos
***********************************************************************/

#include "KinectMotor.h"

#include <Misc/ThrowStdErr.h>

#include "USBContext.h"
#include "USBDeviceList.h"

/****************************
Methods of class KinectMotor:
****************************/

KinectMotor::KinectMotor(USBContext& usbContext,size_t index)
	{
	/* Get the index-th Kinect motor device from the context: */
	USBDeviceList deviceList(usbContext);
	USBDevice::operator=(deviceList.getDevice(0x045e,0x02b0,index));
	if(!isValid())
		Misc::throwStdErr("KinectMotor::KinectMotor: Less than %d Kinect motor devices detected",int(index));
	
	/* Open and prepare the device: */
	open();
	setConfiguration(1);
	claimInterface(0);
	}

void KinectMotor::setPitch(int pitch)
	{
	/* Convert the pitch value to unsigned int: */
	if(pitch<0)
		pitch+=65536;
	writeControl(0x40,0x31,pitch,0x0000,0,0);
	}

void KinectMotor::readAccelerometers(float accels[3])
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
		Misc::throwStdErr("KinectMotor::readAccelerometers: Short control packet, received %d bytes instead of 8",int(dataSize));
	}
