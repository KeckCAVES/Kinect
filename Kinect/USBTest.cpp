/***********************************************************************
USBTest - Test program for USB library and basic device handshaking.
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

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <Misc/ThrowStdErr.h>
#include <Misc/Time.h>
#include <Misc/FunctionCalls.h>
#include <Kinect/USBContext.h>
#include <Kinect/USBDevice.h>
#include <Kinect/USBDeviceList.h>
#include <Kinect/KinectMotor.h>
#include <Kinect/KinectCamera.h>

void dumpControl(USBDevice& device,unsigned int bmRequest,unsigned int bRequest,unsigned int wValue,unsigned int wIndex,size_t maxSize = 4096)
	{
	/* Allocate a buffer: */
	unsigned char* buffer=new unsigned char[maxSize];
	
	try
		{
		/* Issue the request: */
		size_t resultSize=0;
		while(resultSize==0)
			{
			usleep(1000);
			resultSize=device.readControl(bmRequest|0x80U,bRequest,wValue,wIndex,buffer,maxSize);
			}
		
		/* Process the result: */
		std::cout<<"Received "<<resultSize<<" bytes for request "<<int(bmRequest)<<", "<<int(bRequest)<<", "<<int(wValue)<<", "<<int(wIndex)<<":"<<std::endl;
		for(size_t row=0;row<resultSize;row+=8)
			{
			std::cout<<std::setfill('0')<<std::setw(2)<<std::hex<<int(buffer[row+0]);
			for(size_t column=1;column<8&&column<resultSize-row;++column)
				std::cout<<"   "<<std::setfill('0')<<std::setw(2)<<std::hex<<int(buffer[row+column]);
			std::cout<<std::dec<<std::endl;
			}
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		}
	
	/* Clean up: */
	delete[] buffer;
	}

void playWithMotor(USBContext& context,int pitch)
	{
	/* Enumerate all devices: */
	USBDeviceList deviceList(context);
	
	/* Get the first Kinect motor device: */
	USBDevice kinectMotor(deviceList.getDevice(0x045e,0x02b0));
	if(!kinectMotor.isValid())
		Misc::throwStdErr("USBTest: No Kinect motor device found");
	
	/* Open the device: */
	kinectMotor.open();
	kinectMotor.setConfiguration(1);
	kinectMotor.claimInterface(0);
	
	dumpControl(kinectMotor,0x40,0x32,0x0000,0x0000);
	
	/* Move the motor: */
	if(pitch<0)
		pitch+=65536;
	kinectMotor.writeControl(0x40,0x31,pitch,0x0000,0,0);
	
	#if 0
	/* Read the accelerometer values: */
	while(true)
		{
		unsigned char data[32];
		size_t dataSize=kinectMotor.readControl(0x40,0x32,0x0000,0x0000,sizeof(data),data);
		if(dataSize>=8)
			{
			int val[3];
			for(int i=0;i<3;++i)
				{
				val[i]=(int(data[i*2+2])<<8)|int(data[i*2+3]);
				if(val[i]>=0x8000)
					val[i]-=65536;
				}
			std::cout<<std::setw(6)<<val[0]<<", "<<std::setw(6)<<val[1]<<", "<<std::setw(6)<<val[2]<<std::endl;
			}
		else
			std::cout<<"Short data of length"<<dataSize<<std::endl;
		usleep(100000);
		}
	#endif
	}

void depthCallback(const FrameBuffer& frameBuffer)
	{
	std::cout<<"Received depth frame"<<std::endl;
	}

void playWithCamera(USBContext& context)
	{
	/* Open the camera: */
	KinectCamera kinectCamera(context);
	
	/* Perform the magic chant and start streaming: */
	kinectCamera.startStreaming(0,Misc::createFunctionCall(depthCallback));
	
	/* Read depth frames for ten seconds: */
	Misc::Time end=Misc::Time::now();
	sleep(10);
	
	/* Stop streaming: */
	kinectCamera.stopStreaming();
	
	/* Reset the camera until we learn how to turn off light generation: */
	// kinectCamera.reset();
	}

int main(int argc,char* argv[])
	{
	/* Create a USB context: */
	USBContext context;
	context.setDebugLevel(3);
	context.startEventHandling();
	
	#if 0
	{
	/* Get the first Kinect camera device: */
	KinectCamera kinectCamera(context,0);
	
	/* Open the device: */
	kinectCamera.open();
	kinectCamera.setConfiguration(1);
	
	/* Send a control message: */
	unsigned short commandBuffer[4];
	commandBuffer[0]=0x4d47U;
	commandBuffer[1]=0;
	commandBuffer[2]=0x0000U;
	commandBuffer[3]=0x1000U;
	kinectCamera.writeControl(0x40,0x00,0x0000,0x0000,reinterpret_cast<unsigned char*>(commandBuffer),sizeof(commandBuffer));
	
	/* Dump the reply: */
	dumpControl(kinectCamera,0x40,0x00,0x0000,0x0000);
	}
	#endif
	
	if(argc>1&&strcmp(argv[1],"reset")==0)
		{
		/* Get the first Kinect camera device: */
		int cameraIndex=atoi(argv[2]);
		KinectCamera kinectCamera(context,cameraIndex);
		
		/* Open and reset the device: */
		kinectCamera.open();
		kinectCamera.reset();
		}
	
	if(argc>2&&strcmp(argv[1],"led")==0)
		{
		/* Enumerate all devices: */
		USBDeviceList deviceList(context);
		
		/* Get the first Kinect motor device: */
		KinectMotor kinectMotor(context,0);
		
		/* Set the LED state: */
		kinectMotor.setLED((KinectMotor::LEDState)atoi(argv[2]));
		}
	
	if(argc>2&&strcmp(argv[1],"pitch")==0)
		{
		/* Enumerate all devices: */
		USBDeviceList deviceList(context);
		
		/* Get the first Kinect motor device: */
		KinectMotor kinectMotor(context,0);
		
		/* Move the motor: */
		kinectMotor.setPitch(atoi(argv[2]));
		}
	
	return 0;
	}
