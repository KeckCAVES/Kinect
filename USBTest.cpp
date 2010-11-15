/***********************************************************************
USBTest - Test program for USB library and basic device handshaking.
Copyright (c) 2010 Oliver Kreylos
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

#include "USBContext.h"
#include "USBDevice.h"
#include "USBDeviceList.h"
#include "KinectCamera.h"

void dumpControl(USBDevice& device,unsigned int bmRequest,unsigned int bRequest,unsigned int wValue,unsigned int wIndex,size_t maxSize = 4096)
	{
	/* Allocate a buffer: */
	unsigned char* buffer=new unsigned char[maxSize];
	
	try
		{
		/* Issue the request: */
		size_t resultSize=device.readControl(bmRequest|0x80U,bRequest,wValue,wIndex,buffer,maxSize);
		
		/* Process the result: */
		std::cout<<"Received "<<resultSize<<" bytes for request "<<int(bmRequest)<<", "<<int(bRequest)<<", "<<int(wValue)<<", "<<int(wIndex)<<":"<<std::endl;
		for(int row=0;row<resultSize;row+=8)
			{
			std::cout<<std::setfill('0')<<std::setw(2)<<std::hex<<int(buffer[row+0]);
			for(int column=1;column<8&&column<resultSize-row;++column)
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
	kinectCamera.startStreaming(0,new Misc::VoidFunctionCall<const FrameBuffer&>(depthCallback));
	
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
	if(argc>1&&strcmp(argv[1],"reset")==0)
		{
		/* Create a USB context: */
		USBContext context;
		context.setDebugLevel(3);
		
		/* Enumerate all devices: */
		USBDeviceList deviceList(context);
		
		/* Get the first Kinect camera device: */
		USBDevice kinectCamera(deviceList.getDevice(0x045e,0x02ae));
		if(!kinectCamera.isValid())
			Misc::throwStdErr("USBTest: No Kinect camera device found");
		
		/* Open and reset the device: */
		kinectCamera.open();
		kinectCamera.reset();
		
		return 0;
		}
	
	try
		{
		/* Create a USB context: */
		USBContext context;
		context.setDebugLevel(3);
		context.startEventHandling();
		
		playWithMotor(context,atoi(argv[1]));
		
		// playWithCamera(context);
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
