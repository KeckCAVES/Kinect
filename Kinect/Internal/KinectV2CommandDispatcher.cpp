/***********************************************************************
KinectV2CommandDispatcher - Class to exchange commands and command
replies with a Kinect v2 device via USB bulk transfers.
Copyright (c) 2014-2017 Oliver Kreylos

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

#include <Kinect/Internal/KinectV2CommandDispatcher.h>

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <Misc/Utility.h>
#include <Misc/ThrowStdErr.h>
#include <IO/File.h>
#include <IO/FixedMemoryFile.h>
#include <IO/OpenFile.h>
#include <USB/Device.h>
#include <Kinect/Internal/KinectV2DepthStreamReader.h>

namespace Kinect {

/******************************************
Methods of class KinectV2CommandDispatcher:
******************************************/

size_t KinectV2CommandDispatcher::execute(const KinectV2CommandDispatcher::Command& command,unsigned int numParameters)
	{
	/* Send the command header: */
	size_t commandSize=(5+numParameters)*sizeof(Misc::UInt32);
	size_t sendSize=device.bulkTransfer(0x02U,const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(&command)),commandSize);
	if(sendSize!=commandSize)
		Misc::throwStdErr("Kinect::KinectV2CommandDispatcher::execute: Incomplete command transfer; %u of %u bytes",(unsigned int)sendSize,(unsigned int)commandSize);
	
	/* Process optional command replies: */
	replySize=0;
	
	if(command.maxResponseSize>0)
		{
		size_t maxResponseSize=Misc::max(size_t(command.maxResponseSize),sizeof(Reply));
		if(maxResponseSize>replyBufferSize)
			{
			delete[] replyBuffer;
			replyBufferSize=maxResponseSize;
			replyBuffer=new Misc::UInt8[replyBufferSize];
			}
		
		/* Read the command response into the reply buffer, assuming it's data: */
		size_t receiveSize=device.bulkTransfer(0x81U,replyBuffer,maxResponseSize);
		
		/* Check if the reply is a reply-complete message: */
		if(receiveSize!=sizeof(Reply)||reinterpret_cast<Reply*>(replyBuffer)->magic!=Reply::replyCompleteMagic)
			{
			/* Yup, we got somethin': */
			replySize=receiveSize;
			if(replySize>maxResponseSize)
				{
				/* Some warning would be in order here... */
				replySize=maxResponseSize;
				}
			}
		else
			{
			/* Didn't actually get any data, oh well: */
			if(reinterpret_cast<Reply*>(replyBuffer)->seq!=command.seq)
				Misc::throwStdErr("Kinect::KinectV2CommandDispatcher::execute: invalid command reply sequence number, %u instead of %u",reinterpret_cast<Reply*>(replyBuffer)->seq,command.seq);
			
			return 0;
			}
		}
	
	/* Read the response complete message: */
	Reply reply;
	size_t receiveSize=device.bulkTransfer(0x81U,reinterpret_cast<unsigned char*>(&reply),sizeof(Reply));
	if(receiveSize!=sizeof(Reply))
		Misc::throwStdErr("Kinect::KinectV2CommandDispatcher::execute: invalid command reply size of %u bytes",(unsigned int)receiveSize);
	if(reply.magic!=Reply::replyCompleteMagic)
		Misc::throwStdErr("Kinect::KinectV2CommandDispatcher::execute: invalid command reply magic value %u",reply.magic);
	if(reply.seq!=command.seq)
		Misc::throwStdErr("Kinect::KinectV2CommandDispatcher::execute: invalid command reply sequence number, %u instead of %u",reply.seq,command.seq);
	
	return replySize;
	}

KinectV2CommandDispatcher::KinectV2CommandDispatcher(USB::Device& sDevice)
	:device(sDevice),
	 nextSeq(0),
	 replyBufferSize(0),replyBuffer(0),replySize(0)
	{
	}

KinectV2CommandDispatcher::~KinectV2CommandDispatcher(void)
	{
	/* Delete the reply buffer: */
	delete[] replyBuffer;
	}

size_t KinectV2CommandDispatcher::execute(unsigned int command,unsigned int maxReplySize)
	{
	/* Fill a command structure: */
	Command cmd(nextSeq,maxReplySize,command);
	++nextSeq;
	
	/* Execute the command: */
	return execute(cmd,0);
	}

size_t KinectV2CommandDispatcher::execute(unsigned int command,unsigned int parameter0,unsigned int maxReplySize)
	{
	/* Fill a command structure: */
	Command cmd(nextSeq,maxReplySize,command);
	cmd.parameters[0]=parameter0;
	++nextSeq;
	
	/* Execute the command: */
	return execute(cmd,1);
	}

size_t KinectV2CommandDispatcher::execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int maxReplySize)
	{
	/* Fill a command structure: */
	Command cmd(nextSeq,maxReplySize,command);
	cmd.parameters[0]=parameter0;
	cmd.parameters[1]=parameter1;
	++nextSeq;
	
	/* Execute the command: */
	return execute(cmd,2);
	}

size_t KinectV2CommandDispatcher::execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int parameter2,unsigned int maxReplySize)
	{
	/* Fill a command structure: */
	Command cmd(nextSeq,maxReplySize,command);
	cmd.parameters[0]=parameter0;
	cmd.parameters[1]=parameter1;
	cmd.parameters[2]=parameter2;
	++nextSeq;
	
	/* Execute the command: */
	return execute(cmd,3);
	}

size_t KinectV2CommandDispatcher::execute(unsigned int command,unsigned int parameter0,unsigned int parameter1,unsigned int parameter2,unsigned int parameter3,unsigned int maxReplySize)
	{
	/* Fill a command structure: */
	Command cmd(nextSeq,maxReplySize,command);
	cmd.parameters[0]=parameter0;
	cmd.parameters[1]=parameter1;
	cmd.parameters[2]=parameter2;
	cmd.parameters[3]=parameter3;
	++nextSeq;
	
	/* Execute the command: */
	return execute(cmd,4);
	}

void KinectV2CommandDispatcher::saveReply(const char* fileName) const
	{
	/* Open the file: */
	IO::FilePtr file=IO::openFile(fileName,IO::File::WriteOnly);
	
	/* Write the reply buffer: */
	file->write(replyBuffer,replySize);
	}

Misc::UInt8* KinectV2CommandDispatcher::detachReply(void)
	{
	/* Forget the reply buffer and return it to the caller: */
	Misc::UInt8* result=replyBuffer;
	replyBufferSize=0;
	replyBuffer=0;
	return result;
	}

void KinectV2CommandDispatcher::initInterfaces(void)
	{
	/* Set the required alternate device configuration: */
	if(device.getConfiguration()!=1)
		device.setConfiguration(1);
	
	/* Claim the color and depth streaming interfaces: */
	device.claimInterface(0);
	device.claimInterface(1);
	
	#ifdef LIBUSB_SET_ISOCH_DELAY // Older versions of libusb-1.0 don't have this
	/* Set isochronous delay: */
	device.writeControl(LIBUSB_RECIPIENT_DEVICE,LIBUSB_SET_ISOCH_DELAY,40U,0U,0,0,1000U);
	#endif
	
	#ifdef LIBUSB_REQUEST_SET_SEL // Older versions of libusb-1.0 don't have this
	/* Set power management delays: */
	Misc::UInt8 setSelReq[6]={0x55U,0x00U,0x55U,0x00U,0x00U,0x00U};
	device.writeControl(LIBUSB_RECIPIENT_DEVICE,LIBUSB_REQUEST_SET_SEL,0U,0U,setSelReq,sizeof(setSelReq),1000U);
	#endif
	
	/* Disable streaming interface: */
	device.setAlternateSetting(1,0);
	device.writeControl(LIBUSB_RECIPIENT_DEVICE,LIBUSB_REQUEST_SET_FEATURE,0x30U,0U,0,0,1000U);
	device.writeControl(LIBUSB_RECIPIENT_DEVICE,LIBUSB_REQUEST_SET_FEATURE,0x31U,0U,0,0,1000U);
	
	/* Disable sensors: */
	device.writeControl(LIBUSB_RECIPIENT_INTERFACE,LIBUSB_REQUEST_SET_FEATURE,0U,0x300U,0,0,1000U);
	}

void KinectV2CommandDispatcher::downloadTables(KinectV2DepthStreamReader* depthStreamReader)
	{
	/* Read first data block: */
	execute(0x02U,0x00U,0x200U);
	
	/* Read second data block: */
	execute(0x14U,0x5cU);
	
	/* Read third data block: */
	execute(0x22U,0x01U,0x80U);
	
	/* Read depth camera parameters: */
	execute(0x22U,0x03U,0x1c0000U);
	
	/* Extract depth camera parameters: */
	{
	IO::FixedMemoryFile depthCameraParamsTableBlock(detachReply(),getReplySize());
	depthCameraParamsTableBlock.ref();
	depthCameraParamsTableBlock.setEndianness(Misc::LittleEndian);
	
	/* Extract projection parameters: */
	depthCameraParams.sx=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.sy=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParamsTableBlock.skip<Misc::Float32>(1);
	depthCameraParams.cx=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.cy=depthCameraParamsTableBlock.read<Misc::Float32>();
	
	/* Extract lens distortion correction parameters: */
	depthCameraParams.k1=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.k2=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.p1=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.p2=depthCameraParamsTableBlock.read<Misc::Float32>();
	depthCameraParams.k3=depthCameraParamsTableBlock.read<Misc::Float32>();
	}
	
	/* Read depth camera P0 tables: */
	execute(0x22U,0x02U,0x1c0000U);
	
	/* Forward the P0 table block to the depth stream reader if it exists: */
	if(depthStreamReader!=0)
		{
		IO::FixedMemoryFile p0TableBlock(detachReply(),getReplySize());
		p0TableBlock.ref();
		depthStreamReader->loadP0Tables(&p0TableBlock);
		}
	
	/* Read color camera parameters: */
	execute(0x22U,0x04U,0x1c0000U);
	
	/* Extract color camera parameters: */
	{
	IO::FixedMemoryFile colorCameraParamsTableBlock(detachReply(),getReplySize());
	colorCameraParamsTableBlock.ref();
	colorCameraParamsTableBlock.setEndianness(Misc::LittleEndian);
	
	/* Extract projection parameters: */
	colorCameraParamsTableBlock.skip<Misc::UInt8>(1);
	colorCameraParams.sx=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.sy=colorCameraParams.sx;
	colorCameraParams.cx=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.cy=colorCameraParamsTableBlock.read<Misc::Float32>();
	
	/* Extract shift between color and depth cameras: */
	colorCameraParams.shiftD=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.shiftM=colorCameraParamsTableBlock.read<Misc::Float32>();
	
	/* Coefficients of bivariate cubic polynomial to map depth pixels to color pixels in x: */
	colorCameraParams.pxx3y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx0y3=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx2y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx1y2=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx2y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx0y2=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx1y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx1y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx0y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pxx0y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	
	/* Coefficients of bivariate cubic polynomial to map depth pixels to color pixels in y: */
	colorCameraParams.pyx3y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx0y3=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx2y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx1y2=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx2y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx0y2=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx1y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx1y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx0y1=colorCameraParamsTableBlock.read<Misc::Float32>();
	colorCameraParams.pyx0y0=colorCameraParamsTableBlock.read<Misc::Float32>();
	}
	}

void KinectV2CommandDispatcher::startSensors(void)
	{
	/* Enable sensors: */
	device.writeControl(LIBUSB_RECIPIENT_INTERFACE,LIBUSB_REQUEST_SET_FEATURE,0U,0x0U,0,0,1000U);
	
	/* Read camera status: */
	execute(0x16U,0x90000U,0x4U);
	
	/* Initialize data streams: */
	execute(0x09U,0x0U);
	
	/* Enable streaming interface: */
	device.setAlternateSetting(1,1);
	
	/* Read camera status again: */
	execute(0x16U,0x90000U,0x4U);
	
	/* Enable streaming: */
	execute(0x2bU,0x01U,0x0U);
	}

void KinectV2CommandDispatcher::stopSensors(void)
	{
	/* Disable depth streaming interface: */
	device.setAlternateSetting(1,0);
	
	/* Execute unknown command: */
	execute(0x0aU,0x0U);
	
	/* Disable streaming: */
	execute(0x2bU,0x00U,0x0U);
	
	/* Disable sensors: */
	device.writeControl(LIBUSB_RECIPIENT_INTERFACE,LIBUSB_REQUEST_SET_FEATURE,0U,0x300U,0,0,1000U);
	}

}
