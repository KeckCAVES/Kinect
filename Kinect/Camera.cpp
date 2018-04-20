/***********************************************************************
Camera - Wrapper class to represent the color and depth camera interface
aspects of the Kinect sensor.
Copyright (c) 2010-2018 Oliver Kreylos

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

#include <Kinect/Camera.h>

#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <string>
#include <iostream>
#include <Misc/ThrowStdErr.h>
#include <Misc/MessageLogger.h>
#include <Misc/FunctionCalls.h>
#include <Misc/FileTests.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <USB/DeviceList.h>
#include <IO/File.h>
#include <IO/Directory.h>
#include <IO/FixedMemoryFile.h>
#include <Math/Math.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/Margin.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <Kinect/Internal/Config.h>
#include <Kinect/FrameBuffer.h>

#define KINECT_CAMERA_DUMP_INIT 0

namespace Kinect {

/**********************************************
Methods of class Camera::CalibrationParameters:
**********************************************/

void Camera::CalibrationParameters::read(int subsection,IO::File& file)
	{
	switch(subsection)
		{
		case 0:
			dxCenter=file.read<Misc::SInt32>();
			ax=file.read<Misc::SInt32>();
			bx=file.read<Misc::SInt32>();
			cx=file.read<Misc::SInt32>();
			dx=file.read<Misc::SInt32>();
			dxStart=file.read<Misc::SInt32>();
			ay=file.read<Misc::SInt32>();
			by=file.read<Misc::SInt32>();
			cy=file.read<Misc::SInt32>();
			dy=file.read<Misc::SInt32>();
			dyStart=file.read<Misc::SInt32>();
			dxBetaStart=file.read<Misc::SInt32>();
			dyBetaStart=file.read<Misc::SInt32>();
			rolloutBlank=file.read<Misc::SInt32>();
			rolloutSize=file.read<Misc::SInt32>();
			dxBetaInc=file.read<Misc::SInt32>();
			dyBetaInc=file.read<Misc::SInt32>();
			dxdxStart=file.read<Misc::SInt32>();
			dxdyStart=file.read<Misc::SInt32>();
			dydxStart=file.read<Misc::SInt32>();
			dydyStart=file.read<Misc::SInt32>();
			dxdxdxStart=file.read<Misc::SInt32>();
			dydxdxStart=file.read<Misc::SInt32>();
			dxdxdyStart=file.read<Misc::SInt32>();
			dydxdyStart=file.read<Misc::SInt32>();
			backComp1=file.read<Misc::SInt32>();
			dydydxStart=file.read<Misc::SInt32>();
			backComp2=file.read<Misc::SInt32>();
			dydydyStart=file.read<Misc::SInt32>();
			break;
		
		case 1:
			startLines=file.read<Misc::UInt16>();
			endLines=file.read<Misc::UInt16>();
			croppingLines=file.read<Misc::UInt16>();
			break;
		
		case 2:
			constantShift=file.read<Misc::UInt16>();
			break;
		
		case 3:
			dcmosEmitterDist=file.read<Misc::Float32>();
			dcmosRcmosDist=file.read<Misc::Float32>();
			referenceDistance=file.read<Misc::Float32>();
			referencePixelSize=file.read<Misc::Float32>();
			break;
		}
	}

void Camera::CalibrationParameters::read(IO::File& file)
	{
	for(int i=0;i<4;++i)
		read(i,file);
	}

void Camera::CalibrationParameters::write(IO::File& file) const
	{
	file.write<Misc::SInt32>(dxCenter);
	file.write<Misc::SInt32>(ax);
	file.write<Misc::SInt32>(bx);
	file.write<Misc::SInt32>(cx);
	file.write<Misc::SInt32>(dx);
	file.write<Misc::SInt32>(dxStart);
	file.write<Misc::SInt32>(ay);
	file.write<Misc::SInt32>(by);
	file.write<Misc::SInt32>(cy);
	file.write<Misc::SInt32>(dy);
	file.write<Misc::SInt32>(dyStart);
	file.write<Misc::SInt32>(dxBetaStart);
	file.write<Misc::SInt32>(dyBetaStart);
	file.write<Misc::SInt32>(rolloutBlank);
	file.write<Misc::SInt32>(rolloutSize);
	file.write<Misc::SInt32>(dxBetaInc);
	file.write<Misc::SInt32>(dyBetaInc);
	file.write<Misc::SInt32>(dxdxStart);
	file.write<Misc::SInt32>(dxdyStart);
	file.write<Misc::SInt32>(dydxStart);
	file.write<Misc::SInt32>(dydyStart);
	file.write<Misc::SInt32>(dxdxdxStart);
	file.write<Misc::SInt32>(dydxdxStart);
	file.write<Misc::SInt32>(dxdxdyStart);
	file.write<Misc::SInt32>(dydxdyStart);
	file.write<Misc::SInt32>(backComp1);
	file.write<Misc::SInt32>(dydydxStart);
	file.write<Misc::SInt32>(backComp2);
	file.write<Misc::SInt32>(dydydyStart);
	
	file.write<Misc::UInt16>(startLines);
	file.write<Misc::UInt16>(endLines);
	file.write<Misc::UInt16>(croppingLines);
	
	file.write<Misc::UInt16>(constantShift);
	
	file.write<Misc::Float32>(dcmosEmitterDist);
	file.write<Misc::Float32>(dcmosRcmosDist);
	file.write<Misc::Float32>(referenceDistance);
	file.write<Misc::Float32>(referencePixelSize);
	}

/***************************************
Methods of class Camera::StreamingState:
***************************************/

Camera::StreamingState::StreamingState(libusb_device_handle* handle,unsigned int endpoint,Camera* sCamera,int sPacketFlagBase,int sPacketSize,const unsigned int sFrameSize[2],size_t sRawFrameSize,Camera::StreamingCallback* sStreamingCallback)
	:camera(sCamera),
	 packetFlagBase(sPacketFlagBase),
	 packetSize(sPacketSize),numPackets(16),numTransfers(32),
	 transferBuffers(0),transfers(0),numActiveTransfers(0),
	 rawFrameSize(sRawFrameSize),rawFrameBuffer(new unsigned char[rawFrameSize*2]),
	 activeBuffer(0),writePtr(rawFrameBuffer),bufferSpace(rawFrameSize),
	 readyFrame(0),cancelDecoding(false),
	 streamingCallback(sStreamingCallback)
	{
	/* Copy the frame size: */
	frameSize[0]=sFrameSize[0];
	frameSize[1]=sFrameSize[1];
	
	/* Initialize the streaming data structures: */
	transferBuffers=new unsigned char*[numTransfers];
	transfers=new libusb_transfer*[numTransfers];
	for(int i=0;i<numTransfers;++i)
		{
		/* Allocate a transfer buffer and a transfer object: */
		transferBuffers[i]=new unsigned char[packetSize*numPackets];
		transfers[i]=libusb_alloc_transfer(numPackets);
		if(transfers[i]!=0)
			{
			/* Initialize the transfer object: */
			libusb_fill_iso_transfer(transfers[i],handle,endpoint,transferBuffers[i],packetSize*numPackets,numPackets,transferCallback,this,0);
			libusb_set_iso_packet_lengths(transfers[i],packetSize);
			if(libusb_submit_transfer(transfers[i])==0)
				++numActiveTransfers;
			else
				Misc::formattedConsoleError("Kinect::Camera: Error submitting USB transfer %d",i);
			}
		else
			Misc::formattedConsoleError("Kinect::Camera: Error allocating USB transfer %d",i);
		}
	}

Camera::StreamingState::~StreamingState(void)
	{
	/* Cancel all transfers: */
	cancelDecoding=true;
	for(int i=0;i<numTransfers;++i)
		libusb_cancel_transfer(transfers[i]);
	
	/* Stop the decoding thread: */
	#if 0
	decodingThread.cancel();
	#else
	{
	Threads::MutexCond::Lock frameReadyLock(frameReadyCond);
	frameReadyCond.signal();
	}
	#endif
	decodingThread.join();
	
	/* Wait for all cancellations to complete: */
	while(numActiveTransfers>0)
		{
		/* Wait for a bit: */
		usleep(1000);
		}
	
	/* Destroy the streaming data structures: */
	for(int i=0;i<numTransfers;++i)
		{
		/* Delete the transfer object and buffer: */
		libusb_free_transfer(transfers[i]);
		delete[] transferBuffers[i];
		}
	delete[] transfers;
	delete[] transferBuffers;
	
	/* Destroy the raw frame buffer: */
	delete[] rawFrameBuffer;
	
	/* Destroy the streaming callback: */
	delete streamingCallback;
	}

void Camera::StreamingState::transferCallback(libusb_transfer* transfer)
	{
	/* Get the object pointer: */
	StreamingState* thisPtr=static_cast<StreamingState*>(transfer->user_data);
	
	if(transfer->status==LIBUSB_TRANSFER_COMPLETED)
		{
		/* Process all isochronous packets in the completed transfer: */
		unsigned char* packetPtr=transfer->buffer;
		for(int i=0;i<transfer->num_iso_packets;++i)
			{
			size_t packetSize=transfer->iso_packet_desc[i].actual_length;
			if(packetSize>=12&&packetPtr[0]==0x52U&&packetPtr[1]==0x42U)
				{
				#if KINECT_CAMERA_DUMP_HEADERS
				if(thisPtr->headerFile!=0)
					thisPtr->headerFile->write(packetPtr,12);
				#endif
				
				/* Parse the packet header: */
				size_t payloadSize=packetSize-12*sizeof(unsigned char); // Each packet has a 12-byte header
				int packetType=packetPtr[3]-thisPtr->packetFlagBase;
				
				/* Check if this is the beginning of a new frame: */
				if(packetType==0x01)
					{
					/* Sample the timer: */
					Time now;
					
					/* Activate the next double buffer half: */
					thisPtr->activeBuffer=1-thisPtr->activeBuffer;
					thisPtr->writePtr=thisPtr->rawFrameBuffer+thisPtr->rawFrameSize*thisPtr->activeBuffer;
					thisPtr->bufferSpace=thisPtr->rawFrameSize;
					
					/*************************************************************
					This is where we would synchronize clocks to account for
					random OS delays, subtract expected hardware latency, etc. pp.
					*************************************************************/
					
					/* Time-stamp the new frame: */
					thisPtr->activeFrameTimeStamp=double(now-thisPtr->camera->timeBase);
					}
				
				/* Check for a data packet: */
				if(packetType==0x01||packetType==0x02||packetType==0x05)
					{
					/* Append the packet data to the active raw frame buffer: */
					if(thisPtr->bufferSpace>=payloadSize)
						{
						memcpy(thisPtr->writePtr,packetPtr+12,payloadSize);
						thisPtr->writePtr+=payloadSize;
						thisPtr->bufferSpace-=payloadSize;
						}
					}
				
				/* Check if this is the end of the current frame: */
				if(packetType==0x05)
					{
					Threads::MutexCond::Lock frameReadyLock(thisPtr->frameReadyCond);
					
					/* Check if the frame was received intact: */
					thisPtr->readyFrameIntact=true; // thisPtr->bufferSpace==0;
					if(thisPtr->readyFrameIntact)
						{
						/* Submit the raw frame to the frame decoder: */
						thisPtr->readyFrame=thisPtr->rawFrameBuffer+thisPtr->rawFrameSize*thisPtr->activeBuffer;
						thisPtr->readyFrameTimeStamp=thisPtr->activeFrameTimeStamp;
						thisPtr->frameReadyCond.signal();
						}
					}
				}
			
			/* Go to the next packet in the current USB transfer (even if a packet is short, the next one starts at the preset offset): */
			packetPtr+=thisPtr->packetSize;
			}
		
		/* Resubmit the transfer if not shutting down: */
		if(thisPtr->cancelDecoding||libusb_submit_transfer(transfer)!=0)
			{
			/* Mark this transfer as inactive: */
			--thisPtr->numActiveTransfers;
			
			/* Check if submitting the transfer failed due to an error: */
			if(!thisPtr->cancelDecoding)
				Misc::consoleError("Kinect::Camera: Error submitting USB transfer; camera may stop working soon");
			}
		}
	else if(transfer->status==LIBUSB_TRANSFER_CANCELLED)
		{
		/* Decrement the number of active transfers: */
		--thisPtr->numActiveTransfers;
		}
	}

/***********************
Methods of class Camera:
***********************/

size_t Camera::sendMessage(Camera::USBWord messageType,const Camera::USBWord* messageData,size_t messageSize,void* replyBuffer,size_t replyBufferSize)
	{
	if(messageSize>252)
		Misc::throwStdErr("Kinect::Camera::sendMessage: Message too long");
	if(replyBufferSize>=512)
		Misc::throwStdErr("Kinect::Camera::sendMessage: Expected response too long");
	
	/* Fill a message buffer: */
	USBWord messageBuffer[256];
	messageBuffer[0]=0x4d47U; // Magic number
	messageBuffer[1]=messageSize;
	messageBuffer[2]=messageType;
	messageBuffer[3]=messageSequenceNumber;
	++messageSequenceNumber;
	
	/* Copy the message data: */
	memcpy(messageBuffer+4,messageData,messageSize*sizeof(USBWord));
	
	/* Send the message to the device: */
	device.writeControl(0x40,0x00,0x0000,0x0000,reinterpret_cast<unsigned char*>(messageBuffer),(4+messageSize)*sizeof(USBWord));
	
	/* Receive the reply message: */
	size_t replySize;
	do
		{
		/* Wait for a reply: */
		usleep(1000);
		/* Receive the reply in our own message buffer to work around a problem with libusb reporting wrong reply sizes on USB 3.0: */
		replySize=device.readControl(0x40,0x00,0x0000,0x0000,reinterpret_cast<unsigned char*>(messageBuffer),sizeof(messageBuffer));
		}
	while(replySize==0||replySize==sizeof(messageBuffer)); // The second test is to work around a problem with USB 3.0
	
	/* Check the reply's magic number, command, and sequence number: */
	if(replySize<4*sizeof(USBWord)||messageBuffer[0]!=0x4252U||messageBuffer[2]!=messageType||messageBuffer[3]!=messageSequenceNumber-1)
		Misc::throwStdErr("Kinect::Camera::sendMessage: Protocol error while sending message %u",(unsigned int)messageType);
	
	/* Copy the message reply to the provided buffer: */
	memcpy(replyBuffer,messageBuffer,replySize);
	
	return replySize;
	}

bool Camera::sendCommand(Camera::USBWord command,Camera::USBWord value)
	{
	/* Prepare a command message: */
	USBWord commandBuffer[2];
	commandBuffer[0]=command;
	commandBuffer[1]=value;
	USBWord replyBuffer[8];
	size_t replySize=sendMessage(0x0003U,commandBuffer,2,replyBuffer,sizeof(replyBuffer));
	
	/* Check for success message: */
	return replySize==5*sizeof(USBWord)&&replyBuffer[1]==1&&replyBuffer[4]==0x0000U;
	}

Camera::USBWord Camera::readRegister(Camera::USBWord address)
	{
	/* Prepare a command message: */
	USBWord commandBuffer[3];
	commandBuffer[0]=1U; // 1 address only
	commandBuffer[1]=address&0x7fffU; // Clear highest bit to indicate read access
	commandBuffer[2]=0x0000U; // Dummy value
	USBWord replyBuffer[3];
	size_t replySize=sendMessage(0x0095U,commandBuffer,3,replyBuffer,sizeof(replyBuffer));
	
	/* Check for success message: */
	if(replySize!=3*sizeof(USBWord)||replyBuffer[0]!=0x0000U)
		Misc::throwStdErr("Kinect::Camera::readRegister: Protocol error");
	
	/* Return the read register value: */
	return replyBuffer[2];
	}

void Camera::writeRegister(Camera::USBWord address,Camera::USBWord value)
	{
	/* Prepare a command message: */
	USBWord commandBuffer[3];
	commandBuffer[0]=1U; // 1 address only
	commandBuffer[1]=address|0x8000U; // Set highest bit to indicate write access
	commandBuffer[2]=value;
	USBWord replyBuffer[3];
	size_t replySize=sendMessage(0x0095U,commandBuffer,3,replyBuffer,sizeof(replyBuffer));
	
	/* Check for success message: */
	if(replySize!=3*sizeof(USBWord)||replyBuffer[0]!=0x0000U)
		Misc::throwStdErr("Kinect::Camera::writeRegister: Protocol error");
	}

namespace {

/***********************************
Helper functions for Bayer decoding:
***********************************/

inline FrameSource::ColorComponent avg(FrameSource::ColorComponent v1,FrameSource::ColorComponent v2)
	{
	return FrameSource::ColorComponent(((unsigned int)(v1)+(unsigned int)(v2)+1U)>>1);
	}

inline FrameSource::ColorComponent avg(FrameSource::ColorComponent v1,FrameSource::ColorComponent v2,FrameSource::ColorComponent v3)
	{
	return FrameSource::ColorComponent(((unsigned int)(v1)+(unsigned int)(v2)+(unsigned int)(v3)+1U)/3U);
	}

inline FrameSource::ColorComponent avg(FrameSource::ColorComponent v1,FrameSource::ColorComponent v2,FrameSource::ColorComponent v3,FrameSource::ColorComponent v4)
	{
	return FrameSource::ColorComponent(((unsigned int)(v1)+(unsigned int)(v2)+(unsigned int)(v3)+(unsigned int)(v4)+2U)>>2);
	}

}

void* Camera::colorDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next color frame: */
		ColorComponent* framePtr;
		double frameTimeStamp;
		{
		Threads::MutexCond::Lock frameReadyLock(streamers[COLOR]->frameReadyCond);
		while(!streamers[COLOR]->cancelDecoding&&streamers[COLOR]->readyFrame==0)
			streamers[COLOR]->frameReadyCond.wait(frameReadyLock);
		if(streamers[COLOR]->cancelDecoding)
			break;
		framePtr=streamers[COLOR]->readyFrame;
		frameTimeStamp=streamers[COLOR]->readyFrameTimeStamp;
		streamers[COLOR]->readyFrame=0;
		}
		
		/* Allocate a new decoded color buffer: */
		int width=streamers[COLOR]->frameSize[0];
		int height=streamers[COLOR]->frameSize[1];
		FrameBuffer decodedFrame(width,height,width*height*sizeof(ColorPixel));
		decodedFrame.timeStamp=frameTimeStamp;
		
		/* Decode the raw color buffer (which is in Bayer GRBG pattern): */
		int stride=width;
		const ColorComponent* rRowPtr=framePtr;
		ColorComponent* cRowPtr=decodedFrame.getData<ColorComponent>();
		cRowPtr+=(height-1)*stride*3; // Flip the color image vertically
		
		/* Convert the first row: */
		const ColorComponent* rPtr=rRowPtr;
		ColorComponent* cPtr=cRowPtr;
		
		/* Convert the first row's first (G) pixel: */
		*(cPtr++)=rPtr[1];
		*(cPtr++)=rPtr[0];
		*(cPtr++)=rPtr[stride];
		++rPtr;
		
		/* Convert the first row's central pixels: */
		for(int x=1;x<width-1;x+=2)
			{
			/* Convert the odd (R) pixel: */
			*(cPtr++)=rPtr[0];
			*(cPtr++)=avg(rPtr[-1],rPtr[1],rPtr[stride]);
			*(cPtr++)=avg(rPtr[stride-1],rPtr[stride+1]);
			++rPtr;
			
			/* Convert the even (G) pixel: */
			*(cPtr++)=avg(rPtr[-1],rPtr[1]);
			*(cPtr++)=rPtr[0];
			*(cPtr++)=rPtr[stride];
			++rPtr;
			}
		
		/* Convert the first row's last (R) pixel: */
		*(cPtr++)=rPtr[0];
		*(cPtr++)=avg(rPtr[-1],rPtr[stride]);
		*(cPtr++)=rPtr[stride-1];
		++rPtr;
		
		rRowPtr+=stride;
		cRowPtr-=stride*3;
		
		/* Convert the central rows: */
		for(int y=1;y<height-1;y+=2)
			{
			/* Convert the odd row: */
			rPtr=rRowPtr;
			cPtr=cRowPtr;
			
			/* Convert the odd row's first (B) pixel: */
			*(cPtr++)=avg(rPtr[-stride+1],rPtr[stride+1]);
			*(cPtr++)=avg(rPtr[-stride],rPtr[1],rPtr[stride]);
			*(cPtr++)=rPtr[0];
			++rPtr;
			
			/* Convert the odd row's central pixels: */
			for(int x=1;x<width-1;x+=2)
				{
				/* Convert the odd (G) pixel: */
				*(cPtr++)=avg(rPtr[-stride],rPtr[stride]);
				*(cPtr++)=rPtr[0];
				*(cPtr++)=avg(rPtr[-1],rPtr[1]);
				++rPtr;
				
				/* Convert the even (B) pixel: */
				*(cPtr++)=avg(rPtr[-stride-1],rPtr[-stride+1],rPtr[stride-1],rPtr[stride+1]);
				*(cPtr++)=avg(rPtr[-stride],rPtr[-1],rPtr[1],rPtr[stride]);
				*(cPtr++)=rPtr[0];
				++rPtr;
				}
			
			/* Convert the odd row's last (G) pixel: */
			*(cPtr++)=avg(rPtr[-stride],rPtr[stride]);
			*(cPtr++)=rPtr[0];
			*(cPtr++)=rPtr[-1];
			++rPtr;
			
			rRowPtr+=stride;
			cRowPtr-=stride*3;
			
			/* Convert the even row: */
			rPtr=rRowPtr;
			cPtr=cRowPtr;
			
			/* Convert the even row's first (G) pixel: */
			*(cPtr++)=rPtr[1];
			*(cPtr++)=rPtr[0];
			*(cPtr++)=avg(rPtr[-stride],rPtr[stride]);
			++rPtr;
			
			/* Convert the even row's central pixels: */
			for(int x=1;x<width-1;x+=2)
				{
				/* Convert the odd (R) pixel: */
				*(cPtr++)=rPtr[0];
				*(cPtr++)=avg(rPtr[-stride],rPtr[-1],rPtr[1],rPtr[stride]);
				*(cPtr++)=avg(rPtr[-stride-1],rPtr[-stride+1],rPtr[stride-1],rPtr[stride+1]);
				++rPtr;
				
				/* Convert the even (G) pixel: */
				*(cPtr++)=avg(rPtr[-1],rPtr[1]);
				*(cPtr++)=rPtr[0];
				*(cPtr++)=avg(rPtr[-stride],rPtr[stride]);
				++rPtr;
				}
			
			/* Convert the even row's last (R) pixel: */
			*(cPtr++)=rPtr[0];
			*(cPtr++)=avg(rPtr[-stride],rPtr[-1],rPtr[stride]);
			*(cPtr++)=avg(rPtr[-stride-1],rPtr[stride-1]);
			++rPtr;
			
			rRowPtr+=stride;
			cRowPtr-=stride*3;
			}
		
		/* Convert the last row: */
		rPtr=rRowPtr;
		cPtr=cRowPtr;
		
		/* Convert the last row's first (B) pixel: */
		*(cPtr++)=rPtr[-stride+1];
		*(cPtr++)=avg(rPtr[-stride],rPtr[1]);
		*(cPtr++)=rPtr[0];
		++rPtr;
		
		/* Convert the last row's central pixels: */
		for(int x=1;x<width-1;x+=2)
			{
			/* Convert the odd (G) pixel: */
			*(cPtr++)=rPtr[-stride];
			*(cPtr++)=rPtr[0];
			*(cPtr++)=avg(rPtr[-1],rPtr[1]);
			++rPtr;
			
			/* Convert the even (B) pixel: */
			*(cPtr++)=avg(rPtr[-stride-1],rPtr[-stride+1]);
			*(cPtr++)=avg(rPtr[-stride],rPtr[-1],rPtr[1]);
			*(cPtr++)=rPtr[0];
			++rPtr;
			}
		
		/* Convert the last row's last (G) pixel: */
		*(cPtr++)=rPtr[-stride];
		*(cPtr++)=rPtr[0];
		*(cPtr++)=rPtr[-1];
		
		/* Pass the decoded color buffer to the streaming callback function: */
		(*streamers[COLOR]->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

void* Camera::depthDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	typedef Misc::UInt8 Byte;
	
	while(true)
		{
		/* Wait for the next depth frame: */
		Byte* framePtr;
		double frameTimeStamp;
		{
		Threads::MutexCond::Lock frameReadyLock(streamers[DEPTH]->frameReadyCond);
		while(!streamers[DEPTH]->cancelDecoding&&streamers[DEPTH]->readyFrame==0)
			streamers[DEPTH]->frameReadyCond.wait(frameReadyLock);
		if(streamers[DEPTH]->cancelDecoding)
			break;
		framePtr=streamers[DEPTH]->readyFrame;
		frameTimeStamp=streamers[DEPTH]->readyFrameTimeStamp;
		streamers[DEPTH]->readyFrame=0;
		}
		
		/* Allocate a new decoded depth buffer: */
		int width=streamers[DEPTH]->frameSize[0];
		int height=streamers[DEPTH]->frameSize[1];
		FrameBuffer decodedFrame(width,height,width*height*sizeof(DepthPixel));
		decodedFrame.timeStamp=frameTimeStamp;
		
		/* Decode the raw depth buffer: */
		Byte* sPtr=framePtr;
		DepthPixel* dRowPtr=decodedFrame.getData<DepthPixel>();
		dRowPtr+=width*(height-1);
		
		/* Process rows: */
		for(int y=0;y<height;++y,dRowPtr-=width) // Flip the depth image vertically
			{
			DepthPixel* dPtr=dRowPtr;
			
			/* Process pixels in groups of eight: */
			for(int x=0;x<width;x+=8,sPtr+=11,dPtr+=8)
				{
				/* Convert a run of 11 8-bit bytes into 8 11-bit pixels: */
				dPtr[0]=(DepthPixel(sPtr[0])<<3)|(DepthPixel(sPtr[1])>>5);
				dPtr[1]=((DepthPixel(sPtr[1])&0x1fU)<<6)|(DepthPixel(sPtr[2])>>2);
				dPtr[2]=((DepthPixel(sPtr[2])&0x03U)<<9)|(DepthPixel(sPtr[3])<<1)|(DepthPixel(sPtr[4])>>7);
				dPtr[3]=((DepthPixel(sPtr[4])&0x7fU)<<4)|(DepthPixel(sPtr[5])>>4);
				dPtr[4]=((DepthPixel(sPtr[5])&0x0fU)<<7)|(DepthPixel(sPtr[6])>>1);
				dPtr[5]=((DepthPixel(sPtr[6])&0x01U)<<10)|(DepthPixel(sPtr[7])<<2)|(DepthPixel(sPtr[8])>>6);
				dPtr[6]=((DepthPixel(sPtr[8])&0x3fU)<<5)|(DepthPixel(sPtr[9])>>3);
				dPtr[7]=((DepthPixel(sPtr[9])&0x07U)<<8)|DepthPixel(sPtr[10]);
				}
			}
		
		/* Handle background capture and removal: */
		processDepthFrameBackground(decodedFrame);
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*streamers[DEPTH]->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

namespace {

inline unsigned int getNybble(Misc::UInt8*& sPtr,bool& sFull)
	{
	unsigned int result;
	if(sFull)
		{
		/* Return the high nybble: */
		result=(*sPtr>>4)&0x0fU;
		
		/* Mark the high nybble read: */
		sFull=false;
		}
	else
		{
		/* Return the low nybble: */
		result=(*sPtr)&0x0fU;
		
		/* Go to the next source byte and mark the high nybble unread: */
		++sPtr;
		sFull=true;
		}
	
	return result;
	}

}

void* Camera::compressedDepthDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	typedef Misc::UInt8 Byte;
	
	while(true)
		{
		/* Wait for the next depth frame: */
		Byte* framePtr;
		double frameTimeStamp;
		{
		Threads::MutexCond::Lock frameReadyLock(streamers[DEPTH]->frameReadyCond);
		while(!streamers[DEPTH]->cancelDecoding&&streamers[DEPTH]->readyFrame==0)
			streamers[DEPTH]->frameReadyCond.wait(frameReadyLock);
		if(streamers[DEPTH]->cancelDecoding)
			break;
		framePtr=streamers[DEPTH]->readyFrame;
		frameTimeStamp=streamers[DEPTH]->readyFrameTimeStamp;
		streamers[DEPTH]->readyFrame=0;
		}
		
		/* Allocate a new decoded depth buffer: */
		int width=streamers[DEPTH]->frameSize[0];
		int height=streamers[DEPTH]->frameSize[1];
		FrameBuffer decodedFrame(width,height,width*height*sizeof(DepthPixel));
		decodedFrame.timeStamp=frameTimeStamp;
		
		/* Decode the raw depth buffer: */
		Byte* sPtr=framePtr;
		bool sFull=true;
		DepthPixel* dRowPtr=decodedFrame.getData<DepthPixel>();
		dRowPtr+=width*(height-1);
		
		/* Process rows: */
		for(int y=0;y<height;++y,dRowPtr-=width) // Flip the depth image vertically
			{
			DepthPixel* dPtr=dRowPtr;
			DepthPixel* dEnd=dPtr+width;
			
			/* Process RLE/differential code groups from the raw depth stream: */
			unsigned int lastPixel=0x7ffU;
			while(dPtr!=dEnd)
				{
				/* Parse the next code group: */
				unsigned int code=getNybble(sPtr,sFull);
				if(code==0x0fU) // It's either a literal depth value or a large-step differential
					{
					unsigned int value=getNybble(sPtr,sFull);
					if(value<0x08U) // It's a literal depth value
						{
						/* Read the rest of the literal depth value: */
						for(int i=0;i<3;++i)
							value=(value<<4)|getNybble(sPtr,sFull);
						
						/* Store the depth value: */
						lastPixel=value;
						*dPtr=DepthPixel(lastPixel);
						++dPtr;
						}
					else // It's a large-step differential
						{
						/* Read the rest of the differential: */
						value=(value<<4)|getNybble(sPtr,sFull);
						
						/* Calculate and store the depth value: */
						lastPixel=(lastPixel+value)-0xc0U;
						*dPtr=DepthPixel(lastPixel);
						++dPtr;
						}
					}
				else if(code==0x0eU) // It's an RLE span
					{
					/* Read the repetition count: */
					unsigned int numReps=getNybble(sPtr,sFull)+1;
					
					/* Copy the last pixel value: */
					while(numReps>0&&dPtr!=dEnd)
						{
						*dPtr=DepthPixel(lastPixel);
						++dPtr;
						--numReps;
						}
					}
				else // It's a small-step differential
					{
					/* Calculate and store the depth value: */
					lastPixel=(lastPixel+code)-0x06U;
					*dPtr=DepthPixel(lastPixel);
					++dPtr;
					}
				}
			}
		
		/* Handle background capture and removal: */
		processDepthFrameBackground(decodedFrame);
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*streamers[DEPTH]->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

namespace {

/****************
Helper functions:
****************/

std::string getKinectSerialNumber(libusb_device* device,USB::DeviceList* deviceList)
	{
	/* Determine the Kinect's model number: */
	USB::Device camera(device);
	libusb_device_descriptor dd=camera.getDeviceDescriptor();
	
	/* Check whether the Kinect's serial number is stored with the camera or audio sub-device: */
	if(dd.idProduct==0x02bfU||dd.bcdDevice>0x010bU)
		{
		/*******************************************************************
		Return the serial number of the Kinect audio device on the same hub
		as this camera device:
		*******************************************************************/
		
		std::string result;
		
		/* Enumerate all USB devices to find the audio sub-device sharing the same internal hub as the camera sub-device: */
		USB::DeviceList* myDeviceList=0;
		if(deviceList==0)
			deviceList=myDeviceList=new USB::DeviceList;
		
		/* Get the Kinect camera device's parent device, i.e., the Kinect's internal USB hub: */
		libusb_device* hub=deviceList->getParent(device);
		
		/* Determine the vendor/product ID for the matching Kinect audio sub-device: */
		USB::VendorProductId kinectAudioId(0x045eU,dd.idProduct==0x02bfU?0x02beU:0x02adU);
		
		/* Find the Kinect audio device connected to the same hub: */
		for(size_t i=0;i<deviceList->getNumDevices();++i)
			if(deviceList->getVendorProductId(i)==kinectAudioId&&deviceList->getParent(deviceList->getDevice(i))==hub)
				{
				/* Return the audio device's serial number: */
				USB::Device audio(deviceList->getDevice(i));
				result=audio.getSerialNumber();
				break;
				}
		
		delete myDeviceList;
		return result;
		}
	else
		{
		/* Return the camera sub-device's serial number: */
		return camera.getSerialNumber();
		}
	}

}

void Camera::initialize(USB::DeviceList* deviceList)
	{
	/* Determine the Kinect's model number: */
	libusb_device_descriptor dd=device.getDeviceDescriptor();
	
	/* Get the Kinect's serial number: */
	serialNumber=getKinectSerialNumber(device.getDevice(),deviceList);
	
	/* Initialize calibration parameter reply sizes (fourth reply's size depends on Kinect model): */
	calibrationParameterReplySizes[0]=126;
	calibrationParameterReplySizes[1]=16;
	calibrationParameterReplySizes[2]=12;
	calibrationParameterReplySizes[3]=(dd.idProduct==0x02bfU||dd.bcdDevice>0x010bU)?342:330; // Behavior change in Kinect-for-Xbox model 1473 and Kinect-for-Windows
	
	/* Check whether the camera requires alternative interface settings: */
	needAltInterface=dd.idProduct==0x02bfU; // Behavior change in Kinect-for-Windows
	hasNearMode=dd.idProduct==0x02bfU; // New feature in Kinect-for-Windows
	
	/* Initialize the camera and streamer states: */
	frameSizes[0]=FS_640_480;
	frameSizes[1]=FS_640_480;
	frameRates[0]=FR_30_HZ;
	frameRates[1]=FR_30_HZ;
	
	streamers[0]=0;
	streamers[1]=0;
	}

void Camera::nearModeToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set near mode flag: */
	setNearMode(cbData->set);
	}

void Camera::irIntensitySliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set IR intensity value: */
	setIrIntensity(int(Math::floor(cbData->value+0.5)));
	}

void Camera::colorSharpeningSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set color sharpening value: */
	setSharpening(int(Math::floor(cbData->value+0.5)));
	}

namespace {

/**************
Helper classes:
**************/

class KinectCameraMatcher // Class to match a Kinect camera device
	{
	/* Methods: */
	public:
	bool operator()(const libusb_device_descriptor& dd) const
		{
		/* Check for Kinect-for-Xbox and Kinect-for-Windows camera devices: */
		return dd.idVendor==0x045eU&&(dd.idProduct==0x02aeU||dd.idProduct==0x02bfU);
		}
	};

}

size_t Camera::getNumDevices(void)
	{
	/* Get the list of devices on all local USB buses: */
	USB::DeviceList deviceList;
	
	/* Return the number of Kinect cameras: */
	return deviceList.getNumDevices(KinectCameraMatcher());
	}

Camera::Camera(libusb_device* sDevice)
	:device(sDevice),
	 needAltInterface(false),hasNearMode(false),
	 messageSequenceNumber(0x2000U),
	 compressDepthFrames(true),smoothDepthFrames(true),irIntensity(30U),nearMode(false),sharpening(0)
	 #if KINECT_CAMERA_DUMP_HEADERS
	 ,headerFile(0)
	 #endif
	{
	/* Initialize the camera: */
	initialize(0);
	}

Camera::Camera(size_t index)
	:needAltInterface(false),hasNearMode(false),
	 messageSequenceNumber(0x2000U),
	 compressDepthFrames(true),smoothDepthFrames(true),irIntensity(30U),nearMode(false),sharpening(0)
	 #if KINECT_CAMERA_DUMP_HEADERS
	 ,headerFile(0)
	 #endif
	{
	/* Get the index-th Kinect camera device from the context: */
	USB::DeviceList deviceList;
	device=deviceList.getDevice(KinectCameraMatcher(),index);
	if(!device.isValid())
		Misc::throwStdErr("Kinect::Camera::Camera: Fewer than %d Kinect camera devices detected",int(index)+1);
	
	/* Initialize the camera: */
	initialize(&deviceList);
	}

Camera::Camera(const char* serialNumber)
	:needAltInterface(false),hasNearMode(false),
	 messageSequenceNumber(0x2000U),
	 compressDepthFrames(true),smoothDepthFrames(true),irIntensity(30U),nearMode(false),sharpening(0)
	 #if KINECT_CAMERA_DUMP_HEADERS
	 ,headerFile(0)
	 #endif
	{
	/* Enumerate all Kinect cameras on the USB bus: */
	USB::DeviceList deviceList;
	
	/* Search for all Kinect cameras: */
	libusb_device* dev=0;
	KinectCameraMatcher kcm;
	for(size_t i=0;dev==0&&i<deviceList.getNumDevices();++i)
		{
		/* Check if the device is a Kinect camera: */
		libusb_device_descriptor dd;
		if(kcm(deviceList.getDeviceDescriptor(i,dd)))
			{
			/* Get the device's serial number: */
			std::string devSerialNumber=getKinectSerialNumber(deviceList.getDevice(i),&deviceList);
			if(devSerialNumber==serialNumber)
				dev=deviceList.getDevice(i);
			}
		}
	if(dev==0)
		Misc::throwStdErr("Kinect::Camera::Camera: Kinect device with serial number %s not found",serialNumber);
	
	/* Initialize the camera: */
	device=dev;
	initialize(&deviceList);
	}

Camera::~Camera(void)
	{
	/* Stop streaming if necessary: */
	if(streamers[0]!=0||streamers[1]!=0)
		stopStreaming();
	}

FrameSource::DepthCorrection* Camera::getDepthCorrectionParameters(void)
	{
	/* Assemble the name of the depth correction parameters file: */
	std::string depthCorrectionFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	depthCorrectionFileName.push_back('/');
	depthCorrectionFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_DEPTHCORRECTIONFILENAMEPREFIX);
	depthCorrectionFileName.push_back('-');
	depthCorrectionFileName.append(serialNumber);
	depthCorrectionFileName.append(".dat");
	
	/* Check if a file of the given name exists and is readable: */
	if(IO::Directory::getCurrent()->getPathType(depthCorrectionFileName.c_str())==Misc::PATHTYPE_FILE)
		{
		try
			{
			/* Open the depth correction file: */
			IO::FilePtr depthCorrectionFile(IO::Directory::getCurrent()->openFile(depthCorrectionFileName.c_str()));
			depthCorrectionFile->setEndianness(Misc::LittleEndian);
			
			/* Read and return a depth correction object: */
			return new DepthCorrection(*depthCorrectionFile);
			}
		catch(std::runtime_error err)
			{
			/* Log an error: */
			Misc::formattedConsoleError("Kinect::Camera::getDepthCorrectionParameters: Could not load depth correction file %s due to exception %s",depthCorrectionFileName.c_str(),err.what());
			
			/* Return a default depth correction object: */
			return FrameSource::getDepthCorrectionParameters();
			}
		}
	else
		{
		/* Return a default depth correction object: */
		return FrameSource::getDepthCorrectionParameters();
		}
	}

FrameSource::IntrinsicParameters Camera::getIntrinsicParameters(void)
	{
	/* Assemble the name of the intrinsic parameter file: */
	std::string intrinsicParameterFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	intrinsicParameterFileName.push_back('/');
	intrinsicParameterFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX);
	intrinsicParameterFileName.push_back('-');
	intrinsicParameterFileName.append(serialNumber);
	if(frameSizes[COLOR]==FS_1280_1024)
		intrinsicParameterFileName.append("-high");
	intrinsicParameterFileName.append(".dat");
	
	IntrinsicParameters result;
	try
		{
		/* Open the parameter file: */
		IO::FilePtr parameterFile(IO::Directory::getCurrent()->openFile(intrinsicParameterFileName.c_str()));
		parameterFile->setEndianness(Misc::LittleEndian);
		
		/* Read the parameter file: */
		Misc::Float64 depthMatrix[16];
		parameterFile->read(depthMatrix,4*4);
		result.depthProjection=IntrinsicParameters::PTransform::fromRowMajor(depthMatrix);
		Misc::Float64 colorMatrix[16];
		parameterFile->read(colorMatrix,4*4);
		result.colorProjection=IntrinsicParameters::PTransform::fromRowMajor(colorMatrix);
		}
	catch(std::runtime_error err)
		{
		/* Log an error: */
		Misc::formattedConsoleError("Kinect::Camera::getIntrinsicParameters: Could not load intrinsic parameter file %s due to exception %s",intrinsicParameterFileName.c_str(),err.what());
		
		/* Extract intrinsic parameters from the Kinect's factory calibration data: */
		CalibrationParameters calib;
		getCalibrationParameters(calib);
		
		/* Calculate the depth-to-distance conversion formula: */
		double numerator=(4.0*double(calib.dcmosEmitterDist)*double(calib.referenceDistance))/double(calib.referencePixelSize);
		double denominator=4.0*double(calib.dcmosEmitterDist)/double(calib.referencePixelSize)+4.0*double(calib.constantShift)+1.5;
		
		/* Calculate the unprojection scale factor: */
		double scale=2.0*double(calib.referencePixelSize)/double(calib.referenceDistance);
		
		/* Construct the depth pixel unprojection matrix: */
		IntrinsicParameters::PTransform::Matrix& depthMatrix=result.depthProjection.getMatrix();
		depthMatrix=IntrinsicParameters::PTransform::Matrix::zero;
		depthMatrix(0,0)=scale;
		depthMatrix(0,3)=-scale*double(getActualFrameSize(DEPTH)[0])*0.5;
		depthMatrix(1,1)=scale;
		depthMatrix(1,3)=-scale*double(getActualFrameSize(DEPTH)[1])*0.5;
		depthMatrix(2,3)=-1.0;
		depthMatrix(3,2)=-1.0/numerator;
		depthMatrix(3,3)=denominator/numerator;
		
		/* Construct the color projection matrix from the depth pixel transformation polynomial: */
		result.colorProjection=IntrinsicParameters::PTransform::identity;
		}
	
	return result;
	}

const unsigned int* Camera::getActualFrameSize(int camera) const
	{
	static const unsigned int actualFrameSizes[2][2]={{640,480},{1280,1024}};
	return actualFrameSizes[frameSizes[camera]];
	}

FrameSource::DepthRange Camera::getDepthRange(void) const
	{
	return DepthRange(300,1100);
	}

void Camera::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	/* Open and prepare the device: */
	device.open();
	// device.setConfiguration(1); // This seems to confuse the device
	device.claimInterface(0,true); // Must detach kernel driver; Kinect supported as UVC camera in newer kernels
	if(needAltInterface)
		{
		/* Switch the camera to an alternative interface setting: */
		device.setAlternateSetting(0,1);
		}
	
	/***********************************************************
	Query the device's serial number(?) and firmware version(?):
	***********************************************************/
	
	/* This seems to wake up the device; try it a few times: */
	USBWord replyBuffer[64];
	for(int i=0;i<5;++i)
		{
		try
			{
			sendMessage(0x0000U,0,0,replyBuffer,sizeof(replyBuffer));
			sendMessage(0x0005U,0,0,replyBuffer,sizeof(replyBuffer));
			}
		catch(std::runtime_error err)
			{
			Misc::formattedConsoleWarning("Kinect::Camera::startStreaming: Caught exception %s; retrying to wake up Kinect camera %s",err.what(),getSerialNumber().c_str());
			usleep(100000);
			}
		}
	
	/***********************************
	Disable the color and depth cameras:
	***********************************/
	
	bool sequenceOk=true;
	sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0000U); // Disable color streaming
	sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0000U); // Disable depth streaming
	if(!sequenceOk)
		Misc::throwStdErr("Kinect::Camera::startStreaming: Failed to disable cameras");
	
	#if KINECT_CAMERA_DUMP_HEADERS
	std::string headerFileName="Headers-";
	headerFileName.append(getSerialNumber());
	headerFileName.append(".dat");
	headerFile=IO::openFile(headerFileName.c_str(),IO::File::WriteOnly);
	headerFile->setEndianness(Misc::LittleEndian);
	#endif
	
	/* Check if caller wants to receive color frames: */
	if(newColorStreamingCallback!=0)
		{
		/* Create the color streaming state: */
		const unsigned int* colorFrameSize=getActualFrameSize(COLOR);
		size_t rawFrameSize=colorFrameSize[0]*colorFrameSize[1]; // Bayer pattern; one byte per pixel
		streamers[COLOR]=new StreamingState(device.getDeviceHandle(),0x81U,this,0x80U,1920,colorFrameSize,rawFrameSize,newColorStreamingCallback);
		
		#if KINECT_CAMERA_DUMP_HEADERS
		streamers[COLOR]->headerFile=headerFile;
		#endif
		
		/* Start the color decoding thread: */
		streamers[COLOR]->decodingThread.start(this,&Camera::colorDecodingThreadMethod);
		}
	
	/* Check if caller wants to receive depth frames: */
	if(newDepthStreamingCallback!=0)
		{
		/* Create the depth streaming state: */
		const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
		size_t rawFrameSize=(depthFrameSize[0]*depthFrameSize[1]*11+7)/8; // Packed bitstream; 11 bits per pixel
		streamers[DEPTH]=new StreamingState(device.getDeviceHandle(),0x82U,this,0x70U,1760,depthFrameSize,rawFrameSize,newDepthStreamingCallback);
		
		#if KINECT_CAMERA_DUMP_HEADERS
		streamers[DEPTH]->headerFile=headerFile;
		#endif
		
		/* Start the depth decoding thread: */
		if(compressDepthFrames)
			streamers[DEPTH]->decodingThread.start(this,&Camera::compressedDepthDecodingThreadMethod);
		else
			streamers[DEPTH]->decodingThread.start(this,&Camera::depthDecodingThreadMethod);
		}
	
	/**********************************************************
	Initialize the color and depth cameras and start streaming:
	**********************************************************/
	
	sequenceOk=true;
	
	/* Configure color camera: */
	sequenceOk=sequenceOk&&sendCommand(0x000cU,0x0000U); // Request Bayer-encoded color images
	switch(frameSizes[COLOR])
		{
		case FS_640_480:
			sequenceOk=sequenceOk&&sendCommand(0x000dU,0x0001U); // Request 640x480 color images
			break;
		
		case FS_1280_1024:
			sequenceOk=sequenceOk&&sendCommand(0x000dU,0x0002U); // Request 1280x1024 color images
			break;
		}
	sequenceOk=sequenceOk&&sendCommand(0x000eU,getActualFrameRate(COLOR)); // Request selected color image frame rate
	
	/* Configure depth camera: */
	if(compressDepthFrames)
		sequenceOk=sequenceOk&&sendCommand(0x0012U,0x0001U); // Request RLE/differential compressed depth images
	else
		sequenceOk=sequenceOk&&sendCommand(0x0012U,0x0003U); // Request 11-bit packed depth images
	switch(frameSizes[DEPTH])
		{
		case FS_640_480:
			sequenceOk=sequenceOk&&sendCommand(0x0013U,0x0001U); // Request 640x480 depth images
			break;
		
		case FS_1280_1024:
			sequenceOk=sequenceOk&&sendCommand(0x0013U,0x0002U); // Request 1280x1024 depth images
			break;
		}
	sequenceOk=sequenceOk&&sendCommand(0x0014U,getActualFrameRate(DEPTH)); // Request selected depth image frame rate
	sequenceOk=sequenceOk&&sendCommand(0x0015U,irIntensity); // Select the IR projector's intensity
	if(smoothDepthFrames)
		sequenceOk=sequenceOk&&sendCommand(0x0016U,0x0001U); // Enable depth smoothing
	else
		sequenceOk=sequenceOk&&sendCommand(0x0016U,0x0000U); // Disable depth smoothing
	sequenceOk=sequenceOk&&sendCommand(0x0018U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0002U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0105U,0x0000U); // Disable IR pattern checking
	sequenceOk=sequenceOk&&sendCommand(0x0024U,0x0001U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x002dU,0x0001U); // Unknown semantics
	
	if(streamers[DEPTH]!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0002U); // Enable depth streaming
		sequenceOk=sequenceOk&&sendCommand(0x0017U,0x0000U); // Request normal orientation of depth images
		sequenceOk=sequenceOk&&sendCommand(0x0015U,irIntensity); // Set IR intensity
		
		if(hasNearMode)
			{
			/* Switch between near mode and regular mode on Kinect-for-Windows: */
			sequenceOk=sequenceOk&&sendCommand(0x02efU,nearMode?0x0000U:0x0190U);
			}
		}
	
	if(streamers[COLOR]!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0001U); // Enable color streaming
		sequenceOk=sequenceOk&&sendCommand(0x0047U,0x0000U); // Request normal orientation of color images
		
		#if 0
		/* Set the color camera's sharpening value: */
		if(sequenceOk)
			{
			try
				{
				writeRegister(0x0105U,sharpening);
				}
			catch(std::runtime_error)
				{
				sequenceOk=false;
				}
			}
		#endif
		}
	
	if(!sequenceOk)
		{
		/* Reset camera device to non-streaming state and clean up: */
		stopStreaming();
		
		/* Signal an error: */
		Misc::throwStdErr("Kinect::Camera::startStreaming: Failed to initialize streaming mode");
		}
	}

void Camera::stopStreaming(void)
	{
	if(streamers[DEPTH]!=0&&hasNearMode&&nearMode)
		{
		/* Reset to far mode: */
		sendCommand(0x02efU,0x0190U);
		}
	
	/* Send commands to stop streaming: */
	sendCommand(0x0005U,0x0000U); // Disable color streaming
	sendCommand(0x0006U,0x0000U); // Disable depth streaming (and turn off IR projector)
	
	/* Destroy the streaming states: */
	for(int i=0;i<2;++i)
		{
		delete streamers[i];
		streamers[i]=0;
		}
	
	/* Destroy the background removal buffer: */
	backgroundCaptureNumFrames=0;
	delete[] backgroundFrame;
	backgroundFrame=0;
	removeBackground=false;
	
	#if KINECT_CAMERA_DUMP_HEADERS
	headerFile=0;
	#endif
	
	/* Release the interface and re-attach the kernel driver: */
	if(needAltInterface)
		{
		/* Switch the camera back to the original interface setting: */
		device.setAlternateSetting(0,0);
		}
	device.releaseInterface(0);
	// device.setConfiguration(1); // This seems to confuse the device
	// device.reset(); // This seems to confuse the device
	device.close();
	}

std::string Camera::getSerialNumber(void)
	{
	return serialNumber;
	}

void Camera::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Call the base class method: */
	DirectFrameSource::configure(configFileSection);
	
	/* Select the color frame resolution: */
	setFrameSize(COLOR,configFileSection.retrieveValue<bool>("./colorHiRes",getFrameSize(COLOR)==FS_1280_1024)?FS_1280_1024:FS_640_480);
	
	/* Select the frame rate: */
	unsigned int frameRate=configFileSection.retrieveValue<unsigned int>("./frameRate",getFrameRate(COLOR)==FR_30_HZ?30:15);
	setFrameRate(COLOR,frameRate>=23?FR_30_HZ:FR_15_HZ);
	setFrameRate(DEPTH,frameRate>=23?FR_30_HZ:FR_15_HZ);
	
	/* Select depth compression: */
	setCompressDepthFrames(configFileSection.retrieveValue<bool>("./compressDepth",compressDepthFrames));
	
	/* Select depth smoothing: */
	setSmoothDepthFrames(configFileSection.retrieveValue<bool>("./smoothDepth",smoothDepthFrames));
	
	/* Select IR intensity: */
	setIrIntensity(configFileSection.retrieveValue<unsigned int>("./irIntensity",irIntensity));
	
	/* Select near mode: */
	if(hasNearMode)
		setNearMode(configFileSection.retrieveValue<bool>("./nearMode",nearMode));
	
	/* Set color camera sharpening value: */
	setSharpening(configFileSection.retrieveValue<unsigned int>("./colorSharpening",getSharpening()));
	}

void Camera::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Create the base class settings dialog: */
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	
	const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	
	if(hasNearMode)
		{
		/* Create a button to toggle near mode: */
		GLMotif::Margin* nearModeMargin=new GLMotif::Margin("NearModeMargin",settingsDialog,false);
		nearModeMargin->setAlignment(GLMotif::Alignment::LEFT);
		
		GLMotif::ToggleButton* nearModeToggle=new GLMotif::ToggleButton("NearModeToggle",nearModeMargin,"Near Mode");
		nearModeToggle->setBorderWidth(0.0f);
		nearModeToggle->setBorderType(GLMotif::Widget::PLAIN);
		nearModeToggle->setToggle(nearMode);
		nearModeToggle->getValueChangedCallbacks().add(this,&Camera::nearModeToggleCallback);
		
		nearModeMargin->manageChild();
		}
	
	/* Create a panel of sliders to set IR intensity and color sharpening: */
	GLMotif::RowColumn* sliderBox=new GLMotif::RowColumn("SliderBox",settingsDialog,false);
	sliderBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	sliderBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	sliderBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("IRIntensityLabel",sliderBox,"IR Intensity");
	
	GLMotif::TextFieldSlider* irIntensitySlider=new GLMotif::TextFieldSlider("IRIntensitySlider",sliderBox,3,ss.fontHeight*5.0f);
	irIntensitySlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	irIntensitySlider->setValueType(GLMotif::TextFieldSlider::UINT);
	irIntensitySlider->setValueRange(1,50,1);
	irIntensitySlider->setValue(irIntensity);
	irIntensitySlider->getValueChangedCallbacks().add(this,&Camera::irIntensitySliderCallback);
	
	new GLMotif::Label("ColorSharpeningLabel",sliderBox,"Color Sharpening");
	
	GLMotif::TextFieldSlider* colorSharpeningSlider=new GLMotif::TextFieldSlider("ColorSharpeningSlider",sliderBox,2,ss.fontHeight*4.0f);
	colorSharpeningSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	colorSharpeningSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	colorSharpeningSlider->setValueRange(0,7,1);
	colorSharpeningSlider->setValue(sharpening);
	colorSharpeningSlider->getValueChangedCallbacks().add(this,&Camera::colorSharpeningSliderCallback);
	
	sliderBox->manageChild();
	}

void Camera::getCalibrationParameters(Camera::CalibrationParameters& calib)
	{
	/* Temporarily open the device: */
	bool tempOpen=!device.isOpen();
	if(tempOpen)
		device.open();
	
	/* Set up the common command buffer: */
	USBWord cmdBuffer[5];
	cmdBuffer[1]=0x0000U; // Calibration parameter format
	switch(frameSizes[COLOR])
		{
		case FS_640_480:
			cmdBuffer[2]=1U; // "Medium" resolution
			break;
		
		case FS_1280_1024:
			cmdBuffer[2]=2U; // "High" resolution
			break;
		}
	switch(frameRates[COLOR])
		{
		case FR_15_HZ:
			cmdBuffer[3]=15U;
			break;
		
		case FR_30_HZ:
			cmdBuffer[3]=30U;
			break;
		}
	cmdBuffer[4]=0x0000U; // Offset into calibration parameter area
	
	/* Request the calibration parameters in four subsets: */
	static const USBWord subsetOpcodes[4]={0x0040U,0x0041U,0x0000U,0x0000U};
	for(int subset=0;subset<4;++subset)
		{
		/* Request the subset of calibration parameters: */
		cmdBuffer[0]=subsetOpcodes[subset];
		
		/* Send the message and receive the reply: */
		IO::FixedMemoryFile replyBuffer(calibrationParameterReplySizes[subset]);
		if(sendMessage(subset<3?0x0016U:0x0004U,cmdBuffer,subset<3?5:1,replyBuffer.getMemory(),calibrationParameterReplySizes[subset])!=calibrationParameterReplySizes[subset])
			Misc::throwStdErr("Kinect::Camera::getCalibrationParameters: Protocol error while requesting parameter subset");

		/* Extract the subset of calibration parameters: */
		replyBuffer.skip<USBWord>(4); // Skip the reply header
		replyBuffer.skip<USBWord>(1); // Skip the parameter set size
		if(subset==3)
			replyBuffer.skip<USBWord>(46); // Skip some unknown parameters
		calib.read(subset,replyBuffer);
		}
	
	/* Close the device again if it was temporarily opened: */
	if(tempOpen)
		device.close();
	}

void Camera::setFrameSize(int camera,Camera::FrameSize newFrameSize)
	{
	/* The depth camera can only do 640x480: */
	if(camera==DEPTH)
		newFrameSize=FS_640_480;
	
	/* Set the camera's frame size: */
	frameSizes[camera]=newFrameSize;
	
	/* Limit the camera's frame rate to 15 Hz if high-res frames were selected: */
	if(newFrameSize==FS_1280_1024)
		frameRates[camera]=FR_15_HZ;
	}

void Camera::setFrameRate(int camera,Camera::FrameRate newFrameRate)
	{
	/* Set the camera's frame rate: */
	frameRates[camera]=newFrameRate;
	
	/* Limit the camera's frame size to 640x480 if 30 Hz was selected: */
	if(newFrameRate==FR_30_HZ)
		frameSizes[camera]=FS_640_480;
	}

unsigned int Camera::getActualFrameRate(int camera) const
	{
	static const unsigned int actualFrameRates[2]={15,30};
	return actualFrameRates[frameRates[camera]];
	}

void Camera::setCompressDepthFrames(bool newCompressDepthFrames)
	{
	compressDepthFrames=newCompressDepthFrames; 
	}

void Camera::setSmoothDepthFrames(bool newSmoothDepthFrames)
	{
	smoothDepthFrames=newSmoothDepthFrames; 
	}

void Camera::setIrIntensity(unsigned short newIrIntensity)
	{
	/* Clamp requested IR intensity to the valid range: */
	irIntensity=Math::clamp(newIrIntensity,(unsigned short)1,(unsigned short)50);
	
	/* Change the IR intensity live if depth streaming is active: */
	if(streamers[DEPTH]!=0)
		sendCommand(0x0015U,irIntensity);
	}

void Camera::setNearMode(bool newNearMode)
	{
	if(hasNearMode)
		{
		nearMode=newNearMode;
		
		/* Select near mode or regular mode on Kinect-for-Windows if depth streaming is active: */
		if(streamers[DEPTH]!=0)
			sendCommand(0x02efU,nearMode?0x0000U:0x0190U);
		}
	}

unsigned int Camera::getSharpening(void)
	{
	if(streamers[COLOR]!=0)
		{
		/* Query the sharpening register value: */
		sharpening=readRegister(0x0105U);
		}
	
	return sharpening;
	}

void Camera::setSharpening(unsigned int newSharpening)
	{
	/* Limit the sharpening value: */
	sharpening=Math::clamp(newSharpening,0U,7U);
	
	if(streamers[COLOR]!=0)
		{
		/* Set the sharpening value: */
		writeRegister(0x0105U,sharpening);
		}
	}

}
