/***********************************************************************
KinectCamera - Wrapper class to represent the color and depth camera
interface aspects of the Kinect sensor.
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

#include "KinectCamera.h"

#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <IO/ValueSource.h>

#include "USBContext.h"
#include "USBDeviceList.h"
#include "FrameBuffer.h"

#define KINECT_DUMP_INIT 0

namespace {

/****************
Helper functions:
****************/

static int hexDigitValuesBuffer[257];
static int* hexDigitValues=hexDigitValuesBuffer+1;

void initHexDigits(void)
	{
	/* Initialize the hex digit value array: */
	for(int i=-1;i<256;++i)
		hexDigitValues[i]=-1;
	for(int i=0;i<10;++i)
		hexDigitValues['0'+i]=i;
	for(int i=0;i<6;++i)
		{
		hexDigitValues['A'+i]=10+i;
		hexDigitValues['a'+i]=10+i;
		}
	}

inline unsigned int readUInt(IO::ValueSource& source)
	{
	unsigned int result=0x0U;
	
	/* Accumulate hex digits until no more hex digits are found: */
	while(hexDigitValues[source.peekc()]>=0)
		{
		result<<=4;
		result|=(unsigned int)hexDigitValues[source.getChar()];
		}
	source.skipWs();
	
	return result;
	}

inline size_t readUCharBuffer(IO::ValueSource& source,unsigned char* buffer,size_t bufferSize)
	{
	size_t result=0;
	
	/* Read pairs of hex digits and store them as unsigned characters: */
	unsigned char* bufPtr=buffer;
	while(hexDigitValues[source.peekc()]>=0&&result<bufferSize)
		{
		/* Read one or two hex digits: */
		unsigned int val=hexDigitValues[source.getChar()]<<4;
		if(hexDigitValues[source.peekc()]>=0)
			val|=hexDigitValues[source.getChar()];
		
		/* Store the parsed character: */
		*bufPtr=(unsigned char)val;
		++bufPtr;
		++result;
		}
	
	/* Skip any leftover hex digits to get to a defined state in case of overflow: */
	while(hexDigitValues[source.peekc()]>=0)
		{
		source.getChar();
		if(hexDigitValues[source.peekc()]>=0)
			source.getChar();
		++result;
		}
	
	source.skipWs();
	
	return result;
	}

}

/*********************************************
Methods of class KinectCamera::StreamingState:
*********************************************/

KinectCamera::StreamingState::StreamingState(libusb_device_handle* handle,unsigned int endpoint,int sPacketFlagBase,int sPacketSize,const int sFrameSize[2],size_t sRawFrameSize,KinectCamera::StreamingCallback* sStreamingCallback)
	:packetFlagBase(sPacketFlagBase),
	 packetSize(sPacketSize),numPackets(16),numTransfers(32),
	 transferBuffers(0),transfers(0),numActiveTransfers(0),
	 rawFrameSize(sRawFrameSize),rawFrameBuffer(new unsigned char[rawFrameSize*2]),
	 activeBuffer(0),writePtr(rawFrameBuffer),bufferSpace(rawFrameSize),
	 readyFrame(0),
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
				std::cerr<<"Error submitting transfer "<<i<<std::endl;
			}
		else
			std::cerr<<"Error allocating transfer "<<i<<std::endl;
		}
	}

KinectCamera::StreamingState::~StreamingState(void)
	{
	/* Cancel all transfers: */
	for(int i=0;i<numTransfers;++i)
		libusb_cancel_transfer(transfers[i]);
	
	/* Stop the decoding thread: */
	decodingThread.cancel();
	decodingThread.join();
	
	/* Wait for all cancellations to complete: */
	while(numActiveTransfers>0)
		{
		/* Wait for a bit: */
		usleep(10);
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

void KinectCamera::StreamingState::transferCallback(libusb_transfer* transfer)
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
			if(packetSize>0)
				{
				/* Parse the packet header: */
				size_t payloadSize=packetSize-12*sizeof(unsigned char); // Each packet has a 12-byte header
				switch(packetPtr[3]-thisPtr->packetFlagBase)
					{
					case 0x01: // Start of new frame
						/* Activate the next double buffer half: */
						thisPtr->activeBuffer=1-thisPtr->activeBuffer;
						thisPtr->writePtr=thisPtr->rawFrameBuffer+thisPtr->rawFrameSize*thisPtr->activeBuffer;
						thisPtr->bufferSpace=thisPtr->rawFrameSize;
						
						/* Copy the packet data into the raw frame buffer: */
						memcpy(thisPtr->writePtr,packetPtr+12,payloadSize);
						thisPtr->writePtr+=payloadSize;
						thisPtr->bufferSpace-=payloadSize;
						break;
					
					case 0x02: // Frame continuation
					case 0x05: // End of frame
						if(thisPtr->bufferSpace>=payloadSize)
							{
							/* Copy the packet data into the raw frame buffer: */
							memcpy(thisPtr->writePtr,packetPtr+12,payloadSize);
							thisPtr->writePtr+=payloadSize;
							thisPtr->bufferSpace-=payloadSize;
							}
						
						/* Check for end of frame and correct frame size: */
						if(packetPtr[3]-thisPtr->packetFlagBase==0x05&&thisPtr->bufferSpace==0)
							{
							/* Submit the raw frame to the frame decoder: */
							Threads::MutexCond::Lock frameReadyLock(thisPtr->frameReadyCond);
							thisPtr->readyFrame=thisPtr->rawFrameBuffer+thisPtr->rawFrameSize*thisPtr->activeBuffer;
							thisPtr->frameReadyCond.signal(frameReadyLock);
							}
						break;
					}
				}
			
			packetPtr+=thisPtr->packetSize;
			}
		
		/* Resubmit the transfer: */
		libusb_submit_transfer(transfer);
		}
	else if(transfer->status==LIBUSB_TRANSFER_CANCELLED)
		{
		/* Decrement the number of active transfers: */
		--thisPtr->numActiveTransfers;
		}
	}

/*****************************
Methods of class KinectCamera:
*****************************/

size_t KinectCamera::sendMessage(unsigned short messageType,const unsigned short* messageData,size_t messageSize,void* replyBuffer,size_t replyBufferSize)
	{
	if(messageSize>252)
		Misc::throwStdErr("KinectCamera: Message too long");
	
	/* Fill a message buffer: */
	unsigned short messageBuffer[256];
	messageBuffer[0]=0x4d47U; // Magic number
	messageBuffer[1]=messageSize;
	messageBuffer[2]=messageType;
	messageBuffer[3]=messageSequenceNumber;
	++messageSequenceNumber;
	
	/* Copy the message data: */
	for(size_t i=0;i<messageSize;++i)
		messageBuffer[4+i]=messageData[i];
	
	/* Send the message to the device: */
	writeControl(0x40,0x00,0x0000,0x0000,reinterpret_cast<unsigned char*>(messageBuffer),(4+messageSize)*sizeof(unsigned short));
	
	/* Receive the reply message: */
	size_t replySize=0;
	while(replySize==0)
		{
		/* Wait for a reply: */
		usleep(1000);
		replySize=readControl(0x40,0x00,0x0000,0x0000,static_cast<unsigned char*>(replyBuffer),replyBufferSize);
		}
	
	/* Check the reply's magic number, command, and sequence number: */
	unsigned short* rb=static_cast<unsigned short*>(replyBuffer);
	if(rb[0]!=0x4252U||rb[2]!=messageBuffer[2]||rb[3]!=messageBuffer[3])
		Misc::throwStdErr("KinectCamera: Protocol error while sending message %u",(unsigned int)messageType);
	
	return replySize;
	}

bool KinectCamera::sendCommand(unsigned short command,unsigned short value)
	{
	/* Prepare a command message: */
	unsigned short commandBuffer[2];
	commandBuffer[0]=command;
	commandBuffer[1]=value;
	unsigned short replyBuffer[8];
	size_t replySize=sendMessage(0x0003U,commandBuffer,2,replyBuffer,sizeof(replyBuffer));
	
	/* Check for success message: */
	return replySize==5*sizeof(unsigned short)&&replyBuffer[1]==1&&replyBuffer[4]==0x0000U;
	}

namespace {

/***********************************
Helper functions for Bayer decoding:
***********************************/

inline unsigned char avg(unsigned char v1,unsigned char v2)
	{
	return (unsigned char)(((unsigned int)(v1)+(unsigned int)(v2)+1U)/2U);
	}

inline unsigned char avg(unsigned char v1,unsigned char v2,unsigned char v3)
	{
	return (unsigned char)(((unsigned int)(v1)+(unsigned int)(v2)+(unsigned int)(v3)+1U)/3U);
	}

inline unsigned char avg(unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4)
	{
	return (unsigned char)(((unsigned int)(v1)+(unsigned int)(v2)+(unsigned int)(v3)+(unsigned int)(v4)+2U)/4U);
	}

}

void* KinectCamera::colorDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	while(true)
		{
		/* Wait for the next color frame: */
		unsigned char* framePtr;
		{
		Threads::MutexCond::Lock frameReadyLock(colorStreamer->frameReadyCond);
		while(colorStreamer->readyFrame==0)
			colorStreamer->frameReadyCond.wait(frameReadyLock);
		framePtr=colorStreamer->readyFrame;
		colorStreamer->readyFrame=0;
		}
		
		/* Allocate a new decoded color buffer: */
		int width=colorStreamer->frameSize[0];
		int height=colorStreamer->frameSize[1];
		FrameBuffer decodedFrame(width,height,width*height*3*sizeof(unsigned char));
		
		/* Decode the raw color buffer (which is in Bayer GRBG pattern): */
		int stride=width;
		const unsigned char* rRowPtr=framePtr;
		unsigned char* cRowPtr=static_cast<unsigned char*>(decodedFrame.getBuffer());
		cRowPtr+=(height-1)*stride*3; // Flip the depth image vertically
		
		/* Convert the first row: */
		const unsigned char* rPtr=rRowPtr;
		unsigned char* cPtr=cRowPtr;
		
		/* Convert the first row's first (G) pixel: */
		*(cPtr++)=rPtr[1];
		*(cPtr++)=rPtr[0];
		*(cPtr++)=rPtr[stride];
		++rPtr;
		
		/* Convert the first row's central pixels: */
		for(unsigned int x=1;x<width-1;x+=2)
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
		for(unsigned int y=1;y<height-1;y+=2)
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
			for(unsigned x=1;x<width-1;x+=2)
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
			for(unsigned x=1;x<width-1;x+=2)
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
		for(unsigned int x=1;x<width-1;x+=2)
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
		(*colorStreamer->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

void* KinectCamera::depthDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	while(true)
		{
		/* Wait for the next depth frame: */
		unsigned char* framePtr;
		{
		Threads::MutexCond::Lock frameReadyLock(depthStreamer->frameReadyCond);
		while(depthStreamer->readyFrame==0)
			depthStreamer->frameReadyCond.wait(frameReadyLock);
		framePtr=depthStreamer->readyFrame;
		depthStreamer->readyFrame=0;
		}
		
		/* Allocate a new decoded depth buffer: */
		int width=depthStreamer->frameSize[0];
		int height=depthStreamer->frameSize[1];
		FrameBuffer decodedFrame(width,height,width*height*sizeof(unsigned short));
		
		/* Decode the raw depth buffer: */
		unsigned char* sPtr=framePtr;
		unsigned short* dRowPtr=static_cast<unsigned short*>(decodedFrame.getBuffer());
		dRowPtr+=width*(height-1);
		
		/* Process rows: */
		unsigned short* bfPtr=backgroundFrame;
		for(int y=0;y<height;++y,dRowPtr-=width) // Flip the depth image vertically
			{
			unsigned short* dPtr=dRowPtr;
			
			/* Process pixels in groups of eight: */
			for(int x=0;x<width;x+=8,sPtr+=11,dPtr+=8,bfPtr+=8)
				{
				/* Convert a run of 11 8-bit bytes into 8 11-bit pixels: */
				dPtr[0]=((unsigned short)sPtr[0]<<3)|((unsigned short)sPtr[1]>>5);
				dPtr[1]=(((unsigned short)sPtr[1]&0x1fU)<<6)|((unsigned short)sPtr[2]>>2);
				dPtr[2]=(((unsigned short)sPtr[2]&0x03U)<<9)|((unsigned short)sPtr[3]<<1)|((unsigned short)sPtr[4]>>7);
				dPtr[3]=(((unsigned short)sPtr[4]&0x7fU)<<4)|((unsigned short)sPtr[5]>>4);
				dPtr[4]=(((unsigned short)sPtr[5]&0x0fU)<<7)|((unsigned short)sPtr[6]>>1);
				dPtr[5]=(((unsigned short)sPtr[6]&0x01U)<<10)|((unsigned short)sPtr[7]<<2)|((unsigned short)sPtr[8]>>6);
				dPtr[6]=(((unsigned short)sPtr[8]&0x3fU)<<5)|((unsigned short)sPtr[9]>>3);
				dPtr[7]=(((unsigned short)sPtr[9]&0x07U)<<8)|(unsigned short)sPtr[10];
				
				/* Check if we're in the middle of capturing a background frame: */
				if(numBackgroundFrames>0)
					{
					/* Update the pixels' background depth values: */
					for(int i=0;i<8;++i)
						if(bfPtr[i]>dPtr[i])
							bfPtr[i]=dPtr[i];
					}
				
				if(removeBackground)
					{
					/* Remove background pixels: */
					for(int i=0;i<8;++i)
						if(dPtr[i]>=bfPtr[i])
							dPtr[i]=invalidDepth; // Mark the pixel as invalid
					}
				}
			}
		
		if(numBackgroundFrames>0)
			--numBackgroundFrames;
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*depthStreamer->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

KinectCamera::KinectCamera(USBContext& usbContext,size_t index)
	:messageSequenceNumber(0x2000U),
	 colorStreamer(0),depthStreamer(0),
	 numBackgroundFrames(0),backgroundFrame(0),removeBackground(false)
	{
	/* Get the index-th Kinect camera device from the context: */
	USBDeviceList deviceList(usbContext);
	USBDevice::operator=(deviceList.getDevice(0x045e,0x02ae,index));
	if(!isValid())
		Misc::throwStdErr("KinectCamera::KinectCamera: Less than %d Kinect camera devices detected",int(index));
	}

KinectCamera::~KinectCamera(void)
	{
	delete colorStreamer;
	delete depthStreamer;
	delete[] backgroundFrame;
	
	releaseInterface(0);
	// setConfiguration(1); // This seems to confuse the device
	
	// reset(); // This seems to confuse the device
	}

void KinectCamera::startStreaming(KinectCamera::StreamingCallback* newColorStreamingCallback,KinectCamera::StreamingCallback* newDepthStreamingCallback)
	{
	/* Open and prepare the device: */
	open();
	// setConfiguration(1); // This seems to confuse the device
	claimInterface(0,true);
	
	/* Check if caller wants to receive color frames: */
	if(newColorStreamingCallback!=0)
		{
		/* Create the color streaming state: */
		const int colorFrameSize[2]={640,480};
		size_t rawFrameSize=colorFrameSize[0]*colorFrameSize[1]; // Bayer pattern; one byte per pixel
		colorStreamer=new StreamingState(getDeviceHandle(),0x81U,0x80U,1920,colorFrameSize,rawFrameSize,newColorStreamingCallback);
		
		/* Start the color decoding thread: */
		colorStreamer->decodingThread.start(this,&KinectCamera::colorDecodingThreadMethod);
		}
	
	/* Check if caller wants to receive depth frames: */
	if(newDepthStreamingCallback!=0)
		{
		/* Create the depth streaming state: */
		const int depthFrameSize[2]={640,480};
		size_t rawFrameSize=(depthFrameSize[0]*depthFrameSize[1]*11+7)/8; // Packed bitstream; 11 bits per pixel
		depthStreamer=new StreamingState(getDeviceHandle(),0x82U,0x70U,1760,depthFrameSize,rawFrameSize,newDepthStreamingCallback);
		
		/* Start the depth decoding thread: */
		depthStreamer->decodingThread.start(this,&KinectCamera::depthDecodingThreadMethod);
		}
	
	/***********************************************************
	Query the device's serial number(?) and firmware version(?):
	***********************************************************/
	
	/* This seems to wake up the device: */
	unsigned short replyBuffer[64];
	size_t replySize=sendMessage(0x0000U,0,0,replyBuffer,sizeof(replyBuffer));
	
	replySize=sendMessage(0x0005U,0,0,replyBuffer,sizeof(replyBuffer));
	
	/**********************************************************
	Initialize the color and depth cameras and start streaming:
	**********************************************************/
	
	bool sequenceOk=true;
	sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0000U); // Disable color streaming
	sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0000U); // Disable depth streaming
	sequenceOk=sequenceOk&&sendCommand(0x000cU,0x0000U); // Request regular color frame format
	sequenceOk=sequenceOk&&sendCommand(0x000dU,0x0001U); // Request regular color frame format
	sequenceOk=sequenceOk&&sendCommand(0x000eU,0x001eU); // Request Bayer-encoded color images
	sequenceOk=sequenceOk&&sendCommand(0x0012U,0x0003U); // Request 11-bit depth images
	sequenceOk=sequenceOk&&sendCommand(0x0013U,0x0001U); // Request regular depth frame format
	sequenceOk=sequenceOk&&sendCommand(0x0014U,0x001eU); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0016U,0x0001U); // Enable depth smoothing
	sequenceOk=sequenceOk&&sendCommand(0x0018U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0002U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0105U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0024U,0x0001U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x002dU,0x0001U); // Unknown semantics
	
	if(depthStreamer!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0002U); // Enable depth streaming
		sequenceOk=sequenceOk&&sendCommand(0x0017U,0x0000U); // Request normal orientation of depth images
		}
	
	if(colorStreamer!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0001U); // Enable color streaming
		sequenceOk=sequenceOk&&sendCommand(0x0047U,0x0000U); // Request normal orientation of color images
		}
	
	if(!sequenceOk)
		Misc::throwStdErr("KinectCamera: Failed to initialize streaming mode");
	}

void KinectCamera::captureBackground(unsigned int newNumBackgroundFrames)
	{
	/* Bail out if there is no depth streamer: */
	if(depthStreamer==0)
		return;
	
	/* Initialize the background frame buffer: */
	if(backgroundFrame==0)
		backgroundFrame=new unsigned short[640*480];
	
	/* Initialize the background frame to "empty:" */
	unsigned short* bfPtr=backgroundFrame;
	for(unsigned int y=0;y<480;++y)
		for(unsigned int x=0;x<640;++x,++bfPtr)
			*bfPtr=invalidDepth;
	
	/* Start capturing background frames: */
	numBackgroundFrames=newNumBackgroundFrames;
	}

void KinectCamera::setRemoveBackground(bool newRemoveBackground)
	{
	/* Bail out if there is no background frame buffer: */
	if(backgroundFrame==0)
		return;
	
	/* Set the background removal flag: */
	removeBackground=newRemoveBackground;
	}

void KinectCamera::stopStreaming(void)
	{
	/* Send commands to stop streaming: */
	sendCommand(0x0005U,0x0000U); // Disable color streaming
	sendCommand(0x0006U,0x0000U); // Disable depth streaming (and turn off IR projector)
	
	/* Destroy the streaming states: */
	delete colorStreamer;
	colorStreamer=0;
	delete depthStreamer;
	depthStreamer=0;
	
	/* Destroy the background removal buffer: */
	numBackgroundFrames=0;
	delete[] backgroundFrame;
	backgroundFrame=0;
	removeBackground=false;
	}
