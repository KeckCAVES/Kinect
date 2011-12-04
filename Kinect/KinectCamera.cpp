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

#include <Kinect/KinectCamera.h>

#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Kinect/USBContext.h>
#include <Kinect/USBDeviceList.h>
#include <Kinect/FrameBuffer.h>

#define KINECT_DUMP_INIT 0
#define KINECTSTREAMER_USE_CAMERA_TIMESTAMP 0

/*********************************************
Methods of class KinectCamera::StreamingState:
*********************************************/

KinectCamera::StreamingState::StreamingState(libusb_device_handle* handle,unsigned int endpoint,Misc::Timer& sFrameTimer,double& sFrameTimerOffset,int sPacketFlagBase,int sPacketSize,const unsigned int sFrameSize[2],size_t sRawFrameSize,KinectCamera::StreamingCallback* sStreamingCallback)
	:frameTimer(sFrameTimer),frameTimerOffset(sFrameTimerOffset),
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
	#if 0
	decodingThread.cancel();
	#else
	{
	Threads::MutexCond::Lock frameReadyLock(frameReadyCond);
	cancelDecoding=true;
	frameReadyCond.signal();
	}
	#endif
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
			if(packetSize>=12&&packetPtr[0]==0x52U&&packetPtr[1]==0x42U)
				{
				// DEBUGGING
				//if(thisPtr->headerFile!=0)
				//	thisPtr->headerFile->write(packetPtr,12);
				
				/* Parse the packet header: */
				size_t payloadSize=packetSize-12*sizeof(unsigned char); // Each packet has a 12-byte header
				int packetType=packetPtr[3]-thisPtr->packetFlagBase;
				
				/* Check if this is the beginning of a new frame: */
				if(packetType==0x01)
					{
					/* Activate the next double buffer half: */
					thisPtr->activeBuffer=1-thisPtr->activeBuffer;
					thisPtr->writePtr=thisPtr->rawFrameBuffer+thisPtr->rawFrameSize*thisPtr->activeBuffer;
					thisPtr->bufferSpace=thisPtr->rawFrameSize;
					
					/* Time-stamp the new frame: */
					#if KINECTSTREAMER_USE_CAMERA_TIMESTAMP
					unsigned int timeStamp=(unsigned int)packetPtr[11];
					for(int j=10;j>=8;--j)
						timeStamp=(timeStamp<<8)|(unsigned int)packetPtr[j];
					thisPtr->activeFrameTimeStamp=double(timeStamp)/2000000.0;
					#else
					thisPtr->activeFrameTimeStamp=thisPtr->frameTimer.peekTime()+thisPtr->frameTimerOffset;
					#endif
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
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next color frame: */
		unsigned char* framePtr;
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
		FrameBuffer decodedFrame(width,height,width*height*3*sizeof(unsigned char));
		decodedFrame.timeStamp=frameTimeStamp;
		
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

void* KinectCamera::depthDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next depth frame: */
		unsigned char* framePtr;
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
		FrameBuffer decodedFrame(width,height,width*height*sizeof(unsigned short));
		decodedFrame.timeStamp=frameTimeStamp;
		
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
						if(dPtr[i]+backgroundRemovalFuzz>=bfPtr[i])
							dPtr[i]=invalidDepth; // Mark the pixel as really far away
					}
				}
			}
		
		if(numBackgroundFrames>0)
			{
			--numBackgroundFrames;
			
			/* Check if this was the last captured background frame, and whether to call a callback function: */
			if(numBackgroundFrames==0&&backgroundCaptureCallback!=0)
				{
				/* Call the callback: */
				(*backgroundCaptureCallback)(*this);
				
				/* Remove the callback object: */
				delete backgroundCaptureCallback;
				backgroundCaptureCallback=0;
				}
			}
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*streamers[DEPTH]->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

namespace {

inline unsigned int getNybble(unsigned char*& sPtr,bool& sFull)
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

void* KinectCamera::compressedDepthDecodingThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next depth frame: */
		unsigned char* framePtr;
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
		FrameBuffer decodedFrame(width,height,width*height*sizeof(unsigned short));
		decodedFrame.timeStamp=frameTimeStamp;
		
		/* Decode the raw depth buffer: */
		unsigned char* sPtr=framePtr;
		bool sFull=true;
		unsigned short* dRowPtr=static_cast<unsigned short*>(decodedFrame.getBuffer());
		dRowPtr+=width*(height-1);
		
		/* Process rows: */
		unsigned short* bfPtr=backgroundFrame;
		for(int y=0;y<height;++y,dRowPtr-=width) // Flip the depth image vertically
			{
			unsigned short* dPtr=dRowPtr;
			unsigned short* dEnd=dPtr+width;
			
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
						*dPtr=(unsigned short)(lastPixel);
						++dPtr;
						}
					else // It's a large-step differential
						{
						/* Read the rest of the differential: */
						value=(value<<4)|getNybble(sPtr,sFull);
						
						/* Calculate and store the depth value: */
						lastPixel=(lastPixel+value)-0xc0U;
						*dPtr=(unsigned short)lastPixel;
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
						*dPtr=(unsigned short)lastPixel;
						++dPtr;
						--numReps;
						}
					}
				else // It's a small-step differential
					{
					/* Calculate and store the depth value: */
					lastPixel=(lastPixel+code)-0x06U;
					*dPtr=(unsigned short)lastPixel;
					++dPtr;
					}
				}
			}
		
		/* Open the depth frame to fill holes: */
		FrameBuffer openedFrame(width,height,width*height*sizeof(unsigned short));
		openedFrame.timeStamp=frameTimeStamp;
		ptrdiff_t offsets[9];
		for(int y=0;y<3;++y)
			for(int x=0;x<3;++x)
				offsets[y*3+x]=(y-1)*width+(x-1);
		unsigned short* sRowPtr=static_cast<unsigned short*>(decodedFrame.getBuffer())+width;
		dRowPtr=static_cast<unsigned short*>(openedFrame.getBuffer())+width;
		for(int y=1;y<height-1;++y,sRowPtr+=width,dRowPtr+=width)
			{
			unsigned short* sPtr=sRowPtr+1;
			unsigned short* dPtr=dRowPtr+1;
			for(int x=1;x<width-1;++x,++sPtr,++dPtr)
				{
				if(*sPtr!=invalidDepth)
					*dPtr=*sPtr;
				else
					{
					/* Gather depth values from the surrounding pixels: */
					unsigned int sum=0;
					unsigned int num=0;
					for(int i=0;i<9;++i)
						if(sPtr[offsets[i]]!=invalidDepth)
							{
							sum+=sPtr[offsets[i]];
							++num;
							}
					
					/* Calculate the average depth value: */
					if(num!=0)
						*dPtr=(sum+(num/2))/num;
					else
						*dPtr=invalidDepth;
					}
				}
			}
		decodedFrame=openedFrame;
		
		/* Check if we're in the middle of capturing a background frame: */
		if(numBackgroundFrames>0)
			{
			/* Update the pixels' background depth values: */
			unsigned short* dPtr=static_cast<unsigned short*>(decodedFrame.getBuffer());
			bfPtr=backgroundFrame;
			for(int i=width*height;i>0;--i,++dPtr,++bfPtr)
				{
				if(*bfPtr>*dPtr)
					*bfPtr=*dPtr;
				}
			}
		
		/* Check if we're removing background: */
		if(removeBackground)
			{
			/* Remove background pixels: */
			unsigned short* dPtr=static_cast<unsigned short*>(decodedFrame.getBuffer());
			bfPtr=backgroundFrame;
			for(int i=width*height;i>0;--i,++dPtr,++bfPtr)
				{
				if(*dPtr+backgroundRemovalFuzz>=*bfPtr)
					*dPtr=invalidDepth; // Mark the pixel as invalid
				}
			}
		
		if(numBackgroundFrames>0)
			{
			--numBackgroundFrames;
			
			/* Check if this was the last captured background frame, and whether to call a callback function: */
			if(numBackgroundFrames==0&&backgroundCaptureCallback!=0)
				{
				/* Call the callback: */
				(*backgroundCaptureCallback)(*this);
				
				/* Remove the callback object: */
				delete backgroundCaptureCallback;
				backgroundCaptureCallback=0;
				}
			}
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*streamers[DEPTH]->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

KinectCamera::KinectCamera(libusb_device* sDevice)
	:USBDevice(sDevice),
	 messageSequenceNumber(0x2000U),
	 frameTimerOffset(0.0),
	 compressDepthFrames(true),
	 numBackgroundFrames(0),backgroundFrame(0),
	 backgroundCaptureCallback(0),
	 removeBackground(false),backgroundRemovalFuzz(5)
	{
	/* Initialize the camera and streamer states: */
	frameSizes[0]=FS_640_480;
	frameSizes[1]=FS_640_480;
	frameRates[0]=FR_30_HZ;
	frameRates[1]=FR_30_HZ;
	
	streamers[0]=0;
	streamers[1]=0;
	
	// DEBUGGING
	//headerFile=0;
	}

KinectCamera::KinectCamera(USBContext& usbContext,size_t index)
	:messageSequenceNumber(0x2000U),
	 frameTimerOffset(0.0),
	 compressDepthFrames(true),
	 numBackgroundFrames(0),backgroundFrame(0),
	 backgroundCaptureCallback(0),
	 removeBackground(false),backgroundRemovalFuzz(5)
	{
	/* Get the index-th Kinect camera device from the context: */
	USBDeviceList deviceList(usbContext);
	USBDevice::operator=(deviceList.getDevice(0x045e,0x02ae,index));
	if(!isValid())
		Misc::throwStdErr("KinectCamera::KinectCamera: Less than %d Kinect camera devices detected",int(index));
	
	/* Initialize the camera and streamer states: */
	frameSizes[0]=FS_640_480;
	frameSizes[1]=FS_640_480;
	frameRates[0]=FR_30_HZ;
	frameRates[1]=FR_30_HZ;
	
	streamers[0]=0;
	streamers[1]=0;
	
	// DEBUGGING
	//headerFile=0;
	}

KinectCamera::~KinectCamera(void)
	{
	delete streamers[0];
	delete streamers[1];
	delete[] backgroundFrame;
	delete backgroundCaptureCallback;
	
	releaseInterface(0);
	// setConfiguration(1); // This seems to confuse the device
	
	// reset(); // This seems to confuse the device
	
	// DEBUGGING
	//delete headerFile;
	}

void KinectCamera::setFrameSize(int camera,KinectCamera::FrameSize newFrameSize)
	{
	/* Set the camera's frame size: */
	frameSizes[camera]=newFrameSize;
	
	/* Limit the camera's frame rate to 15 Hz if high-res frames were selected: */
	if(newFrameSize==FS_1280_1024)
		frameRates[camera]=FR_15_HZ;
	}

const unsigned int* KinectCamera::getActualFrameSize(int camera) const
	{
	static const unsigned int actualFrameSizes[2][2]={{640,480},{1280,1024}};
	return actualFrameSizes[frameSizes[camera]];
	}

void KinectCamera::setFrameRate(int camera,KinectCamera::FrameRate newFrameRate)
	{
	/* Set the camera's frame rate: */
	frameRates[camera]=newFrameRate;
	
	/* Limit the camera's frame size to 640x480 if 30 Hz was selected: */
	if(newFrameRate==FR_30_HZ)
		frameSizes[camera]=FS_640_480;
	}

unsigned int KinectCamera::getActualFrameRate(int camera) const
	{
	static const unsigned int actualFrameRates[2]={15,30};
	return actualFrameRates[frameRates[camera]];
	}

void KinectCamera::resetFrameTimer(double newFrameTimerOffset)
	{
	frameTimer.elapse();
	frameTimerOffset=newFrameTimerOffset;
	}

void KinectCamera::setCompressDepthFrames(bool newCompressDepthFrames)
	{
	compressDepthFrames=newCompressDepthFrames; 
	}

void KinectCamera::startStreaming(KinectCamera::StreamingCallback* newColorStreamingCallback,KinectCamera::StreamingCallback* newDepthStreamingCallback)
	{
	/* Open and prepare the device: */
	open();
	// setConfiguration(1); // This seems to confuse the device
	claimInterface(0,true);
	
	/***********************************************************
	Query the device's serial number(?) and firmware version(?):
	***********************************************************/
	
	/* This seems to wake up the device: */
	unsigned short replyBuffer[64];
	size_t replySize=sendMessage(0x0000U,0,0,replyBuffer,sizeof(replyBuffer));
	
	replySize=sendMessage(0x0005U,0,0,replyBuffer,sizeof(replyBuffer));
	
	/***********************************
	Disable the color and depth cameras:
	***********************************/
	
	bool sequenceOk=true;
	sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0000U); // Disable color streaming
	sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0000U); // Disable depth streaming
	if(!sequenceOk)
		Misc::throwStdErr("KinectCamera: Failed to disable cameras");
	
	// DEBUGGING
	//delete headerFile;
	//std::string headerFileName="Headers-";
	//headerFileName.append(getSerialNumber());
	//headerFileName.append(".dat");
	//headerFile=new Misc::File(headerFileName.c_str(),"wb",Misc::File::LittleEndian);
	
	/* Check if caller wants to receive color frames: */
	if(newColorStreamingCallback!=0)
		{
		/* Create the color streaming state: */
		const unsigned int* colorFrameSize=getActualFrameSize(COLOR);
		size_t rawFrameSize=colorFrameSize[0]*colorFrameSize[1]; // Bayer pattern; one byte per pixel
		streamers[COLOR]=new StreamingState(getDeviceHandle(),0x81U,frameTimer,frameTimerOffset,0x80U,1920,colorFrameSize,rawFrameSize,newColorStreamingCallback);
		
		// DEBUGGING
		//streamers[COLOR]->headerFile=headerFile;
		
		/* Start the color decoding thread: */
		streamers[COLOR]->decodingThread.start(this,&KinectCamera::colorDecodingThreadMethod);
		}
	
	/* Check if caller wants to receive depth frames: */
	if(newDepthStreamingCallback!=0)
		{
		/* Create the depth streaming state: */
		const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
		size_t rawFrameSize=(depthFrameSize[0]*depthFrameSize[1]*11+7)/8; // Packed bitstream; 11 bits per pixel
		streamers[DEPTH]=new StreamingState(getDeviceHandle(),0x82U,frameTimer,frameTimerOffset,0x70U,1760,depthFrameSize,rawFrameSize,newDepthStreamingCallback);
		
		// DEBUGGING
		//streamers[DEPTH]->headerFile=headerFile;
		
		/* Start the depth decoding thread: */
		if(compressDepthFrames)
			streamers[DEPTH]->decodingThread.start(this,&KinectCamera::compressedDepthDecodingThreadMethod);
		else
			streamers[DEPTH]->decodingThread.start(this,&KinectCamera::depthDecodingThreadMethod);
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
	sequenceOk=sequenceOk&&sendCommand(0x0013U,0x0001U); // Request 640x480 depth images
	sequenceOk=sequenceOk&&sendCommand(0x0014U,0x001eU); // Request 30 Hz depth image frame rate
	sequenceOk=sequenceOk&&sendCommand(0x0016U,0x0001U); // Enable depth smoothing
	sequenceOk=sequenceOk&&sendCommand(0x0018U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0002U,0x0000U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x0105U,0x0000U); // Disable IR pattern checking
	sequenceOk=sequenceOk&&sendCommand(0x0024U,0x0001U); // Unknown semantics
	sequenceOk=sequenceOk&&sendCommand(0x002dU,0x0001U); // Unknown semantics
	
	if(streamers[DEPTH]!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0006U,0x0002U); // Enable depth streaming
		sequenceOk=sequenceOk&&sendCommand(0x0017U,0x0000U); // Request normal orientation of depth images
		}
	
	if(streamers[COLOR]!=0)
		{
		sequenceOk=sequenceOk&&sendCommand(0x0005U,0x0001U); // Enable color streaming
		sequenceOk=sequenceOk&&sendCommand(0x0047U,0x0000U); // Request normal orientation of color images
		}
	
	if(!sequenceOk)
		Misc::throwStdErr("KinectCamera: Failed to initialize streaming mode");
	}

void KinectCamera::captureBackground(unsigned int newNumBackgroundFrames,bool replace,KinectCamera::BackgroundCaptureCallback* newBackgroundCaptureCallback)
	{
	/* Remember the background capture callback: */
	delete backgroundCaptureCallback;
	backgroundCaptureCallback=newBackgroundCaptureCallback;
	
	/* Initialize the background frame buffer: */
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(backgroundFrame==0)
		{
		backgroundFrame=new unsigned short[depthFrameSize[0]*depthFrameSize[1]];
		replace=true;
		}
	
	if(replace)
		{
		/* Initialize the background frame to "empty:" */
		unsigned short* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				*bfPtr=invalidDepth;
		}
	
	/* Start capturing background frames: */
	numBackgroundFrames=newNumBackgroundFrames;
	}

void KinectCamera::loadBackground(const char* fileNamePrefix)
	{
	/* Construct the full background file name: */
	std::string fileName=fileNamePrefix;
	fileName.push_back('-');
	fileName.append(getSerialNumber());
	fileName.append(".background");
	
	/* Open and read the background file: */
	IO::FilePtr file(IO::openFile(fileName.c_str()));
	loadBackground(*file);
	}

void KinectCamera::loadBackground(IO::File& file)
	{
	/* Set the file's endianness: */
	file.setEndianness(Misc::LittleEndian);
	
	/* Read the frame header: */
	unsigned int fileFrameSize[2];
	file.read<unsigned int>(fileFrameSize,2);
	
	/* Check if the file matches the current depth buffer size: */
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(fileFrameSize[0]!=depthFrameSize[0]||fileFrameSize[1]!=depthFrameSize[1])
		Misc::throwStdErr("KinectCamera::loadBackground: Background file's size does not match camera's");
	
	/* Create the background frame buffer: */
	if(backgroundFrame==0)
		backgroundFrame=new unsigned short[depthFrameSize[0]*depthFrameSize[1]];
	
	/* Read the background file: */
	file.read<unsigned short>(backgroundFrame,depthFrameSize[0]*depthFrameSize[1]);
	}

void KinectCamera::setMaxDepth(unsigned int newMaxDepth,bool replace)
	{
	/* Limit the depth value to the valid range: */
	if(newMaxDepth>invalidDepth)
		newMaxDepth=invalidDepth;
	unsigned short nmd=(unsigned short)(newMaxDepth);
	
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(backgroundFrame==0)
		{
		/* Create the background frame buffer: */
		backgroundFrame=new unsigned short[depthFrameSize[0]*depthFrameSize[1]];
		replace=true;
		}
	
	if(replace)
		{
		/* Initialize the background frame to the max depth value */
		unsigned short* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				*bfPtr=nmd;
		}
	else
		{
		/* Modify the existing background frame buffer: */
		unsigned short* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				if(*bfPtr>nmd)
					*bfPtr=nmd;
		}
	}

void KinectCamera::saveBackground(const char* fileNamePrefix)
	{
	/* Bail out if there is no background frame: */
	if(backgroundFrame==0)
		return;
	
	/* Construct the full background file name: */
	std::string fileName=fileNamePrefix;
	fileName.push_back('-');
	fileName.append(getSerialNumber());
	fileName.append(".background");
	
	/* Save the background file: */
	IO::FilePtr file(IO::openFile(fileName.c_str(),IO::File::WriteOnly));
	file->setEndianness(Misc::LittleEndian);
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	file->write<unsigned int>(depthFrameSize,2);
	file->write<unsigned short>(backgroundFrame,depthFrameSize[0]*depthFrameSize[1]);
	}

void KinectCamera::setRemoveBackground(bool newRemoveBackground)
	{
	/* Bail out if there is no background frame buffer: */
	if(backgroundFrame==0)
		return;
	
	/* Set the background removal flag: */
	removeBackground=newRemoveBackground;
	}

void KinectCamera::setBackgroundRemovalFuzz(int newBackgroundRemovalFuzz)
	{
	backgroundRemovalFuzz=(short int)(newBackgroundRemovalFuzz);
	}

void KinectCamera::stopStreaming(void)
	{
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
	numBackgroundFrames=0;
	delete[] backgroundFrame;
	backgroundFrame=0;
	removeBackground=false;
	}
