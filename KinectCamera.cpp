/***********************************************************************
KinectCamera - Wrapper class to represent the color and depth camera
interface aspects of the Kinect sensor.
Copyright (c) 2010 Oliver Kreylos
***********************************************************************/

#include "KinectCamera.h"

#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <Misc/FileCharacterSource.h>
#include <Misc/ValueSource.h>

#include "USBContext.h"
#include "USBDeviceList.h"
#include "FrameBuffer.h"
#include "SimpleReadBuffer.h"
#include "SimpleWriteBuffer.h"

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

inline unsigned int readUInt(Misc::ValueSource& source)
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

inline size_t readUCharBuffer(Misc::ValueSource& source,unsigned char* buffer,size_t bufferSize)
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

void KinectCamera::chant(Misc::ValueSource& chantSource)
	{
	/* Parse the incantation: */
	if(chantSource.peekc()=='#')
		{
		/* Ignore comment lines: */
		chantSource.skipLine();
		chantSource.skipWs();
		return;
		}
	
	/* Read the command: */
	unsigned int command=readUInt(chantSource);
	
	/* Check for comma: */
	if(chantSource.peekc()!=',')
		Misc::throwStdErr("KinectCamera::chant: Missing comma in incantation");
	chantSource.readChar();
	
	/* Read the command value: */
	unsigned int value=readUInt(chantSource);
	
	/* Check for comma: */
	if(chantSource.peekc()!=',')
		Misc::throwStdErr("KinectCamera::chant: Missing comma in incantation");
	chantSource.readChar();
	
	/* Read the chant call: */
	unsigned char call[1024];
	size_t callSize=readUCharBuffer(chantSource,call,sizeof(call));
	if(callSize>sizeof(call))
		Misc::throwStdErr("KinectCamera::chant: Buffer overflow in incantation");
	if(callSize&0x1)
		Misc::throwStdErr("KinectCamera::chant: Odd call size in incantation");
	
	/* Check for comma: */
	if(chantSource.peekc()!=',')
		Misc::throwStdErr("KinectCamera::chant: Missing comma in incantation");
	chantSource.readChar();
	
	/* Read the chant reply: */
	unsigned char reply[1024];
	size_t replySize=readUCharBuffer(chantSource,reply,sizeof(reply));
	if(replySize>sizeof(reply))
		Misc::throwStdErr("KinectCamera::chant: Buffer overflow in incantation");
	if(replySize&0x1)
		Misc::throwStdErr("KinectCamera::chant: Odd reply size in incantation");
	
	/* Check for newline: */
	if(chantSource.peekc()!='\n')
		Misc::throwStdErr("KinectCamera::chant: Overlong line in incantation");
	chantSource.skipLine();
	chantSource.skipWs();
	
	/* Create a little-endian write buffer to marshal data to the device: */
	SimpleWriteBuffer buffer(4096);
	
	/* Write the magic number: */
	buffer.write<unsigned short>(0x4d47U);
	
	/* Write the call length: */
	buffer.write<unsigned short>(callSize/2); // Calls are actually composed of unsigned shorts, not unsigned bytes
	
	/* Write the command and value: */
	buffer.write<unsigned short>(command);
	buffer.write<unsigned short>(value);
	
	/* Write the call body: */
	buffer.write<unsigned char>(call,callSize);
	
	/* Send the call to the device: */
	std::cout<<"Chanting call: "<<command<<", "<<value<<", "<<callSize<<", "<<replySize<<"; size "<<buffer.getDataSize()<<std::endl;
	writeControl(0x40,0x00,0x0000,0x0000,buffer.getData(),buffer.getDataSize());
	
	/* Read the reply from the device: */
	SimpleReadBuffer read(4096);
	std::cout<<"Listening for reply"<<std::flush;
	do
		{
		usleep(1000);
		std::cout<<'.'<<std::flush;
		read.fill(readControl(0x40,0x00,0x0000,0x0000,read.getBuffer(),read.getBufferSize()));
		}
	while(read.getDataSize()==0);
	std::cout<<"got "<<read.getDataSize()<<std::endl;
	
	/* Read the reply magic number, reply length, command, and value: */
	unsigned int replyMagic=read.read<unsigned short>();
	unsigned int replyLen=read.read<unsigned short>();
	unsigned int replyCommand=read.read<unsigned short>();
	unsigned int replyValue=read.read<unsigned short>();
	
	/* Check the reply: */
	bool replyOk=replyMagic==0x4252U;
	replyOk=replyOk&&replyLen*2==read.getDataSize()&&replyLen*2==replySize;
	replyOk=replyOk&&replyCommand==command;
	replyOk=replyOk&&replyValue==value;
	for(size_t i=0;i<replySize&&replyOk;++i)
		replyOk=reply[i]==read.read<unsigned char>();
	if(!replyOk)
		Misc::throwStdErr("KinectCamera::chant: Bad reply in incantation; %u, %u, %u, %u",replyMagic,replyLen*2,replyCommand,replyValue);
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
		cRowPtr+=(height-1)*stride*3;
		
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
		for(int y=0;y<height;++y,dRowPtr-=width)
			{
			unsigned short* dPtr=dRowPtr;
			
			/* Process pixels in groups of eight: */
			for(int x=0;x<width;x+=8,sPtr+=11,dPtr+=8)
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
				
				/* Replace invalid depths with zero: */
				for(int j=0;j<8;++j)
					if(dPtr[j]==0x07ffU)
						dPtr[j]=0x0U;
				}
			}
		
		/* Pass the decoded depth buffer to the streaming callback function: */
		(*depthStreamer->streamingCallback)(decodedFrame);
		}
	
	return 0;
	}

KinectCamera::KinectCamera(USBContext& usbContext,size_t index)
	:colorStreamer(0),depthStreamer(0)
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
	
	releaseInterface(0);
	// setConfiguration(1);
	
	// reset();
	}

void KinectCamera::startStreaming(KinectCamera::StreamingCallback* newColorStreamingCallback,KinectCamera::StreamingCallback* newDepthStreamingCallback)
	{
	/* Open and prepare the device: */
	open();
	// setConfiguration(1);
	claimInterface(0);
	
	/* Check if caller wants to receive color frames: */
	if(newColorStreamingCallback!=0)
		{
		/* Create the color streaming state: */
		const int colorFrameSize[2]={640,480};
		colorStreamer=new StreamingState(getDeviceHandle(),0x81U,0x80U,1920,colorFrameSize,307200,newColorStreamingCallback);
		
		/* Start the color decoding thread: */
		colorStreamer->decodingThread.start(this,&KinectCamera::colorDecodingThreadMethod);
		}
	
	/* Check if caller wants to receive depth frames: */
	if(newDepthStreamingCallback!=0)
		{
		/* Create the depth streaming state: */
		const int depthFrameSize[2]={640,480};
		depthStreamer=new StreamingState(getDeviceHandle(),0x82U,0x70U,1760,depthFrameSize,422400,newDepthStreamingCallback);
		
		/* Start the depth decoding thread: */
		depthStreamer->decodingThread.start(this,&KinectCamera::depthDecodingThreadMethod);
		}
	
	/********************************************************************
	Send the magic incantations to start color and depth video capture on
	the device:
	********************************************************************/
	
	/* Read and send the magic chant file: */
	Misc::FileCharacterSource chantSourceFile("MagicIncantation.txt");
	Misc::ValueSource chantSource(chantSourceFile);
	chantSource.setPunctuation("#,\n");
	chantSource.skipWs();
	initHexDigits();
	while(!chantSource.eof())
		chant(chantSource);
	std::cout<<"Chant completed!"<<std::endl;
	}

void KinectCamera::stopStreaming(void)
	{
	/* Destroy the streaming states: */
	delete colorStreamer;
	colorStreamer=0;
	delete depthStreamer;
	depthStreamer=0;
	}
