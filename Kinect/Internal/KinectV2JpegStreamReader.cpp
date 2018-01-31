/***********************************************************************
KinectV2JpegStreamReader - Class to read JPEG-compressed RGB images
asynchronously from a stream of USB transfer buffers.
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

#include <Kinect/Internal/KinectV2JpegStreamReader.h>

#include <libusb-1.0/libusb.h>
#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <Misc/MessageLogger.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/CameraV2.h>

// DEBUGGING
#include <iostream>

namespace Kinect {

/*****************************************
Methods of class KinectV2JpegStreamReader:
*****************************************/

void KinectV2JpegStreamReader::getNextTransfer(void)
	{
	/* Keep grabbing transfer buffers until a non-empty one is found: */
	do
		{
		/* Release the current transfer buffer if there is one: */
		if(currentTransfer!=0)
			transferPool->release(currentTransfer);
		
		/* Wait for the next transfer buffer: */
		{
		Threads::MutexCond::Lock inQueueLock(inQueueCond);
		
		/* Block while the input queue is empty: */
		while(inQueue.empty())
			inQueueCond.wait(inQueueLock);
		
		/* Get and remove the first transfer from the input queue: */
		currentTransfer=inQueue.pop_front();
		}
		}
	while(currentTransfer->getTransfer().actual_length==0);
	
	/* Set the JPEG decompressor's input buffer to the next transfer's data: */
	sourceManager.bytes_in_buffer=currentTransfer->getTransfer().actual_length;
	sourceManager.next_input_byte=currentTransfer->getTransfer().buffer;
	frameSize+=currentTransfer->getTransfer().actual_length;
	}

void KinectV2JpegStreamReader::errorExitFunction(j_common_ptr cinfo)
	{
	/* Mark the current image as invalid: */
	static_cast<KinectV2JpegStreamReader*>(cinfo->client_data)->error=true;
	
	/* Log an error message: */
	jpeg_error_mgr* err=cinfo->err;
	std::string errorMessage="KinectV2JpegStreamReader: ";
	errorMessage.append(err->jpeg_message_table[err->msg_code]);
	Misc::formattedConsoleError(errorMessage.c_str(),err->msg_parm.i[0],err->msg_parm.i[1],err->msg_parm.i[2],err->msg_parm.i[3],err->msg_parm.i[4],err->msg_parm.i[5],err->msg_parm.i[6],err->msg_parm.i[7]);
	}

void KinectV2JpegStreamReader::initSourceFunction(j_decompress_ptr cinfo)
	{
	/* Nothing to do */
	}

boolean KinectV2JpegStreamReader::fillInputBufferFunction(j_decompress_ptr cinfo)
	{
	KinectV2JpegStreamReader* thisPtr=static_cast<KinectV2JpegStreamReader*>(cinfo->client_data);
	
	/* Get the next transfer buffer: */
	thisPtr->getNextTransfer();
	
	return true;
	}

void KinectV2JpegStreamReader::skipInputDataFunction(j_decompress_ptr cinfo,long count)
	{
	KinectV2JpegStreamReader* thisPtr=static_cast<KinectV2JpegStreamReader*>(cinfo->client_data);
	
	if(count<0)
		throw std::runtime_error("KinectV2JpegStreamReader: Unable to skip backwards");
	size_t skip=size_t(count);
	
	while(skip>0U)
		{
		/* Skip inside the current transfer: */
		size_t skipStep=thisPtr->sourceManager.bytes_in_buffer;
		if(skipStep>skip)
			skipStep=skip;
		thisPtr->sourceManager.bytes_in_buffer-=skipStep;
		thisPtr->sourceManager.next_input_byte+=skipStep;
		skip-=skipStep;
		
		/* Get the next transfer if the current transfer is finished: */
		if(thisPtr->sourceManager.bytes_in_buffer==0U)
			thisPtr->getNextTransfer();
		}
	}

void KinectV2JpegStreamReader::termSourceFunction(j_decompress_ptr cinfo)
	{
	/* Nothing to do */
	}

void* KinectV2JpegStreamReader::decompressionThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	// unsigned int frameIndex=0;
	FrameSource::Time now;
	while(true)
		{
		while(true)
			{
			/* Wait for the transfer buffer starting a new image: */
			getNextTransfer();
			
			/* Sample the real-time clock: */
			now.set();
			
			/* Check the magic number and the JPEG header: */
			const Misc::UInt32* rpPtr=reinterpret_cast<const Misc::UInt32*>(sourceManager.next_input_byte);
			const unsigned char* jPtr=reinterpret_cast<const unsigned char*>(rpPtr+2);
			if(rpPtr[1]==0x42424242U&&
			   jPtr[0]==0xffU&&jPtr[1]==0xd8U)
				break;
			
			/* Skip until the end of the current batch of transfers: */
			size_t skip=currentTransfer->getTransfer().actual_length;
			do
				{
				getNextTransfer();
				skip+=currentTransfer->getTransfer().actual_length;
				}
			while(currentTransfer->getTransfer().actual_length==currentTransfer->getTransfer().length);
			}
		
		/* Shave the Kinect2 image header off the first transfer buffer: */
		// const Misc::UInt32* header=reinterpret_cast<const Misc::UInt32*>(sourceManager.next_input_byte);
		// unsigned int frameNumber=header[0];
		// unsigned int magic0=header[1];
		sourceManager.bytes_in_buffer-=2*sizeof(Misc::UInt32);
		sourceManager.next_input_byte+=2*sizeof(Misc::UInt32);
		
		/* Let the decompressor read the JPEG headers and prepare for decompression: */
		frameSize=sourceManager.bytes_in_buffer;
		error=false;
		jpeg_read_header(&decompressor,true);
		decompressor.dct_method=JDCT_FASTEST;
		decompressor.do_fancy_upsampling=false;
		decompressor.do_block_smoothing=false;
		
		// DEBUGGING
		// std::cout<<"Image's color space: "<<decompressor.jpeg_color_space<<", selected output color space: "<<decompressor.out_color_space<<std::endl;
		
		jpeg_start_decompress(&decompressor);
		
		/* Create a frame buffer to hold the decompressed image: */
		FrameBuffer decompressedFrame(decompressor.output_width,decompressor.output_height,decompressor.output_height*decompressor.output_width*sizeof(FrameSource::ColorPixel));
		
		/*************************************************************
		This is where we would synchronize clocks to account for
		random OS delays, subtract expected hardware latency, etc. pp.
		*************************************************************/
		
		/* Time-stamp the new frame: */
		decompressedFrame.timeStamp=double(now-camera.timeBase);
		
		/* Subtract approximate color image capture latency: */
		decompressedFrame.timeStamp-=0.090;
		
		/* Create row pointers to flip the image during reading: */
		if(imageHeight!=decompressedFrame.getSize(1))
			{
			delete[] imageRowPointers;
			imageHeight=decompressedFrame.getSize(1);
			imageRowPointers=new FrameSource::ColorPixel*[imageHeight];
			}
		FrameSource::ColorPixel* rowPtr=decompressedFrame.getData<FrameSource::ColorPixel>()+(imageHeight-1)*decompressedFrame.getSize(0);
		for(int y=0;y<imageHeight;++y,rowPtr-=decompressedFrame.getSize(0))
			imageRowPointers[y]=rowPtr;
		
		/* Decompress all pixel rows in the result image: */
		JDIMENSION scanline=0;
		while(scanline<decompressor.output_height&&!error)
			scanline+=jpeg_read_scanlines(&decompressor,reinterpret_cast<JSAMPLE**>(imageRowPointers+scanline),decompressor.output_height-scanline);
		
		/* Finish decompressing and release the current transfer: */
		if(error)
			jpeg_abort_decompress(&decompressor);
		else
			jpeg_finish_decompress(&decompressor);
		transferPool->release(currentTransfer);
		currentTransfer=0;
		sourceManager.bytes_in_buffer=0;
		sourceManager.next_input_byte=0;
		
		if(!error)
			{
			/* Call the callback: */
			(*imageReadyCallback)(decompressedFrame);
			}
		}
	
	return 0;
	}

KinectV2JpegStreamReader::KinectV2JpegStreamReader(CameraV2& sCamera)
	:camera(sCamera),
	 transferPool(0),currentTransfer(0),
	 imageHeight(0),imageRowPointers(0),
	 imageReadyCallback(0)
	{
	/* Initialize the JPEG error manager: */
	jpeg_std_error(&errorManager);
	errorManager.error_exit=errorExitFunction;
	
	/* Initialize the JPEG source manager: */
	sourceManager.init_source=initSourceFunction;
	sourceManager.fill_input_buffer=fillInputBufferFunction;
	sourceManager.skip_input_data=skipInputDataFunction;
	sourceManager.resync_to_restart=jpeg_resync_to_restart; // Use default function
	sourceManager.term_source=termSourceFunction;
	
	/* Clear the input buffer: */
	sourceManager.bytes_in_buffer=0;
	sourceManager.next_input_byte=0;
	
	/* Initialize the JPEG decompressor: */
	decompressor.err=&errorManager;
	jpeg_create_decompress(&decompressor);
	decompressor.src=&sourceManager;
	decompressor.client_data=this;
	}

KinectV2JpegStreamReader::~KinectV2JpegStreamReader(void)
	{
	/* Stop streaming if necessary: */
	if(transferPool!=0)
		stopStreaming();
	
	/* Destroy the JPEG decompressor: */
	delete[] imageRowPointers;
	jpeg_destroy_decompress(&decompressor);
	}

USB::TransferPool::UserTransferCallback*  KinectV2JpegStreamReader::startStreaming(USB::TransferPool* newTransferPool,KinectV2JpegStreamReader::ImageReadyCallback* newImageReadyCallback)
	{
	/* Remember the source transfer pool: */
	transferPool=newTransferPool;
	
	/* Install the callback function: */
	delete imageReadyCallback;
	imageReadyCallback=newImageReadyCallback;
	
	/* Start the background decompression thread: */
	decompressionThread.start(this,&KinectV2JpegStreamReader::decompressionThreadMethod);
	
	/* Create and return a transfer callback: */
	return Misc::createFunctionCall(this,&KinectV2JpegStreamReader::postTransfer,newTransferPool);
	}

void KinectV2JpegStreamReader::stopStreaming(void)
	{
	/* Shut down the decompressor thread: */
	decompressionThread.cancel();
	decompressionThread.join();
	
	/* Release all remaining queued transfers: */
	while(!inQueue.empty())
		transferPool->release(inQueue.pop_front());
	transferPool=0;
	
	/* Delete the callback function: */
	delete imageReadyCallback;
	imageReadyCallback=0;
	}

}
