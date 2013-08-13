/***********************************************************************
LossyDepthFrameReader - Class to read lossily compressed depth frames
from a source.
Copyright (c) 2013 Oliver Kreylos

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

#include <Kinect/LossyDepthFrameReader.h>

#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Math/Constants.h>
#include <Video/Config.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Video/Colorspaces.h>
#include <Video/TheoraFrame.h>
#include <Video/TheoraInfo.h>
#include <Video/TheoraComment.h>
#include <Video/TheoraPacket.h>
#endif
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

/**************************************
Methods of class LossyDepthFrameReader:
**************************************/

LossyDepthFrameReader::LossyDepthFrameReader(IO::File& sSource)
	:source(sSource),
	 sourceHasTheora(false)
	{
	/* Read the frame size from the source: */
	for(int i=0;i<2;++i)
		size[i]=source.read<Misc::UInt32>();
	
	/* Read the stream header's size: */
	size_t streamHeaderSize=source.read<Misc::UInt32>();
	sourceHasTheora=streamHeaderSize>0;
	
	if(sourceHasTheora)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		
		/* Create the setup structures: */
		Video::TheoraInfo theoraInfo;
		Video::TheoraComment theoraComments;
		Video::TheoraDecoder::Setup theoraSetup;
		
		/* Read and process all stream header packets: */
		while(streamHeaderSize>0)
			{
			/* Read and process the next header packet: */
			Video::TheoraPacket packet;
			packet.read(source);
			Video::TheoraDecoder::processHeader(packet,theoraInfo,theoraComments,theoraSetup);
			streamHeaderSize-=packet.getWireSize();
			}
		
		/* Initialize the Theora decoder: */
		theoraDecoder.init(theoraInfo,theoraSetup);
		
		#else
		
		/* Skip the stream header packets: */
		source.skip<Misc::UInt8>(streamHeaderSize);
		
		#endif
		}
	}

LossyDepthFrameReader::~LossyDepthFrameReader(void)
	{
	}

FrameBuffer LossyDepthFrameReader::readNextFrame(void)
	{
	/* Create the result frame: */
	FrameBuffer result(size[0],size[1],size[1]*size[0]*sizeof(FrameSource::DepthPixel));
	
	/* Return a dummy frame if the file is over: */
	if(source.eof())
		{
		result.timeStamp=Math::Constants<double>::max;
		return result;
		}
	
	/* Read the frame's time stamp from the source: */
	result.timeStamp=source.read<Misc::Float64>();
	
	if(sourceHasTheora)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		
		/* Read and process a single Theora packet: */
		{
		/* Read and process the next packet: */
		Video::TheoraPacket packet;
		packet.read(source);
		
		theoraDecoder.processPacket(packet);
		}
		
		/* Extract the decompressed frame: */
		Video::TheoraFrame theoraFrame;
		theoraDecoder.decodeFrame(theoraFrame);
		
		/* Convert the decompressed frame from Y'CbCr 4:2:0 to 11-bit depth: */
		FrameSource::DepthPixel* resultRowPtr=static_cast<FrameSource::DepthPixel*>(result.getBuffer());
		const unsigned char* ypRowPtr=static_cast<const unsigned char*>(theoraFrame.planes[0].data)+theoraFrame.offsets[0];
		const unsigned char* cbRowPtr=static_cast<const unsigned char*>(theoraFrame.planes[1].data)+theoraFrame.offsets[1];
		const unsigned char* crRowPtr=static_cast<const unsigned char*>(theoraFrame.planes[2].data)+theoraFrame.offsets[2];
		for(unsigned int y=0;y<size[1];y+=2)
			{
			FrameSource::DepthPixel* resultPtr=resultRowPtr;
			const unsigned char* ypPtr=ypRowPtr;
			const unsigned char* cbPtr=cbRowPtr;
			const unsigned char* crPtr=crRowPtr;
			for(unsigned int x=0;x<size[0];x+=2)
				{
				/* Assemble the block's 4 11-bit depth values from the 4 8-bit yp values and the 8-bit cb and cr values: */
				resultPtr[0]=((FrameSource::DepthPixel(ypPtr[0])<<3)&0x7f8U)|(FrameSource::DepthPixel(*cbPtr)>>4);
				resultPtr[1]=((FrameSource::DepthPixel(ypPtr[1])<<3)&0x7f8U)|(FrameSource::DepthPixel(*cbPtr)&0x0fU);
				resultPtr[size[0]]=((FrameSource::DepthPixel(ypPtr[theoraFrame.planes[0].stride])<<3)&0x7f8U)|(FrameSource::DepthPixel(*cbPtr)>>4);
				resultPtr[size[0]+1]=((FrameSource::DepthPixel(ypPtr[theoraFrame.planes[0].stride+1])<<3)&0x7f8U)|(FrameSource::DepthPixel(*cbPtr)&0x0fU);
				
				/* Go to the next pixel block: */
				resultPtr+=2;
				ypPtr+=2;
				++cbPtr;
				++crPtr;
				}
			
			/* Go to the next pixel block row: */
			resultRowPtr+=size[0]*2;
			ypRowPtr+=theoraFrame.planes[0].stride*2;
			cbRowPtr+=theoraFrame.planes[1].stride;
			crRowPtr+=theoraFrame.planes[2].stride;
			}
		
		#else
		
		/**********************
		Skip one Theora packet:
		**********************/
		
		/* Skip packet flags: */
		source.skip<char>(1);
		
		/* Skip 64-bit granule position and packet number: */
		source.skip<Misc::SInt64>(2);
		
		/* Read the packet data size: */
		size_t packetSize=source.read<unsigned int>();
		source.skip<Misc::UInt8>(packetSize);
		
		/* Initialize the frame to all invalid pixels: */
		FrameSource::DepthPixel* resultPtr=static_cast<FrameSource::DepthPixel*>(result.getBuffer());
		for(unsigned int y=0;y<size[1];++y)
			for(unsigned int x=0;x<size[0];++x,++resultPtr)
				*resultPtr=FrameSource::invalidDepth;
		
		#endif
		}
	else
		{
		/* Initialize the frame to all invalid pixels: */
		FrameSource::DepthPixel* resultPtr=static_cast<FrameSource::DepthPixel*>(result.getBuffer());
		for(unsigned int y=0;y<size[1];++y)
			for(unsigned int x=0;x<size[0];++x,++resultPtr)
				*resultPtr=FrameSource::invalidDepth;
		}
	
	return result;
	}

}
