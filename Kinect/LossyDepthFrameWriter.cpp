/***********************************************************************
LossyDepthFrameWriter - Class to write lossily compressed depth frames
to a sink.
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

#include <Kinect/LossyDepthFrameWriter.h>

#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <IO/File.h>
#include <IO/VariableMemoryFile.h>
#include <Video/Config.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Video/FrameBuffer.h>
#include <Video/OggPage.h>
#include <Video/TheoraInfo.h>
#include <Video/TheoraComment.h>
#endif
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

/**************************************
Methods of class LossyDepthFrameWriter:
**************************************/

LossyDepthFrameWriter::LossyDepthFrameWriter(IO::File& sSink,const unsigned int sSize[2])
	:FrameWriter(sSize),
	 sink(sSink)
	{
	/* Write the frame size to the sink: */
	for(int i=0;i<2;++i)
		sink.write<Misc::UInt32>(size[i]);
	
	#if VIDEO_CONFIG_HAVE_THEORA
	
	/* Initialize the Theora encoder: */
	Video::TheoraInfo theoraInfo;
	theoraInfo.setImageSize(size);
	theoraInfo.colorspace=TH_CS_UNSPECIFIED;
	theoraInfo.pixel_fmt=TH_PF_420;
	theoraInfo.target_bitrate=0;
	theoraInfo.quality=48;
	theoraInfo.setGopSize(64);
	theoraInfo.fps_numerator=30;
	theoraInfo.fps_denominator=1;
	theoraInfo.aspect_numerator=1;
	theoraInfo.aspect_denominator=1;
	theoraEncoder.init(theoraInfo);
	if(!theoraEncoder.isValid())
		Misc::throwStdErr("LossyDepthFrameWriter::LossyDepthFrameWriter: Error initializing Theora encoder");
	
	/* Set the encoder to maximum speed: */
	theoraEncoder.setSpeedLevel(theoraEncoder.getMaxSpeedLevel());
	
	/* Create the frame converter structure: */
	theoraFrame.init420(theoraInfo);
	
	/* Set up a comment structure: */
	Video::TheoraComment comments;
	comments.setVendorString("Kinect lossy depth stream");
	
	/* Write the Theora stream headers into a temporary buffer to calculate their size before writing them to the sink: */
	IO::VariableMemoryFile theoraHeaders;
	theoraEncoder.writeHeaders(comments,theoraHeaders);
	
	/* Write the header size and header data to the sink: */
	sink.write<Misc::UInt32>(Misc::UInt32(theoraHeaders.getDataSize()));
	theoraHeaders.writeToSink(sink);
	
	#else
	
	/* Write a dummy header sequence: */
	sink.write<Misc::UInt32>(0);
	
	#endif
	}

LossyDepthFrameWriter::~LossyDepthFrameWriter(void)
	{
	}

size_t LossyDepthFrameWriter::writeFrame(const FrameBuffer& frame)
	{
	size_t result=0;
	
	/* Write the frame's time stamp to the sink: */
	sink.write<Misc::Float64>(frame.timeStamp);
	result+=sizeof(Misc::Float64);
	
	#if VIDEO_CONFIG_HAVE_THEORA
	
	/* Convert the new raw depth frame to Y'CbCr 4:2:0 by processing pixels in 2x2 blocks: */
	const FrameSource::DepthPixel* fRowPtr=static_cast<const FrameSource::DepthPixel*>(frame.getBuffer());
	unsigned char* ypRowPtr=theoraFrame.planes[0].data;
	unsigned char* cbRowPtr=theoraFrame.planes[1].data;
	unsigned char* crRowPtr=theoraFrame.planes[2].data;
	for(unsigned int y=0;y<size[1];y+=2)
		{
		const FrameSource::DepthPixel* fPtr=fRowPtr;
		unsigned char* ypPtr=ypRowPtr;
		unsigned char* cbPtr=cbRowPtr;
		unsigned char* crPtr=crRowPtr;
		for(unsigned int x=0;x<size[0];x+=2)
			{
			/* Distribute the block's 4 11-bit depth values among the 4 8-bit yp values and the 8-bit cb and cr values: */
			ypPtr[0]=(unsigned char)((fPtr[0]>>3)&0xfeU);
			ypPtr[1]=(unsigned char)((fPtr[1]>>3)&0xfeU);
			ypPtr[theoraFrame.planes[0].stride]=(unsigned char)((fPtr[size[0]]>>3)&0xfeU);
			ypPtr[theoraFrame.planes[0].stride+1]=(unsigned char)((fPtr[size[0]+1]>>3)&0xfeU);
			*cbPtr=(unsigned char)((fPtr[0]<<4)|(fPtr[1]&0x0fU));
			*crPtr=(unsigned char)((fPtr[size[0]]<<4)|(fPtr[size[0]+1]&0x0fU));
			
			/* Go to the next pixel block: */
			fPtr+=2;
			ypPtr+=2;
			++cbPtr;
			++crPtr;
			}
		
		/* Go to the next pixel block row: */
		fRowPtr+=size[0]*2;
		ypRowPtr+=theoraFrame.planes[0].stride*2;
		cbRowPtr+=theoraFrame.planes[1].stride;
		crRowPtr+=theoraFrame.planes[2].stride;
		}
	
	/* Feed the converted Y'CbCr 4:2:0 frame to the Theora encoder: */
	theoraEncoder.encodeFrame(theoraFrame);
	
	/* Write all encoded Theora packets to the sink: */
	Video::TheoraPacket packet;
	while(theoraEncoder.emitPacket(packet))
		{
		/* Write the packet to the sink: */
		packet.write(sink);
		result+=packet.getWireSize();
		}
	
	#endif
	
	return result;
	}

}
