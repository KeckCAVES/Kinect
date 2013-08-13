/***********************************************************************
ColorFrameWriter - Class to write compressed color frames to a sink.
Copyright (c) 2010-2013 Oliver Kreylos

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

#include <Kinect/ColorFrameWriter.h>

#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <IO/File.h>
#include <IO/VariableMemoryFile.h>
#include <Video/Config.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Video/FrameBuffer.h>
#include <Video/ImageExtractorRGB8.h>
#include <Video/OggPage.h>
#include <Video/TheoraInfo.h>
#include <Video/TheoraComment.h>
#endif
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

/*********************************
Methods of class ColorFrameWriter:
*********************************/

ColorFrameWriter::ColorFrameWriter(IO::File& sSink,const unsigned int sSize[2])
	:FrameWriter(sSize),
	 sink(sSink)
	 #if VIDEO_CONFIG_HAVE_THEORA
	 ,
	 imageExtractor(0)
	 #endif
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
		Misc::throwStdErr("ColorFrameWriter::ColorFrameWriter: Error initializing Theora encoder");
	
	/* Set the encoder to maximum speed: */
	theoraEncoder.setSpeedLevel(theoraEncoder.getMaxSpeedLevel());
	
	/* Create the frame converter structures: */
	imageExtractor=new Video::ImageExtractorRGB8(size);
	theoraFrame.init420(theoraInfo);
	
	/* Set up a comment structure: */
	Video::TheoraComment comments;
	comments.setVendorString("Kinect color stream");
	
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

ColorFrameWriter::~ColorFrameWriter(void)
	{
	#if VIDEO_CONFIG_HAVE_THEORA
	delete imageExtractor;
	#endif
	}

size_t ColorFrameWriter::writeFrame(const FrameBuffer& frame)
	{
	size_t result=0;
	
	/* Write the frame's time stamp to the sink: */
	sink.write<Misc::Float64>(frame.timeStamp);
	result+=sizeof(Misc::Float64);
	
	#if VIDEO_CONFIG_HAVE_THEORA
	
	/* Convert the new raw RGB frame to Y'CbCr 4:2:0: */
	Video::FrameBuffer tempFrame;
	tempFrame.start=const_cast<FrameSource::ColorComponent*>(static_cast<const FrameSource::ColorComponent*>(frame.getBuffer())); // It's OK
	imageExtractor->extractYpCbCr420(&tempFrame,theoraFrame.planes[0].data,theoraFrame.planes[0].stride,theoraFrame.planes[1].data,theoraFrame.planes[1].stride,theoraFrame.planes[2].data,theoraFrame.planes[2].stride);
	
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
