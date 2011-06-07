/***********************************************************************
ColorFrameWriter - Class to write compressed color frames to a sink.
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

#ifndef COLORFRAMEWRITER_INCLUDED
#define COLORFRAMEWRITER_INCLUDED

#include <stddef.h>
#include <Video/Config.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Video/TheoraFrame.h>
#include <Video/TheoraEncoder.h>
#endif

/* Forward declarations: */
namespace IO {
class File;
}
namespace Video {
#if VIDEO_CONFIG_HAVE_THEORA
class ImageExtractor;
#endif
}
class FrameBuffer;

class ColorFrameWriter
	{
	/* Elements: */
	private:
	IO::File& sink; // Data sink for compressed color frames
	unsigned int size[2]; // Width and height of the color frames
	#if VIDEO_CONFIG_HAVE_THEORA
	Video::TheoraEncoder theoraEncoder; // Theora encoder object
	Video::ImageExtractor* imageExtractor; // Extractor to convert RGB images to Y'CbCr 4:2:0 images
	Video::TheoraFrame theoraFrame; // Frame buffer for frames in Y'CbCr 4:2:0 pixel format
	#endif
	
	/* Constructors and destructors: */
	public:
	ColorFrameWriter(IO::File& sSink,const unsigned int sSize[2]); // Creates a color frame writer for the given sink and frame size
	~ColorFrameWriter(void);
	
	/* Methods: */
	size_t writeFrame(const FrameBuffer& frame); // Compresses and writes the given color frame; returns compressed frame size in bytes
	};

#endif
