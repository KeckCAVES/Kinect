/***********************************************************************
ColorFrameReader - Class to read compressed color frames from a source.
Copyright (c) 2010-2015 Oliver Kreylos

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

#ifndef KINECT_COLORFRAMEREADER_INCLUDED
#define KINECT_COLORFRAMEREADER_INCLUDED

#include <Video/Config.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Video/TheoraDecoder.h>
#endif
#include <Kinect/FrameReader.h>

/* Forward declarations: */
namespace IO {
class File;
}

namespace Kinect {

class ColorFrameReader:public FrameReader
	{
	/* Elements: */
	private:
	IO::File& source; // Data source for compressed color frames
	bool sourceHasTheora; // Flag whether the source actually contains color frames
	#if VIDEO_CONFIG_HAVE_THEORA
	Video::TheoraDecoder theoraDecoder; // Object to decode the Theora-encoded color frame stream
	#endif
	
	/* Constructors and destructors: */
	public:
	ColorFrameReader(IO::File& sSource); // Creates a color frame reader for the given source
	virtual ~ColorFrameReader(void);
	
	/* Methods from FrameReader: */
	virtual FrameBuffer readNextFrame(void);
	};

}

#endif
