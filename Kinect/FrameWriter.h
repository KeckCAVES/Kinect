/***********************************************************************
FrameWriter - Abstract base class to write color or depth frames to a
sink.
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

#ifndef KINECT_FRAMEWRITER_INCLUDED
#define KINECT_FRAMEWRITER_INCLUDED

#include <stddef.h>

/* Forward declarations: */
namespace Kinect {
class FrameBuffer;
}

namespace Kinect {

class FrameWriter
	{
	/* Elements: */
	protected:
	unsigned int size[2]; // Width and height of provided frames
	
	/* Constructors and destructors: */
	public:
	FrameWriter(const unsigned int sSize[2]); // Creates a frame writer for the given frame size
	virtual ~FrameWriter(void);
	
	/* Methods: */
	const unsigned int* getSize(void) const // Returns the frame size as an array
		{
		return size;
		}
	unsigned int getSize(int dimension) const // Returns frame width or height
		{
		return size[dimension];
		}
	virtual size_t writeFrame(const FrameBuffer& frame) =0; // Writes the given color or depth frame; returns size of written data in bytes
	};

}

#endif
