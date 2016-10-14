/***********************************************************************
DepthFrameWriter - Class to write compressed depth frames to a sink.
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

#ifndef KINECT_DEPTHFRAMEWRITER_INCLUDED
#define KINECT_DEPTHFRAMEWRITER_INCLUDED

#include <stddef.h>
#include <Misc/SizedTypes.h>
#include <Kinect/HilbertCurve.h>
#include <Kinect/FrameWriter.h>

/* Forward declarations: */
namespace IO {
class File;
}

namespace Kinect {

class DepthFrameWriter:public FrameWriter
	{
	/* Elements: */
	private:
	IO::File& sink; // Data sink for the compressed depth frame stream
	HilbertCurve hilbertCurve; // Object to traverse depth frames in Hilbert curve order
	static const unsigned int pixelDeltaNumCodes=32; // Number of codes for pixel deltas
	static const Misc::UInt32 pixelDeltaCodes[pixelDeltaNumCodes][2]; // Huffman code array for pixel deltas
	static const Misc::UInt32 pixelDeltaNodes[pixelDeltaNumCodes-1][2]; // Huffman decoding tree nodes for pixel deltas
	static const unsigned int spanLengthNumCodes=256; // Number of codes for span lengths
	static const Misc::UInt32 spanLengthCodes[spanLengthNumCodes][2]; // Huffman code array for span lengths
	static const Misc::UInt32 spanLengthNodes[spanLengthNumCodes-1][2]; // Huffman decoding tree nodes for span lengths
	Misc::UInt32 currentBits; // Buffer to push bits into the sink buffer
	unsigned int currentBitsLeft; // Number of available bits left in the bit buffer
	size_t compressedSize; // Aggregated size of compressed frame during writing
	
	/* Private methods: */
	void writeManyBits(Misc::UInt32 bits,unsigned int numBits); // Writes the given number of bits to the sink
	void writeBits(Misc::UInt32 bits,unsigned int numBits) // Writes the given number of bits to the sink
		{
		if(numBits<=currentBitsLeft)
			{
			currentBits=(currentBits<<numBits)|bits;
			currentBitsLeft-=numBits;
			}
		else
			{
			/* Fall back to slower default method: */
			writeManyBits(bits,numBits);
			}
		}
	void flush(void); // Flushes the bit buffer
	
	/* Constructors and destructors: */
	public:
	DepthFrameWriter(IO::File& sSink,const unsigned int sSize[2]); // Creates a depth frame writer for the given sink and frame size
	virtual ~DepthFrameWriter(void);
	
	/* Methods from FrameWriter: */
	virtual size_t writeFrame(const FrameBuffer& frame);
	};

}

#endif
