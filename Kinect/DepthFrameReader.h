/***********************************************************************
DepthFrameReader - Class to read compressed depth frames from a source,
and pass decompressed time-stamped depth frames to a client.
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

#ifndef KINECT_DEPTHFRAMEREADER_INCLUDED
#define KINECT_DEPTHFRAMEREADER_INCLUDED

#include <stddef.h>
#include <Misc/SizedTypes.h>
#include <Kinect/HilbertCurve.h>
#include <Kinect/FrameReader.h>

/* Forward declarations: */
namespace IO {
class File;
}

namespace Kinect {

class DepthFrameReader:public FrameReader
	{
	/* Embedded classes: */
	private:
	struct HuffmanNode // Structure representing a node in a Huffman decoding tree
		{
		/* Elements: */
		public:
		unsigned int left; // Index of left subtree
		unsigned int right; // Index of right subtree
		};
	
	/* Elements: */
	private:
	IO::File& source; // Data source for compressed depth frames
	HilbertCurve hilbertCurve; // Object to traverse depth frames in Hilbert curve order
	unsigned int pixelDeltaNumLeaves; // Number of leaves in the pixel delta Huffman tree
	HuffmanNode* pixelDeltaNodes; // Node array of the pixel delta Huffman tree
	unsigned int spanLengthNumLeaves; // Number of leaves in the span length Huffman tree
	HuffmanNode* spanLengthNodes; // Node array of the span length Huffman tree
	Misc::UInt32 currentBits; // Buffer to extract bits from the source buffer
	Misc::UInt32 currentBitMask; // Mask to extract the next bit from the bit buffer
	
	/* Private methods: */
	void readHuffmanTree(unsigned int& numLeaves,HuffmanNode*& nodes); // Reads a Huffman decoding tree from the source
	void fillBitBuffer(void); // Fills the bit buffer from the source
	Misc::UInt32 getBit(void) // Reads a single bit from the source and returns its state
		{
		/* Fill the bit buffer if it is empty: */
		if(currentBitMask==0x0U)
			fillBitBuffer();
		
		/* Extract one bit from the bit buffer: */
		Misc::UInt32 result=0x0U;
		if(currentBits&currentBitMask)
			result=0x1U;
		currentBitMask>>=1;
		
		return result;
		}
	void flushBits(void); // Clears the bit buffer at the end of a frame
	
	/* Constructors and destructors: */
	public:
	DepthFrameReader(IO::File& sSource); // Creates a depth frame reader associated with the given data source
	virtual ~DepthFrameReader(void);
	
	/* Methods from FrameReader: */
	virtual FrameBuffer readNextFrame(void);
	};

}

#endif
