/***********************************************************************
DepthFrameReader - Class to read compressed depth frames from a source,
and pass decompressed time-stamped depth frames to a client.
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

#include <Kinect/DepthFrameReader.h>

#include <IO/File.h>
#include <Math/Constants.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

/*********************************
Methods of class DepthFrameReader:
*********************************/

void DepthFrameReader::readHuffmanTree(unsigned int& numLeaves,DepthFrameReader::HuffmanNode*& nodes)
	{
	/* Read the number of leaf nodes: */
	numLeaves=source.read<Misc::UInt32>();
	
	/* Allocate and read the tree's node array: */
	nodes=new HuffmanNode[numLeaves-1]; // No need to store leaves; only interior nodes
	
	/* Read all nodes: */
	for(unsigned int i=0;i<numLeaves-1;++i)
		{
		nodes[i].left=source.read<Misc::UInt32>();
		nodes[i].right=source.read<Misc::UInt32>();
		}
	}

void DepthFrameReader::fillBitBuffer(void)
	{
	/* Read more data: */
	source.read(currentBits);
	currentBitMask=0x80000000U;
	}

void DepthFrameReader::flushBits(void)
	{
	/* Mark the bit buffer as empty: */
	currentBits=0x0U;
	currentBitMask=0x0U;
	}

DepthFrameReader::DepthFrameReader(IO::File& sSource)
	:source(sSource),
	 pixelDeltaNumLeaves(0),pixelDeltaNodes(0),
	 spanLengthNumLeaves(0),spanLengthNodes(0),
	 currentBits(0x0U),currentBitMask(0x0U)
	{
	/* Read the frame size from the source: */
	for(int i=0;i<2;++i)
		size[i]=source.read<Misc::UInt32>();
	
	/* Create the Hilbert curve offset array: */
	hilbertCurve.init(size);
	
	/* Read the pixel delta and span length Huffman decoding trees from the source: */
	readHuffmanTree(pixelDeltaNumLeaves,pixelDeltaNodes);
	readHuffmanTree(spanLengthNumLeaves,spanLengthNodes);
	}

DepthFrameReader::~DepthFrameReader(void)
	{
	delete[] pixelDeltaNodes;
	delete[] spanLengthNodes;
	}

FrameBuffer DepthFrameReader::readNextFrame(void)
	{
	/* Create the result frame: */
	FrameBuffer result(size[0],size[1],size[0]*size[1]*sizeof(FrameSource::DepthPixel));
	
	/* Return a dummy frame if the file is over: */
	if(source.eof())
		{
		result.timeStamp=Math::Constants<double>::max;
		return result;
		}
	
	/* Read the frame's time stamp from the source: */
	result.timeStamp=source.read<Misc::Float64>();
	
	/* Process all spans: */
	FrameSource::DepthPixel* resultBuffer=static_cast<FrameSource::DepthPixel*>(result.getBuffer());
	unsigned int numPixels=size[0]*size[1];
	const unsigned int* hcPtr=hilbertCurve.getOffsets();
	while(numPixels>0)
		{
		/* Detect the type of the next span: */
		if(getBit())
			{
			/******************************
			Process a span of valid pixels:
			******************************/
			
			/* Read the 11-bit unencoded value of the initial pixel: */
			unsigned int pixelValue=getBit();
			for(int i=1;i<11;++i)
				{
				pixelValue<<=1;
				pixelValue|=getBit();
				}
			
			/* Process the span's pixels: */
			while(true)
				{
				/* Store the current pixel: */
				resultBuffer[*hcPtr]=FrameSource::DepthPixel(pixelValue);
				++hcPtr;
				--numPixels;
				
				/* Read the Huffman-encoded pixel value delta for the next pixel: */
				unsigned int delta=pixelDeltaNumLeaves+pixelDeltaNumLeaves-2; // Start at the Huffman tree's root node
				while(delta>=pixelDeltaNumLeaves)
					{
					/* Select the next node based on the next bit: */
					if(getBit())
						delta=pixelDeltaNodes[delta-pixelDeltaNumLeaves].right;
					else
						delta=pixelDeltaNodes[delta-pixelDeltaNumLeaves].left;
					}
				if(delta==0) // Zero is span-ending code
					break;
				
				/* Adjust the current pixel value: */
				pixelValue=pixelValue+delta-16U;
				}
			}
		else
			{
			/********************************
			Process a span of invalid pixels:
			********************************/
			
			/* Read the Huffman-encoded span length: */
			unsigned int spanLength=spanLengthNumLeaves+spanLengthNumLeaves-2; // Start at the Huffman tree's root node
			while(spanLength>=spanLengthNumLeaves)
				{
				/* Select the next node based on the next bit: */
				if(getBit())
					spanLength=spanLengthNodes[spanLength-spanLengthNumLeaves].right;
				else
					spanLength=spanLengthNodes[spanLength-spanLengthNumLeaves].left;
				}
			++spanLength; // Compressor encoded spanLength-1, since 0 is impossible
			while(spanLength>0)
				{
				/* Set the current pixel to invalid: */
				resultBuffer[*hcPtr]=FrameSource::invalidDepth;
				++hcPtr;
				--numPixels;
				--spanLength;
				}
			}
		}
	
	/* Flush the bit buffer; frames start at byte-boundaries: */
	flushBits();
	
	return result;
	}

}
