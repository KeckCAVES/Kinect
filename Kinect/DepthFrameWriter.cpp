/***********************************************************************
DepthFrameWriter - Class to write compressed depth frames to a sink.
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

#include <Kinect/DepthFrameWriter.h>

#include <IO/File.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

namespace Kinect {

/*****************************************
Static elements of class DepthFrameWriter:
*****************************************/

const Misc::UInt32 DepthFrameWriter::pixelDeltaCodes[32][2]=
	{
	{0xbU,5},{0x23bU,11},{0x229U,11},{0x222U,11},{0x226U,11},{0x239U,11},{0x224U,11},{0x47fU,12},
	{0x22bU,11},{0x23dU,11},{0x23eU,11},{0x116U,10},{0x8cU,9},{0x20U,7},{0x9U,5},{0x0U,2},
	{0x1U,1},{0x3U,3},{0xaU,5},{0x21U,7},{0x8dU,9},{0x117U,10},{0x110U,10},{0x23cU,11},
	{0x22aU,11},{0x47eU,12},{0x225U,11},{0x238U,11},{0x228U,11},{0x223U,11},{0x227U,11},{0x23aU,11}
	};

const Misc::UInt32 DepthFrameWriter::pixelDeltaNodes[31][2]=
	{
	{25,7},{3,29},{6,26},{4,30},{28,2},{24,8},{27,5},{31,1},
	{23,9},{10,32},{22,33},{34,35},{36,37},{11,21},{38,39},{40,41},
	{42,43},{44,45},{12,20},{46,47},{48,49},{50,51},{13,19},{52,53},
	{54,55},{56,14},{18,0},{57,58},{59,17},{15,60},{61,16}
	};

const Misc::UInt32 DepthFrameWriter::spanLengthCodes[256][2]=
	{
	{0x2U,3},{0x0U,3},{0xeU,5},{0x1fU,6},{0x18U,6},{0x9U,6},{0x1bU,7},{0x3dU,7},
	{0x33U,7},{0xbU,6},{0x3bU,8},{0x36U,7},{0x3dU,8},{0x65U,8},{0x6bU,8},{0x3eU,8},
	{0x33U,8},{0xdfU,9},{0x73U,9},{0x22U,8},{0x29U,8},{0xddU,9},{0x60U,9},{0x1a1U,10},
	{0x69U,9},{0x79U,9},{0x1e3U,10},{0xd0U,10},{0xc8U,10},{0xe9U,10},{0xcaU,10},{0x1a9U,10},
	{0x1a7U,10},{0x40U,9},{0xaeU,10},{0x1a6U,10},{0x1a5U,10},{0x62U,9},{0x1a4U,10},{0x50U,9},
	{0x70U,9},{0x7fU,9},{0x1e5U,10},{0xa3U,10},{0x18fU,11},{0x344U,11},{0x357U,11},{0xd7U,10},
	{0xa8U,10},{0x1b8U,10},{0x84U,10},{0x61U,9},{0x83U,10},{0x3c1U,11},{0x152U,11},{0x79fU,12},
	{0x154U,11},{0x85U,10},{0x378U,11},{0x3cdU,11},{0x321U,11},{0x355U,11},{0x373U,11},{0x3c2U,11},
	{0x350U,11},{0x1f9U,11},{0x1c7U,11},{0x347U,11},{0x354U,11},{0x1c6U,11},{0x6acU,12},{0x105U,11},
	{0x193U,11},{0x1a8U,11},{0x79eU,12},{0x3acU,12},{0x645U,12},{0x799U,12},{0x3adU,12},{0x18dU,11},
	{0x79dU,12},{0x118U,11},{0x157U,11},{0x345U,11},{0x3c4U,11},{0x1d5U,11},{0x3a8U,12},{0x64bU,12},
	{0x158U,11},{0x1d1U,11},{0x790U,12},{0x3f7U,12},{0x64aU,12},{0x6a2U,12},{0x356U,12},{0x219U,12},
	{0x23cU,12},{0x318U,12},{0xdc8U,13},{0x389U,12},{0x6f5U,12},{0x11dU,11},{0x64fU,12},{0x6a3U,12},
	{0x11cU,11},{0x196U,11},{0x78aU,12},{0x357U,12},{0x344U,12},{0x646U,12},{0x352U,12},{0x23eU,12},
	{0x2b2U,12},{0x233U,12},{0xc98U,13},{0x2b3U,12},{0x324U,12},{0x345U,12},{0x786U,13},{0xde5U,13},
	{0x232U,12},{0x35bU,12},{0x218U,12},{0x23dU,12},{0xd5aU,13},{0x2a7U,12},{0x208U,12},{0x3c6U,12},
	{0xf22U,13},{0x683U,12},{0xf0cU,13},{0x3f4U,12},{0x346U,12},{0x3c4U,12},{0x2abU,12},{0x2bdU,12},
	{0x647U,12},{0x64dU,12},{0x2b4U,12},{0xc9cU,13},{0x7ecU,13},{0xf03U,13},{0x7ebU,13},{0xf38U,13},
	{0xf17U,13},{0x2aaU,12},{0xdeeU,13},{0x358U,12},{0x38aU,12},{0x3c5U,12},{0x23fU,12},{0x32eU,12},
	{0x649U,12},{0x18eU,11},{0x3a9U,12},{0x388U,12},{0x2b7U,12},{0x3f0U,12},{0x2bcU,12},{0x3c0U,12},
	{0x2adU,12},{0x353U,12},{0x2acU,12},{0x6e5U,12},{0x10eU,11},{0x1aaU,11},{0x640U,12},{0x144U,11},
	{0x10fU,11},{0x340U,11},{0x6f4U,12},{0x3c1U,12},{0x641U,12},{0x3f1U,12},{0x3a0U,12},{0xf0eU,13},
	{0xd05U,13},{0xd1aU,13},{0xde4U,13},{0x2a6U,12},{0x209U,12},{0xf39U,13},{0x632U,13},{0x68fU,13},
	{0xd18U,13},{0xf24U,13},{0xf25U,13},{0xdecU,13},{0xde6U,13},{0xdefU,13},{0xde7U,13},{0x72U,9},
	{0x28bU,12},{0x3aeU,12},{0xf00U,13},{0x787U,13},{0xd1bU,13},{0xf31U,13},{0xf26U,13},{0xc99U,13},
	{0x359U,12},{0x2beU,12},{0x2b5U,12},{0xf23U,13},{0x56cU,13},{0x75eU,13},{0xf27U,13},{0x32fU,12},
	{0xf16U,13},{0x3c7U,12},{0x68eU,13},{0x11aU,11},{0xd5bU,13},{0x21bU,12},{0x785U,13},{0xc89U,13},
	{0x237U,12},{0xf0fU,13},{0x743U,13},{0x64bU,13},{0x46dU,13},{0x435U,13},{0x1bdaU,14},{0x1e60U,14},
	{0x46cU,13},{0x57fU,13},{0xf08U,14},{0xf09U,14},{0x1bdbU,14},{0x514U,13},{0x1e61U,14},{0x633U,13},
	{0x742U,13},{0xd19U,13},{0xdc9U,13},{0x716U,13},{0x64aU,13},{0x7eaU,13},{0x75fU,13},{0xc90U,13},
	{0x6b4U,13},{0x6b5U,13},{0xd04U,13},{0xf01U,13},{0xf02U,13},{0x7edU,13},{0x515U,13},{0x434U,13},
	{0x56dU,13},{0xc91U,13},{0xc88U,13},{0x717U,13},{0x57eU,13},{0xc9dU,13},{0xf0dU,13},{0x1U,1}
	};

const Misc::UInt32 DepthFrameWriter::spanLengthNodes[255][2]=
	{
	{226,227},{222,228},{223,230},{247,221},{224,220},{229,246},{204,248},{252,225},
	{182,231},{236,219},{210,183},{240,241},{235,251},{232,218},{205,238},{256,214},
	{118,195},{237,142},{140,245},{250,215},{239,249},{114,199},{139,253},{242,176},
	{184,233},{177,196},{124,212},{98,234},{178,119},{188,190},{187,257},{146,189},
	{194,243},{244,141},{130,254},{175,217},{208,144},{128,203},{185,186},{198,206},
	{258,197},{143,181},{126,180},{122,95},{259,213},{120,113},{260,216},{96,123},
	{111,150},{261,192},{179,125},{145,134},{162,160},{112,115},{138,202},{262,156},
	{158,135},{201,263},{97,264},{116,265},{151,207},{108,117},{132,266},{110,161},
	{94,107},{147,200},{267,121},{155,99},{148,268},{174,269},{86,154},{75,78},
	{193,270},{159,171},{271,272},{133,149},{127,209},{157,173},{131,273},{274,91},
	{166,172},{275,76},{109,136},{276,152},{92,87},{277,137},{278,102},{279,129},
	{280,281},{93,103},{70,282},{283,163},{284,285},{170,100},{286,287},{288,289},
	{290,291},{106,292},{90,293},{294,295},{296,77},{297,80},{74,55},{298,71},
	{299,300},{164,168},{81,301},{211,302},{104,101},{303,304},{167,305},{54,306},
	{56,307},{308,82},{88,309},{310,311},{312,313},{314,79},{153,44},{315,72},
	{105,316},{317,318},{73,319},{165,320},{321,322},{323,324},{69,66},{325,89},
	{326,85},{327,328},{329,330},{331,332},{333,65},{334,335},{336,60},{337,338},
	{339,340},{341,342},{169,343},{45,83},{344,67},{64,345},{68,61},{346,46},
	{347,62},{58,348},{349,350},{351,53},{63,352},{84,353},{354,355},{356,59},
	{357,358},{359,52},{50,57},{360,361},{362,363},{364,365},{366,43},{48,367},
	{368,369},{370,371},{34,372},{373,374},{28,375},{30,376},{27,377},{378,379},
	{380,47},{381,382},{383,29},{384,385},{386,387},{388,389},{390,391},{392,393},
	{394,23},{395,396},{38,36},{35,32},{397,31},{398,399},{49,400},{401,402},
	{403,404},{405,26},{406,42},{407,408},{33,409},{410,411},{412,413},{39,414},
	{415,416},{417,418},{22,51},{37,419},{420,421},{422,24},{423,424},{40,425},
	{191,18},{426,427},{428,25},{429,41},{430,431},{432,433},{434,435},{436,437},
	{438,21},{439,17},{440,441},{442,443},{444,445},{19,446},{447,20},{448,449},
	{450,451},{452,16},{453,454},{455,456},{457,10},{458,12},{15,459},{460,13},
	{461,462},{463,14},{464,465},{466,467},{468,469},{470,471},{472,473},{474,6},
	{475,476},{477,478},{479,8},{480,481},{11,482},{483,7},{484,5},{485,9},
	{486,487},{488,489},{4,490},{491,492},{493,3},{494,495},{496,497},{498,499},
	{2,500},{501,502},{503,504},{1,505},{0,506},{507,508},{509,255},
	};

/*********************************
Methods of class DepthFrameWriter:
*********************************/

void DepthFrameWriter::writeManyBits(Misc::UInt32 bits,unsigned int numBits)
	{
	while(numBits>0)
		{
		/* Figure out how many bits can be copied in one go: */
		unsigned int numCopyBits=numBits;
		if(numCopyBits>currentBitsLeft)
			numCopyBits=currentBitsLeft;
		
		/* Copy the determined number of bits: */
		currentBits=(currentBits<<numCopyBits)|(bits>>(numBits-numCopyBits));
		currentBitsLeft-=numCopyBits;
		if(currentBitsLeft==0)
			{
			/* Write the bit buffer to the sink: */
			sink.write(currentBits);
			compressedSize+=sizeof(Misc::UInt32);
			
			/* Clear the bit buffer: */
			currentBits=0x0U;
			currentBitsLeft=32;
			}
		
		/* Prepare to write the rest of bits: */
		numBits-=numCopyBits;
		if(numBits>0)
			bits=bits&((0x1U<<numBits)-0x1U);
		}
	}

void DepthFrameWriter::flush(void)
	{
	/* Check if there are bits in the bit buffer: */
	if(currentBitsLeft<32)
		{
		/* Push the leftover bits to the left: */
		currentBits<<=currentBitsLeft;
		
		/* Write the bit buffer to the sink: */
		sink.write(currentBits);
		compressedSize+=sizeof(Misc::UInt32);
		
		/* Clear the bit buffer: */
		currentBits=0x0U;
		currentBitsLeft=32;
		}
	}

DepthFrameWriter::DepthFrameWriter(IO::File& sSink,const unsigned int sSize[2])
	:FrameWriter(sSize),
	 sink(sSink),
	 currentBits(0x0U),currentBitsLeft(32)
	{
	/* Create the Hilbert curve offset array: */
	hilbertCurve.init(size);
	
	/* Write the frame size to the sink: */
	for(int i=0;i<2;++i)
		sink.write<Misc::UInt32>(size[i]);
	
	/* Write the pixel delta Huffman decoding tree to the sink: */
	unsigned int pdnc=pixelDeltaNumCodes;
	sink.write<Misc::UInt32>(pdnc);
	sink.write(&pixelDeltaNodes[0][0],(pixelDeltaNumCodes-1)*2);
	
	/* Write the span length Huffman decoding tree to the sink: */
	unsigned int slnc=spanLengthNumCodes;
	sink.write<Misc::UInt32>(slnc);
	sink.write(&spanLengthNodes[0][0],(spanLengthNumCodes-1)*2);
	}

DepthFrameWriter::~DepthFrameWriter(void)
	{
	}

size_t DepthFrameWriter::writeFrame(const FrameBuffer& frame)
	{
	compressedSize=0;
	
	/* Write the frame's time stamp: */
	sink.write<Misc::Float64>(frame.timeStamp);
	compressedSize+=sizeof(Misc::Float64);
	
	/* Process all pixels: */
	const FrameSource::DepthPixel* frameBuffer=static_cast<const FrameSource::DepthPixel*>(frame.getBuffer());
	unsigned int numPixels=size[0]*size[1];
	const unsigned int* hcPtr=hilbertCurve.getOffsets();
	while(numPixels>0)
		{
		/* Check if the next span is valid or invalid: */
		if(frameBuffer[*hcPtr]!=FrameSource::invalidDepth)
			{
			/******************************
			Process a span of valid pixels:
			******************************/
			
			/* Write the span header and the initial pixel value: */
			Misc::UInt32 pixelValue=frameBuffer[*hcPtr];
			writeBits(0x800U|pixelValue,12); // 1 bit span header, 11 bits initial pixel value
			
			/* Write the rest of pixels in the span: */
			++hcPtr;
			--numPixels;
			while(numPixels>0&&frameBuffer[*hcPtr]>=pixelValue-15&&frameBuffer[*hcPtr]<=pixelValue+15)
				{
				/* Write the Huffman-encoded pixel value delta: */
				unsigned int delta=frameBuffer[*hcPtr]+16-pixelValue;
				writeBits(pixelDeltaCodes[delta][0],pixelDeltaCodes[delta][1]);
				
				pixelValue=frameBuffer[*hcPtr];
				++hcPtr;
				--numPixels;
				}
			
			/* Write the span terminator: */
			writeBits(pixelDeltaCodes[0][0],pixelDeltaCodes[0][1]);
			}
		else
			{
			/********************************
			Process a span of invalid pixels:
			********************************/
			
			/* Skip all following invalid pixels: */
			++hcPtr;
			--numPixels;
			unsigned int spanLength=1;
			while(numPixels>0&&frameBuffer[*hcPtr]==FrameSource::invalidDepth&&spanLength<256)
				{
				++hcPtr;
				--numPixels;
				++spanLength;
				}
			
			/* Write the span header and the Huffman-encoded span length minus 1: */
			writeBits(spanLengthCodes[spanLength-1][0],spanLengthCodes[spanLength-1][1]+1); // Write one extra zero bit for the span header
			}
		}
	
	/* Flush the bit buffer: */
	flush();
	
	return compressedSize;
	}

}
