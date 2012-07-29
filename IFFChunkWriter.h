/***********************************************************************
IFFChunkWriter - Helper class to simplify writing data to IFF container
files.
Copyright (c) 2012 Oliver Kreylos

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

#ifndef IFFCHUNKWRITER_INCLUDED
#define IFFCHUNKWRITER_INCLUDED

#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <IO/VariableMemoryFile.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Box.h>

class IFFChunkWriter:public IO::VariableMemoryFile
	{
	/* Embedded classes: */
	public:
	typedef Misc::Float32 Scalar;
	typedef Geometry::Point<Scalar,3> Point;
	typedef Geometry::Vector<Scalar,3> Vector;
	typedef Geometry::Box<Scalar,3> Box;
	
	/* Elements: */
	private:
	IO::FilePtr dest; // The data sink to which the chunk will be written
	char chunkId[4]; // Unterminated four-character chunk ID
	bool subChunk; // Flag if the chunk is a subchunk, i.e., only has a 2 byte length field
	/* Constructors and destructors: */
	public:
	IFFChunkWriter(IO::FilePtr sDest,const char* sChunkId,bool sSubChunk =false) // Creates a chunk for the given data sink with the given chunk ID; if subChunk flag is true, chunkSize is only 2 bytes long
		:dest(sDest),subChunk(sSubChunk)
		{
		/* Copy the chunk ID: */
		for(int i=0;i<4;++i)
			chunkId[i]=sChunkId[i];
		
		/* Copy the destination file's endianness: */
		setSwapOnWrite(dest->mustSwapOnWrite());
		
		/* Reference self: */
		ref();
		}
	
	/* New methods: */
	void writeString(const char* string) // Writes a NUL-terminated string
		{
		/* Write the string and terminator: */
		size_t len=strlen(string);
		write<char>(string,len+1);
		
		/* Write padding byte if length including terminator is odd: */
		if((len&0x1U)==0x0U)
			write<char>('\0');
		}
	void writeVarIndex(unsigned int index) // Writes a variable-length index
		{
		if(index>=0xff00U)
			write<Misc::UInt32>(index|0xff000000U);
		else
			write<Misc::UInt16>(index);
		}
	template <class ScalarParam>
	void writePoint(const Geometry::Point<ScalarParam,3>& p) // Writes a 3D point in right-handed space
		{
		/* Swap y and z to go to left-handed space: */
		write<Scalar>(p[0]);
		write<Scalar>(p[2]);
		write<Scalar>(p[1]);
		}
	template <class ScalarParam>
	void writeVector(const Geometry::Vector<ScalarParam,3>& v) // Writes a 3D vector in right-handed space
		{
		/* Swap y and z to go to left-handed space: */
		write<Scalar>(v[0]);
		write<Scalar>(v[2]);
		write<Scalar>(v[1]);
		}
	template <class ScalarParam>
	void writeBox(const Geometry::Box<ScalarParam,3>& b) // Writes a 3D box in right-handed space
		{
		/* Swap y and z to go to left-handed space: */
		write<Scalar>(b.min[0]);
		write<Scalar>(b.min[2]);
		write<Scalar>(b.min[1]);
		write<Scalar>(b.max[0]);
		write<Scalar>(b.max[2]);
		write<Scalar>(b.max[1]);
		}
	void writeColor(float r,float g,float b) // Writes a color value
		{
		write<Misc::Float32>(r);
		write<Misc::Float32>(g);
		write<Misc::Float32>(b);
		}
	void writeChunk(void) const // Writes the chunk to the destination file
		{
		/* Write the chunk ID: */
		dest->write<char>(chunkId,4);
		
		/* Write the chunk size: */
		size_t chunkSize=getDataSize();
		if(subChunk)
			{
			/* Check if the chunk is too large: */
			if(chunkSize>=size_t(0x1U)<<16)
				Misc::throwStdErr("IFFChunkWriter::writeChunk: Subchunk is too large to write");
			
			/* Write the chunk size: */
			dest->write<Misc::UInt16>(Misc::UInt16(chunkSize));
			}
		else
			{
			/* Check if the chunk is too large: */
			if(chunkSize>=size_t(0x1U)<<32)
				Misc::throwStdErr("IFFChunkWriter::writeChunk: Chunk is too large to write");
			
			/* Write the chunk size: */
			dest->write<Misc::UInt32>(Misc::UInt32(chunkSize));
			}
		
		/* Write the chunk contents: */
		writeToSink(*dest);
		
		/* Write the pad byte if necessary: */
		if(chunkSize&0x1U)
			dest->write<Misc::UInt8>(0U);
		}
	};

#endif
