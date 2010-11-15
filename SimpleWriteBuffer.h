/***********************************************************************
SimpleWriteBuffer - Class providing an endianness-safe typed interface
to write data into a fixed-size memory buffer.
Copyright (c) 2010 Oliver Kreylos

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

#ifndef SIMPLEWRITEBUFFER_INCLUDED
#define SIMPLEWRITEBUFFER_INCLUDED

#include <string.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/Endianness.h>

class SimpleWriteBuffer
	{
	/* Elements: */
	private:
	bool mustSwapEndianness; // Flag if buffer endianness is different from machine endianness
	size_t bufferSize; // Maximum size of buffer
	unsigned char* buffer; // Allocated buffer
	unsigned char* writePtr; // Pointer to the current write position in the buffer
	size_t free; // Free size in buffer
	
	/* Constructors and destructors: */
	public:
	SimpleWriteBuffer(size_t sBufferSize) // Allocates buffer of the given size
		:bufferSize(sBufferSize),buffer(new unsigned char[bufferSize]),
		 writePtr(buffer),free(bufferSize)
		{
		/* Determine the buffer's endianness swapping behavior: */
		#if __BYTE_ORDER==__LITTLE_ENDIAN
		mustSwapEndianness=false;
		#endif
		#if __BYTE_ORDER==__BIG_ENDIAN
		mustSwapEndianness=true;
		#endif
		}
	~SimpleWriteBuffer(void)
		{
		delete[] buffer;
		}
	
	/* Methods: */
	const unsigned char* getData(void) const // Returns the write buffer
		{
		return buffer;
		}
	size_t getDataSize(void) const // Returns the amount of data in the write buffer
		{
		return bufferSize-free;
		}
	void clear(void) // Clears the buffer
		{
		writePtr=buffer;
		free=bufferSize;
		}
	
	/* Endianness-safe binary I/O interface: */
	bool mustSwapOnWrite(void) // Returns true if the buffer must endianness-swap data on write
		{
		return mustSwapEndianness;
		}
	void writeRaw(const void* data,size_t dataSize) // Writes a chunk of data into the buffer
		{
		if(dataSize<=free)
			{
			/* Write the data: */
			memcpy(writePtr,data,dataSize);
			writePtr+=dataSize;
			free-=dataSize;
			}
		else
			Misc::throwStdErr("SimpleWriteBuffer::write: Buffer overflow by %u bytes",(unsigned int)(dataSize-free));
		}
	template <class DataParam>
	void write(const DataParam& data) // Writes single value
		{
		if(mustSwapEndianness)
			{
			DataParam temp=data;
			Misc::swapEndianness(temp);
			writeRaw(&temp,sizeof(DataParam));
			}
		else
			writeRaw(&data,sizeof(DataParam));
		}
	template <class DataParam>
	void write(const DataParam* data,size_t numItems) // Writes array of values
		{
		if(mustSwapEndianness)
			{
			for(size_t i=0;i<numItems;++i)
				{
				DataParam temp=data[i];
				Misc::swapEndianness(temp);
				writeRaw(&temp,sizeof(DataParam));
				}
			}
		else
			writeRaw(data,numItems*sizeof(DataParam));
		}
	};

#endif
