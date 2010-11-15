/***********************************************************************
SimpleReadBuffer - Class providing an endianness-safe typed interface
to read data from a fixed-size memory buffer.
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

#ifndef SIMPLEREADBUFFER_INCLUDED
#define SIMPLEREADBUFFER_INCLUDED

#include <string.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/Endianness.h>

class SimpleReadBuffer
	{
	/* Elements: */
	private:
	bool mustSwapEndianness; // Flag if buffer endianness is different from machine endianness
	size_t bufferSize; // Maximum size of buffer
	unsigned char* buffer; // Allocated buffer
	unsigned char* readPtr; // Pointer to the current read position in the buffer
	size_t used; // Used size in buffer
	
	/* Constructors and destructors: */
	public:
	SimpleReadBuffer(size_t sBufferSize) // Allocates buffer of the given size
		:bufferSize(sBufferSize),buffer(new unsigned char[bufferSize]),
		 readPtr(buffer),used(0)
		{
		/* Determine the buffer's endianness swapping behavior: */
		#if __BYTE_ORDER==__LITTLE_ENDIAN
		mustSwapEndianness=false;
		#endif
		#if __BYTE_ORDER==__BIG_ENDIAN
		mustSwapEndianness=true;
		#endif
		}
	~SimpleReadBuffer(void)
		{
		delete[] buffer;
		}
	
	/* Methods: */
	unsigned char* getBuffer(void) // Returns the read buffer
		{
		return buffer;
		}
	size_t getBufferSize(void) const // Returns the size of the read buffer
		{
		return bufferSize;
		}
	void fill(size_t dataSize) // Signals that the buffer contains the given amount of data, and rewinds
		{
		readPtr=buffer;
		used=dataSize;
		}
	size_t getDataSize(void) const // Returns the amount of unread data in the buffer
		{
		return used;
		}
	
	/* Endianness-safe binary I/O interface: */
	bool mustSwapOnRead(void) // Returns true if the buffer must endianness-swap data on read
		{
		return mustSwapEndianness;
		}
	void readRaw(void* data,size_t dataSize) // Reads a chunk of data from the buffer
		{
		if(dataSize<=used)
			{
			/* Read the data: */
			memcpy(data,readPtr,dataSize);
			readPtr+=dataSize;
			used-=dataSize;
			}
		else
			Misc::throwStdErr("SimpleReadBuffer::read: Buffer underflow by %u bytes",(unsigned int)(dataSize-used));
		}
	template <class DataParam>
	DataParam read(void) // Reads single value
		{
		DataParam result;
		readRaw(&result,sizeof(DataParam));
		if(mustSwapEndianness)
			Misc::swapEndianness(result);
		return result;
		}
	template <class DataParam>
	void read(DataParam& data) // Ditto
		{
		readRaw(&data,sizeof(DataParam));
		if(mustSwapEndianness)
			Misc::swapEndianness(data);
		}
	template <class DataParam>
	void read(const DataParam* data,size_t numItems) // Reads array of values
		{
		readRaw(data,numItems*sizeof(DataParam));
		if(mustSwapEndianness)
			Misc::swapEndianness(data,numItems);
		}
	};

#endif
