/***********************************************************************
FrameBuffer - Class for reference-counted decoded color or depth frame
buffers.
Copyright (c) 2010-2012 Oliver Kreylos

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

#ifndef KINECT_FRAMEBUFFER_INCLUDED
#define KINECT_FRAMEBUFFER_INCLUDED

#define KINECT_FRAMEBUFFER_DEBUGLOCK 0

#if KINECT_FRAMEBUFFER_DEBUGLOCK
#include <assert.h>
#endif
#include <new>
#if KINECT_FRAMEBUFFER_DEBUGLOCK
#include <iostream>
#endif
#include <Threads/Atomic.h>

namespace Kinect {

class FrameBuffer
	{
	/* Embedded classes: */
	private:
	struct BufferHeader
		{
		/* Elements: */
		public:
		Threads::Atomic<unsigned int> refCount; // Reference counter
		#if KINECT_FRAMEBUFFER_DEBUGLOCK
		int destroyed;
		#endif
		
		/* Constructors and destructors: */
		BufferHeader(void)
			:refCount(1)
			#if KINECT_FRAMEBUFFER_DEBUGLOCK
			 ,destroyed(0)
			#endif
			{
			}
		~BufferHeader(void)
			{
			#if KINECT_FRAMEBUFFER_DEBUGLOCK
			destroyed=1;
			#endif
			}
		
		/* Methods: */
		void ref(void) // References the buffer
			{
			#if KINECT_FRAMEBUFFER_DEBUGLOCK
			assert(destroyed==0);
			#endif
			
			refCount.preAdd(1);
			}
		bool unref(void) // Unreferences the buffer; returns true if buffer becomes orphaned
			{
			#if KINECT_FRAMEBUFFER_DEBUGLOCK
			assert(destroyed==0);
			#endif
			
			return refCount.preSub(1)==0;
			}
		};
	
	/* Elements: */
	private:
	int size[2]; // Width and height of the frame
	void* buffer; // Pointer to the reference-counted frame buffer
	public:
	double timeStamp; // Frame's time stamp in originating camera's own clock
	
	/* Constructors and destructors: */
	public:
	FrameBuffer(void) // Creates invalid frame buffer
		:buffer(0),timeStamp(0.0)
		{
		size[0]=size[1]=0;
		}
	FrameBuffer(int sizeX,int sizeY,size_t bufferSize) // Allocates a new frame buffer of the given frame size and size in bytes
		:buffer(0),timeStamp(0.0)
		{
		/* Copy the frame size: */
		size[0]=sizeX;
		size[1]=sizeY;
		
		/* Allocate the enlarged frame buffer: */
		unsigned char* paddedBuffer=new unsigned char[bufferSize+sizeof(BufferHeader)];
		new(paddedBuffer) BufferHeader;
		
		/* Store the actual buffer pointer: */
		buffer=paddedBuffer+sizeof(BufferHeader);
		}
	FrameBuffer(const FrameBuffer& source) // Copy constructor
		:buffer(source.buffer),timeStamp(source.timeStamp)
		{
		/* Copy the frame size: */
		size[0]=source.size[0];
		size[1]=source.size[1];
		
		/* Reference the source's buffer: */
		if(buffer!=0)
			static_cast<BufferHeader*>(buffer)[-1].ref();
		}
	FrameBuffer& operator=(const FrameBuffer& source) // Assignment operator
		{
		if(buffer!=source.buffer)
			{
			#if KINECT_FRAMEBUFFER_DEBUGLOCK
			void* oldBuffer=buffer;
			#endif
			
			/* Unreference the current buffer: */
			if(buffer!=0)
				{
				if(static_cast<BufferHeader*>(buffer)[-1].unref())
					{
					/* Delete the unused buffer: */
					static_cast<BufferHeader*>(buffer)[-1].~BufferHeader();
					delete[] (static_cast<unsigned char*>(buffer)-sizeof(BufferHeader));
					}
				}
			
			/* Copy the frame size: */
			size[0]=source.size[0];
			size[1]=source.size[1];
			
			/* Reference the source's buffer: */
			buffer=source.buffer;
			if(buffer!=0)
				static_cast<BufferHeader*>(buffer)[-1].ref();
			
			/* Copy the time stamp: */
			timeStamp=source.timeStamp;
			}
		return *this;
		}
	~FrameBuffer(void)
		{
		/* Unreference the current buffer: */
		if(buffer!=0)
			{
			if(static_cast<BufferHeader*>(buffer)[-1].unref())
				{
				/* Delete the unused buffer: */
				static_cast<BufferHeader*>(buffer)[-1].~BufferHeader();
				delete[] (static_cast<unsigned char*>(buffer)-sizeof(BufferHeader));
				}
			}
		}
	
	/* Methods: */
	const int* getSize(void) const // Returns the frame size
		{
		return size;
		}
	int getSize(int dimension) const // Returns the frame size in one dimension
		{
		return size[dimension];
		}
	const void* getBuffer(void) const // Returns the frame buffer
		{
		return buffer;
		}
	void* getBuffer(void) // Ditto
		{
		return buffer;
		}
	};

}

#endif
