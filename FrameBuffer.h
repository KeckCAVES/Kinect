/***********************************************************************
FrameBuffer - Class for reference-counted decoded color or depth frame
buffers.
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

#ifndef FRAMEBUFFER_INCLUDED
#define FRAMEBUFFER_INCLUDED

class FrameBuffer
	{
	/* Elements: */
	private:
	int size[2]; // Width and height of the frame
	void* buffer; // Pointer to the reference-counted frame buffer
	
	/* Constructors and destructors: */
	public:
	FrameBuffer(void) // Creates invalid frame buffer
		:buffer(0)
		{
		size[0]=size[1]=0;
		}
	FrameBuffer(int sizeX,int sizeY,size_t bufferSize) // Allocates a new frame buffer of the given frame size and size in bytes
		:buffer(0)
		{
		/* Copy the frame size: */
		size[0]=sizeX;
		size[1]=sizeY;
		
		/* Allocate the enlarged frame buffer: */
		unsigned int* paddedBuffer=new unsigned int[(bufferSize+sizeof(unsigned int)-1)/sizeof(unsigned int)+1];
		buffer=paddedBuffer+1;
		
		/* Initialize the buffer's reference count: */
		paddedBuffer[0]=1;
		}
	FrameBuffer(const FrameBuffer& source) // Copy constructor
		:buffer(source.buffer)
		{
		/* Copy the frame size: */
		size[0]=source.size[0];
		size[1]=source.size[1];
		
		/* Reference the source's buffer: */
		if(buffer!=0)
			++(static_cast<unsigned int*>(buffer)[-1]);
		}
	FrameBuffer& operator=(const FrameBuffer& source) // Assignment operator
		{
		if(buffer!=source.buffer)
			{
			/* Unreference the current buffer: */
			if(buffer!=0&&--static_cast<unsigned int*>(buffer)[-1]==0)
				{
				/* Delete the unused buffer: */
				delete[] (static_cast<unsigned int*>(buffer)-1);
				}
			
			/* Copy the frame size: */
			size[0]=source.size[0];
			size[1]=source.size[1];
			
			/* Reference the source's buffer: */
			buffer=source.buffer;
			if(buffer!=0)
				++(static_cast<unsigned int*>(buffer)[-1]);
			}
		}
	~FrameBuffer(void)
		{
		/* Unreference the current buffer: */
		if(buffer!=0&&(--(static_cast<unsigned int*>(buffer)[-1]))==0)
			{
			/* Delete the unused buffer: */
			delete[] (static_cast<unsigned int*>(buffer)-1);
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

#endif
