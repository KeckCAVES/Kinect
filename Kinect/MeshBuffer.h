/***********************************************************************
MeshBuffer - Class for reference-counted projected depth frames stored
as triangle meshes.
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

#ifndef KINECT_MESHBUFFER_INCLUDED
#define KINECT_MESHBUFFER_INCLUDED

#include <new>
#include <Threads/Atomic.h>
#include <GL/gl.h>
#include <GL/GLVertex.h>

namespace Kinect {

class MeshBuffer
	{
	/* Embedded classes: */
	public:
	typedef GLVertex<void,0,void,0,void,GLfloat,3> Vertex; // Type for vertices
	typedef GLuint Index; // Type for triangle vertex indices
	
	private:
	struct BufferHeader
		{
		/* Elements: */
		public:
		Threads::Atomic<unsigned int> refCount; // Reference counter
		unsigned int maxNumVertices; // Number of vertices for which the buffer has been allocated
		Vertex* vertices; // Pointer to the vertex array
		unsigned int maxNumTriangles; // Number of triangles for which the buffer has been allocated
		Index* triangleIndices; // Pointer to the triangle vertex index array
		
		/* Constructors and destructors: */
		BufferHeader(unsigned int sMaxNumVertices,unsigned int sMaxNumTriangles)
			:refCount(1),
			 maxNumVertices(sMaxNumVertices),vertices(reinterpret_cast<Vertex*>(this+1)),
			 maxNumTriangles(sMaxNumTriangles),triangleIndices(reinterpret_cast<Index*>(vertices+maxNumVertices))
			{
			}
		
		/* Methods: */
		void ref(void) // References the buffer
			{
			refCount.preAdd(1);
			}
		bool unref(void) // Unreferences the buffer; returns true if buffer becomes orphaned
			{
			return refCount.preSub(1)==0;
			}
		bool isPrivate(void) // Returns true if the buffer is referenced by exactly one pointer
			{
			return refCount.ifCompareAndSwap(1,1); // Atomically check if the current ref count is 1; if so, set it to one (no-op) and return true
			}
		};
	
	/* Elements: */
	private:
	BufferHeader* buffer; // Pointer to the reference-counted buffer
	public:
	unsigned int numVertices; // Number of vertices in the mesh
	unsigned int numTriangles; // Number of triangles in the mesh
	double timeStamp; // Frame's time stamp in originating camera's own clock
	
	/* Constructors and destructors: */
	public:
	MeshBuffer(void) // Creates invalid mesh buffer
		:buffer(0),
		 numVertices(0),numTriangles(0),
		 timeStamp(0.0)
		{
		}
	MeshBuffer(unsigned int allocNumVertices,unsigned int allocNumTriangles) // Allocates a new mesh buffer for the given number of vertices and triangles
		:buffer(0),
		 numVertices(0),numTriangles(0),
		 timeStamp(0.0)
		{
		/* Calculate the required buffer size: */
		size_t bufferSize=sizeof(BufferHeader)+allocNumVertices*sizeof(Vertex)+allocNumTriangles*3*sizeof(Index);
		
		/* Allocate the mesh buffer including the header: */
		unsigned char* paddedBuffer=new unsigned char[bufferSize];
		buffer=new(paddedBuffer) BufferHeader(allocNumVertices,allocNumTriangles);
		}
	MeshBuffer(const MeshBuffer& source) // Copy constructor
		:buffer(source.buffer),
		 numVertices(source.numVertices),numTriangles(source.numTriangles),
		 timeStamp(source.timeStamp)
		{
		/* Reference the source's buffer: */
		if(buffer!=0)
			buffer->ref();
		}
	MeshBuffer& operator=(const MeshBuffer& source) // Assignment operator
		{
		/* Reference the new buffer: */
		BufferHeader* newBuffer=source.buffer;
		if(newBuffer!=0)
			newBuffer->ref();
		
		/* Unreference the current buffer: */
		if(buffer!=0&&buffer->unref())
			{
			/* Delete the unused buffer: */
			buffer->~BufferHeader();
			delete[] (reinterpret_cast<unsigned char*>(buffer));
			}
		
		/* Attach to the new buffer: */
		buffer=newBuffer;
		
		/* Copy the mesh information: */
		numVertices=source.numVertices;
		numTriangles=source.numTriangles;
		
		/* Copy the time stamp: */
		timeStamp=source.timeStamp;
		
		return *this;
		}
	~MeshBuffer(void)
		{
		/* Unreference the current buffer: */
		if(buffer!=0&&buffer->unref())
			{
			/* Delete the unused buffer: */
			buffer->~BufferHeader();
			delete[] (reinterpret_cast<unsigned char*>(buffer));
			}
		}
	
	/* Methods: */
	bool isValid(void) const // Returns true if the buffer is valid, i.e., has attached storage
		{
		return buffer!=0;
		}
	
	/* Methods that can only be called on valid buffers: */
	bool isPrivate(void) // Returns true if there is exactly one reference to the buffer
		{
		return buffer->isPrivate();
		}
	unsigned int getMaxNumVertices(void) const // Returns the number of vertices the buffer can hold
		{
		return buffer->maxNumVertices;
		}
	const Vertex* getVertices(void) const // Returns a pointer to the buffer's vertex array
		{
		return buffer->vertices;
		}
	Vertex* getVertices(void) // Ditto
		{
		return buffer->vertices;
		}
	unsigned int getMaxNumTriangles(void) const // Returns the number of triangles the buffer can hold
		{
		return buffer->maxNumTriangles;
		}
	const Index* getTriangleIndices(void) const // Returns a pointer to the buffer's triangle vertex index array
		{
		return buffer->triangleIndices;
		}
	Index* getTriangleIndices(void) // Ditto
		{
		return buffer->triangleIndices;
		}
	};

}

#endif
