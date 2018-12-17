/***********************************************************************
SpaceCarver - Utility to convert a set of colocated Kinect facades into
a watertight mesh using a space carving approach.
Copyright (c) 2011-2018 Oliver Kreylos

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

#include <stdlib.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/Array.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Geometry/ComponentArray.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/GeometryMarshallers.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/DepthFrameReader.h>

typedef Geometry::Point<double,3> Point;
typedef Geometry::Box<double,3> Box;
typedef Geometry::ComponentArray<double,3> Size;
typedef Geometry::OrthogonalTransformation<double,3> OGTransform;
typedef Geometry::ProjectiveTransformation<double,3> Projection;
typedef unsigned char Voxel;
typedef Misc::Array<Voxel,3> Grid;
typedef Grid::Index Index;

FrameBuffer open(const FrameBuffer& frame)
	{
	FrameBuffer result(frame.getSize(0),frame.getSize(1),frame.getSize(0)*frame.getSize(1)*sizeof(unsigned short));
	
	int stride=frame.getSize(0);
	int noffs[9];
	int* noffPtr=noffs;
	for(int dy=-1;dy<=1;++dy)
		for(int dx=-1;dx<=1;++dx,++noffPtr)
			*noffPtr=dy*stride+dx;
	const unsigned short* fPtr=frame.getTypedBuffer<unsigned short>();
	unsigned short* rPtr=result.getTypedBuffer<unsigned short>();
	for(int x=0;x<frame.getSize(0);++x,++fPtr,++rPtr)
		*rPtr=*fPtr;
	for(int y=1;y<frame.getSize(1)-1;++y)
		{
		*rPtr=*fPtr;
		++fPtr;
		++rPtr;
		for(int x=1;x<frame.getSize(0)-1;++x,++fPtr,++rPtr)
			{
			if(*fPtr>=0x07feU)
				{
				unsigned int nd=0;
				unsigned int nn=0;
				for(int i=0;i<9;++i)
					if(fPtr[noffs[i]]<0x07feU)
						{
						nd+=fPtr[noffs[i]];
						++nn;
						}
				if(nn>0)
					*rPtr=(nd+nn/2)/nn;
				else
					*rPtr=0x07ffU;
				}
			else
				*rPtr=*fPtr;
			}
		*rPtr=*fPtr;
		++fPtr;
		++rPtr;
		}
	for(int x=0;x<frame.getSize(0);++x,++fPtr,++rPtr)
		*rPtr=*fPtr;
	
	return result;
	}

FrameBuffer close(const FrameBuffer& frame)
	{
	FrameBuffer result(frame.getSize(0),frame.getSize(1),frame.getSize(0)*frame.getSize(1)*sizeof(unsigned short));
	
	int stride=frame.getSize(0);
	int noffs[9];
	int* noffPtr=noffs;
	for(int dy=-1;dy<=1;++dy)
		for(int dx=-1;dx<=1;++dx,++noffPtr)
			*noffPtr=dy*stride+dx;
	const unsigned short* fPtr=frame.getTypedBuffer<unsigned short>();
	unsigned short* rPtr=result.getTypedBuffer<unsigned short>();
	for(int x=0;x<frame.getSize(0);++x,++fPtr,++rPtr)
		*rPtr=*fPtr;
	for(int y=1;y<frame.getSize(1)-1;++y)
		{
		*rPtr=*fPtr;
		++fPtr;
		++rPtr;
		for(int x=1;x<frame.getSize(0)-1;++x,++fPtr,++rPtr)
			{
			if(*fPtr>=0x07feU)
				{
				for(int i=0;i<9;++i)
					rPtr[noffs[i]]=0x07ffU;
				}
			else
				*rPtr=*fPtr;
			}
		*rPtr=*fPtr;
		++fPtr;
		++rPtr;
		}
	for(int x=0;x<frame.getSize(0);++x,++fPtr,++rPtr)
		*rPtr=*fPtr;
	
	return result;
	}

int main(int argc,char* argv[])
	{
	/* Set up the volumetric grid: */
	Box gridBox=Box(Point(-32.0,-64.0,16.0),Point(32.0,0.0,80.0));
	Index gridSize(256,256,256);
	Grid grid(gridSize);
	
	/* Initialize the grid: */
	for(Grid::iterator gIt=grid.begin();gIt!=grid.end();++gIt)
		*gIt=Voxel(255);
	
	/* Carve away the n-th facade from each depth stream file listed on the command line: */
	unsigned int facadeIndex=atoi(argv[1]);
	for(int depthFileIndex=2;depthFileIndex<argc;++depthFileIndex)
		{
		try
			{
			/* Open the depth file: */
			IO::AutoFile depthFile(IO::openFile(argv[depthFileIndex]));
			depthFile->setEndianness(IO::File::LittleEndian);
			
			/* Read the facade projection matrix and the projector transformation: */
			Projection depthTransform;
			depthFile->read<double>(depthTransform.getMatrix().getEntries(),4*4);
			OGTransform projectorTransform=Misc::Marshaller<OGTransform>::read(*depthFile);
			
			/* Calculate the joint projective transformation from 3D world space into depth image space: */
			Projection proj=Geometry::invert(Projection(projectorTransform)*depthTransform);
			
			/* Create a depth frame reader: */
			DepthFrameReader depthFrameReader(*depthFile);
			
			/* Read the n-th facade: */
			FrameBuffer frame;
			for(unsigned int i=0;i<facadeIndex;++i)
				frame=depthFrameReader.readNextFrame();
			
			/* Run a sequence of morphological open and close operators on the frame to fill holes: */
			#if 1
			for(int i=0;i<8;++i)
				frame=open(frame);
			#endif
			#if 1
			for(int i=0;i<8;++i)
				frame=close(frame);
			#endif
			
			/* Carve the facade out of the grid: */
			std::cout<<"Processing depth file "<<argv[depthFileIndex]<<std::flush;
			double fmax[2];
			for(int i=0;i<2;++i)
				fmax[i]=double(frame.getSize(i));
			unsigned short* frameBuffer=frame.getTypedBuffer<unsigned short>();
			Size cellSize;
			for(int i=0;i<3;++i)
				cellSize[i]=(gridBox.max[i]-gridBox.min[i])/double(gridSize[i]);
			Index index;
			Point gridp;
			gridp[0]=gridBox.min[0]+0.5*cellSize[0];
			for(index[0]=0;index[0]<gridSize[0];++index[0],gridp[0]+=cellSize[0])
				{
				gridp[1]=gridBox.min[1]+0.5*cellSize[1];
				for(index[1]=0;index[1]<gridSize[1];++index[1],gridp[1]+=cellSize[1])
					{
					gridp[2]=gridBox.min[2]+0.5*cellSize[2];
					for(index[2]=0;index[2]<gridSize[2];++index[2],gridp[2]+=cellSize[2])
						{
						/* Project the grid point into the depth frame: */
						Point fp=proj.transform(gridp);
						
						/* Check if the projected grid point is inside the depth frame: */
						if(fp[0]>=0.0&&fp[0]<fmax[0]&&fp[1]>=0.0&&fp[1]<fmax[1])
							{
							/* Check if the grid point is outside the facade: */
							int x=int(fp[0]);
							int y=int(fp[1]);
							unsigned short depth=frameBuffer[y*frame.getSize(0)+x];
							if(fp[2]<double(depth))
								grid(index)=Voxel(0);
							}
						else
							grid(index)=Voxel(0);
						}
					}
				std::cout<<'.'<<std::flush;
				}
			std::cout<<"done"<<std::endl;
			}
		catch(const std::runtime_error& err)
			{
			std::cerr<<"Ignoring depth file "<<argv[depthFileIndex]<<" due to exception "<<err.what()<<std::endl;
			}
		catch(...)
			{
			std::cerr<<"Ignoring depth file "<<argv[depthFileIndex]<<" due to spurious exception"<<std::endl;
			}
		}
	
	/* Save the result grid to a volume file: */
	IO::AutoFile volFile(IO::openFile("SpaceCarverOut.vol",IO::File::WriteOnly));
	volFile->setEndianness(IO::File::BigEndian);
	for(int i=0;i<3;++i)
		volFile->write<int>(int(gridSize[i]));
	volFile->write<int>(0);
	for(int i=0;i<3;++i)
		volFile->write<float>((gridBox.max[i]-gridBox.min[i])*double(gridSize[i]-1)/double(gridSize[i]));
	volFile->write<Voxel>(grid.getArray(),grid.getNumElements());
	
	return 0;
	}
