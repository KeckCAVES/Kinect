/***********************************************************************
HilbertCurve - Helper class to create an index array to traverse a 2D
array in the order of a space-filling Hilbert curve.
Copyright (c) 2010-2011 Oliver Kreylos

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

#include <Kinect/HilbertCurve.h>

namespace Kinect {

/*****************************
Methods of class HilbertCurve:
*****************************/

void HilbertCurve::createCurve(const unsigned int arraySize[2],const unsigned int pos[2],unsigned int size,int entryCorner,int mainFlipBit,unsigned int*& hcPtr)
	{
	if(size==1)
		{
		/* Check if the leaf node is valid: */
		if(pos[0]<arraySize[0]&&pos[1]<arraySize[1])
			{
			/* Store the offset of the leaf node: */
			*hcPtr=pos[1]*arraySize[0]+pos[0];
			++hcPtr;
			}
		}
	else
		{
		static const int childStateTemplate[2][4][3]=
			{
			{{0,0,1},{2,0,0},{3,0,0},{1,3,1}},
			{{0,0,0},{1,0,1},{3,0,1},{2,3,0}}
			};
		
		/**************************************
		Recurse into the children of this node:
		**************************************/
		
		unsigned int childSize=size>>1;
		for(int i=0;i<4;++i)
			{
			/* Find the index of the child to recurse into: */
			int child=childStateTemplate[mainFlipBit][i][0]^entryCorner;
			unsigned int childPos[2];
			for(int j=0;j<2;++j)
				{
				childPos[j]=pos[j];
				if(child&(1<<j))
					childPos[j]+=childSize;
				}
			
			/* Recurse: */
			int childEntryCorner=childStateTemplate[mainFlipBit][i][1]^entryCorner;
			int childMainFlipBit=childStateTemplate[mainFlipBit][i][2];
			createCurve(arraySize,childPos,childSize,childEntryCorner,childMainFlipBit,hcPtr);
			}
		}
	}

HilbertCurve::HilbertCurve(void)
	:offsets(0)
	{
	}

HilbertCurve::~HilbertCurve(void)
	{
	delete[] offsets;
	}

void HilbertCurve::init(const unsigned int arraySize[2])
	{
	/* Allocate the offsets array: */
	offsets=new unsigned int[arraySize[1]*arraySize[0]];
	
	/* Call the recursive method: */
	unsigned int pos[2];
	for(int i=0;i<2;++i)
		pos[i]=0;
	unsigned int size;
	for(size=1;size<arraySize[0]||size<arraySize[1];size<<=1)
		;
	unsigned int* hcPtr=offsets;
	createCurve(arraySize,pos,size,0,0,hcPtr);
	}

}
