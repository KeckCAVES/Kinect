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

#ifndef KINECT_HILBERTCURVE_INCLUDED
#define KINECT_HILBERTCURVE_INCLUDED

namespace Kinect {

class HilbertCurve
	{
	/* Elements: */
	private:
	unsigned int* offsets; // Array of pixel offsets
	
	/* Private methods: */
	void createCurve(const unsigned int arraySize[2],const unsigned int pos[2],unsigned int size,int entryCorner,int mainFlipBit,unsigned int*& hcPtr); // Creates the Hilbert curve recursively
	
	/* Constructors and destructors: */
	public:
	HilbertCurve(void); // Creates uninitialized Hilbert curve
	~HilbertCurve(void);
	
	/* Methods: */
	void init(const unsigned int arraySize[2]); // Initializes the Hilbert curve for the given array size
	const unsigned int* getOffsets(void) const // Returns the array offset array
		{
		return offsets;
		}
	unsigned int operator()(unsigned int index) const // Returns the array offset of the given point along the Hilbert curve
		{
		return offsets[index];
		}
	};

}

#endif
