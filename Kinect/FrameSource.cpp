/***********************************************************************
FrameSource - Base class for objects that create streams of depth and
color frames.
Copyright (c) 2011-2013 Oliver Kreylos

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

#include <Kinect/FrameSource.h>

#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Kinect/FrameBuffer.h>

namespace Kinect {

/*********************************************
Methods of class FrameSource::DepthCorrection:
*********************************************/

FrameSource::DepthCorrection::DepthCorrection(int sDegree,const int sNumSegments[2])
	:degree(sDegree),
	 controlPoints(0)
	{
	/* Copy elements: */
	for(int i=0;i<2;++i)
		numSegments[i]=sNumSegments[i];
	
	/* Allocate and initialize the control point array: */
	int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(int i=0;i<numControlPoints;++i)
		{
		controlPoints[i].scale=1.0f;
		controlPoints[i].offset=0.0f;
		}
	}

FrameSource::DepthCorrection::DepthCorrection(IO::File& file)
	:controlPoints(0)
	{
	/* Read the B-spline degree and number of segments: */
	degree=file.read<Misc::SInt32>();
	for(int i=0;i<2;++i)
		numSegments[i]=file.read<Misc::SInt32>();
	
	/* Allocate and read the control point array: */
	int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(int i=0;i<numControlPoints;++i)
		{
		controlPoints[i].scale=file.read<Misc::Float32>();
		controlPoints[i].offset=file.read<Misc::Float32>();
		}
	}

FrameSource::DepthCorrection::DepthCorrection(const FrameSource::DepthCorrection& source)
	:degree(source.degree),
	 controlPoints(0)
	{
	/* Copy elements: */
	for(int i=0;i<2;++i)
		numSegments[i]=source.numSegments[i];
	
	/* Allocate and copy the control point array: */
	int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	controlPoints=new PixelCorrection[numControlPoints];
	for(int i=0;i<numControlPoints;++i)
		controlPoints[i]=source.controlPoints[i];
	}

FrameSource::DepthCorrection::~DepthCorrection(void)
	{
	/* Delete the control point array: */
	delete[] controlPoints;
	}

void FrameSource::DepthCorrection::write(IO::File& file) const
	{
	/* Write the B-spline degree and number of segments: */
	file.write<Misc::SInt32>(degree);
	for(int i=0;i<2;++i)
		file.write<Misc::SInt32>(numSegments[i]);
	
	/* Write the control point array: */
	int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
	for(int i=0;i<numControlPoints;++i)
		{
		file.write<Misc::Float32>(controlPoints[i].scale);
		file.write<Misc::Float32>(controlPoints[i].offset);
		}
	}

namespace {

/****************
Helper functions:
****************/

/* Calculate the value of a univariate uniform non-rational B-spline: */

inline float bs(int i,int n,float x)
	{
	/* Check whether x is inside the B-spline's support [i, i+n+1): */
	if(x<float(i)||x>=float(i+n+1))
		return 0.0f;
	
	/* Calculate the B-spline using Cox-deBoor recursion: */
	float bsTemp[21]; // Maximum degree is 20
	for(int j=0;j<=n;++j)
		bsTemp[j]=x>=float(i+j)&&x<float(i+j+1)?1.0:0.0;
	
	for(int ni=1;ni<=n;++ni)
		for(int j=0;j<=n-ni;++j)
			bsTemp[j]=((x-float(i+j))*bsTemp[j]+(float(i+j+ni+1)-x)*bsTemp[j+1])/double(ni);
	
	return bsTemp[0];
	}

}

FrameSource::DepthCorrection::PixelCorrection FrameSource::DepthCorrection::getPixelCorrection(unsigned int x,unsigned int y,const unsigned int frameSize[2]) const
	{
	/* Convert the pixel position to B-spline space: */
	float dx=((float(x)+0.5f)*float(numSegments[0]))/float(frameSize[0]);
	float dy=((float(y)+0.5f)*float(numSegments[1]))/float(frameSize[1]);
	
	/* Evaluate the B-spline: */
	PixelCorrection result;
	result.scale=0.0f;
	result.offset=0.0f;
	for(int i=0;i<numSegments[1]+degree;++i)
		{
		float bsi=bs(i-degree,degree,dy);
		for(int j=0;j<numSegments[0]+degree;++j)
			{
			float bsj=bs(j-degree,degree,dx);
			result.scale+=controlPoints[(i*(numSegments[0]+degree)+j)].scale*bsi*bsj;
			result.offset+=controlPoints[(i*(numSegments[0]+degree)+j)].offset*bsi*bsj;
			}
		}
	
	return result;
	}

namespace {

/****************
Helper functions:
****************/

/* Evaluate a bivariate uniform non-rational B-spline: */

inline FrameSource::DepthCorrection::PixelCorrection bspline(int degree,const int numSegments[2],const FrameSource::DepthCorrection::PixelCorrection controlPoints[],float x,float y)
	{
	/* Find the segment index containing the evaluation point: */
	int i0=Math::floor(y);
	int j0=Math::floor(x);
	
	/* Run deBoor's algorithm to evaluate the B-spline: */
	FrameSource::DepthCorrection::PixelCorrection bsTemp[16][16]; // Maximum degree is 15
	for(int i=0;i<=degree;++i)
		for(int j=0;j<=degree;++j)
			bsTemp[i][j]=controlPoints[(i0+i)*(numSegments[0]+degree)+(j0+j)];
	for(int ni=0;ni<degree;++ni)
		{
		int sd=degree-ni;
		for(int j=0;j<sd;++j)
			{
			float w1=(x-float(j0+j-sd+1))/float(sd);
			float w0=(float(j0+j+1)-x)/float(sd);
			for(int i=0;i<=sd;++i)
				{
				bsTemp[i][j].scale=w1*bsTemp[i][j+1].scale+w0*bsTemp[i][j].scale;
				bsTemp[i][j].offset=w1*bsTemp[i][j+1].offset+w0*bsTemp[i][j].offset;
				}
			}
		
		for(int i=0;i<sd;++i)
			{
			float w1=(y-float(i0+i-sd+1))/float(sd);
			float w0=(float(i0+i+1)-y)/float(sd);
			for(int j=0;j<=sd;++j)
				{
				bsTemp[i][j].scale=w1*bsTemp[i+1][j].scale+w0*bsTemp[i][j].scale;
				bsTemp[i][j].offset=w1*bsTemp[i+1][j].offset+w0*bsTemp[i][j].offset;
				}
			}
		}
	
	return bsTemp[0][0];
	}

}

FrameSource::DepthCorrection::PixelCorrection* FrameSource::DepthCorrection::getPixelCorrection(const unsigned int frameSize[2]) const
	{
	/* Allocate the result array: */
	PixelCorrection* result=new PixelCorrection[frameSize[1]*frameSize[0]];
	
	PixelCorrection* rPtr=result;
	for(unsigned int y=0;y<frameSize[1];++y)
		{
		float dy=((float(y)+0.5f)*float(numSegments[1]))/float(frameSize[1]);
		for(unsigned x=0;x<frameSize[0];++x,++rPtr)
			{
			float dx=((float(x)+0.5f)*float(numSegments[0]))/float(frameSize[0]);
			*rPtr=bspline(degree,numSegments,controlPoints,dx,dy);
			}
		}
	
	return result;
	}

/****************************
Methods of class FrameSource:
****************************/

FrameSource::FrameSource(void)
	{
	}

FrameSource::~FrameSource(void)
	{
	}

FrameSource::DepthCorrection* FrameSource::getDepthCorrectionParameters(void)
	{
	/* Create and return a dummy depth correction object: */
	int numSegments[2]={1,1};
	return new DepthCorrection(0,numSegments);
	}

}
