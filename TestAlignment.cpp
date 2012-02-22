/***********************************************************************
TestAlignment - Utility to test the alignment quality between multiple
Kinect cameras.
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

#include <string.h>
#include <iostream>
#include <vector>
#include <IO/OpenFile.h>
#include <IO/ValueSource.h>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/GeometryValueCoders.h>

typedef double Scalar;
typedef Geometry::Point<Scalar,3> Point;
typedef Geometry::OrthogonalTransformation<Scalar,3> OGTransform;

int main(int argc,char* argv[])
	{
	/* Parse the command line: */
	const char* fileName[2]={0,0};
	OGTransform transform[2];
	int numTransforms=0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"T")==0)
				{
				++i;
				if(numTransforms<2)
					{
					transform[numTransforms]=Misc::ValueCoder<OGTransform>::decode(argv[i],argv[i]+strlen(argv[i]),0);
					++numTransforms;
					}
				}
			}
		else if(fileName[0]==0)
			fileName[0]=argv[i];
		else if(fileName[1]==0)
			fileName[1]=argv[i];
		}
	
	/* Read and transform the two point sets: */
	std::vector<Point> points[2];
	for(int pointSet=0;pointSet<2;++pointSet)
		{
		/* Open the input file: */
		IO::ValueSource reader(IO::openFile(fileName[pointSet]));
		reader.setWhitespace(',',true);
		reader.setPunctuation('\n',true);
		reader.skipWs();
		
		/* Read points until end of file: */
		while(!reader.eof())
			{
			Point p;
			for(int i=0;i<3;++i)
				p[i]=Scalar(reader.readNumber());
			points[pointSet].push_back(transform[pointSet].transform(p));
			reader.skipLine();
			reader.skipWs();
			}
		}
	size_t numPoints=points[0].size();
	if(numPoints>points[1].size())
		numPoints=points[1].size();
	
	/* Calculate the RMS distance between the two point sets: */
	double rms2=0.0;
	for(size_t i=0;i<numPoints;++i)
		rms2+=Geometry::sqrDist(points[0][i],points[1][i]);
	std::cout<<"RMS distance: "<<Math::sqrt(rms2/double(numPoints))<<std::endl;
	
	return 0;
	}
