/***********************************************************************
Depth to distance formula calibration utility.
Copyright (c) 2011 Oliver Kreylos

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
#include <stdlib.h>
#include <iostream>
#include <IO/OpenFile.h>
#include <IO/CSVSource.h>
#include <Math/Matrix.h>

void calibrateLinear(IO::CSVSource& csv,int distCol,int depthCol)
	{
	/* Create the least-squares matrix: */
	Math::Matrix ata(2,2,0.0);
	Math::Matrix atb(2,1,0.0);
	
	/* Process all measurements: */
	while(!csv.eof())
		{
		/* Read the next entry: */
		double dist=0.0;
		double depth=0.0;
		for(int colIndex=0;colIndex<=distCol||colIndex<=depthCol;++colIndex)
			{
			if(colIndex==distCol)
				dist=csv.readField<double>();
			else if(colIndex==depthCol)
				depth=csv.readField<double>();
			else
				csv.skipField();
			}
		std::cout<<"Distance: "<<dist<<" cm, depth: "<<depth<<std::endl;
		if(!csv.eor())
			csv.skipRecord();
		
		/* Accumulate the least-squares matrix: */
		ata(0,0)+=1.0;
		ata(0,1)+=-dist;
		ata(1,0)+=-dist;
		ata(1,1)+=dist*dist;
		atb(0)+=-dist*depth;
		atb(1)+=dist*dist*depth;
		}
	
	/* Solve the least-squares system: */
	Math::Matrix x=atb.divideFullPivot(ata);
	std::cout<<"Depth conversion formula: dist(cm) = "<<x(0)<<" / ("<<x(1)<<" - depth)"<<std::endl;
	}

void calibrateSquare(IO::CSVSource& csv,int distCol,int depthCol)
	{
	/* Create the least-squares matrix: */
	Math::Matrix ata(3,3,0.0);
	Math::Matrix atb(3,1,0.0);
	
	/* Process all measurements: */
	while(!csv.eof())
		{
		/* Read the next entry: */
		double dist=0.0;
		double depth=0.0;
		for(int colIndex=0;colIndex<=distCol||colIndex<=depthCol;++colIndex)
			{
			if(colIndex==distCol)
				dist=csv.readField<double>();
			else if(colIndex==depthCol)
				depth=csv.readField<double>();
			else
				csv.skipField();
			}
		std::cout<<"Distance: "<<dist<<" cm, depth: "<<depth<<std::endl;
		if(!csv.eor())
			csv.skipRecord();
		
		/* Accumulate the least-squares matrix: */
		ata(0,0)+=1.0;
		ata(0,1)+=-dist;
		ata(0,2)+=dist*depth*depth;
		ata(1,0)+=-dist;
		ata(1,1)+=dist*dist;
		ata(1,2)+=-dist*dist*depth*depth;
		ata(2,0)+=dist*depth*depth;
		ata(2,1)+=-dist*dist*depth*depth;
		ata(2,2)+=dist*dist*depth*depth*depth*depth;
		atb(0)+=-dist*depth;
		atb(1)+=dist*dist*depth;
		atb(2)+=-dist*dist*depth*depth*depth;
		}
	
	/* Solve the least-squares system: */
	Math::Matrix x=atb.divideFullPivot(ata);
	std::cout<<"Depth conversion formula: dist(cm) = "<<x(0)<<" / ("<<x(1)<<" - depth - "<<x(2)<<"*depth*depth)"<<std::endl;
	}

int main(int argc,char* argv[])
	{
	/* Parse the command line: */
	const char* csvFileName=0;
	int distCol=1;
	int depthCol=2;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"dist")==0)
				{
				++i;
				distCol=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"depth")==0)
				{
				++i;
				depthCol=atoi(argv[i]);
				}
			}
		else if(csvFileName==0)
			csvFileName=argv[i];
		}
	
	if(csvFileName==0)
		return 1;
	
	/* Read the CSV file: */
	IO::CSVSource csv(IO::openFile(csvFileName));
	
	/* Skip the header line: */
	csv.skipRecord();
	
	calibrateLinear(csv,distCol,depthCol);
	
	return 0;
	}
