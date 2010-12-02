/***********************************************************************
AlignPoints - Utility to align two sets of measurements of the same set
of points using a variety of transformation types.
Copyright (c) 2009-2010 Oliver Kreylos

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
#include <stdio.h>
#include <iostream>
#include <vector>
#include <Misc/File.h>
#include <Misc/FileCharacterSource.h>
#include <Misc/ValueSource.h>
#include <Math/Constants.h>
#define GEOMETRY_NONSTANDARD_TEMPLATES
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Application.h>

#include "ONTransformFitter.h"
#include "OGTransformFitter.h"
#include "LevenbergMarquardtMinimizer.h"

template <class TransformFitterParam>
inline
void
findTransform(std::vector<typename TransformFitterParam::Point>& points0,
              std::vector<typename TransformFitterParam::Point>& points1)
	{
	size_t numPoints=points0.size();
	if(numPoints>points1.size())
		numPoints=points1.size();
	
	/* Calculate the point set's centroids: */
	typename TransformFitterParam::Point::AffineCombiner cc0;
	for(size_t i=0;i<numPoints;++i)
		cc0.addPoint(points0[i]);
	typename TransformFitterParam::Point::AffineCombiner cc1;
	for(size_t i=0;i<numPoints;++i)
		cc1.addPoint(points1[i]);
	
	/* Move the first point set to the centroid: */
	typename TransformFitterParam::Transform centroidTransform=TransformFitterParam::Transform::translateToOriginFrom(cc0.getPoint());
	std::vector<typename TransformFitterParam::Point> cPoints0;
	for(size_t i=0;i<numPoints;++i)
		cPoints0.push_back(centroidTransform.transform(points0[i]));
	
	/* Initialize the iteration transformation: */
	typename TransformFitterParam::Transform bestTransform=TransformFitterParam::Transform::translateFromOriginTo(cc1.getPoint());
	typename TransformFitterParam::Scalar bestDistance=Math::Constants<typename TransformFitterParam::Scalar>::max;
	
	for(int i=0;i<5;++i)
		{
		/* Minimize the distance: */
		TransformFitterParam tf(numPoints,&cPoints0[0],&points1[0]);
		tf.setTransform(bestTransform);
		typename TransformFitterParam::Scalar result=LevenbergMarquardtMinimizer<TransformFitterParam>::minimize(tf);
		if(bestDistance>result)
			{
			bestTransform=tf.getTransform();
			bestDistance=result;
			}
		}
	
	/* Undo the centroid transformation: */
	bestTransform*=centroidTransform;
	
	/* Print the result: */
	std::cout<<"Best distance: "<<bestDistance<<std::endl;
	std::cout<<"Best transform: "<<Misc::ValueCoder<typename TransformFitterParam::Transform>::encode(bestTransform)<<std::endl;
	
	/* Transform the first point set: */
	for(typename std::vector<typename TransformFitterParam::Point>::iterator pIt=points0.begin();pIt!=points0.end();++pIt)
		*pIt=bestTransform.transform(*pIt);
	}

template <class PointParam,class TransformParam>
inline
void
transform(
	std::vector<PointParam>& points,
	const TransformParam& transform)
	{
	/* Transform all points: */
	for(typename std::vector<PointParam>::iterator pIt=points.begin();pIt!=points.end();++pIt)
		*pIt=transform.transform(*pIt);
	}

class AlignPoints:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef double Scalar;
	typedef Geometry::Point<Scalar,3> Point;
	typedef Geometry::OrthonormalTransformation<Scalar,3> ONTransform;
	typedef Geometry::OrthogonalTransformation<Scalar,3> OGTransform;
	typedef Geometry::ProjectiveTransformation<Scalar,3> PTransform;
	
	/* Elements: */
	std::vector<Point> points[2]; // The two point sets
	
	/* Constructors and destructors: */
	public:
	AlignPoints(int& argc,char**& argv,char**& appDefaults);
	
	/* Methods from Vrui::Application: */
	virtual void display(GLContextData& contextData) const;
	};

AlignPoints::AlignPoints(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults)
	{
	/* Parse the command line: */
	const char* fileName[2]={0,0};
	int transformMode=1;
	Scalar preScale=Scalar(1);
	const char* outputFileName=0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"ON")==0)
				transformMode=1;
			else if(strcasecmp(argv[i]+1,"OG")==0)
				transformMode=2;
			else if(strcasecmp(argv[i]+1,"S")==0)
				{
				++i;
				preScale=Scalar(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"O")==0)
				{
				++i;
				outputFileName=argv[i];
				}
			}
		else if(fileName[0]==0)
			fileName[0]=argv[i];
		else if(fileName[1]==0)
			fileName[1]=argv[i];
		}
	
	/* Read the two point sets: */
	for(int pointSet=0;pointSet<2;++pointSet)
		{
		/* Open the input file: */
		Misc::FileCharacterSource file(fileName[pointSet]);
		Misc::ValueSource reader(file);
		reader.setWhitespace(',',true);
		reader.setPunctuation('\n',true);
		reader.skipWs();
		
		/* Read points until end of file: */
		while(!reader.eof())
			{
			Point p;
			for(int i=0;i<3;++i)
				p[i]=Scalar(reader.readNumber());
			points[pointSet].push_back(p);
			reader.skipLine();
			reader.skipWs();
			}
		}
	size_t numPoints=points[0].size();
	if(numPoints>points[1].size())
		numPoints=points[1].size();
	
	if(preScale!=Scalar(1))
		{
		/* Scale the first point set: */
		for(std::vector<Point>::iterator pIt=points[0].begin();pIt!=points[0].end();++pIt)
			*pIt=Point::origin+(*pIt-Point::origin)*preScale;
		}
	
	/* Align the point sets: */
	switch(transformMode)
		{
		case 1: // Orthonormal transformation
			findTransform<ONTransformFitter>(points[0],points[1]);
			break;
		
		case 2: // Orthogonal transformation
			findTransform<OGTransformFitter>(points[0],points[1]);
			break;
		}
	
	if(outputFileName!=0)
		{
		/* Write the transformed point set: */
		Misc::File outputFile(outputFileName,"wt");
		for(std::vector<Point>::const_iterator pIt=points[0].begin();pIt!=points[0].end();++pIt)
			fprintf(outputFile.getFilePtr(),"%10.6f, %10.6f, %10.6f\n",double((*pIt)[0]),double((*pIt)[1]),double((*pIt)[2]));
		}
	}

void AlignPoints::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glLineWidth(1.0f);
	
	for(int pointSet=0;pointSet<2;++pointSet)
		{
		if(pointSet==0)
			glColor3f(1.0f,0.0f,1.0f);
		else
			glColor3f(0.0f,1.0f,0.0f);
		
		glBegin(GL_POINTS);
		for(std::vector<Point>::const_iterator pIt=points[pointSet].begin();pIt!=points[pointSet].end();++pIt)
			glVertex(*pIt);
		glEnd();
		}
	
	glBegin(GL_LINES);
	for(size_t i=0;i<points[0].size()&&i<points[1].size();++i)
		{
		glColor3f(1.0f,0.0f,1.0f);
		glVertex(points[0][i]);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex(points[1][i]);
		}
	glEnd();
	
	glPopAttrib();
	}

int main(int argc,char* argv[])
	{
	char** appDefaults=0;
	AlignPoints app(argc,argv,appDefaults);
	app.run();
	
	return 0;
	}
