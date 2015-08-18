/***********************************************************************
AlignPoints - Utility to align two sets of measurements of the same set
of points using a variety of transformation types.
Copyright (c) 2009-2014 Oliver Kreylos

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
#include <IO/ValueSource.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#define GEOMETRY_NONSTANDARD_TEMPLATES
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Box.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/OutputOperators.h>
#include <Geometry/LevenbergMarquardtMinimizer.h>
#include <GL/gl.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/OpenFile.h>
#include <Vrui/Application.h>

#include "ONTransformFitter.h"
#include "OGTransformFitter.h"
#include "PTransformFitter.h"

template <class TransformFitterParam>
inline
typename TransformFitterParam::Transform
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
		Geometry::LevenbergMarquardtMinimizer<TransformFitterParam> minimizer;
		minimizer.maxNumIterations=500000;
		minimizer.epsilon2=1.0e-40;
		TransformFitterParam tf(numPoints,&cPoints0[0],&points1[0]);
		tf.setTransform(bestTransform);
		typename TransformFitterParam::Scalar result=minimizer.minimize(tf);
		if(bestDistance>result)
			{
			bestTransform=tf.getTransform();
			bestDistance=result;
			}
		}
	
	/* Undo the centroid transformation: */
	bestTransform*=centroidTransform;
	
	/* Print the result: */
	std::cout<<"Best residual: "<<bestDistance<<std::endl;
	
	/* Transform the first point set: */
	for(typename std::vector<typename TransformFitterParam::Point>::iterator pIt=points0.begin();pIt!=points0.end();++pIt)
		*pIt=bestTransform.transform(*pIt);
	double rmsd=0.0;
	for(size_t i=0;i<numPoints;++i)
		rmsd+=Geometry::sqrDist(points0[i],points1[i]);
	std::cout<<"Best distance: "<<Math::sqrt(rmsd/double(numPoints))<<std::endl;
	
	return bestTransform;
	}

template <class TransformFitterParam>
inline
typename TransformFitterParam::Transform
findTransform2(std::vector<typename TransformFitterParam::Point>& points0,
               std::vector<typename TransformFitterParam::Point>& points1)
	{
	return TransformFitterParam::Transform::identity;
	}

template <>
inline
OGTransformFitter::Transform
findTransform2<OGTransformFitter>(std::vector<OGTransformFitter::Point>& points0,
                                  std::vector<OGTransformFitter::Point>& points1)
	{
	typedef OGTransformFitter::Point Point;
	typedef OGTransformFitter::Transform Transform;
	
	#if 0
	
	/* Rotate the first point set: */
	Transform t=Transform::rotate(Transform::Rotation::rotateAxis(Transform::Vector(1,0.5,0),Math::rad(22.0)));
	for(std::vector<Point>::iterator pIt=points0.begin();pIt!=points0.end();++pIt)
		*pIt=t.transform(*pIt);
	
	#endif
	
	size_t numPoints=points0.size();
	if(numPoints>points1.size())
		numPoints=points1.size();
	
	/* Calculate both point sets' centroids: */
	Point::AffineCombiner cc0;
	Point::AffineCombiner cc1;
	for(size_t i=0;i<numPoints;++i)
		{
		cc0.addPoint(points0[i]);
		cc1.addPoint(points1[i]);
		}
	Point c0=cc0.getPoint();
	Point c1=cc1.getPoint();
	std::cout<<"Centroids: "<<c0<<", "<<c1<<", result translation = "<<c1-c0<<std::endl;
	
	/* Calculate both point sets' inner products: */
	double ip0=0.0;
	double ip1=0.0;
	for(size_t pi=0;pi<numPoints;++pi)
		{
		Point::Vector d0=points0[pi]-c0;
		Point::Vector d1=points1[pi]-c1;
		ip0+=Math::sqr(d0[0])+Math::sqr(d0[1])+Math::sqr(d0[2]);
		ip1+=Math::sqr(d1[0])+Math::sqr(d1[1])+Math::sqr(d1[2]);
		}
	
	/* Calculate the normalizing scaling factors: */
	double scale0=Math::sqrt(ip0);
	double scale1=Math::sqrt(ip1);
	std::cout<<"Scales: "<<scale0<<", "<<scale1<<", result scale = "<<scale1/scale0<<std::endl;
	
	/* Move both point sets to their centroids and scale them to uniform size: */
	Transform centroidTransform0=Transform::translateToOriginFrom(c0);
	centroidTransform0.leftMultiply(Transform::scale(1.0/scale0));
	Transform centroidTransform1=Transform::translateToOriginFrom(c1);
	centroidTransform1.leftMultiply(Transform::scale(1.0/scale1));
	std::vector<Point> cPoints0;
	std::vector<Point> cPoints1;
	for(size_t i=0;i<numPoints;++i)
		{
		cPoints0.push_back(centroidTransform0.transform(points0[i]));
		cPoints1.push_back(centroidTransform1.transform(points1[i]));
		}
	
	/* Calculate the inner product between the two point sets: */
	double m[3][3];
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			m[i][j]=0.0;
	for(size_t pi=0;pi<numPoints;++pi)
		for(int i=0;i<3;++i)
			for(int j=0;j<3;++j)
				m[i][j]+=cPoints0[pi][i]*cPoints1[pi][j];
	
	/* Calculate the coefficients of the quaternion-based characteristic polynomial of the quaternion key matrix: */
	double q4=1.0;
	double q3=0.0;
	double q2=0.0;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			q2-=2.0*Math::sqr(m[i][j]);
	double q1=8.0*(m[0][0]*m[1][2]*m[2][1]+m[1][1]*m[2][0]*m[0][2]+m[2][2]*m[0][1]*m[1][0])
	         -8.0*(m[0][0]*m[1][1]*m[2][2]+m[1][2]*m[2][0]*m[0][1]+m[2][1]*m[1][0]*m[0][2]);
	double qd0=Math::sqr(Math::sqr(m[0][1])+Math::sqr(m[0][2])-Math::sqr(m[1][0])-Math::sqr(m[2][0]));
	double qd1=(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])-2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]))
	          *(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])+2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]));
	double qd2=(-(m[0][2]+m[2][0])*(m[1][2]-m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]+m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]+m[2][2]));
	double qd3=(-(m[0][2]+m[2][0])*(m[1][2]+m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]-m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd4=((m[0][1]+m[1][0])*(m[1][2]+m[2][1])+(m[0][2]+m[2][0])*(m[0][0]-m[1][1]+m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]-m[2][1])+(m[0][2]+m[2][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd5=((m[0][1]+m[1][0])*(m[1][2]-m[2][1])+(m[0][2]-m[2][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]+m[2][1])+(m[0][2]-m[2][0])*(m[0][0]+m[1][1]-m[2][2]));
	double q0=qd0+qd1+qd2+qd3+qd4+qd5;
	
	/* Calculate the optimal rotation: */
	double lambda=Math::mid(ip0,ip1);
	double lambda0;
	do
		{
		lambda0=lambda;
		double poly=(((q4*lambda+q3)*lambda+q2)*lambda+q1)*lambda+q0;
		double dPoly=((4.0*q4*lambda+3.0*q3)*lambda+2.0*q2)*lambda+q1;
		lambda-=poly/dPoly;
		}
	while(Math::abs(lambda-lambda0)<1.0e-8);
	std::cout<<"Largest eigenvalue of key matrix: "<<lambda<<std::endl;
	
	/* Find the eigenvector corresponding to the largest eigenvalue: */
	Math::Matrix k(4,4);
	k(0,0)=m[0][0]+m[1][1]+m[2][2];
	k(0,1)=m[1][2]-m[2][1];
	k(0,2)=m[2][0]-m[0][2];
	k(0,3)=m[0][1]-m[1][0];
	k(1,0)=m[1][2]-m[2][1];
	k(1,1)=m[0][0]-m[1][1]-m[2][2];
	k(1,2)=m[0][1]+m[1][0];
	k(1,3)=m[2][0]+m[0][2];
	k(2,0)=m[2][0]-m[0][2];
	k(2,1)=m[0][1]+m[1][0];
	k(2,2)=-m[0][0]+m[1][1]-m[2][2];
	k(2,3)=m[1][2]+m[2][1];
	k(3,0)=m[0][1]-m[1][0];
	k(3,1)=m[2][0]+m[0][2];
	k(3,2)=m[1][2]+m[2][1];
	k(3,3)=-m[0][0]-m[1][1]+m[2][2];
	
	#if 1
	
	/* Test the polynomial: */
	for(double x=-2.0;x<=2.0;x+=1.0/16.0)
		{
		double p1=(((q4*x+q3)*x+q2)*x+q1)*x+q0;
		
		Math::Matrix kp(4,4);
		for(int i=0;i<3;++i)
			for(int j=0;j<4;++j)
				kp(i,j)=i==j?x-k(i,j):-k(i,j);
		double p2=kp.determinant();
		std::cout<<x<<": "<<p1<<", "<<p2<<std::endl;
		}
	
	
	#endif
	
	std::pair<Math::Matrix,Math::Matrix> jacobi=k.jacobiIteration();
	std::cout<<"Eigenvalues of key matrix: ";
	for(int i=0;i<4;++i)
		std::cout<<", "<<jacobi.second(i);
	std::cout<<std::endl;
	double maxE=jacobi.second(0);
	int maxEIndex=0;
	for(int i=1;i<4;++i)
		if(maxE<jacobi.second(i))
			{
			maxE=jacobi.second(i);
			maxEIndex=i;
			}
	std::cout<<"Largest eigenvector: "<<jacobi.first(0,maxEIndex)<<", "<<jacobi.first(1,maxEIndex)<<", "<<jacobi.first(2,maxEIndex)<<", "<<jacobi.first(3,maxEIndex)<<std::endl;
	Transform::Rotation rotation=Transform::Rotation::fromQuaternion(jacobi.first(1,maxEIndex),jacobi.first(2,maxEIndex),jacobi.first(3,maxEIndex),jacobi.first(0,maxEIndex));
	std::cout<<"Result rotation: "<<rotation<<std::endl;
	
	Transform result=Geometry::invert(centroidTransform1);
	result*=Transform::rotate(rotation);
	result*=centroidTransform0;
	
	/* Transform the first point set: */
	for(std::vector<Point>::iterator pIt=points0.begin();pIt!=points0.end();++pIt)
		*pIt=result.transform(*pIt);
	double rmsd=0.0;
	for(size_t i=0;i<numPoints;++i)
		rmsd+=Geometry::sqrDist(points0[i],points1[i]);
	std::cout<<"Best distance: "<<Math::sqrt(rmsd/double(numPoints))<<std::endl;
	
	return result;
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
	typedef Geometry::Box<Scalar,3> Box;
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
	OGTransform preTransform=OGTransform::identity;
	const char* outputFileName=0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"ON")==0)
				transformMode=1;
			else if(strcasecmp(argv[i]+1,"OG")==0)
				transformMode=2;
			else if(strcasecmp(argv[i]+1,"P")==0)
				transformMode=3;
			else if(strcasecmp(argv[i]+1,"S")==0)
				{
				++i;
				preScale=Scalar(atof(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"PRE")==0)
				{
				++i;
				preTransform=Misc::ValueCoder<OGTransform>::decode(argv[i],argv[i]+strlen(argv[i]),0);
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
	std::vector<bool> invalidPoints;
	for(int pointSet=0;pointSet<2;++pointSet)
		{
		/* Open the input file: */
		IO::ValueSource reader(Vrui::openFile(fileName[pointSet]));
		reader.setWhitespace(',',true);
		reader.setPunctuation('\n',true);
		reader.skipWs();
		
		/* Read points until end of file: */
		while(!reader.eof())
			{
			Point p;
			bool valid=true;
			try
				{
				for(int i=0;i<3;++i)
					p[i]=Scalar(reader.readNumber());
				}
			catch(IO::ValueSource::NumberError)
				{
				for(int i=0;i<3;++i)
					p[i]=Math::Constants<Scalar>::max;
				valid=false;
				}
			points[pointSet].push_back(p);
			if(invalidPoints.size()<points[pointSet].size())
				invalidPoints.push_back(!valid);
			else
				invalidPoints[points[pointSet].size()-1]=!valid||invalidPoints[points[pointSet].size()-1];
			reader.skipLine();
			reader.skipWs();
			}
		}
	size_t numPoints=points[0].size();
	if(numPoints>points[1].size())
		numPoints=points[1].size();
	
	/* Remove invalid point pairs: */
	for(size_t i=invalidPoints.size();i>0;--i)
		if(i<=numPoints&&invalidPoints[i-1])
			{
			for(int pointSet=0;pointSet<2;++pointSet)
				points[pointSet].erase(points[pointSet].begin()+i-1);
			--numPoints;
			}
	
	/* Print point pairs: */
	for(size_t i=0;i<numPoints;++i)
		std::cout<<points[0][i]<<" <=> "<<points[1][i]<<std::endl;
	
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
			{
			if(preTransform.getScaling()!=1.0)
				{
				OGTransform finalTransform=preTransform;
				finalTransform*=findTransform<ONTransformFitter>(points[0],points[1]);
				std::cout<<"Best transformation: "<<Misc::ValueCoder<OGTransform>::encode(finalTransform)<<std::endl;
				}
			else
				{
				ONTransform finalTransform(preTransform.getTranslation(),preTransform.getRotation());
				finalTransform*=findTransform<ONTransformFitter>(points[0],points[1]);
				std::cout<<"Best transformation: "<<Misc::ValueCoder<ONTransform>::encode(finalTransform)<<std::endl;
				}
			break;
			}
		
		case 2: // Orthogonal transformation
			{
			OGTransform finalTransform=preTransform;
			#if 1
			OGTransform first=findTransform2<OGTransformFitter>(points[0],points[1]);
			finalTransform*=findTransform<OGTransformFitter>(points[0],points[1]);
			finalTransform*=first;
			finalTransform.renormalize();
			#else
			finalTransform*=findTransform<OGTransformFitter>(points[0],points[1]);
			#endif
			std::cout<<"Best transformation: "<<Misc::ValueCoder<OGTransform>::encode(finalTransform)<<std::endl;
			break;
			}
		
		case 3: // Projective transformation
			{
			PTransform finalTransform=preTransform;
			
			/* Calculate an orthogonal transformation first: */
			PTransform bestOg=findTransform2<OGTransformFitter>(points[0],points[1]);
			
			/* Fine-tune it with a projective transformation: */
			finalTransform*=findTransform<PTransformFitter>(points[0],points[1]);
			finalTransform*=bestOg;
			std::cout<<"Best transformation: "<<Misc::ValueCoder<PTransform>::encode(finalTransform)<<std::endl;
			break;
			}
		}
	
	if(outputFileName!=0)
		{
		/* Write the transformed point set: */
		Misc::File outputFile(outputFileName,"wt");
		for(std::vector<Point>::const_iterator pIt=points[0].begin();pIt!=points[0].end();++pIt)
			fprintf(outputFile.getFilePtr(),"%10.6f, %10.6f, %10.6f\n",double((*pIt)[0]),double((*pIt)[1]),double((*pIt)[2]));
		}
	
	/* Calculate the two point sets' joint bounding box: */
	Box bbox=Box::empty;
	for(int i=0;i<2;++i)
		for(std::vector<Point>::const_iterator pIt=points[i].begin();pIt!=points[i].end();++pIt)
			bbox.addPoint(*pIt);
	
	Point center=Geometry::mid(bbox.min,bbox.max);
	Scalar size=Geometry::dist(bbox.min,bbox.max);
	Vrui::setNavigationTransformation(Vrui::Point(center),Vrui::Scalar(size));
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
