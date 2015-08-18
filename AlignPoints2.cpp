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
#include <string>
#include <vector>
#include <iostream>
#include <Misc/File.h>
#include <IO/ValueSource.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#include <Math/SimplexMinimizer.h>
#define GEOMETRY_NONSTANDARD_TEMPLATES
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Box.h>
#include <Geometry/ValuedPoint.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/OutputOperators.h>
#include <Geometry/LevenbergMarquardtMinimizer.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLColor.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/OpenFile.h>
#include <Vrui/Application.h>

#include "ONTransformFitter.h"
#include "OGTransformFitter.h"
#include "PTransformFitter.h"
#include "PTransformSimplexFitter.h"

#define VERBOSE 0

template <class TargetTransformParam>
inline
TargetTransformParam
convertTransform(
	const Geometry::OrthogonalTransformation<double,3>& t)
	{
	return t;
	}

template <>
inline
Geometry::OrthonormalTransformation<double,3>
convertTransform<Geometry::OrthonormalTransformation<double,3> >(
	const Geometry::OrthogonalTransformation<double,3>& t)
	{
	return Geometry::OrthonormalTransformation<double,3>(t.getTranslation(),t.getRotation());
	}

template <class TransformFitterParam>
class PairAlignerBase // Generic base class to align two point sets via different classes of transformations
	{
	/* Embedded classes: */
	public:
	typedef TransformFitterParam TransformFitter;
	typedef typename TransformFitter::Scalar Scalar;
	typedef typename TransformFitter::Point Point;
	typedef std::vector<Point> PointList;
	typedef Geometry::ValuedPoint<Point,bool> VPoint;
	typedef std::vector<VPoint> VPointList;
	typedef typename TransformFitter::Transform Transform;
	typedef Geometry::OrthogonalTransformation<Scalar,3> OGTransform;
	
	/* Methods: */
	static size_t reduce(const VPointList& vpoints0,const VPointList& vpoints1,PointList& points0,PointList& points1) // Removes invalid point pairs
		{
		/* Extract valid point pairs from the input point sets: */
		size_t result=0;
		size_t numPoints=Math::min(vpoints0.size(),vpoints1.size());
		points0.reserve(numPoints);
		points1.reserve(numPoints);
		typename VPointList::const_iterator vp0It;
		typename VPointList::const_iterator vp1It;
		for(vp0It=vpoints0.begin(),vp1It=vpoints1.begin();vp0It!=vpoints0.end()&&vp1It!=vpoints1.end();++vp0It,++vp1It)
			if(vp0It->value&&vp1It->value)
				{
				points0.push_back(*vp0It);
				points1.push_back(*vp1It);
				++result;
				}
		return result;
		}
	static Transform center(PointList& points) // Centers a point set around the origin; returns used transformation
		{
		/* Calculate the point set's centroid and a transformation to move the centroid to the origin: */
		typename Point::AffineCombiner cc;
		for(typename PointList::iterator pIt=points.begin();pIt!=points.end();++pIt)
			cc.addPoint(*pIt);
		typename Point::Vector t=Point::origin-cc.getPoint();
		
		/* Transform the point set: */
		for(typename PointList::iterator pIt=points.begin();pIt!=points.end();++pIt)
			*pIt+=t;
		
		/* Return the result transformation: */
		return Transform::translate(t);
		}
	static OGTransform preAlign(size_t numPoints,const PointList& points0,const PointList& points1) // Finds optimal transformation from first to second point set, both assumed centered around the origin
		{
		/* Calculate both point sets' inner products: */
		double ip0=0.0;
		double ip1=0.0;
		for(size_t pi=0;pi<numPoints;++pi)
			{
			ip0+=Geometry::sqr(points0[pi]);
			ip1+=Geometry::sqr(points1[pi]);
			}
		
		/* Calculate the normalizing scaling factors: */
		double scale0=Math::sqrt(ip0);
		double scale1=Math::sqrt(ip1);
		#if VERBOSE
		std::cout<<"Scales: "<<scale0<<", "<<scale1<<", result scale = "<<scale1/scale0<<std::endl;
		#endif
		
		/* Calculate the inner product between the two normalized point sets: */
		double m[3][3];
		for(int i=0;i<3;++i)
			for(int j=0;j<3;++j)
				m[i][j]=0.0;
		for(size_t pi=0;pi<numPoints;++pi)
			for(int i=0;i<3;++i)
				for(int j=0;j<3;++j)
					m[i][j]+=points0[pi][i]*points1[pi][j]/(scale0*scale1);
		
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
		#if VERBOSE
		std::cout<<"Largest eigenvalue of key matrix: "<<lambda<<std::endl;
		#endif
		
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
		
		#if VERBOSE
		
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
		#if VERBOSE
		std::cout<<"Eigenvalues of key matrix: ";
		for(int i=0;i<4;++i)
			std::cout<<", "<<jacobi.second(i);
		std::cout<<std::endl;
		#endif
		double maxE=jacobi.second(0);
		int maxEIndex=0;
		for(int i=1;i<4;++i)
			if(maxE<jacobi.second(i))
				{
				maxE=jacobi.second(i);
				maxEIndex=i;
				}
		#if VERBOSE
		std::cout<<"Largest eigenvector: "<<jacobi.first(0,maxEIndex)<<", "<<jacobi.first(1,maxEIndex)<<", "<<jacobi.first(2,maxEIndex)<<", "<<jacobi.first(3,maxEIndex)<<std::endl;
		#endif
		typename Transform::Rotation rotation=Transform::Rotation::fromQuaternion(jacobi.first(1,maxEIndex),jacobi.first(2,maxEIndex),jacobi.first(3,maxEIndex),jacobi.first(0,maxEIndex));
		#if VERBOSE
		std::cout<<"Result rotation: "<<rotation<<std::endl;
		#endif
		
		/* Calculate the result transformation by combining the optimal rotation with the optimal scale factor: */
		return OGTransform(Transform::Vector::zero,rotation,scale1/scale0);
		}
	static double calcRmsd(size_t numPoints,const PointList& points0,const Transform& transform,const PointList& points1)
		{
		/* Calculate the RMS between the transformed first and second point set: */
		double d2=0.0;
		for(size_t i=0;i<numPoints;++i)
			d2+=double(Geometry::sqrDist(transform.transform(points0[i]),points1[i]));
		
		return Math::sqrt(d2/double(numPoints));
		}
	static double calcRmsd(const VPointList& points0,const Transform& transform,const VPointList& points1)
		{
		/* Get the number of common points in both point sets, just in case: */
		size_t numPoints=Math::min(points0.size(),points1.size());
		
		/* Calculate the RMS between the transformed first and second point set: */
		double d2=0.0;
		size_t np=0;
		for(size_t i=0;i<numPoints;++i)
			if(points0[i].value&&points1[i].value) // Only consider pairs of valid points
				{
				d2+=double(Geometry::sqrDist(transform.transform(points0[i]),points1[i]));
				++np;
				}
		
		return Math::sqrt(d2/double(np));
		}
	static VPointList transform(const VPointList& points,const Transform& transform)
		{
		/* Transform all points: */
		VPointList result;
		for(typename VPointList::const_iterator pIt=points.begin();pIt!=points.end();++pIt)
			result.push_back(VPoint(transform.transform(*pIt),pIt->value));
		
		return result;
		}
	};

template <class TransformFitterParam>
class PairAligner:public PairAlignerBase<TransformFitterParam> // Generic class to align two point sets via different classes of transformations
	{
	/* Embedded classes: */
	public:
	typedef PairAlignerBase<TransformFitterParam> Base;
	typedef typename Base::TransformFitter TransformFitter;
	typedef typename Base::Scalar Scalar;
	typedef typename Base::Point Point;
	typedef typename Base::PointList PointList;
	typedef typename Base::VPoint VPoint;
	typedef typename Base::VPointList VPointList;
	typedef typename Base::Transform Transform;
	typedef typename Base::OGTransform OGTransform;
	
	/* Methods: */
	static Transform align(const VPointList& vpoints0,const VPointList& vpoints1) // Finds optimal transformation from first to second point set
		{
		/* Reduce the point sets: */
		PointList points0,points1;
		size_t numPoints=Base::reduce(vpoints0,vpoints1,points0,points1);
		
		/* Center both point sets around the origin: */
		Transform centroidTransform0=Base::center(points0);
		Transform centroidTransform1=Base::center(points1);
		
		/* Initialize the iteration transformation: */
		Transform bestTransform=convertTransform<Transform>(Base::preAlign(numPoints,points0,points1));
		std::cout<<"Pre-alignment RMSD: "<<Base::calcRmsd(numPoints,points0,bestTransform,points1)<<std::endl;
		#if 0
		Transform preTransform=bestTransform;
		for(typename PointList::iterator p0It=points0.begin();p0It!=points0.end();++p0It)
			*p0It=preTransform.transform(*p0It);
		bestTransform=Transform::identity;
		#endif
		Scalar bestDistance=Math::Constants<Scalar>::max;
		
		for(int i=0;i<5;++i)
			{
			/* Minimize the distance: */
			Geometry::LevenbergMarquardtMinimizer<TransformFitter> minimizer;
			minimizer.maxNumIterations=100000;
			minimizer.tau=1.0e-1;
			TransformFitter tf(numPoints,&points0[0],&points1[0]);
			tf.setTransform(bestTransform);
			Scalar result=minimizer.minimize(tf);
			if(bestDistance>result)
				{
				bestTransform=tf.getTransform();
				bestDistance=result;
				}
			}
		
		/* Undo the centroid transformation: */
		bestTransform.leftMultiply(Geometry::invert(centroidTransform1));
		// bestTransform*=preTransform;
		bestTransform*=centroidTransform0;
		
		/* Print the result: */
		#if VERBOSE
		std::cout<<"Best residual: "<<bestDistance<<std::endl;
		#endif
		
		return bestTransform;
		}
	};

template <>
class PairAligner<PTransformSimplexFitter>:public PairAlignerBase<PTransformSimplexFitter> // Specialized class for projective transformations and simplex minimization
	{
	/* Embedded classes: */
	public:
	typedef PairAlignerBase<PTransformSimplexFitter> Base;
	typedef typename Base::TransformFitter TransformFitter;
	typedef typename Base::Scalar Scalar;
	typedef typename Base::Point Point;
	typedef typename Base::PointList PointList;
	typedef typename Base::VPoint VPoint;
	typedef typename Base::VPointList VPointList;
	typedef typename Base::Transform Transform;
	typedef typename Base::OGTransform OGTransform;
	typedef typename TransformFitter::Vertex Vertex;
	typedef typename TransformFitter::Value Value;
	typedef Math::SimplexMinimizer<TransformFitter> SimplexMinimizer;
	typedef typename SimplexMinimizer::ValuedVertex ValuedVertex;
	
	/* Methods: */
	static Transform align(const VPointList& vpoints0,const VPointList& vpoints1) // Finds optimal transformation from first to second point set
		{
		/* Reduce the point sets: */
		PointList points0,points1;
		size_t numPoints=Base::reduce(vpoints0,vpoints1,points0,points1);
		
		/* Center both point sets around the origin: */
		Transform centroidTransform0=Base::center(points0);
		Transform centroidTransform1=Base::center(points1);
		
		/* Initialize the iteration transformation: */
		OGTransform pre=Base::preAlign(numPoints,points0,points1);
		Vertex initialVertex(convertTransform<Transform>(pre));
		std::cout<<"Pre-transformation: "<<initialVertex<<std::endl;
		TransformFitter tf(numPoints,&points0[0],&points1[0]);
		Value initialValue=tf(initialVertex);
		std::cout<<"Pre-alignment RMSD: "<<Math::sqrt(initialValue/Value(numPoints))<<std::endl;
		
		/* Create a minimizer and run the minimization loop: */
		SimplexMinimizer minimizer;
		minimizer.maxNumSteps=100000;
		ValuedVertex best=minimizer.minimize(tf,initialVertex,initialValue*0.01);
		std::cout<<"Post-alignment RMSD: "<<Math::sqrt(best.second/Value(numPoints))<<std::endl;
		std::cout<<"Post-alignment transformation: "<<best.first<<std::endl;
		
		/* Undo the centroid transformation: */
		best.first.leftMultiply(Geometry::invert(centroidTransform1));
		best.first*=centroidTransform0;
		
		return best.first;
		}
	};

template <class TransformFitterParam>
inline
void
calcBundleAlignment(
	unsigned int numPointSets,
	std::vector<Geometry::ValuedPoint<typename TransformFitterParam::Point,bool> > pointSets[])
	{
	typedef TransformFitterParam TransformFitter;
	typedef PairAligner<TransformFitter> PA;
	typedef typename TransformFitter::Transform Transform;
	
	/* Calculate all pairwise alignment transformations: */
	Transform* transforms=new Transform[numPointSets*numPointSets];
	for(unsigned int i=0;i<numPointSets;++i)
		{
		/* Transformation from set i to set i is identity: */
		transforms[i*numPointSets+i]=Transform::identity;
		
		for(unsigned int j=i+1;j<numPointSets;++j)
			{
			/* Transformation from set i to set j: */
			transforms[i*numPointSets+j]=PA::align(pointSets[i],pointSets[j]);
			
			/* Transformation from set j to set i is just the inverse: */
			transforms[j*numPointSets+i]=Geometry::invert(transforms[i*numPointSets+j]);
			}
		}
	
	/* Calculate residuals between first and all other point sets: */
	for(unsigned int i=1;i<numPointSets;++i)
		{
		/* Calculate direct RMSD between sets 0 and i: */
		std::cout<<"Direct RMSD between "<<0<<" and "<<i<<":   "<<PA::calcRmsd(pointSets[i],transforms[i*numPointSets+0],pointSets[0])<<std::endl;
		}
	
	/* Calculate residuals between all pairs of point sets: */
	for(unsigned int i=1;i<numPointSets;++i)
		for(unsigned int j=i+1;j<numPointSets;++j)
			{
			/* Calculate direct RMSD between sets i and j: */
			std::cout<<"Direct RMSD between "<<i<<" and "<<j<<":   "<<PA::calcRmsd(pointSets[i],transforms[i*numPointSets+j],pointSets[j])<<std::endl;
			
			/* Calculate indirect RMSD when going through point set 0: */
			Transform t=transforms[0*numPointSets+j]*transforms[i*numPointSets+0];
			std::cout<<"Indirect RMSD between "<<i<<" and "<<j<<": "<<PA::calcRmsd(pointSets[i],t,pointSets[j])<<std::endl;
			}
	
	/* Print transformations from all other point sets to the first one: */
	for(unsigned int i=1;i<numPointSets;++i)
		std::cout<<"Best transformation from set "<<i<<" to set 0: "<<transforms[i*numPointSets+0]<<std::endl;
	
	/* Transform all point sets to the first one: */
	for(unsigned int i=1;i<numPointSets;++i)
		pointSets[i]=PA::transform(pointSets[i],transforms[i*numPointSets+0]);
	}

class AlignPoints:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef double Scalar;
	typedef Geometry::Point<Scalar,3> Point;
	typedef Geometry::Box<Scalar,3> Box;
	typedef Geometry::ValuedPoint<Point,bool> VPoint;
	typedef Geometry::OrthonormalTransformation<Scalar,3> ONTransform;
	typedef Geometry::OrthogonalTransformation<Scalar,3> OGTransform;
	typedef Geometry::ProjectiveTransformation<Scalar,3> PTransform;
	
	/* Elements: */
	unsigned int numPointSets; // Number of point sets to be aligned
	std::vector<VPoint>* points; // Array of point sets with per-point validity flags
	
	/* Constructors and destructors: */
	public:
	AlignPoints(int& argc,char**& argv,char**& appDefaults);
	virtual ~AlignPoints(void);
	
	/* Methods from Vrui::Application: */
	virtual void display(GLContextData& contextData) const;
	};

AlignPoints::AlignPoints(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 points(0)
	{
	/* Parse the command line: */
	std::vector<std::string> pointFileNames;
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
			else if(strcasecmp(argv[i]+1,"PS")==0)
				transformMode=4;
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
		else
			pointFileNames.push_back(argv[i]);
		}
	
	/* Read all point sets: */
	numPointSets=pointFileNames.size();
	points=new std::vector<VPoint>[numPointSets];
	for(unsigned int pointSet=0;pointSet<numPointSets;++pointSet)
		{
		/* Open the input file: */
		IO::ValueSource reader(Vrui::openFile(pointFileNames[pointSet].c_str()));
		reader.setWhitespace(',',true);
		reader.setPunctuation('\n',true);
		reader.skipWs();
		
		/* Read points until end of file: */
		while(!reader.eof())
			{
			VPoint p;
			p.value=true;
			try
				{
				for(int i=0;i<3;++i)
					p[i]=Scalar(reader.readNumber());
				}
			catch(IO::ValueSource::NumberError)
				{
				for(int i=0;i<3;++i)
					p[i]=Math::Constants<Scalar>::max;
				p.value=false;
				}
			points[pointSet].push_back(p);
			reader.skipLine();
			reader.skipWs();
			}
		}
	
	#if 0
	
	if(preScale!=Scalar(1))
		{
		/* Scale the first point set: */
		for(std::vector<VPoint>::iterator pIt=points[0].begin();pIt!=points[0].end();++pIt)
			*pIt=VPoint(Point::origin+(*pIt-Point::origin)*preScale,pIt->value);
		}
	
	#endif
	
	/* Align the point sets: */
	switch(transformMode)
		{
		case 1: // Orthonormal transformation
			calcBundleAlignment<ONTransformFitter>(numPointSets,points);
			break;
		
		case 2: // Orthogonal transformation
			calcBundleAlignment<OGTransformFitter>(numPointSets,points);
			break;
		
		case 3: // Projective transformation
			calcBundleAlignment<PTransformFitter>(numPointSets,points);
			break;
		
		case 4: // Projective transformation with simplex minimization
			calcBundleAlignment<PTransformSimplexFitter>(numPointSets,points);
			break;
		}
	
	#if 0
	
	if(outputFileName!=0)
		{
		/* Write the transformed point set: */
		Misc::File outputFile(outputFileName,"wt");
		for(std::vector<Point>::const_iterator pIt=points[0].begin();pIt!=points[0].end();++pIt)
			fprintf(outputFile.getFilePtr(),"%10.6f, %10.6f, %10.6f\n",double((*pIt)[0]),double((*pIt)[1]),double((*pIt)[2]));
		}
	
	#endif
	
	/* Calculate the point sets' joint bounding box: */
	Box bbox=Box::empty;
	for(unsigned int i=0;i<numPointSets;++i)
		for(std::vector<VPoint>::const_iterator pIt=points[i].begin();pIt!=points[i].end();++pIt)
			if(pIt->value)
				bbox.addPoint(*pIt);
	
	Point center=Geometry::mid(bbox.min,bbox.max);
	Scalar size=Geometry::dist(bbox.min,bbox.max);
	Vrui::setNavigationTransformation(Vrui::Point(center),Vrui::Scalar(size));
	}

AlignPoints::~AlignPoints(void)
	{
	delete[] points;
	}

void AlignPoints::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glLineWidth(1.0f);
	
	static const GLColor<GLfloat,3> pointSetColors[]=
		{
		GLColor<GLfloat,3>(1.0f,0.0f,0.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(0.0f,0.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,0.0f)
		};
	
	for(unsigned int pointSet=0;pointSet<numPointSets;++pointSet)
		{
		glBegin(GL_POINTS);
		glColor(pointSetColors[pointSet]);
		for(std::vector<VPoint>::const_iterator pIt=points[pointSet].begin();pIt!=points[pointSet].end();++pIt)
			if(pIt->value)
				glVertex(*pIt);
		glEnd();
		}
	
	glBegin(GL_LINES);
	for(unsigned int pointSet0=0;pointSet0<numPointSets-1;++pointSet0)
		for(unsigned int pointSet1=pointSet0+1;pointSet1<numPointSets;++pointSet1)
			{
			for(size_t i=0;i<points[pointSet0].size()&&i<points[pointSet1].size();++i)
				{
				if(points[pointSet0][i].value&&points[pointSet1][i].value)
					{
					glColor(pointSetColors[pointSet0]);
					glVertex(points[pointSet0][i]);
					glColor(pointSetColors[pointSet1]);
					glVertex(points[pointSet1][i]);
					}
				}
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
