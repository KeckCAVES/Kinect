/***********************************************************************
PTransformSimplexFitter - Functor plug-in to find the best projective
transformation transforming a source point set into a target point set
using a simplex minimizer.
Copyright (c) 2014 Oliver Kreylos

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

#ifndef PTRANSFORMSIMPLEXFITTER_INCLUDED
#define PTRANSFORMSIMPLEXFITTER_INCLUDED

#include <Geometry/Point.h>
#include <Geometry/ProjectiveTransformation.h>

class PTransformSimplexFitter
	{
	/* Embedded classes: */
	public:
	typedef double Scalar; // Scalar type
	static const unsigned int dimension=16; // Dimension of optimization space
	typedef Geometry::Point<Scalar,3> Point;
	typedef Geometry::ProjectiveTransformation<Scalar,3> Transform;
	
	struct Vertex:public Transform // Type for simplex vertices
		{
		/* Constructors and destructors: */
		public:
		Vertex(void)
			{
			}
		Vertex(const Transform& source)
			:Transform(source)
			{
			}
		
		/* Methods: */
		public:
		void move(unsigned int componentIndex,Scalar scale)
			{
			if(componentIndex%4!=3)
				scale*=Scalar(0.001);
			getMatrix().getEntries()[componentIndex]+=scale;
			}
		static Vertex move(const Vertex& vertex,const Vertex& faceCenter,Scalar distance)
			{
			Vertex result;
			const Transform::Matrix& vm=vertex.getMatrix();
			const Transform::Matrix& fcm=faceCenter.getMatrix();
			Transform::Matrix& rm=result.getMatrix();
			for(int i=0;i<4;++i)
				for(int j=0;j<4;++j)
					rm(i,j)=fcm(i,j)+(fcm(i,j)-vm(i,j))*distance;
			return result;
			}
		static Vertex calcFaceCenter(const Vertex vertices[],unsigned int worstVertex)
			{
			Vertex result;
			Transform::Matrix& rm=result.getMatrix();
			rm=Transform::Matrix::zero;
			for(unsigned int vertexIndex=0;vertexIndex<=dimension;++vertexIndex)
				if(vertexIndex!=worstVertex)
					rm+=vertices[vertexIndex].getMatrix();
			rm*=Scalar(1)/Scalar(dimension);
			return result;
			}
		static bool isTooSmall(const Vertex vertices[])
			{
			/* Calculate the value range of all matrix components: */
			for(int i=0;i<4;++i)
				for(int j=0;j<4;++j)
					{
					Scalar min=vertices[0].getMatrix()(i,j);
					Scalar max=vertices[0].getMatrix()(i,j);
					for(unsigned int vertexIndex=1;vertexIndex<=dimension;++vertexIndex)
						{
						if(min>vertices[vertexIndex].getMatrix()(i,j))
							min=vertices[vertexIndex].getMatrix()(i,j);
						else if(max<vertices[vertexIndex].getMatrix()(i,j))
							max=vertices[vertexIndex].getMatrix()(i,j);
						}
					if(max-min>Scalar(1.0e-20))
						return false;
					}
			return true;
			}
		};
	
	typedef double Value;
	
	/* Elements: */
	private:
	size_t numPoints; // Number of source and target points
	const Point* sp; // Array of source points
	const Point* tp; // Array of target points
	
	/* Constructors and destructors: */
	public:
	PTransformSimplexFitter(size_t sNumPoints,const Point* sSp,const Point* sTp)
		:numPoints(sNumPoints),sp(sSp),tp(sTp)
		{
		}
	
	/* Methods: */
	Value operator()(const Vertex& vertex) const
		{
		Value d2(0);
		for(size_t i=0;i<numPoints;++i)
			d2+=Geometry::sqrDist(vertex.transform(sp[i]),tp[i]);
		return d2;
		}
	};

#endif
