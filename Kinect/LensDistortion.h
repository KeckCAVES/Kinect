/***********************************************************************
LensDistortion - Lens distortion formula used by Kinect v2.
Copyright (c) 2015-2017 Oliver Kreylos

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

#ifndef KINECT_LENSDISTORTION_INCLUDED
#define KINECT_LENSDISTORTION_INCLUDED

#include <Geometry/Point.h>
#include <Geometry/Vector.h>

/* Forward declarations: */
namespace IO {
class File;
}
namespace Geometry {
template <class ScalarParam,int dimensionParam>
class ProjectiveTransformation;
}

namespace Kinect {

class LensDistortion
	{
	/* Embedded classes: */
	public:
	typedef double Scalar; // Scalar type for calculations
	typedef Geometry::Point<Scalar,2> Point; // Type for points in distorted and undistorted image space
	typedef Geometry::Vector<Scalar,2> Vector; // Type for vectors in distorted and undistorted image space
	
	/* Elements: */
	private:
	Point center; // Distortion center
	Scalar kappas[3]; // Radial distortion coefficients
	Scalar rhos[2]; // Tangential distortion coefficients
	Scalar undistortMaxError; // Convergence threshold for Newton-Raphson iteration in undistortion formula
	int undistortMaxSteps; // Maximum number of Newton-Raphson steps in undistortion formula
	Scalar fx,sk,cx,fy,cy; // Projection parameters to convert between pixel space and tangent space
	
	/* Constructors and destructors: */
	public:
	LensDistortion(void); // Creates an identity lens distortion correction formula
	LensDistortion(IO::File& file); // Reads lens distortion correction formula from given binary file
	
	/* Methods: */
	bool isIdentity(void) const // Returns true if this is a no-op identity lens distortion correction
		{
		return kappas[0]==Scalar(0)&&kappas[1]==Scalar(0)&&kappas[2]==Scalar(0)&&rhos[0]==Scalar(0)&&rhos[1]==Scalar(0);
		}
	bool isRadial(void) const // Returns true if this is a radial-only lens distortion correction formula
		{
		return rhos[0]==Scalar(0)&&rhos[1]==Scalar(0);
		}
	const Point& getCenter(void) const
		{
		return center;
		}
	const Scalar* getKappas(void) const
		{
		return kappas;
		}
	Scalar getKappa(int index) const
		{
		return kappas[index];
		}
	const Scalar* getRhos(void) const
		{
		return rhos;
		}
	Scalar getRho(int index) const
		{
		return rhos[index];
		}
	void write(IO::File& file) const; // Writes lens distortion correction formula to given binary file
	void setCenter(const Point& newCenter);
	void setKappas(const Scalar newKappas[3]);
	void setKappa(int index,Scalar newKappa);
	void setRhos(const Scalar newRhos[2]);
	void setRho(int index,Scalar newRho);
	void read(IO::File& file); // Reads lens distortion correction formula from given binary file
	void setProjection(Geometry::ProjectiveTransformation<Scalar,3>& unprojection); // Calculates conversion parameters between pixel and tangent space from the given depth unprojection transformation
	Point distort(const Point& undistorted) const // Calculates forward lens distortion correction formula for the given tangent-space point
		{
		Vector d=undistorted-center;
		Scalar r2=d.sqr();
		Scalar div=Scalar(1)+(kappas[0]+(kappas[1]+kappas[2]*r2)*r2)*r2; // Cubic radial distortion formula in r^2
		return Point(center[0]+d[0]*div+Scalar(2)*rhos[0]*d[0]*d[1]+rhos[1]*(r2+Scalar(2)*d[0]*d[0]), // Tangential distortion formula in x
		             center[1]+d[1]*div+rhos[0]*(r2+Scalar(2)*d[1]*d[1])+Scalar(2)*rhos[1]*d[0]*d[1]); // Tangential distortion formula in y
		}
	Point distortPixel(const Point& undistortedPixel) const // Ditto, for pixel-space point
		{
		/* Transform the point to tangent space: */
		Point utp;
		utp[1]=(undistortedPixel[1]-cy)/fy;
		utp[0]=(undistortedPixel[0]-sk*utp[1]-cx)/fx;
		
		/* Calculate the distorted point in tangent space: */
		Point dtp=distort(utp);
		
		/* Return the distorted point transformed back to pixel space: */
		return Point(dtp[0]*fx+dtp[1]*sk+cx,dtp[1]*fy+cy);
		}
	Scalar distortScale(const Point& undistorted) const; // Calculates the differential scaling factor of the forward distortion correction formula at the given undistorted tangent-space point
	Scalar distortScalePixel(const Point& undistortedPixel) const // Ditto, for pixel-space point
		{
		/* Transform the point to tangent space: */
		Point utp;
		utp[1]=(undistortedPixel[1]-cy)/fy;
		utp[0]=(undistortedPixel[0]-sk*utp[1]-cx)/fx;
		
		/* Return the distortion scale at the tangent-space position: */
		return distortScale(utp);
		}
	Scalar getUndistortMaxError(void) const
		{
		return undistortMaxError;
		}
	int getUndistortMaxSteps(void) const
		{
		return undistortMaxSteps;
		}
	void setUndistortMaxError(Scalar newUndistortMaxError);
	void setUndistortMaxSteps(int newUndistortMaxSteps);
	Point undistort(const Point& distorted) const; // Calculates inverse lens distortion correction formula via Newton-Raphson iteration for the given tangent-space point
	Point undistortPixel(const Point& distortedPixel) const // Ditto, for pixel-space point
		{
		/* Transform the point to tangent space: */
		Point dtp;
		dtp[1]=(distortedPixel[1]-cy)/fy;
		dtp[0]=(distortedPixel[0]-sk*dtp[1]-cx)/fx;
		
		/* Calculate the undistorted point in tangent space: */
		Point utp=undistort(dtp);
		
		/* Return the undistorted point transformed back to pixel space: */
		return Point(utp[0]*fx+utp[1]*sk+cx,utp[1]*fy+cy);
		}
	Point undistortPixel(unsigned int x,unsigned int y) const // Ditto, for midpoint of given pixel
		{
		/* Transform the point to tangent space: */
		Point dtp;
		dtp[1]=(Scalar(y)+Scalar(0.5)-cy)/fy;
		dtp[0]=(Scalar(x)+Scalar(0.5)-sk*dtp[1]-cx)/fx;
		
		/* Calculate the undistorted point in tangent space: */
		Point utp=undistort(dtp);
		
		/* Return the undistorted point transformed back to pixel space: */
		return Point(utp[0]*fx+utp[1]*sk+cx,utp[1]*fy+cy);
		}
	};

}

#endif
