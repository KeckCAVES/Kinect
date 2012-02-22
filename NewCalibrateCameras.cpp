/***********************************************************************
NewCalibrateCameras - Utility to calibrate a Kinect's color camera to
its depth camera based on a set of tie points.
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

#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <Math/Matrix.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Plane.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/OutputOperators.h>
#include <Geometry/GeometryMarshallers.h>

typedef Geometry::ProjectiveTransformation<double,2> Homography;
typedef Homography::Point Point;
typedef Point::Vector Vector;
typedef Geometry::Plane<double,3> Plane;

struct TiePoint
	{
	/* Elements: */
	public:
	Homography depthHom; // Depth homography
	Plane gridPlane; // Plane equation of grid in depth image space
	Homography colorHom; // Color homography
	};

void calibrate(const std::vector<TiePoint>& tiePoints,const int gridSize[2],const double tileSize[2],const unsigned int depthFrameSize[2],const unsigned int colorFrameSize[2])
	{
	/* Initialize the depth camera's intrinsic parameter matrix: */
	Math::Matrix depthV(6,6,0.0);
	
	/* Process all tie points: */
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Enter the tie point's depth homography into the intrinsic parameter matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(depthFrameSize[0]);
		hm*=tpIt->depthHom.getMatrix();
		Homography::Matrix scale(1.0);
		scale(0,0)=1.0/tileSize[0];
		scale(1,1)=1.0/tileSize[1];
		hm*=scale;
		double row[3][6];
		static const int is[3]={0,0,1};
		static const int js[3]={1,0,1};
		for(int r=0;r<3;++r)
			{
			int i=is[r];
			int j=js[r];
			row[r][0]=hm(0,i)*hm(0,j);
			row[r][1]=hm(0,i)*hm(1,j)+hm(1,i)*hm(0,j);
			row[r][2]=hm(0,i)*hm(2,j)+hm(2,i)*hm(0,j);
			row[r][3]=hm(1,i)*hm(1,j);
			row[r][4]=hm(1,i)*hm(2,j)+hm(2,i)*hm(1,j);
			row[r][5]=hm(2,i)*hm(2,j);
			}
		for(int i=0;i<6;++i)
			row[1][i]-=row[2][i];
		for(int r=0;r<2;++r)
			{
			for(unsigned int i=0;i<6;++i)
				for(unsigned int j=0;j<6;++j)
					depthV(i,j)+=row[r][i]*row[r][j];
			}
		}
	
	/* Find the intrinsic parameter linear system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> depthQe=depthV.jacobiIteration();
	unsigned int minEIndex=0;
	double minE=Math::abs(depthQe.second(0));
	for(unsigned int i=1;i<6;++i)
		{
		if(minE>Math::abs(depthQe.second(i)))
			{
			minEIndex=i;
			minE=Math::abs(depthQe.second(i));
			}
		}
	std::cout<<"Smallest eigenvalue of v = "<<depthQe.second(minEIndex)<<std::endl;
	
	/* Calculate the intrinsic parameters: */
	Math::Matrix b=depthQe.first.getColumn(minEIndex);
	std::cout<<b(0)<<", "<<b(1)<<", "<<b(2)<<", "<<b(3)<<", "<<b(4)<<", "<<b(5)<<std::endl;
	double v0=(b(1)*b(2)-b(0)*b(4))/(b(0)*b(3)-Math::sqr(b(1)));
	double lambda=b(5)-(Math::sqr(b(2))+v0*(b(1)*b(2)-b(0)*b(4)))/b(0);
	double alpha=Math::sqrt(lambda/b(0));
	double beta=Math::sqrt(lambda*b(0)/(b(0)*b(3)-Math::sqr(b(1))));
	double gamma=-b(1)*Math::sqr(alpha)*beta/lambda;
	double u0=gamma*v0/beta-b(2)*Math::sqr(alpha)/lambda;
	
	std::cout<<"Intrinsic camera parameters:"<<std::endl;
	std::cout<<alpha<<" "<<gamma<<" "<<u0<<std::endl;
	std::cout<<0.0<<" "<<beta<<" "<<v0<<std::endl;
	std::cout<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;
	
	/* Create the intrinsic camera parameter matrix: */
	Math::Matrix a(3,3,1.0);
	a.set(0,0,alpha);
	a.set(0,1,gamma);
	a.set(0,2,u0);
	a.set(1,1,beta);
	a.set(1,2,v0);
	Math::Matrix aInv=a.inverse();
	
	/* Calculate extrinsic parameters for each tie point to get measurements for the depth formula regression: */
	Math::Matrix depthAta(2,2,0.0);
	Math::Matrix depthAtb(2,1,0.0);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Convert the tie point's depth homography to a matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(depthFrameSize[0]);
		hm*=tpIt->depthHom.getMatrix();
		Homography::Matrix scale(1.0);
		scale(0,0)=1.0/tileSize[0];
		scale(1,1)=1.0/tileSize[1];
		hm*=scale;
		Math::Matrix h(3,3);
		for(unsigned int i=0;i<3;++i)
			for(unsigned int j=0;j<3;++j)
				h(i,j)=hm(i,j);
		
		/* Calculate the extrinsic parameters: */
		double lambda=0.5/(aInv*h.getColumn(0)).mag()+0.5/(aInv*h.getColumn(1)).mag();
		Math::Matrix r1=lambda*aInv*h.getColumn(0);
		Math::Matrix r2=lambda*aInv*h.getColumn(1);
		Math::Matrix r3(3,1);
		for(unsigned int i=0;i<3;++i)
			r3.set(i,r1((i+1)%3)*r2((i+2)%3)-r1((i+2)%3)*r2((i+1)%3)); // 'Tis a cross product, in case you're wondering
		Math::Matrix t=lambda*aInv*h.getColumn(2);
		
		/* Create the extrinsic parameter matrix: */
		Math::Matrix rt(3,4);
		rt.setColumn(0,r1);
		rt.setColumn(1,r2);
		rt.setColumn(2,r3);
		rt.setColumn(3,t);
		
		Math::Matrix wgc(4,1);
		wgc(0)=tileSize[0]*double(gridSize[0])*0.5;
		wgc(1)=tileSize[1]*double(gridSize[1])*0.5;
		wgc(2)=0.0;
		wgc(3)=1.0;
		Math::Matrix cgc=rt*wgc;
		if(cgc(2)<0.0)
			{
			/* Flip the extrinsic matrix to move the grid to positive z: */
			Math::Matrix flip(3,3,-1.0);
			rt=flip*rt;
			cgc=rt*wgc;
			}
		std::cout<<"Grid center: "<<cgc(0)<<", "<<cgc(1)<<", "<<cgc(2)<<std::endl;
		
		/* Transform all world grid points with the extrinsic matrix to get their camera-space z values: */
		for(int y=0;y<gridSize[1];++y)
			for(int x=0;x<gridSize[0];++x)
				{
				/* Create the world point: */
				Math::Matrix wp(4,1);
				wp(0)=tileSize[0]*double(x);
				wp(1)=tileSize[1]*double(y);
				wp(2)=0.0;
				wp(3)=1.0;
				
				/* Transform the world point to camera space: */
				Math::Matrix cp=rt*wp;
				double dist=cp(2);
				
				/* Get the depth frame value from the grid's plane in depth image space: */
				Point dip=tpIt->depthHom.transform(Point(x,y));
				const Plane::Vector& n=tpIt->gridPlane.getNormal();
				double o=tpIt->gridPlane.getOffset();
				double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
				
				/* Enter the depth / z pair into the depth formula accumulator: */
				depthAta(0,0)+=1.0;
				depthAta(0,1)+=-dist;
				depthAta(1,0)+=-dist;
				depthAta(1,1)+=dist*dist;
				depthAtb(0)+=-dist*depth;
				depthAtb(1)+=dist*dist*depth;
				}
		}
	
	/* Solve the depth formula least-squares system: */
	Math::Matrix depthX=depthAtb.divideFullPivot(depthAta);
	std::cout<<"Depth conversion formula: dist = "<<depthX(0)<<" / ("<<depthX(1)<<" - depth)"<<std::endl;
	
	/* Calculate the full depth unprojection matrix: */
	Math::Matrix depthProj(4,4,0.0);
	depthProj(0,0)=1.0/alpha;
	depthProj(0,1)=-gamma/(alpha*beta);
	depthProj(0,3)=-u0/alpha+v0*gamma/(alpha*beta);
	depthProj(1,1)=1.0/beta;
	depthProj(1,3)=-v0/beta;
	depthProj(2,3)=-1.0;
	depthProj(3,2)=-1.0/depthX(0);
	depthProj(3,3)=depthX(1)/depthX(0);
	
	Math::Matrix colorAta(12,12,0.0);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		for(int y=0;y<gridSize[1];++y)
			for(int x=0;x<gridSize[0];++x)
				{
				/* Enter the tie point into the color calibration matrix linear system: */
				Point dip=tpIt->depthHom.transform(Point(x,y));
				const Plane::Vector& n=tpIt->gridPlane.getNormal();
				double o=tpIt->gridPlane.getOffset();
				double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
				Math::Matrix dwp(4,1);
				dwp(0)=dip[0]+double(depthFrameSize[0]);
				dwp(1)=dip[1];
				dwp(2)=depth;
				dwp(3)=1.0;
				dwp=depthProj*dwp;
				for(int i=0;i<3;++i)
					dwp(i)/=dwp(3);
				Point cip=tpIt->colorHom.transform(Point(x,y));
				cip[0]/=double(colorFrameSize[0]);
				cip[1]/=double(colorFrameSize[1]);

				double eq[2][12];
				eq[0][0]=dwp(0);
				eq[0][1]=dwp(1);
				eq[0][2]=dwp(2);
				eq[0][3]=1.0;
				eq[0][4]=0.0;
				eq[0][5]=0.0;
				eq[0][6]=0.0;
				eq[0][7]=0.0;
				eq[0][8]=-cip[0]*dwp(0);
				eq[0][9]=-cip[0]*dwp(1);
				eq[0][10]=-cip[0]*dwp(2);
				eq[0][11]=-cip[0];

				eq[1][0]=0.0;
				eq[1][1]=0.0;
				eq[1][2]=0.0;
				eq[1][3]=0.0;
				eq[1][4]=dwp(0);
				eq[1][5]=dwp(1);
				eq[1][6]=dwp(2);
				eq[1][7]=1.0;
				eq[1][8]=-cip[1]*dwp(0);
				eq[1][9]=-cip[1]*dwp(1);
				eq[1][10]=-cip[1]*dwp(2);
				eq[1][11]=-cip[1];

				for(int row=0;row<2;++row)
					{
					for(unsigned int i=0;i<12;++i)
						for(unsigned int j=0;j<12;++j)
							colorAta(i,j)+=eq[row][i]*eq[row][j];
					}
				}
		}
	
	/* Find the color calibration system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> colorQe=colorAta.jacobiIteration();
	minEIndex=0;
	minE=Math::abs(colorQe.second(0,0));
	for(unsigned int i=1;i<12;++i)
		{
		if(minE>Math::abs(colorQe.second(i,0)))
			{
			minEIndex=i;
			minE=Math::abs(colorQe.second(i,0));
			}
		}
	
	/* Create the normalized color homography: */
	Math::Matrix colorHom(3,4);
	double scale=colorQe.first(11,minEIndex);
	for(int i=0;i<3;++i)
		for(int j=0;j<4;++j)
			colorHom(i,j)=colorQe.first(i*4+j,minEIndex)/scale;
	
	/* Create the full color unprojection matrix by extending the homography: */
	Math::Matrix colorProj(4,4);
	for(unsigned int i=0;i<2;++i)
		for(unsigned int j=0;j<4;++j)
			colorProj(i,j)=colorHom(i,j);
	for(unsigned int j=0;j<4;++j)
		colorProj(2,j)=j==2?1.0:0.0;
	for(unsigned int j=0;j<4;++j)
		colorProj(3,j)=colorHom(2,j);
	
	/* Modify the color unprojection matrix by the depth projection matrix: */
	colorProj*=depthProj;
	
	/* Write the calibration file: */
	IO::FilePtr calibFile(IO::openFile("CameraCalibrationMatrices.dat",IO::File::WriteOnly));
	calibFile->setEndianness(Misc::LittleEndian);
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<double>(depthProj(i,j));
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<double>(colorProj(i,j));
	}

int main(int argc,char* argv[])
	{
	/* Open the tie point file: */
	IO::FilePtr tiePointFile(IO::openFile(argv[1]));
	tiePointFile->setEndianness(Misc::LittleEndian);
	
	/* Read the grid dimensions: */
	int gridSize[2];
	tiePointFile->read<int>(gridSize,2);
	double tileSize[2];
	tiePointFile->read<double>(tileSize,2);
	
	/* Read the frame buffer dimensions: */
	unsigned int depthFrameSize[2];
	tiePointFile->read<unsigned int>(depthFrameSize,2);
	unsigned int colorFrameSize[2];
	tiePointFile->read<unsigned int>(colorFrameSize,2);
	
	/* Read all tie points: */
	unsigned int numTiePoints=tiePointFile->read<unsigned int>();
	std::vector<TiePoint> tiePoints;
	tiePoints.reserve(numTiePoints);
	for(unsigned int i=0;i<numTiePoints;++i)
		{
		TiePoint tp;
		tp.depthHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tp.gridPlane=Misc::Marshaller<Plane>::read(*tiePointFile);
		tp.colorHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tiePoints.push_back(tp);
		#if 0
		std::cout<<"Tie point "<<i<<":"<<std::endl;
		std::cout<<tp.depthHom<<std::endl;
		std::cout<<tp.gridPlane<<std::endl;
		std::cout<<tp.colorHom<<std::endl;
		std::cout<<std::endl;
		#endif
		}
	
	/* Run the calibration: */
	calibrate(tiePoints,gridSize,tileSize,depthFrameSize,colorFrameSize);
	
	return 0;
	}
