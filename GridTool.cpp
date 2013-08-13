/***********************************************************************
GridTool - Calibration tool for RawKinectViewer.
Copyright (c) 2010-2013 Oliver Kreylos

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

#include "GridTool.h"

#include <string>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Camera.h>

#include "RawKinectViewer.h"

/*********************************
Static elements of class GridTool:
*********************************/

GridToolFactory* GridTool::factory=0;
int GridTool::gridSize[2];
double GridTool::tileSize[2];

/*************************
Methods of class GridTool:
*************************/

GridTool::Homography GridTool::calcHomography(const GridTool::Point gridPoints[4],const GridTool::Point imagePoints[4])
	{
	/* Create the linear system: */
	Math::Matrix a(9,9,0.0);
	for(int p=0;p<4;++p)
		{
		for(int i=0;i<2;++i)
			{
			for(int j=0;j<2;++j)
				{
				a(p*2+i,i*3+j)=gridPoints[p][j];
				a(p*2+i,6+j)=-imagePoints[p][i]*gridPoints[p][j];
				}
			a(p*2+i,i*3+2)=1.0;
			a(p*2+i,8)=-imagePoints[p][i];
			}
		}
	
	/* Calculate the linear system's null space: */
	Math::Matrix x=a.kernel();
	
	/* Create the result homography: */
	Homography result;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			result.getMatrix()(i,j)=x(i*3+j,0);
	return result;
	}

void GridTool::initHoms(void)
	{
	/* Initialize the depth and image grid homographies: */
	lastDraggedPoints[0]=Point(0.0,0.0);
	lastDraggedPoints[1]=Point(double(gridSize[0]),0.0);
	lastDraggedPoints[2]=Point(0.0,double(gridSize[1]));
	lastDraggedPoints[3]=Point(double(gridSize[0]),double(gridSize[1]));
	Point imagePoints[4];
	imagePoints[0]=Point(-double(application->depthFrameSize[0])+100.0,100.0);
	imagePoints[1]=Point(-100.0,100.0);
	imagePoints[2]=Point(-double(application->depthFrameSize[0])+100.0,double(application->depthFrameSize[1])-100.0);
	imagePoints[3]=Point(-100.0,double(application->depthFrameSize[1])-100.0);
	homs[0]=calcHomography(lastDraggedPoints,imagePoints);
	imagePoints[0]=Point(100.0,100.0);
	imagePoints[1]=Point(double(application->colorFrameSize[0])-100.0,100.0);
	imagePoints[2]=Point(100.0,double(application->colorFrameSize[1])-100.0);
	imagePoints[3]=Point(double(application->colorFrameSize[0])-100.0,double(application->colorFrameSize[1])-100.0);
	homs[1]=calcHomography(lastDraggedPoints,imagePoints);
	}

void GridTool::startDrag(void)
	{
	/* Get the interaction point: */
	Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
	
	draggingMode=IDLE;
	double minDist2=Math::Constants<double>::max;
	
	/* Calculate the distance to the grids' move handles: */
	for(int hom=0;hom<2;++hom)
		{
		Point mh=homs[hom].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
		double mhDist2=Geometry::sqrDist(p,mh);
		if(minDist2>mhDist2)
			{
			minDist2=mhDist2;
			draggingMode=MOVE;
			draggedHom=hom;
			dragOffset=mh-p;
			}
		}
	
	/* Calculate the distance to the grids' rotation handles: */
	for(int hom=0;hom<2;++hom)
		{
		Point rh=homs[hom].transform(Point(double(gridSize[0]+1),double(gridSize[1])*0.5));
		double rhDist2=Geometry::sqrDist(p,rh);
		if(minDist2>rhDist2)
			{
			minDist2=rhDist2;
			draggingMode=ROTATE;
			draggedHom=hom;
			dragOffset=rh-p;
			}
		}
	
	/* Find the grid point closest to the interaction point: */
	Point draggedPoint(0,0);
	for(int y=0;y<=gridSize[1];++y)
		for(int x=0;x<=gridSize[0];++x)
			{
			Point gp=Point(x,y);
			for(int hom=0;hom<2;++hom)
				{
				Point ip=homs[hom].transform(gp);
				double ipDist2=Geometry::sqrDist(p,ip);
				if(minDist2>ipDist2)
					{
					minDist2=ipDist2;
					draggingMode=VERTEX;
					draggedHom=hom;
					draggedPoint=gp;
					dragOffset=ip-p;
					}
				}
			}
	
	if(draggingMode==VERTEX)
		{
		/* Determine which previously dragged point to replace: */
		draggedPointIndex=0;
		double dpDist2=Geometry::sqrDist(lastDraggedPoints[0],draggedPoint);
		for(int i=1;i<4;++i)
			{
			double dist2=Geometry::sqrDist(lastDraggedPoints[i],draggedPoint);
			if(dpDist2>dist2)
				{
				draggedPointIndex=i;
				dpDist2=dist2;
				}
			}
		
		/* Replace the dragged point: */
		lastDraggedPoints[draggedPointIndex]=draggedPoint;
		}
	}

void GridTool::dragInPlane(const GridTool::Point& moveHandle,const GridTool::Point& rotateHandle)
	{
	/* Project the grid's center point and rotation handle into the world-space depth plane: */
	Point3 wmh(moveHandle[0]+double(application->depthFrameSize[0]),moveHandle[1],0.0);
	wmh[2]=(camDepthPlane.getOffset()-camDepthPlane.getNormal()*wmh)/camDepthPlane.getNormal()[2];
	wmh=application->intrinsicParameters.depthProjection.transform(wmh);
	Point3 wrh(rotateHandle[0]+double(application->depthFrameSize[0]),rotateHandle[1],0.0);
	wrh[2]=(camDepthPlane.getOffset()-camDepthPlane.getNormal()*wrh)/camDepthPlane.getNormal()[2];
	wrh=application->intrinsicParameters.depthProjection.transform(wrh);
	
	/* Construct a coordinate frame around the world-space center point: */
	Vector3 x=wrh-wmh;
	x.normalize();
	Vector3 y=worldDepthPlane.getNormal()^x;
	y.normalize();
	
	Point gridPoints[4];
	for(int i=0;i<4;++i)
		{
		gridPoints[i]=Point::origin;
		if(i&0x1)
			gridPoints[i][0]+=double(gridSize[0]);
		if(i&0x2)
			gridPoints[i][1]+=double(gridSize[1]);
		}
	
	Point imagePoints[2][4];
	for(int i=0;i<4;++i)
		{
		/* Construct the world-space position of the i-th grid corner: */
		Point3 wp=wmh;
		if(i&0x1)
			wp+=x*double(gridSize[0])*tileSize[0]*0.5;
		else
			wp-=x*double(gridSize[0])*tileSize[0]*0.5;
		if(i&0x2)
			wp+=y*double(gridSize[1])*tileSize[1]*0.5;
		else
			wp-=y*double(gridSize[1])*tileSize[1]*0.5;

		/* Project the grid corner into depth camera space: */
		Point3 dcp=application->intrinsicParameters.depthProjection.inverseTransform(wp);
		imagePoints[0][i]=Point(dcp[0]-double(application->depthFrameSize[0]),dcp[1]);
		
		/* Project the grid corner into color camera space: */
		Point3 ccp=application->intrinsicParameters.colorProjection.transform(dcp);
		imagePoints[1][i]=Point(ccp[0]*double(application->colorFrameSize[0]),ccp[1]*double(application->colorFrameSize[1]));
		}
	
	/* Update the depth and color homographies: */
	for(int i=0;i<2;++i)
		homs[i]=calcHomography(gridPoints,imagePoints[i]);
	}

void GridTool::createTiePoint(void)
	{
	/* Calculate the grid's plane equation in depth image space: */
	Geometry::PCACalculator<3> pca;
	
	const float* afdPtr=application->averageFrameDepth;
	const float* affPtr=application->averageFrameForeground;
	float foregroundCutoff=float(application->averageNumFrames)*0.5f;
	const RawKinectViewer::PixelCorrection* dcPtr=application->depthCorrection;
	for(unsigned int y=0;y<application->depthFrameSize[1];++y)
		{
		double dy=double(y)+0.5;
		for(unsigned int x=0;x<application->depthFrameSize[0];++x,++afdPtr,++affPtr,++dcPtr)
			{
			double dx=double(x)-double(application->depthFrameSize[0])+0.5;
			if(*affPtr>=foregroundCutoff)
				{
				/* Determine the pixel's grid position: */
				Point gp=homs[0].inverseTransform(Point(dx,dy));
				if(gp[0]>=0.0&&gp[0]<double(gridSize[0])&&gp[1]>=0.0&&gp[1]<double(gridSize[1]))
					if((int(gp[0])+int(gp[1]))%2==0)
						{
						double gx=gp[0]-Math::floor(gp[0]);
						double gy=gp[1]-Math::floor(gp[1]);
						if(gx>=0.2&&gx<0.8&&gy>=0.2&&gy<0.8)
							pca.accumulatePoint(Geometry::PCACalculator<3>::Point(dx,dy,dcPtr->correct((*afdPtr)/(*affPtr))));
						}
				}
			}
		}
	
	/* Calculate the grid's plane equation: */
	Geometry::PCACalculator<3>::Point centroid=pca.calcCentroid();
	pca.calcCovariance();
	double evs[3];
	pca.calcEigenvalues(evs);
	Geometry::PCACalculator<3>::Vector normal=pca.calcEigenvector(evs[2]);
	
	/* Check for any nans or infs: */
	bool allFinite=true;
	for(int i=0;i<3;++i)
		{
		allFinite=allFinite&&Math::isFinite(normal[i]);
		allFinite=allFinite&&Math::isFinite(centroid[i]);
		}
	
	if(allFinite)
		{
		/* Create and store a tie point: */
		TiePoint newTp;
		newTp.depthHom=homs[0];
		newTp.gridPlane=Plane(normal,centroid);
		newTp.colorHom=homs[1];
		tiePoints.push_back(newTp);
		}
	else
		Vrui::showErrorMessage("GridTool","Could not create tie point due to undefined grid plane equation");
	}

void GridTool::calibrate(void)
	{
	if(tiePoints.size()<2)
		Vrui::showErrorMessage("GridTool","Need at least two tie points to calibrate");
	
	/* Initialize the depth camera's intrinsic parameter matrix: */
	Math::Matrix depthV(6,6,0.0);
	
	/* Process all tie points: */
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Enter the tie point's depth homography into the intrinsic parameter matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(application->depthFrameSize[0]);
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
	
	#if 0
	/* Calculate the full projection matrix: */
	Math::Matrix proj(4,4,0.0);
	proj(0,0)=alpha;
	proj(0,1)=gamma;
	proj(0,2)=u0;
	proj(1,1)=beta;
	proj(1,2)=v0;
	proj(2,3)=1.0;
	proj(3,2)=1.0;
	#endif
	
	/* Calculate extrinsic parameters for each tie point to get measurements for the depth formula regression: */
	Math::Matrix depthAta(2,2,0.0);
	Math::Matrix depthAtb(2,1,0.0);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Convert the tie point's depth homography to a matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(application->depthFrameSize[0]);
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
		
		/* Transform the center of the grid to check for inversion: */
		Math::Matrix wgc(4,1);
		wgc(0)=tileSize[0]*double(gridSize[0])*0.5;
		wgc(1)=tileSize[1]*double(gridSize[1])*0.5;
		wgc(2)=0.0;
		wgc(3)=1.0;
		if((rt*wgc)(2)<0.0)
			{
			/* Flip the extrinsic matrix to move the grid to positive z: */
			Math::Matrix flip(3,3,-1.0);
			rt=flip*rt;
			}
		
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
				
				/* Get the world point's z coordinate in camera space: */
				double dist=(rt*wp)(2);
				
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
	
	/* Create the color calibration matrix's linear system: */
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
				dwp(0)=dip[0]+double(application->depthFrameSize[0]);
				dwp(1)=dip[1];
				dwp(2)=depth;
				dwp(3)=1.0;
				dwp=depthProj*dwp;
				for(int i=0;i<3;++i)
					dwp(i)/=dwp(3);
				Point cip=tpIt->colorHom.transform(Point(x,y));
				cip[0]/=double(application->colorFrameSize[0]);
				cip[1]/=double(application->colorFrameSize[1]);

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
	std::string calibFileName=KINECT_CONFIG_DIR;
	calibFileName.push_back('/');
	calibFileName.append(KINECT_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX);
	calibFileName.push_back('-');
	calibFileName.append(application->camera->getSerialNumber());
	calibFileName.append(".dat");
	std::cout<<"Writing calibration file "<<calibFileName<<std::endl;
	IO::FilePtr calibFile(Vrui::openFile(calibFileName.c_str(),IO::File::WriteOnly));
	calibFile->setEndianness(Misc::LittleEndian);
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<Misc::Float64>(depthProj(i,j));
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<Misc::Float64>(colorProj(i,j));
	}

void GridTool::printWorldPoints(void)
	{
	typedef RawKinectViewer::IntrinsicParameters::PTransform PTransform;
	
	if(tiePoints.empty())
		{
		Vrui::showErrorMessage("GridTool","No tie points to unproject");
		return;
		}
	
	/* Unproject the grid points of the most recent tie point: */
	const TiePoint& tp=tiePoints.back();
	for(int y=1;y<gridSize[1];++y)
		for(int x=1;x<gridSize[0];++x)
			{
			Point dip=tp.depthHom.transform(Point(x,y));
			const Plane::Vector& n=tp.gridPlane.getNormal();
			double o=tp.gridPlane.getOffset();
			double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
			PTransform::Point wp=application->intrinsicParameters.depthProjection.transform(PTransform::Point(dip[0]+double(application->depthFrameSize[0]),dip[1],depth));
			std::cout<<wp[0]<<", "<<wp[1]<<", "<<wp[2]<<std::endl;
			}
	}

void GridTool::drawGrid(const GridTool::Homography& hom,bool active)
	{
	/* Draw the grid: */
	glBegin(GL_LINES);
	for(int x=0;x<=gridSize[0];++x)
		{
		Point ip1=hom.transform(Point(double(x),0.0));
		glVertex3d(ip1[0],ip1[1],0.01);
		Point ip2=hom.transform(Point(double(x),double(gridSize[1])));
		glVertex3d(ip2[0],ip2[1],0.01);
		}
	for(int y=0;y<=gridSize[1];++y)
		{
		Point ip1=hom.transform(Point(0.0,double(y)));
		glVertex3d(ip1[0],ip1[1],0.01);
		Point ip2=hom.transform(Point(double(gridSize[0]),double(y)));
		glVertex3d(ip2[0],ip2[1],0.01);
		}
	glEnd();
	
	if(active)
		{
		/* Draw the "origin point" and move/rotation handles: */
		glBegin(GL_POINTS);
		Point op=hom.transform(Point(0.5,0.5));
		glVertex3d(op[0],op[1],0.01);
		Point mh=hom.transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
		glVertex3d(mh[0],mh[1],0.01);
		Point rh=hom.transform(Point(double(gridSize[0]+1),double(gridSize[1])*0.5));
		glVertex3d(rh[0],rh[1],0.01);
		glEnd();
		}
	
	#if 0
	/* Draw all grid vertices: */
	glBegin(GL_POINTS);
	for(int y=0;y<gridSize[1];++y)
		for(int x=0;x<gridSize[0];++x)
			if((x+y)%2==0)
				{
				Point ip=hom.transform(Point(double(x)+0.5,double(y)+0.5));
				glVertex3d(ip[0],ip[1],0.01);
				}
	glEnd();
	#endif
	}

GridToolFactory* GridTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new GridToolFactory("GridTool","Draw Grids",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(6);
	factory->setButtonFunction(0,"Drag Grid Corner");
	factory->setButtonFunction(1,"Toggle Depth Plane Lock");
	factory->setButtonFunction(2,"Store Grid");
	factory->setButtonFunction(3,"Toggle Stored Grids");
	factory->setButtonFunction(4,"Calibrate");
	factory->setButtonFunction(5,"Unproject Last Grid");
	
	/* Initialize the calibration grid layout: */
	gridSize[0]=7;
	gridSize[1]=5;
	tileSize[0]=3.5*2.54;
	tileSize[1]=3.5*2.54;
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

void GridTool::setGridSize(int newGridSize0,int newGridSize1)
	{
	gridSize[0]=newGridSize0;
	gridSize[1]=newGridSize1;
	}

void GridTool::setTileSize(double newTileSize0,double newTileSize1)
	{
	tileSize[0]=newTileSize0;
	tileSize[1]=newTileSize1;
	}

GridTool::GridTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 lockToPlane(false),draggingMode(IDLE),
	 showTiePoints(false)
	{
	}

GridTool::~GridTool(void)
	{
	}

void GridTool::initialize(void)
	{
	#if 0
	/* Open the tie point file: */
	IO::FilePtr tiePointFile(Vrui::openFile("CalibrationTiePoints.dat"));
	tiePointFile->setEndianness(Misc::LittleEndian);
	
	/* Read the grid dimensions: */
	Misc::UInt32 gridSize[2];
	tiePointFile->read(gridSize,2);
	Misc::Float64 tileSize[2];
	tiePointFile->read(tileSize,2);
	
	/* Read the depth and color frame sizes: */
	Misc::UInt32 depthFrameSize[2];
	tiePointFile->read(depthFrameSize,2);
	Misc::UInt32 colorFrameSize[2];
	tiePointFile->read(colorFrameSize,2);
	
	/* Read all tie points: */
	unsigned int numTiePoints=tiePointFile->read<Misc::UInt32>();
	for(unsigned int i=0;i<numTiePoints;++i)
		{
		TiePoint tp;
		tp.depthHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tp.gridPlane=Misc::Marshaller<Plane>::read(*tiePointFile);
		tp.colorHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tiePoints.push_back(tp);
		}
	#endif
	
	initHoms();
	}

const Vrui::ToolFactory* GridTool::getFactory(void) const
	{
	return factory;
	}

void GridTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
			{
			/* Try entering dragging mode: */
			startDrag();
			}
		else if(buttonSlotIndex==1)
			{
			if(!lockToPlane)
				{
				/* Check if the application has a valid depth plane: */
				if(application->depthPlaneValid)
					{
					/* Lock to the application's world-space depth plane: */
					lockToPlane=true;
					camDepthPlane=application->camDepthPlane;
					worldDepthPlane=application->worldDepthPlane;
					
					/* Project the depth image grid into the depth plane: */
					Point moveHandle=homs[0].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
					Point rotateHandle=homs[0].transform(Point(double(gridSize[0]+1),double(gridSize[1])*0.5));
					dragInPlane(moveHandle,rotateHandle);
					}
				else
					Vrui::showErrorMessage("GridTool","No valid depth plane to lock to");
				}
			else
				lockToPlane=false;
			}
		else if(buttonSlotIndex==2)
			{
			/* Store the current grids as an intrinsic calibration tie point: */
			createTiePoint();
			}
		else if(buttonSlotIndex==3)
			{
			/* Toggle display of previously collected tie points: */
			showTiePoints=!showTiePoints;
			}
		else if(buttonSlotIndex==4)
			{
			/* Write all tie points to a file: */
			IO::FilePtr tiePointFile(Vrui::openFile("CalibrationTiePoints.dat",IO::File::WriteOnly));
			tiePointFile->setEndianness(Misc::LittleEndian);
			for(int i=0;i<2;++i)
				tiePointFile->write<Misc::UInt32>(gridSize[i]);
			for(int i=0;i<2;++i)
				tiePointFile->write<Misc::Float64>(tileSize[i]);
			for(int i=0;i<2;++i)
				tiePointFile->write<Misc::UInt32>(application->depthFrameSize[i]);
			for(int i=0;i<2;++i)
				tiePointFile->write<Misc::UInt32>(application->colorFrameSize[i]);
			tiePointFile->write<Misc::UInt32>(tiePoints.size());
			for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
				{
				Misc::Marshaller<Homography>::write(tpIt->depthHom,*tiePointFile);
				Misc::Marshaller<Plane>::write(tpIt->gridPlane,*tiePointFile);
				Misc::Marshaller<Homography>::write(tpIt->colorHom,*tiePointFile);
				}
			
			/* Perform intrinsic calibration: */
			calibrate();
			}
		else if(buttonSlotIndex==5)
			{
			/* Print the world positions of the most recently saved tie point for extrinsic calibration: */
			printWorldPoints();
			}
		}
	else
		{
		if(buttonSlotIndex==0)
			{
			/* Stop dragging: */
			draggingMode=IDLE;
			}
		}
	}

void GridTool::frame(void)
	{
	if(draggingMode!=IDLE)
		{
		/* Get the current interaction point: */
		Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		
		/* Calculate the image-space positions of the four most recently dragged vertices: */
		Point imagePoints[4];
		for(int i=0;i<4;++i)
			imagePoints[i]=homs[draggedHom].transform(lastDraggedPoints[i]);
		
		switch(draggingMode)
			{
			case VERTEX:
				{
				if(draggedHom==1||!lockToPlane)
					{
					/* Update the currently dragged grid vertex: */
					imagePoints[draggedPointIndex]=p+dragOffset;
					
					/* Recalculate the dragged homography: */
					homs[draggedHom]=calcHomography(lastDraggedPoints,imagePoints);
					}
				
				break;
				}
			
			case MOVE:
				{
				if(draggedHom==0&&lockToPlane)
					{
					/* Calculate the image-space position of the move and rotate handles and the displacement vector: */
					Point moveHandle=homs[draggedHom].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
					Point rotateHandle=homs[draggedHom].transform(Point(double(gridSize[0]+1),double(gridSize[1])*0.5));
					Vector delta=(p+dragOffset)-moveHandle;
					
					/* Recalculate the depth homography: */
					dragInPlane(moveHandle+delta,rotateHandle+delta);
					}
				else
					{
					/* Calculate the image-space position of the image center and the displacement vector: */
					Point imageCenter=homs[draggedHom].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
					Vector delta=(p+dragOffset)-imageCenter;
					
					/* Move all image points: */
					for(int i=0;i<4;++i)
						imagePoints[i]+=delta;
					
					/* Recalculate the dragged homography: */
					homs[draggedHom]=calcHomography(lastDraggedPoints,imagePoints);
					}
				
				break;
				}
				
			case ROTATE:
				{
				if(draggedHom==0&&lockToPlane)
					{
					/* Calculate the image-space position of the move and dragged rotate handles: */
					Point moveHandle=homs[draggedHom].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
					Point rotateHandle=p+dragOffset;
					
					/* Recalculate the depth homography: */
					dragInPlane(moveHandle,rotateHandle);
					}
				else
					{
					/* Calculate the image-space positions of the grid center and rotation handle: */
					Point imageCenter=homs[draggedHom].transform(Point(double(gridSize[0])*0.5,double(gridSize[1])*0.5));
					Point imageRh=homs[draggedHom].transform(Point(double(gridSize[0]+1),double(gridSize[1])*0.5));
					
					/* Calculate the rotation angle: */
					Vector d1=imageRh-imageCenter;
					double d1Len=Geometry::mag(d1);
					Vector d1n=Geometry::normal(d1);
					double d1nLen=Geometry::mag(d1n);
					Vector d2=(p+dragOffset)-imageCenter;
					double angle=Math::atan2((d1n*d2)/d1nLen,(d1*d2)/d1Len);
					
					/* Rotate all image points: */
					for(int i=0;i<4;++i)
						{
						Vector d=imagePoints[i]-imageCenter;
						imagePoints[i]=imageCenter+Vector(d[0]*Math::cos(angle)-d[1]*Math::sin(angle),d[0]*Math::sin(angle)+d[1]*Math::cos(angle));
						}
					
					/* Recalculate the dragged homography: */
					homs[draggedHom]=calcHomography(lastDraggedPoints,imagePoints);
					}
				
				break;
				}
			
			default:
				;
			}
		}
	}

void GridTool::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glLineWidth(1.0f);
	
	/* Go to navigation coordinates: */
	glPushMatrix();
	const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
	glLoadMatrix(displayState.modelviewNavigational);
	
	if(showTiePoints)
		{
		/* Draw all tie points: */
		glColor3f(0.0f,0.333f,0.0f);
		for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
			{
			drawGrid(tpIt->depthHom,false);
			drawGrid(tpIt->colorHom,false);
			}
		}
	
	/* Draw the current depth and color grids: */
	glColor3f(0.0f,1.0f,0.0f);
	for(int i=0;i<2;++i)
		drawGrid(homs[i],true);
	
	glPopMatrix();
	
	glPopAttrib();
	}
