/***********************************************************************
Depth distortion calibration utility.
Copyright (c) 2012-2015 Oliver Kreylos

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

#include <vector>
#include <iostream>
#include <iomanip>
#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Matrix.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Plane.h>
#include <Geometry/PCACalculator.h>
#include <Kinect/FrameBuffer.h>

struct DepthFrame // Structure describing a depth frame and its best-fit plane
	{
	/* Embedded classes: */
	public:
	typedef double Scalar;
	typedef Geometry::Plane<Scalar,3> Plane;
	
	/* Elements: */
	public:
	Kinect::FrameBuffer frame; // The depth frame's pixels
	Plane plane; // The best-fit plane for the depth frame
	};

int main(int argc,char* argv[])
	{
	/* Open all depth frame files: */
	unsigned int frameSize[2]={640,480};
	std::vector<DepthFrame> depthFrames;
	for(int i=1;i<argc;++i)
		{
		/* Read the depth file: */
		std::cout<<"Reading "<<argv[i]<<"..."<<std::flush;
		IO::FilePtr depthFile(IO::openFile(argv[i]));
		Misc::UInt32 fs[2];
		depthFile->read(fs,2);
		if(fs[0]==frameSize[0]&&fs[1]==frameSize[1])
			{
			/* Read the depth frame: */
			DepthFrame depthFrame;
			depthFrame.frame=Kinect::FrameBuffer(frameSize[0],frameSize[1],frameSize[1]*frameSize[0]*sizeof(float));
			float* dfPtr=depthFrame.frame.getData<float>();
			depthFile->read(dfPtr,frameSize[1]*frameSize[0]);
			
			/* Calculate the best-fitting plane: */
			typedef Geometry::PCACalculator<3>::Point PPoint;
			typedef Geometry::PCACalculator<3>::Vector PVector;
			Geometry::PCACalculator<3> pca;
			for(unsigned int y=0;y<frameSize[1];++y)
				for(unsigned int x=0;x<frameSize[0];++x,++dfPtr)
					if(*dfPtr!=2047.0f)
						pca.accumulatePoint(PPoint(double(x)+0.5,double(y)+0.5,double(*dfPtr)));
			PPoint centroid=pca.calcCentroid();
			pca.calcCovariance();
			double evs[3];
			pca.calcEigenvalues(evs);
			PVector normal=pca.calcEigenvector(evs[2]);
			depthFrame.plane=DepthFrame::Plane(normal,centroid);
			std::cout<<" PCA residual "<<evs[2];
			depthFrames.push_back(depthFrame);
			}
		std::cout<<" done"<<std::endl;
		}
	
	/* Calculate per-pixel affine correction coefficients: */
	float* coefficients=new float[frameSize[1]*frameSize[0]*2];
	float* cPtr=coefficients;
	std::cout<<"Calculating corrrection coefficients...   0%"<<std::flush;
	for(unsigned int y=0;y<frameSize[1];++y)
		{
		for(unsigned int x=0;x<frameSize[0];++x,cPtr+=2)
			{
			/* Build the least-squares linear regression system: */
			Math::Matrix ata(2,2,0.0);
			Math::Matrix atb(2,1,0.0);
			unsigned int numFrames=0;
			for(std::vector<DepthFrame>::iterator dfIt=depthFrames.begin();dfIt!=depthFrames.end();++dfIt)
				{
				double actual=double(dfIt->frame.getData<float>()[y*frameSize[0]+x]);
				if(actual!=2047.0)
					{
					ata(0,0)+=actual*actual;
					ata(0,1)+=actual;
					ata(1,0)+=actual;
					ata(1,1)+=1.0;
					double expected=(dfIt->plane.getOffset()-(double(x)+0.5)*dfIt->plane.getNormal()[0]-(double(y)+0.5)*dfIt->plane.getNormal()[1])/dfIt->plane.getNormal()[2];
					atb(0)+=actual*expected;
					atb(1)+=expected;
					++numFrames;
					}
				}
			
			if(numFrames>=2)
				{
				/* Solve for the regression coefficients: */
				Math::Matrix x=atb/ata;
				cPtr[0]=float(x(0));
				cPtr[1]=float(x(1));
				}
			else
				{
				cPtr[0]=1.0f;
				cPtr[1]=0.0f;
				}
			}
		std::cout<<"\b\b\b\b"<<std::setw(3)<<((y+1)*100U+frameSize[1]/2)/frameSize[1]<<'%'<<std::flush;
		}
	std::cout<<std::endl;
	
	/* Fit planes to depth frames again to compare residuals: */
	for(std::vector<DepthFrame>::iterator dfIt=depthFrames.begin();dfIt!=depthFrames.end();++dfIt)
		{
		/* Calculate the best-fitting plane: */
		typedef Geometry::PCACalculator<3>::Point PPoint;
		Geometry::PCACalculator<3> pca;
		float* dfPtr=dfIt->frame.getData<float>();
		float* cPtr=coefficients;
		for(unsigned int y=0;y<frameSize[1];++y)
			for(unsigned int x=0;x<frameSize[0];++x,++dfPtr,cPtr+=2)
				if(*dfPtr!=2047.0f)
					pca.accumulatePoint(PPoint(double(x)+0.5,double(y)+0.5,double((*dfPtr)*cPtr[0]+cPtr[1])));
		pca.calcCovariance();
		double evs[3];
		pca.calcEigenvalues(evs);
		std::cout<<"Corrected PCA residual "<<evs[2]<<std::endl;
		}
	
	/* Write the coefficient frame: */
	{
	IO::FilePtr coeffFile(IO::openFile("DepthCorrection.dat",IO::File::WriteOnly));
	coeffFile->setEndianness(Misc::LittleEndian);
	coeffFile->write<unsigned int>(frameSize,2);
	coeffFile->write<float>(coefficients,frameSize[1]*frameSize[0]*2);
	}
	
	delete[] coefficients;
	return 0;
	}
