/***********************************************************************
DepthCorrectionTool - Calibration tool for RawKinectViewer.
Copyright (c) 2012-2013 Oliver Kreylos

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

#include "DepthCorrectionTool.h"

#include <string>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <IO/File.h>
#include <Math/Matrix.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/PCACalculator.h>
#include <Vrui/ToolManager.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Camera.h>

#include "RawKinectViewer.h"

/********************************************
Static elements of class DepthCorrectionTool:
********************************************/

DepthCorrectionToolFactory* DepthCorrectionTool::factory=0;

/************************************
Methods of class DepthCorrectionTool:
************************************/

void DepthCorrectionTool::averageDepthFrameReady(int)
	{
	/* Add a new averaged depth frame and calculate the best-fitting plane: */
	DepthFrame df;
	df.frame=Kinect::FrameBuffer(application->depthFrameSize[0],application->depthFrameSize[1],application->depthFrameSize[0]*application->depthFrameSize[1]*sizeof(float));
	float foregroundCutoff=float(application->averageNumFrames)*0.5f;
	float* afdPtr=application->averageFrameDepth;
	float* affPtr=application->averageFrameForeground;
	float* dfPtr=static_cast<float*>(df.frame.getBuffer());
	typedef Geometry::PCACalculator<3>::Point PPoint;
	typedef Geometry::PCACalculator<3>::Vector PVector;
	Geometry::PCACalculator<3> pca;
	for(unsigned int y=0;y<application->depthFrameSize[1];++y)
		for(unsigned int x=0;x<application->depthFrameSize[0];++x,++afdPtr,++affPtr,++dfPtr)
			{
			if(*affPtr>=foregroundCutoff)
				{
				/* Calculate the average depth value: */
				*dfPtr=(*afdPtr)/(*affPtr);

				/* Add the depth pixel to the PCA calculator: */
				pca.accumulatePoint(PPoint(double(x)+0.5,double(y)+0.5,double(*dfPtr)));
				}
			else
				*dfPtr=2047.0f;
			}

	/* Calculate the best-fitting plane: */
	PPoint centroid=pca.calcCentroid();
	pca.calcCovariance();
	double evs[3];
	pca.calcEigenvalues(evs);
	PVector normal=pca.calcEigenvector(evs[2]);
	df.plane=Plane(normal,centroid);
	depthFrames.push_back(df);
	}

DepthCorrectionToolFactory* DepthCorrectionTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new DepthCorrectionToolFactory("DepthCorrectionTool","Calibrate Depth Lens",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Save Plane");
	factory->setButtonFunction(1,"Calibrate");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

DepthCorrectionTool::DepthCorrectionTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	/* Initialize the B-spline approximator: */
	degree=3;
	numSegments[0]=12;
	numSegments[1]=9;
	}

DepthCorrectionTool::~DepthCorrectionTool(void)
	{
	}

const Vrui::ToolFactory* DepthCorrectionTool::getFactory(void) const
	{
	return factory;
	}

namespace {

/****************
Helper functions:
****************/

/* Calculate the value of a univariate uniform non-rational B-spline: */

inline double bs(int i,int n,double x)
	{
	/* Check whether x is inside the B-spline's support [i, i+n+1): */
	if(x<double(i)||x>=double(i+n+1))
		return 0.0;
	
	/* Calculate the B-spline using Cox-deBoor recursion: */
	double bsTemp[21]; // Maximum degree is 20
	for(int j=0;j<=n;++j)
		bsTemp[j]=x>=double(i+j)&&x<double(i+j+1)?1.0:0.0;
	
	for(int ni=1;ni<=n;++ni)
		for(int j=0;j<=n-ni;++j)
			bsTemp[j]=((x-double(i+j))*bsTemp[j]+(double(i+j+ni+1)-x)*bsTemp[j+1])/double(ni);
	
	return bsTemp[0];
	}

}

void DepthCorrectionTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
			{
			/* Request an average depth frame from the main application: */
			application->requestAverageFrame(Misc::createFunctionCall(this,&DepthCorrectionTool::averageDepthFrameReady));
			}
		else
			{
			/* Initialize the B-spline approximation matrices: */
			int numControlPoints=(numSegments[1]+degree)*(numSegments[0]+degree);
			Math::Matrix bsplineAta(numControlPoints,numControlPoints,0.0);
			Math::Matrix bsplineAtb(numControlPoints,2,0.0);
			
			/* Calculate the per-pixel depth correction offsets: */
			double* c=new double[numControlPoints];
			unsigned int pixelOffset=0;
			for(unsigned int y=0;y<application->depthFrameSize[1];++y)
				{
				/* Calculate the B-spline space y coordinate: */
				double dy=(double(y)+0.5)*double(numSegments[1])/double(application->depthFrameSize[1]);
				
				for(unsigned int x=0;x<application->depthFrameSize[0];++x,++pixelOffset)
					{
					/* Calculate the B-spline space x coordinate: */
					double dx=(double(x)+0.5)*double(numSegments[0])/double(application->depthFrameSize[0]);
					
					/* Build the least-squares linear regression system to calculate depth scale and offset: */
					Math::Matrix ata(2,2,0.0);
					Math::Matrix atb(2,1,0.0);
					unsigned int numFrames=0;
					for(std::vector<DepthFrame>::iterator dfIt=depthFrames.begin();dfIt!=depthFrames.end();++dfIt)
						{
						double actual=double(static_cast<float*>(dfIt->frame.getBuffer())[pixelOffset]);
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
						/* Solve for the pixel's correction scale and offset: */
						Math::Matrix x=atb.divideFullPivot(ata);
						double scale=x(0);
						double offset=x(1);
						
						/* Accumulate the pixel's correction scale and offset into the spline appoximation matrix: */
						for(int i=0;i<numSegments[1]+degree;++i)
							for(int j=0;j<numSegments[0]+degree;++j)
								c[i*(numSegments[0]+degree)+j]=bs(i-degree,degree,dy)*bs(j-degree,degree,dx);
						for(int i=0;i<numControlPoints;++i)
							{
							for(int j=0;j<numControlPoints;++j)
								bsplineAta(i,j)+=c[i]*c[j];
							bsplineAtb(i,0)+=c[i]*scale;
							bsplineAtb(i,1)+=c[i]*offset;
							}
						}
					}
				}
			delete[] c;
			
			/* Solve for the approximating B-spline coefficients: */
			Math::Matrix bsplineCoeffs=bsplineAtb.divideFullPivot(bsplineAta);
			Misc::Float32* correctionCoefficients=new Misc::Float32[(numSegments[1]+degree)*(numSegments[0]+degree)*2];
			for(int i=0;i<numControlPoints;++i)
				{
				/* Save scale control point: */
				correctionCoefficients[2*i+0]=Misc::Float32(bsplineCoeffs(i,0));
				
				/* Save offset control point: */
				correctionCoefficients[2*i+1]=Misc::Float32(bsplineCoeffs(i,1));
				}
			
			/* Save the depth correction B-spline coefficients: */
			std::string depthCorrectionFileName=KINECT_CONFIG_DIR;
			depthCorrectionFileName.push_back('/');
			depthCorrectionFileName.append(KINECT_CAMERA_DEPTHCORRECTIONFILENAMEPREFIX);
			depthCorrectionFileName.push_back('-');
			depthCorrectionFileName.append(application->camera->getSerialNumber());
			depthCorrectionFileName.append(".dat");
			std::cout<<"Writing depth correction file "<<depthCorrectionFileName<<std::endl;
			IO::FilePtr depthCorrectionFile(Vrui::openFile(depthCorrectionFileName.c_str(),IO::File::WriteOnly));
			depthCorrectionFile->setEndianness(Misc::LittleEndian);
			
			/* Write the number of B-spline coefficients: */
			depthCorrectionFile->write<Misc::SInt32>(degree);
			for(int i=0;i<2;++i)
				depthCorrectionFile->write<Misc::SInt32>(numSegments[i]);
			
			/* Write the B-spline coefficients: */
			depthCorrectionFile->write<Misc::Float32>(correctionCoefficients,numControlPoints*2);
			
			/* Clean up: */
			delete[] correctionCoefficients;
			}
		}
	}
