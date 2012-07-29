/***********************************************************************
DepthCorrectionTool - Calibration tool for RawKinectViewer.
Copyright (c) 2012 Oliver Kreylos

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

DepthCorrectionToolFactory* DepthCorrectionTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new DepthCorrectionToolFactory("DepthCorrectionTool","Calibrate Depth Lens",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Save Plane");
	factory->setButtonFunction(0,"Calibrate");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

DepthCorrectionTool::DepthCorrectionTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

DepthCorrectionTool::~DepthCorrectionTool(void)
	{
	}

const Vrui::ToolFactory* DepthCorrectionTool::getFactory(void) const
	{
	return factory;
	}

void DepthCorrectionTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
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
		else
			{
			/* Calculate the per-pixel affine depth correction coefficients: */
			Kinect::FrameSource::PixelDepthCorrection* coefficients=new Kinect::FrameSource::PixelDepthCorrection[application->depthFrameSize[1]*application->depthFrameSize[0]];
			Kinect::FrameSource::PixelDepthCorrection* cPtr=coefficients;
			unsigned int pixelOffset=0;
			for(unsigned int y=0;y<application->depthFrameSize[1];++y)
				{
				for(unsigned int x=0;x<application->depthFrameSize[0];++x,++cPtr,++pixelOffset)
					{
					/* Build the least-squares linear regression system: */
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
						/* Solve for the regression coefficients: */
						Math::Matrix x=atb/ata;
						cPtr->scale=float(x(0));
						cPtr->offset=float(x(1));
						}
					else
						{
						/* Use identity correction if the pixel is underdetermined: */
						cPtr->scale=1.0f;
						cPtr->offset=0.0f;
						}
					}
				}
			
			/* Save the depth correction image: */
			std::string depthCorrectionFileName=KINECT_CONFIG_DIR;
			depthCorrectionFileName.push_back('/');
			depthCorrectionFileName.append(KINECT_CAMERA_DEPTHCORRECTIONFILENAMEPREFIX);
			depthCorrectionFileName.push_back('-');
			depthCorrectionFileName.append(application->camera->getSerialNumber());
			depthCorrectionFileName.append(".dat");
			std::cout<<"Writing depth correction file "<<depthCorrectionFileName<<std::endl;
			IO::FilePtr depthCorrectionFile(Vrui::openFile(depthCorrectionFileName.c_str(),IO::File::WriteOnly));
			depthCorrectionFile->setEndianness(Misc::LittleEndian);
			cPtr=coefficients;
			for(unsigned int y=0;y<application->depthFrameSize[1];++y)
				for(unsigned int x=0;x<application->depthFrameSize[0];++x,++cPtr)
					{
					depthCorrectionFile->write<float>(cPtr->scale);
					depthCorrectionFile->write<float>(cPtr->offset);
					}
			
			/* Clean up: */
			delete[] coefficients;
			}
		}
	}
