/***********************************************************************
PlaneTool - Calibration tool for RawKinectViewer.
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

#include "PlaneTool.h"

#include <iostream>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Box.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>
#include <Kinect/Camera.h>

#include "RawKinectViewer.h"

/**********************************
Static elements of class PlaneTool:
**********************************/

PlaneToolFactory* PlaneTool::factory=0;

/**************************
Methods of class PlaneTool:
**************************/

PlaneToolFactory* PlaneTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new PlaneToolFactory("PlaneTool","Extract Planes",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(1);
	factory->setButtonFunction(0,"Draw Rectangle");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

PlaneTool::PlaneTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 dragging(false)
	{
	}

PlaneTool::~PlaneTool(void)
	{
	}

const Vrui::ToolFactory* PlaneTool::getFactory(void) const
	{
	return factory;
	}

void PlaneTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		/* Get the initial rectangle point and start dragging: */
		p0=Point(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		dragging=true;
		}
	else
		{
		/* Find the bounding box of the selected rectangle in distortion-corrected depth image space: */
		Geometry::Box<double,2> rect=Geometry::Box<double,2>::empty;
		Geometry::Box<double,2> imgRect=Geometry::Box<double,2>::empty;
		if(application->intrinsicParameters.depthLensDistortion.isIdentity())
			{
			/* No lens distortion correction needed; build rectangle from corner points: */
			rect.addPoint(Point(application->calcDepthImagePoint(p0).getComponents()));
			rect.addPoint(Point(application->calcDepthImagePoint(p1).getComponents()));
			}
		else
			{
			/* Get the selected rectangle in image space: */
			imgRect.addPoint(p0);
			imgRect.addPoint(p1);
			
			/* Calculate the lens distortion-corrected extents of the selected rectangle: */
			Point::Vector vx=imgRect.max-imgRect.min;
			vx[1]=0.0;
			for(int x=0;x<64;++x)
				{
				/* A point along the bottom edge: */
				Point b0=imgRect.min+vx*(double(x)/64.0);
				rect.addPoint(Point(application->calcDepthImagePoint(b0).getComponents()));
				
				/* A point along the top edge: */
				Point b1=imgRect.max-vx*(double(x)/64.0);
				rect.addPoint(Point(application->calcDepthImagePoint(b1).getComponents()));
				}
			Point::Vector vy=imgRect.max-imgRect.min;
			vy[0]=0.0;
			for(int y=0;y<64;++y)
				{
				/* A point along the left edge: */
				Point b0=imgRect.min+vy*(double(y+1)/64.0);
				rect.addPoint(Point(application->calcDepthImagePoint(b0).getComponents()));
				
				/* A point along the right edge: */
				Point b1=imgRect.max-vy*(double(y+1)/64.0);
				rect.addPoint(Point(application->calcDepthImagePoint(b1).getComponents()));
				}
			}
		
		/* Calculate the rectangle's pixel boundaries: */
		int min[2],max[2];
		for(int i=0;i<2;++i)
			{
			min[i]=Math::floor(rect.min[i]);
			if(min[i]<0)
				min[i]=0;
			max[i]=Math::floor(rect.max[i])+1;
			if(max[i]>int(application->depthFrameSize[i]))
				max[i]=int(application->depthFrameSize[i]);
			}
		
		/* Calculate the selected pixels' plane equation in depth image space: */
		typedef Geometry::PCACalculator<3>::Point PPoint;
		typedef Geometry::PCACalculator<3>::Vector PVector;
		Geometry::PCACalculator<3> pca;
		
		const float* afdRow=application->averageFrameDepth+min[1]*application->depthFrameSize[0];
		const float* affRow=application->averageFrameForeground+min[1]*application->depthFrameSize[0];
		float foregroundCutoff=float(application->averageNumFrames)*0.5f;
		if(application->intrinsicParameters.depthLensDistortion.isIdentity())
			{
			/* No lens distortion correction required: */
			if(application->depthCorrection!=0)
				{
				const RawKinectViewer::PixelCorrection* dcRow=application->depthCorrection+min[1]*application->depthFrameSize[0];
				for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0],dcRow+=application->depthFrameSize[0])
					{
					double dy=double(y)+0.5;
					const float* afdPtr=afdRow+min[0];
					const float* affPtr=affRow+min[0];
					const RawKinectViewer::PixelCorrection* dcPtr=dcRow+min[0];
					for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr,++dcPtr)
						{
						double dx=double(x)+0.5;
						if(*affPtr>=foregroundCutoff)
							pca.accumulatePoint(PPoint(dx,dy,dcPtr->correct((*afdPtr)/(*affPtr))));
						}
					}
				}
			else
				{
				for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0])
					{
					double dy=double(y)+0.5;
					const float* afdPtr=afdRow+min[0];
					const float* affPtr=affRow+min[0];
					for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr)
						{
						double dx=double(x)+0.5;
						if(*affPtr>=foregroundCutoff)
							pca.accumulatePoint(PPoint(dx,dy,(*afdPtr)/(*affPtr)));
						}
					}
				}
			}
		else
			{
			/* Account for lens distortion correction by checking every pixel against the selected rectangle: */
			if(application->depthCorrection!=0)
				{
				const RawKinectViewer::PixelCorrection* dcRow=application->depthCorrection+min[1]*application->depthFrameSize[0];
				for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0],dcRow+=application->depthFrameSize[0])
					{
					double dy=double(y)+0.5;
					const float* afdPtr=afdRow+min[0];
					const float* affPtr=affRow+min[0];
					const RawKinectViewer::PixelCorrection* dcPtr=dcRow+min[0];
					for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr,++dcPtr)
						{
						double dx=double(x)+0.5;
						if(*affPtr>=foregroundCutoff)
							{
							/* Check if the pixel is inside the selected rectangle: */
							if(imgRect.contains(Point(application->getDepthImagePoint(x,y).getComponents())))
								pca.accumulatePoint(PPoint(dx,dy,dcPtr->correct((*afdPtr)/(*affPtr))));
							}
						}
					}
				}
			else
				{
				for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0])
					{
					double dy=double(y)+0.5;
					const float* afdPtr=afdRow+min[0];
					const float* affPtr=affRow+min[0];
					for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr)
						{
						double dx=double(x)+0.5;
						if(*affPtr>=foregroundCutoff)
							{
							/* Check if the pixel is inside the selected rectangle: */
							if(imgRect.contains(Point(application->getDepthImagePoint(x,y).getComponents())))
								pca.accumulatePoint(PPoint(dx,dy,(*afdPtr)/(*affPtr)));
							}
						}
					}
				}
			}
		
		/* Calculate the selected pixels' plane equation: */
		PPoint centroid=pca.calcCentroid();
		pca.calcCovariance();
		double evs[3];
		pca.calcEigenvalues(evs);
		PVector normal=pca.calcEigenvector(evs[2]);
		
		/* Check for any nans or infs: */
		bool allFinite=true;
		for(int i=0;i<3;++i)
			{
			allFinite=allFinite&&Math::isFinite(normal[i]);
			allFinite=allFinite&&Math::isFinite(centroid[i]);
			}
		
		if(allFinite)
			{
			/* Print the approximation residual: */
			std::cout<<"Approximation residual: "<<evs[2]<<std::endl;
			
			/* Print the plane equation in depth image space: */
			std::cout<<"Depth-space plane equation: x * "<<normal<<" = "<<centroid*normal<<std::endl;
			
			/* Get the camera's intrinsic parameters: */
			Kinect::FrameSource::IntrinsicParameters ips=application->camera->getIntrinsicParameters();
			
			/* Transform the plane equation to camera space: */
			PVector v0=Geometry::normal(normal);
			PVector v1=normal^v0;
			PPoint p0=centroid+v0;
			PPoint p1=centroid+v1;
			PPoint cCentroid=ips.depthProjection.transform(centroid);
			PPoint cP0=ips.depthProjection.transform(p0);
			PPoint cP1=ips.depthProjection.transform(p1);
			PVector cNormal=(cP0-cCentroid)^(cP1-cCentroid);
			cNormal.normalize();
			
			double rms2=0.0;
			unsigned int numPoints=0;
			{
			const float* afdRow=application->averageFrameDepth+min[1]*application->depthFrameSize[0];
			const float* affRow=application->averageFrameForeground+min[1]*application->depthFrameSize[0];
			float foregroundCutoff=float(application->averageNumFrames)*0.5f;
			if(application->intrinsicParameters.depthLensDistortion.isIdentity())
				{
				/* No lens distortion correction required: */
				if(application->depthCorrection!=0)
					{
					const RawKinectViewer::PixelCorrection* dcRow=application->depthCorrection+min[1]*application->depthFrameSize[0];
					for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0],dcRow+=application->depthFrameSize[0])
						{
						double dy=double(y)+0.5;
						const float* afdPtr=afdRow+min[0];
						const float* affPtr=affRow+min[0];
						const RawKinectViewer::PixelCorrection* dcPtr=dcRow+min[0];
						for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr,++dcPtr)
							{
							double dx=double(x)+0.5;
							if(*affPtr>=foregroundCutoff)
								{
								/* Check if the pixel is inside the selected rectangle: */
								if(imgRect.contains(Point(application->getDepthImagePoint(x,y).getComponents())))
									{
									rms2+=Math::sqr((ips.depthProjection.transform(PPoint(dx,dy,dcPtr->correct((*afdPtr)/(*affPtr))))-cCentroid)*cNormal);
									++numPoints;
									}
								}
							}
						}
					}
				else
					{
					for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0])
						{
						double dy=double(y)+0.5;
						const float* afdPtr=afdRow+min[0];
						const float* affPtr=affRow+min[0];
						for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr)
							{
							double dx=double(x)+0.5;
							if(*affPtr>=foregroundCutoff)
								{
								/* Check if the pixel is inside the selected rectangle: */
								if(imgRect.contains(Point(application->getDepthImagePoint(x,y).getComponents())))
									{
									rms2+=Math::sqr((ips.depthProjection.transform(PPoint(dx,dy,(*afdPtr)/(*affPtr)))-cCentroid)*cNormal);
									++numPoints;
									}
								}
							}
						}
					}
				}
			else
				{
				/* Account for lens distortion correction by checking every pixel against the selected rectangle: */
				if(application->depthCorrection!=0)
					{
					const RawKinectViewer::PixelCorrection* dcRow=application->depthCorrection+min[1]*application->depthFrameSize[0];
					for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0],dcRow+=application->depthFrameSize[0])
						{
						double dy=double(y)+0.5;
						const float* afdPtr=afdRow+min[0];
						const float* affPtr=affRow+min[0];
						const RawKinectViewer::PixelCorrection* dcPtr=dcRow+min[0];
						for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr,++dcPtr)
							{
							double dx=double(x)+0.5;
							if(*affPtr>=foregroundCutoff)
								{
								rms2+=Math::sqr((ips.depthProjection.transform(PPoint(dx,dy,dcPtr->correct((*afdPtr)/(*affPtr))))-cCentroid)*cNormal);
								++numPoints;
								}
							}
						}
					}
				else
					{
					for(int y=min[1];y<max[1];++y,afdRow+=application->depthFrameSize[0],affRow+=application->depthFrameSize[0])
						{
						double dy=double(y)+0.5;
						const float* afdPtr=afdRow+min[0];
						const float* affPtr=affRow+min[0];
						for(int x=min[0];x<max[0];++x,++afdPtr,++affPtr)
							{
							double dx=double(x)+0.5;
							if(*affPtr>=foregroundCutoff)
								{
								rms2+=Math::sqr((ips.depthProjection.transform(PPoint(dx,dy,(*afdPtr)/(*affPtr)))-cCentroid)*cNormal);
								++numPoints;
								}
							}
						}
					}
				}
			}
			std::cout<<"Camera-space approximation RMS: "<<Math::sqrt(rms2/double(numPoints))<<std::endl;
			
			/* Print the plane equation in camera space: */
			std::cout<<"Camera-space plane equation: x * "<<cNormal<<" = "<<cCentroid*cNormal<<std::endl;
			}
		else
			Vrui::showErrorMessage("PlaneTool","Could not extract plane equation");
		
		dragging=false;
		}
	}

void PlaneTool::frame(void)
	{
	if(dragging)
		{
		/* Get the current rectangle point: */
		p1=Point(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		}
	}

void PlaneTool::display(GLContextData& contextData) const
	{
	if(dragging)
		{
		glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		
		/* Go to navigation coordinates: */
		glPushMatrix();
		const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
		glLoadMatrix(displayState.modelviewNavigational);
		
		/* Draw the current rectangle: */
		glColor3f(0.0f,0.333f,0.0f);
		glBegin(GL_LINE_LOOP);
		glVertex3d(p0[0],p0[1],0.01);
		glVertex3d(p1[0],p0[1],0.01);
		glVertex3d(p1[0],p1[1],0.01);
		glVertex3d(p0[0],p1[1],0.01);
		glEnd();
		
		glPopMatrix();

		glPopAttrib();
		}
	}
