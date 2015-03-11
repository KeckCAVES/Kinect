/***********************************************************************
PlaneTool - Calibration tool for RawKinectViewer.
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

#include "PlaneTool.h"

#include <iostream>
#include <Math/Math.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
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
		/* Stop dragging and extract the plane equation best fitting the currently selected depth pixels: */
		dragging=false;
		int min[2],max[2];
		for(int i=0;i<2;++i)
			{
			double minp=p0[i]<=p1[i]?p0[i]:p1[1];
			min[i]=Math::floor(minp);
			if(i==0)
				min[i]+=application->depthFrameSize[0];
			if(min[i]<0)
				min[i]=0;
			double maxp=p0[i]<=p1[i]?p1[i]:p0[1];
			max[i]=Math::floor(maxp)+1;
			if(i==0)
				max[i]+=application->depthFrameSize[0];
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
			
			/* Print the plane equation in camera space: */
			std::cout<<"Camera-space plane equation: x * "<<cNormal<<" = "<<cCentroid*cNormal<<std::endl;
			}
		else
			Vrui::showErrorMessage("PlaneTool","Could not extract plane equation");
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
