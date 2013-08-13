/***********************************************************************
PointPlaneTool - Calibration tool for RawKinectViewer.
Copyright (c) 2013 Oliver Kreylos

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

#include "PointPlaneTool.h"

#include <Math/Math.h>
#include <Geometry/PCACalculator.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>

#include "RawKinectViewer.h"

/***************************************
Static elements of class PointPlaneTool:
***************************************/

PointPlaneToolFactory* PointPlaneTool::factory=0;

/*******************************
Methods of class PointPlaneTool:
*******************************/

PointPlaneToolFactory* PointPlaneTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new PointPlaneToolFactory("PointPlaneTool","Define Depth Planes",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Pick Point");
	factory->setButtonFunction(1,"Define Plane");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

PointPlaneTool::PointPlaneTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

PointPlaneTool::~PointPlaneTool(void)
	{
	}

const Vrui::ToolFactory* PointPlaneTool::getFactory(void) const
	{
	return factory;
	}

void PointPlaneTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
			{
			/* Select another depth image point: */
			Point newP=Point(application->calcImagePoint(getButtonDeviceRay(0)));
			if(application->averageFrameValid&&newP[0]>=-double(application->depthFrameSize[0])&&newP[0]<0.0&&newP[1]>=0.0&&newP[1]<double(application->depthFrameSize[1]))
				{
				/* Calculate the selected point's depth: */
				unsigned int x=(unsigned int)(newP[0]+double(application->depthFrameSize[0]));
				unsigned int y=(unsigned int)newP[1];
				unsigned int index=y*application->depthFrameSize[0]+x;
				if(application->averageFrameForeground[index]>=float(application->averageNumFrames)*0.5f)
					{
					newP[0]=double(x)+0.5;
					newP[1]=double(y)+0.5;
					newP[2]=double(application->depthCorrection[index].correct(application->averageFrameDepth[index]/application->averageFrameForeground[index]));
					points.push_back(newP);
					}
				}
			}
		else
			{
			if(points.size()>=3)
				{
				/* Calculate the camera-space plane defined by the selected points: */
				Geometry::PCACalculator<3> pca;
				for(std::vector<Point>::iterator pIt=points.begin();pIt!=points.end();++pIt)
					pca.accumulatePoint(*pIt);
				
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
					/* Let the plane point towards the camera: */
					if(normal[2]>0.0)
						normal=-normal;
					
					/* Set the application's depth plane in camera and world space: */
					application->depthPlaneValid=true;
					application->camDepthPlane=RawKinectViewer::Plane(normal,centroid);
					application->worldDepthPlane=application->camDepthPlane; 
					application->worldDepthPlane.transform(application->intrinsicParameters.depthProjection);
					
					/* Clear the selected point set: */
					points.clear();
					}
				else
					{
					/* Show an error message: */
					Vrui::showErrorMessage("Define Plane","Depth plane equation is undefined");
					}
				}
			else
				{
				/* Show an error message: */
				Vrui::showErrorMessage("Define Plane","Need at least three depth points to define depth plane");
				}
			}
		}
	}

void PointPlaneTool::display(GLContextData& contextData) const
	{
	if(!points.empty())
		{
		glPushAttrib(GL_ENABLE_BIT|GL_POINT_BIT);
		glDisable(GL_LIGHTING);
		glPointSize(3.0f);

		/* Go to navigation coordinates: */
		glPushMatrix();
		const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
		glLoadMatrix(displayState.modelviewNavigational);
		
		glBegin(GL_POINTS);
		glColor3f(1.0f,1.0f,1.0f);
		for(std::vector<Point>::const_iterator pIt=points.begin();pIt!=points.end();++pIt)
			glVertex3d((*pIt)[0]-double(application->depthFrameSize[0]),(*pIt)[1],0.01);
		glEnd();
		
		glPopMatrix();
		
		glPopAttrib();
		}
	}
