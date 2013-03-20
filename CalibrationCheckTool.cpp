/***********************************************************************
CalibrationCheckTool - Tool to check the calibration between depth and
color camera inside RawKinectViewer.
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

#include "CalibrationCheckTool.h"

#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>
#include <Kinect/Camera.h>

#include "RawKinectViewer.h"

/*********************************************
Static elements of class CalibrationCheckTool:
*********************************************/

CalibrationCheckToolFactory* CalibrationCheckTool::factory=0;

/*************************************
Methods of class CalibrationCheckTool:
*************************************/

CalibrationCheckToolFactory* CalibrationCheckTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new CalibrationCheckToolFactory("CalibrationCheckTool","Check Calibration",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(1);
	factory->setButtonFunction(0,"Select Depth Point");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

CalibrationCheckTool::CalibrationCheckTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 depthToColor(0),
	 haveDepthPoint(false)
	{
	}

CalibrationCheckTool::~CalibrationCheckTool(void)
	{
	delete[] depthToColor;
	}

void CalibrationCheckTool::initialize(void)
	{
	/* Retrieve the factory calibration data: */
	Kinect::Camera::CalibrationParameters cal;
	application->camera->getCalibrationParameters(cal);
	
	/* Calculate the depth-to-distance conversion formula: */
	double numerator=10.0*(4.0*cal.dcmosEmitterDist*cal.referenceDistance)/cal.referencePixelSize;
	double denominator=4.0*cal.dcmosEmitterDist/cal.referencePixelSize+4.0*cal.constantShift+1.5;
	
	/* Calculate the depth-to-color mapping: */
	double colorShiftA=cal.dcmosRcmosDist/(cal.referencePixelSize*2.0)+0.375;
	double colorShiftB=cal.dcmosRcmosDist*cal.referenceDistance*10.0/(cal.referencePixelSize*2.0);
	dtcOffset=colorShiftA-colorShiftB*denominator/numerator;
	dtcScale=colorShiftB/numerator;
	
	/* Tabulate the bivariate pixel mapping polynomial using forward differencing: */
	depthToColor=new Point2[480*640];
	double ax=cal.ax;
	double bx=cal.bx;
	double cx=cal.cx;
	double dx=cal.dx;
	
	double ay=cal.ay;
	double by=cal.by;
	double cy=cal.cy;
	double dy=cal.dy;
	
	double dx0=(cal.dxStart<<13)>>4;
	double dy0=(cal.dyStart<<13)>>4;
	
	double dxdx0=(cal.dxdxStart<<11)>>3;
	double dxdy0=(cal.dxdyStart<<11)>>3;
	double dydx0=(cal.dydxStart<<11)>>3;
	double dydy0=(cal.dydyStart<<11)>>3;
	
	double dxdxdx0=(cal.dxdxdxStart<<5)<<3;
	double dydxdx0=(cal.dydxdxStart<<5)<<3;
	double dxdxdy0=(cal.dxdxdyStart<<5)<<3;
	double dydxdy0=(cal.dydxdyStart<<5)<<3;
	double dydydx0=(cal.dydydxStart<<5)<<3;
	double dydydy0=(cal.dydydyStart<<5)<<3;
	
	for(int y=0;y<480;++y)
		{
		dxdxdx0+=cx;
		dxdx0  +=dydxdx0/256.0;
		dydxdx0+=dx;
		
		dx0    +=dydx0/64.0;
		dydx0  +=dydydx0/256.0;
		dydydx0+=bx;
		
		dxdxdy0+=cy;
		
		dxdy0  +=dydxdy0/256.0;
		dydxdy0+=dy;
		
		dy0    +=dydy0/64.0;
		dydy0  +=dydydy0/256.0;
		dydydy0+=by;
		
		double coldxdxdy=dxdxdy0;
		double coldxdy=dxdy0;
		double coldy=dy0;
		
		double coldxdxdx=dxdxdx0;
		double coldxdx=dxdx0;
		double coldx=dx0;
		
		for(int x=0;x<640;++x)
			{
			depthToColor[y*640+x][0]=double(x)+coldx/131072.0+1.0;
			depthToColor[y*640+x][1]=double(y)+coldy/131072.0+1.0;
			
			coldx+=coldxdx/64.0;
			coldxdx+=coldxdxdx/256.0;
			coldxdxdx+=ax;
			
			coldy+=coldxdy/64.0;
			coldxdy+=coldxdxdy/256.0;
			coldxdxdy+=ay;
			}
		}
	
	/* Store the row offset between the depth and color images: */
	rowOffset=cal.startLines;
	}

const Vrui::ToolFactory* CalibrationCheckTool::getFactory(void) const
	{
	return factory;
	}

void CalibrationCheckTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	}

void CalibrationCheckTool::frame(void)
	{
	if(getButtonState(0))
		{
		/* Get the interaction point: */
		haveDepthPoint=false;
		Vrui::Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		if(p[0]>=-640.0&&p[0]<0.0&&p[1]>=0.0&&p[1]<480.0)
			{
			int px=int(Math::floor(p[0]+640.0));
			int py=int(Math::floor(p[1]));
			if(application->averageFrameForeground[py*640+px]>=float(application->averageNumFrames)*0.5f)
				{
				/* Remember the depth point: */
				depthPoint=Point2(double(px)+0.5-640.0,double(py)+0.5);
				
				/* Calculate the color point: */
				double depth=application->depthCorrection[py*640+px].correct(application->averageFrameDepth[py*640+px]/application->averageFrameForeground[py*640+px]);
				
				int cx=int(Math::floor(depthToColor[(479-py)*640+px][0]+dtcOffset+dtcScale*depth));
				int cy=479-int(Math::floor(depthToColor[(479-py)*640+px][1]))-rowOffset;
				colorPoint[0]=double(cx)+0.5;
				colorPoint[1]=double(cy)+0.5;
				haveDepthPoint=true;
				}
			}
		}
	}

void CalibrationCheckTool::display(GLContextData& contextData) const
	{
	if(haveDepthPoint)
		{
		glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		
		/* Go to navigation coordinates: */
		glPushMatrix();
		const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
		glLoadMatrix(displayState.modelviewNavigational);
		
		glBegin(GL_LINES);
		glColor3f(0.0f,0.333f,0.0f);
		
		/* Draw the depth-image point: */
		glVertex3d(depthPoint[0],0.0,0.01);
		glVertex3d(depthPoint[0],480.0,0.01);
		glVertex3d(-640.0,depthPoint[1],0.01);
		glVertex3d(0.0,depthPoint[1],0.01);
		
		/* Draw the color-image point: */
		glVertex3d(colorPoint[0],0.0,0.01);
		glVertex3d(colorPoint[0],480.0,0.01);
		glVertex3d(0.0,colorPoint[1],0.01);
		glVertex3d(640.0,colorPoint[1],0.01);
		glEnd();
		
		glPopMatrix();
		
		glPopAttrib();
		}
	}
