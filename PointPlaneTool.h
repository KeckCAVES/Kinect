/***********************************************************************
PointPlaneTool - Calibration tool for RawKinectViewer.
Copyright (c) 2013-2015 Oliver Kreylos

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

#ifndef POINTPLANETOOL_INCLUDED
#define POINTPLANETOOL_INCLUDED

#include <Geometry/Point.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class RawKinectViewer;

class PointPlaneTool;
typedef Vrui::GenericToolFactory<PointPlaneTool> PointPlaneToolFactory;

class PointPlaneTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<PointPlaneTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::Point<double,3> Point;
	
	/* Elements: */
	private:
	static PointPlaneToolFactory* factory; // Pointer to the factory object for this class
	
	std::vector<Point> points; // List of selected depth image points
	
	/* Constructors and destructors: */
	public:
	static PointPlaneToolFactory* initClass(Vrui::ToolManager& toolManager);
	PointPlaneTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~PointPlaneTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void display(GLContextData& contextData) const;
	};

#endif
