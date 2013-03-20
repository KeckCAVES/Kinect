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

#ifndef CALIBRATIONCHECKTOOL_INCLUDED
#define CALIBRATIONCHECKTOOL_INCLUDED

#include <vector>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Plane.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class RawKinectViewer;

class CalibrationCheckTool;
typedef Vrui::GenericToolFactory<CalibrationCheckTool> CalibrationCheckToolFactory;

class CalibrationCheckTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<CalibrationCheckTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::Point<double,2> Point2; // Type for 2D points
	
	/* Elements: */
	private:
	static CalibrationCheckToolFactory* factory; // Pointer to the factory object for this class
	
	double dtcOffset,dtcScale; // Offset and scale factors for the depth to displacement function
	Point2* depthToColor; // Tabulation of the non-linear depth to color image mapping function
	int rowOffset; // Row index offset from depth to color frame
	bool haveDepthPoint; // Flag if the last selected depth image point is valid
	Point2 depthPoint; // Last selected depth-image point
	Point2 colorPoint; // Color-image point to which the last selected depth point is matched
	
	/* Constructors and destructors: */
	public:
	static CalibrationCheckToolFactory* initClass(Vrui::ToolManager& toolManager);
	CalibrationCheckTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~CalibrationCheckTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual void initialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
