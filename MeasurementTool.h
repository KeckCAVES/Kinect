/***********************************************************************
MeasurementTool - Tool to measure 3D positions from the depth image
stream.
Copyright (c) 2014-2015 Oliver Kreylos

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

#ifndef MEASUREMENTTOOL_INCLUDED
#define MEASUREMENTTOOL_INCLUDED

#include <Geometry/Point.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
class RawKinectViewer;

class MeasurementTool;
typedef Vrui::GenericToolFactory<MeasurementTool> MeasurementToolFactory;

class MeasurementTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<MeasurementTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::Point<double,3> Point;
	
	/* Elements: */
	static MeasurementToolFactory* factory; // Pointer to the factory object for this class
	Point selectedPoint; // Selected image point waiting for an average depth frame
	
	/* Private methods: */
	void averageDepthFrameReady(int); // Callback called when an average depth frame has been collected
	
	/* Constructors and destructors: */
	public:
	static MeasurementToolFactory* initClass(Vrui::ToolManager& toolManager);
	MeasurementTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~MeasurementTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	};

#endif
