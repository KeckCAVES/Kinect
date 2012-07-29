/***********************************************************************
PauseTool - Calibration tool for RawKinectViewer.
Copyright (c) 2010-2012 Oliver Kreylos

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

#ifndef PAUSETOOL_INCLUDED
#define PAUSETOOL_INCLUDED

#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class RawKinectViewer;

class PauseTool;
typedef Vrui::GenericToolFactory<PauseTool> PauseToolFactory;

class PauseTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<PauseTool>;
	
	/* Elements: */
	private:
	static PauseToolFactory* factory; // Pointer to the factory object for this class
	
	/* Constructors and destructors: */
	public:
	static PauseToolFactory* initClass(Vrui::ToolManager& toolManager);
	PauseTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~PauseTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	};

#endif
