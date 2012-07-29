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

#include "PauseTool.h"

#include <Vrui/ToolManager.h>

#include "RawKinectViewer.h"

/**********************************
Static elements of class PauseTool:
**********************************/

PauseToolFactory* PauseTool::factory=0;

/**************************
Methods of class PauseTool:
**************************/

PauseToolFactory* PauseTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new PauseToolFactory("PauseTool","Pause Streams",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(1);
	factory->setButtonFunction(0,"Pause");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

PauseTool::PauseTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

PauseTool::~PauseTool(void)
	{
	}

const Vrui::ToolFactory* PauseTool::getFactory(void) const
	{
	return factory;
	}

void PauseTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		application->paused=!application->paused;
	}
