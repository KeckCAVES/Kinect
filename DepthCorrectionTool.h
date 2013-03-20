/***********************************************************************
DepthCorrectionTool - Calibration tool for RawKinectViewer.
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

#ifndef DEPTHCORRECTIONTOOL_INCLUDED
#define DEPTHCORRECTIONTOOL_INCLUDED

#include <vector>
#include <Geometry/Plane.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
class RawKinectViewer;

class DepthCorrectionTool;
typedef Vrui::GenericToolFactory<DepthCorrectionTool> DepthCorrectionToolFactory;

class DepthCorrectionTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<DepthCorrectionTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::Plane<double,3> Plane;
	struct DepthFrame // Structure describing a depth frame and its best-fit plane
		{
		/* Elements: */
		public:
		Kinect::FrameBuffer frame; // The averaged depth frame
		Plane plane; // The best-fit plane for the averaged depth frame
		};
	
	/* Elements: */
	static DepthCorrectionToolFactory* factory; // Pointer to the factory object for this class
	
	int degree; // Degree of B-splines approximating the depth correction offsets
	int numSegments[2]; // Number of B-spline segments (horizontal and vertical) approximating the depth correction offsets
	std::vector<DepthFrame> depthFrames; // List of extracted depth frames
	
	/* Private methods: */
	void averageDepthFrameReady(int); // Callback called when an average depth frame has been collected
	
	/* Constructors and destructors: */
	public:
	static DepthCorrectionToolFactory* initClass(Vrui::ToolManager& toolManager);
	DepthCorrectionTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~DepthCorrectionTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	};

#endif
