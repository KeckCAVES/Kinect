/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
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

#ifndef RAWKINECTVIEWER_INCLUDED
#define RAWKINECTVIEWER_INCLUDED

#include <Threads/TripleBuffer.h>
#include <USB/Context.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/FileSelectionDialog.h>
#include <Vrui/LocatorTool.h>
#include <Vrui/Application.h>
#include <Kinect/FrameBuffer.h>

/* Forward declarations: */
namespace GLMotif {
class PopupMenu;
}
namespace Kinect {
class Camera;
}

class RawKinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		unsigned int colorTextureSize[2]; // Padded size of the color texture
		GLuint colorTextureId; // ID of texture object holding color image
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		unsigned int depthTextureSize[2]; // Padded size of the depth texture
		GLuint depthTextureId; // ID of texture object holding depth image
		unsigned int depthFrameVersion; // Version number of frame currently texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	friend class PauseTool;
	friend class TiePointTool;
	friend class LineTool;
	friend class DepthCorrectionTool;
	friend class GridTool;
	friend class PlaneTool;
	
	/* Elements: */
	USB::Context usbContext; // USB device context
	Kinect::Camera* camera; // Pointer to camera aspect of Kinect device
	const unsigned int* colorFrameSize; // Size of color frames in pixels
	Threads::TripleBuffer<Kinect::FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of current color frame
	const unsigned int* depthFrameSize; // Size of depth frames in pixels
	bool hasDepthCorrection; // Flag whether the camera has per-pixel depth correction coefficients
	Kinect::FrameBuffer depthCorrection; // Buffer containing per-pixel depth correction coefficients
	float depthValueRange[2]; // Range of depth values mapped to the depth color map
	Threads::TripleBuffer<Kinect::FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of current depth frame
	bool paused; // Flag whether the video stream display is paused
	unsigned int averageNumFrames; // Number of depth frames to average
	unsigned int averageFrameCounter; // Number of depth frames still to accumulate
	float* averageFrameDepth; // Average depth values in depth frame
	float* averageFrameForeground; // Ratio of foreground vs background in depth frame
	bool showAverageFrame; // Flag whether to show the averaged frame
	unsigned int selectedPixel[2]; // Coordinates of the selected depth image pixel
	unsigned short selectedPixelPulse[128]; // EKG of depth value of selected pixel
	int selectedPixelCurrentIndex; // Index of most recent value in selected pixel's EKG
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	Vrui::Point calcImagePoint(const Vrui::Ray& physicalRay) const; // Returns image-space point at which the given physical-space ray intersects the image plane
	void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData); // Callback when a locator tool's button is pressed
	void resetNavigationCallback(Misc::CallbackData* cbData);
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void saveAverageFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void saveAverageFrameCallback(Misc::CallbackData* cbData);
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	
	/* Constructors and destructors: */
	public:
	RawKinectViewer(int& argc,char**& argv,char**& appDefaults);
	virtual ~RawKinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

#endif
