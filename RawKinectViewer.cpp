/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
Copyright (c) 2010 Oliver Kreylos

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

#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLContextData.h>
#include <GL/GLObject.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/LocatorTool.h>
#include <Vrui/Application.h>

#include "USBContext.h"
#include "FrameBuffer.h"
#include "KinectCamera.h"

class RawKinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint depthTextureId; // ID of texture object holding depth image
		unsigned int depthFrameVersion; // Version number of frame currently texture object
		GLuint colorTextureId; // ID of texture object holding color image
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	USBContext usbContext; // USB device context
	KinectCamera* kinectCamera; // Pointer to camera aspect of Kinect device
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of current depth frame
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of current color frame
	unsigned int selectedPixel[2]; // Coordinates of the selected depth image pixel
	unsigned short selectedPixelPulse[128]; // EKG of depth value of selected pixel
	int selectedPixelCurrentIndex; // Index of most recent value in selected pixel's EKG
	
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData); // Callback when a locator tool's button is pressed
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
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

/******************************************
Methods of class RawKinectViewer::DataItem:
******************************************/

RawKinectViewer::DataItem::DataItem(void)
	:depthTextureId(0),depthFrameVersion(0),
	 colorTextureId(0),colorFrameVersion(0)
	{
	/* Allocate texture objects: */
	glGenTextures(1,&depthTextureId);
	glGenTextures(1,&colorTextureId);
	}

RawKinectViewer::DataItem::~DataItem(void)
	{
	/* Destroy texture objects: */
	glDeleteTextures(1,&depthTextureId);
	glDeleteTextures(1,&colorTextureId);
	}

/********************************
Methods of class RawKinectViewer:
********************************/

void RawKinectViewer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the depth frame triple buffer: */
	depthFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void RawKinectViewer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the color frame triple buffer: */
	colorFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void RawKinectViewer::locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData)
	{
	Vrui::Point pos=cbData->currentTransformation.getOrigin();
	if(pos[0]>=-640.0&&pos[0]<0.0&&pos[1]>=0.0&&pos[1]<480.0)
		{
		/* Select the pixel under the locator: */
		selectedPixel[0]=int(pos[0]+640.0);
		selectedPixel[1]=int(pos[1]);
		
		/* Start the selected pixel's EKG: */
		selectedPixelCurrentIndex=0;
		const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
		selectedPixelPulse[0]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
		for(int i=1;i<128;++i)
			selectedPixelPulse[i]=0;
		}
	else
		{
		/* Select an invalid pixel: */
		selectedPixel[0]=selectedPixel[1]=~0x0U;
		}
	}

void RawKinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	/* Capture one second worth of background frames: */
	kinectCamera->captureBackground(150);
	}

void RawKinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	kinectCamera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(kinectCamera->getRemoveBackground());
	}

GLMotif::PopupMenu* RawKinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Raw Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to capture a background frame: */
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",mainMenu,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&RawKinectViewer::captureBackgroundCallback);
	
	/* Create a toggle button to enable/disable background removal: */
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",mainMenu,"Remove Background");
	removeBackgroundToggle->setToggle(kinectCamera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&RawKinectViewer::removeBackgroundCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

RawKinectViewer::RawKinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 kinectCamera(0),
	 depthFrameVersion(0),colorFrameVersion(0),
	 mainMenu(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to first Kinect camera device on the host: */
	kinectCamera=new KinectCamera(usbContext);
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Start streaming: */
	kinectCamera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::depthStreamingCallback));
	
	/* Select an invalid pixel: */
	selectedPixel[0]=selectedPixel[1]=~0x0U;
	
	/* Initialize navigation transformation: */
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(640),Vrui::Vector(0,1,0));
	}

RawKinectViewer::~RawKinectViewer(void)
	{
	delete mainMenu;
	
	/* Stop streaming: */
	kinectCamera->stopStreaming();
	
	/* Disconnect from the Kinect camera device: */
	delete kinectCamera;
	}

void RawKinectViewer::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Check if the new tool is a locator tool: */
	Vrui::LocatorTool* lt=dynamic_cast<Vrui::LocatorTool*>(cbData->tool);
	if(lt!=0)
		{
		/* Register callbacks with the locator tool: */
		lt->getButtonPressCallbacks().add(this,&RawKinectViewer::locatorButtonPressCallback);
		}
	}

void RawKinectViewer::frame(void)
	{
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		++depthFrameVersion;
		
		if(selectedPixel[0]!=~0x0&&selectedPixel[1]!=~0x0)
			{
			/* Update the selected pixel's EKG: */
			++selectedPixelCurrentIndex;
			if(selectedPixelCurrentIndex==128)
				selectedPixelCurrentIndex=0;
			const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
			selectedPixelPulse[selectedPixelCurrentIndex]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
			}
		}
	
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	}

void RawKinectViewer::display(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_TEXTURE_BIT);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	
	/* Bind the depth texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(dataItem->depthFrameVersion!=depthFrameVersion)
		{
		/* Upload the depth frame into the texture object: */
		const FrameBuffer& depthFrame=depthFrames.getLockedValue();
		unsigned int width=depthFrame.getSize(0);
		unsigned int height=depthFrame.getSize(1);
		const GLushort* framePtr=static_cast<const GLushort*>(depthFrame.getBuffer());
		
		/* Convert the depth image to unsigned byte: */
		GLubyte* byteFrame=new GLubyte[height*width];
		const GLushort* fPtr=framePtr;
		GLubyte* bfPtr=byteFrame;
		for(unsigned int y=0;y<height;++y)
			for(unsigned int x=0;x<width;++x,++fPtr,++bfPtr)
				*bfPtr=GLubyte(255U-(unsigned int)(*fPtr)*256U/(0x0800U));
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the depth texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE8,width,height,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,byteFrame);
		
		delete[] byteFrame;
		
		/* Mark the cached depth frame as up-to-date: */
		dataItem->depthFrameVersion=depthFrameVersion;
		}
	
	/* Draw the depth image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(-640.0f,0.0f);
	glTexCoord2f(1.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(0.0f,480.0f);
	glTexCoord2f(0.0f,1.0f);
	glVertex2f(-640.0f,480.0f);
	glEnd();
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		const FrameBuffer& colorFrame=colorFrames.getLockedValue();
		unsigned int width=colorFrame.getSize(0);
		unsigned int height=colorFrame.getSize(1);
		const GLubyte* framePtr=static_cast<const GLubyte*>(colorFrame.getBuffer());
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the color texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	
	/* Draw the color image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(1.0f,0.0f);
	glVertex2f(640.0f,0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(640.0f,480.0f);
	glTexCoord2f(0.0f,1.0f);
	glVertex2f(0.0f,480.0f);
	glEnd();
	
	/* Protect the texture objects: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	if(selectedPixel[0]!=~0x0&&selectedPixel[1]!=~0x0)
		{
		/* Draw the selected pixel: */
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		
		glBegin(GL_LINES);
		glColor3f(0.0f,1.0f,0.0f);
		GLfloat spx=GLfloat(selectedPixel[0])-639.5f;
		GLfloat spy=GLfloat(selectedPixel[1])+0.5f;
		glVertex3f(spx-5.0f,spy,0.1f);
		glVertex3f(spx+5.0f,spy,0.1f);
		glVertex3f(spx,spy-5.0f,0.1f);
		glVertex3f(spx,spy+5.0f,0.1f);
		glEnd();
		
		/* Draw the selected pixel's EKG: */
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<128;++i)
			glVertex3f(GLfloat(i)*640.0f/128.0f-640.0f,GLfloat(selectedPixelPulse[i])*0.25-512.0f,0.1f);
		glEnd();
		}
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void RawKinectViewer::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	}

int main(int argc,char* argv[])
	{
	try
		{
		char** appDefaults=0;
		RawKinectViewer app(argc,argv,appDefaults);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
