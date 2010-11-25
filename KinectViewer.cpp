/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
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
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/Application.h>

#include "USBContext.h"
#include "FrameBuffer.h"
#include "KinectCamera.h"
#include "KinectProjector.h"
// #include "MD5MeshAnimator.h"

class KinectViewer:public Vrui::Application
	{
	/* Elements: */
	private:
	USBContext usbContext; // USB device context
	KinectCamera* kinectCamera; // Pointer to camera aspect of Kinect device
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	KinectProjector kinectProjector; // Class to project depth and color frames back into 3D camera space
	Vrui::InputDevice* cameraDevice; // Pointer to the device to which the depth camera is attached
	// MD5MeshAnimator anim; // An animator
	
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void viewCameraCallback(Misc::CallbackData* cbData);
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int& argc,char**& argv,char**& appDefaults);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

/*****************************
Methods of class KinectViewer:
*****************************/

void KinectViewer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the depth frame triple buffer: */
	depthFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void KinectViewer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the color frame triple buffer: */
	colorFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void KinectViewer::viewCameraCallback(Misc::CallbackData* cbData)
	{
	/* Move the origin of model space to the main viewer's head position: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	
	/* Rotate model space such that the negative Z direction is aligned with the main viewer's view direction: */
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	
	Vrui::setNavigationTransformation(nav);
	}

void KinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	/* Capture one second worth of background frames: */
	kinectCamera->captureBackground(150);
	}

void KinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	kinectCamera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(kinectCamera->getRemoveBackground());
	}

GLMotif::PopupMenu* KinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to view from camera's point of view: */
	GLMotif::Button* viewCameraButton=new GLMotif::Button("ViewCameraButton",mainMenu,"View from Camera");
	viewCameraButton->getSelectCallbacks().add(this,&KinectViewer::viewCameraCallback);
	
	/* Create a button to capture a background frame: */
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",mainMenu,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&KinectViewer::captureBackgroundCallback);
	
	/* Create a toggle button to enable/disable background removal: */
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",mainMenu,"Remove Background");
	removeBackgroundToggle->setToggle(kinectCamera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&KinectViewer::removeBackgroundCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

KinectViewer::KinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 kinectCamera(0),
	 kinectProjector("CameraCalibrationMatrices.dat"),
	 cameraDevice(Vrui::findInputDevice("Camera")),
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
	kinectCamera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer>(this,&KinectViewer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer>(this,&KinectViewer::depthStreamingCallback));
	
	/* Initialize navigation transformation: */
	viewCameraCallback(0);
	}

KinectViewer::~KinectViewer(void)
	{
	delete mainMenu;
	
	/* Stop streaming: */
	kinectCamera->stopStreaming();
	
	/* Disconnect from the Kinect camera device: */
	delete kinectCamera;
	}

void KinectViewer::frame(void)
	{
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		/* Push the new frame to the Kinect projector: */
		kinectProjector.setDepthFrame(depthFrames.getLockedValue());
		}
	
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		{
		/* Push the new frame to the Kinect projector: */
		kinectProjector.setColorFrame(colorFrames.getLockedValue());
		}
	
	#if 0
	/* Animate the animated model: */
	anim.frame();
	#endif
	}

void KinectViewer::display(GLContextData& contextData) const
	{
	if(cameraDevice!=0)
		{
		/* Move the camera projector to the tracking device's position and orientation in physical space: */
		glPushMatrix();
		glMultMatrix(cameraDevice->getTransformation());
		glRotated(90.0,1.0,0.0,0.0);
		glScaled(1.0/2.54,1.0/2.54,1.0/2.54);
		
		glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(100.0f,0.0f,0.0f);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(0.0f,100.0f,0.0f);
		glColor3f(0.0f,0.0f,1.0f);
		glVertex3f(0.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,100.0f);
		glEnd();
		}
	
	/* Draw the current depth image: */
	kinectProjector.draw(contextData);
	
	if(cameraDevice!=0)
		{
		/* Return to navigational space: */
		glPopMatrix();
		}
	
	#if 0
	/* Draw the animated model: */
	glPushMatrix();
	glTranslated(7.71,-29.37,-94.57);
	glScaled(0.2,0.2,0.2);
	glRotated(120.0,0.0,1.0,0.0);
	glRotated(-90.0,1.0,0.0,0.0);
	anim.glRenderAction(contextData);
	glPopMatrix();
	#endif
	}

int main(int argc,char* argv[])
	{
	try
		{
		char** appDefaults=0;
		KinectViewer app(argc,argv,appDefaults);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
