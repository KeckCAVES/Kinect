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

#include <string.h>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/SubMenu.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/CascadeButton.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/Application.h>

#include "USBContext.h"
#include "FrameBuffer.h"
#include "KinectCamera.h"
#include "KinectFrameSaver.h"
#include "KinectPlayback.h"
#include "KinectProjector.h"
// #include "MD5MeshAnimator.h"

/***********************************************************************
Setting PLAYBACK to 1 is just a hack right now; ignore it unless you
understand exactly what it's doing.
***********************************************************************/

#define PLAYBACK 0

class KinectViewer:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	class KinectStreamer // Helper class to stream 3D video data from a Kinect camera to a Kinect projector
		{
		/* Elements: */
		public:
		#if PLAYBACK
		KinectPlayback* camera; // Pointer to the camera
		#else
		KinectCamera* camera; // Pointer to the camera
		#endif
		KinectFrameSaver* frameSaver; // Pointer to helper object saving depth and color frames received from the Kinect
		Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
		Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
		KinectProjector* projector; // Pointer to the projector
		Vrui::OGTransform projectorTransform; // Transformation from projector space to shared model space
		bool enabled; // Flag whether the streamer is currently processing and rendering 3D video frames
		
		/* Private methods: */
		void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
		void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
		
		/* Constructors and destructors: */
		public:
		#if PLAYBACK
		KinectStreamer(const char* depthFrameFileName,const char* colorFrameFileName);
		#else
		KinectStreamer(USBContext& usbContext,int cameraIndex); // Creates a streamer for the Kinect camera of the given index in the given USB context
		#endif
		~KinectStreamer(void); // Destroys the streamer
		
		/* Methods: */
		void frame(void); // Called once per Vrui frame to update state
		void display(GLContextData& contextData) const; // Renders the streamer's current state into the given OpenGL context
		};
	
	/* Elements: */
	private:
	USBContext usbContext; // USB device context
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
	Vrui::InputDevice* cameraDevice; // Pointer to the device to which the depth camera is attached
	// MD5MeshAnimator anim; // An animator
	
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void enableStreamerCallback(Misc::CallbackData* cbData,const size_t& streamerIndex);
	void viewCameraCallback(Misc::CallbackData* cbData,const size_t& streamerIndex);
	void captureBackgroundCallback(Misc::CallbackData* cbData,const size_t& streamerIndex);
	void removeBackgroundCallback(Misc::CallbackData* cbData,const size_t& streamerIndex);
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int& argc,char**& argv,char**& appDefaults);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

/*********************************************
Methods of class KinectViewer::KinectStreamer:
*********************************************/

void KinectViewer::KinectStreamer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Save the frame if requested: */
	if(frameSaver!=0)
		frameSaver->saveDepthFrame(frameBuffer);
	
	if(enabled)
		{
		/* Post the new frame into the depth frame triple buffer: */
		depthFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void KinectViewer::KinectStreamer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Save the frame if requested: */
	if(frameSaver!=0)
		frameSaver->saveColorFrame(frameBuffer);
	
	if(enabled)
		{
		/* Post the new frame into the color frame triple buffer: */
		colorFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

#if PLAYBACK
KinectViewer::KinectStreamer::KinectStreamer(const char* depthFrameFileName,const char* colorFrameFileName)
#else
KinectViewer::KinectStreamer::KinectStreamer(USBContext& context,int cameraIndex)
#endif
	:camera(0),frameSaver(0),
	 projector(0),
	 projectorTransform(Vrui::OGTransform::identity),
	 enabled(true)
	{
	#if PLAYBACK
	/* Open the given frame files: */
	camera=new KinectPlayback(depthFrameFileName,colorFrameFileName);
	std::string serialNumber="B00367608415043B";
	#else
	/* Attach to and open the Kinect camera: */
	camera=new KinectCamera(context,cameraIndex);
	camera->open();
	
	/* Get the camera's serial number to load the proper calibration matrices: */
	std::string serialNumber=camera->getSerialNumber();
	#endif
	
	/* Create a Kinect projector with the proper calibration matrices: */
	std::string calibrationFileName="CameraCalibrationMatrices-";
	calibrationFileName.append(serialNumber);
	calibrationFileName.append(".dat");
	projector=new KinectProjector(calibrationFileName.c_str());
	
	/* Read the camera's model space transformation: */
	std::string transformFileName="ProjectorTransform-";
	transformFileName.append(serialNumber);
	transformFileName.append(".txt");
	Misc::File transformFile(transformFileName.c_str(),"rt");
	char transform[1024];
	transformFile.gets(transform,sizeof(transform));
	projectorTransform=Misc::ValueCoder<Vrui::OGTransform>::decode(transform,transform+strlen(transform),0);
	
	/* Start streaming: */
	camera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer::KinectStreamer>(this,&KinectViewer::KinectStreamer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer::KinectStreamer>(this,&KinectViewer::KinectStreamer::depthStreamingCallback));
	}

KinectViewer::KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	camera->stopStreaming();
	delete frameSaver;
	delete projector;
	
	/* Close and disconnect from the Kinect camera device: */
	delete camera;
	}

void KinectViewer::KinectStreamer::frame(void)
	{
	if(enabled)
		{
		/* Lock the most recent frame in the depth frame triple buffer: */
		if(depthFrames.lockNewValue())
			{
			/* Push the new frame to the Kinect projector: */
			projector->setDepthFrame(depthFrames.getLockedValue());
			}
		
		/* Lock the most recent frame in the color frame triple buffer: */
		if(colorFrames.lockNewValue())
			{
			/* Push the new frame to the Kinect projector: */
			projector->setColorFrame(colorFrames.getLockedValue());
			}
		}
	}

void KinectViewer::KinectStreamer::display(GLContextData& contextData) const
	{
	if(enabled)
		{
		/* Transform the projector into model space: */
		glPushMatrix();
		glMultMatrix(projectorTransform);
		
		/* Draw the current depth image: */
		projector->draw(contextData);
		
		/* Return to navigational coordinates: */
		glPopMatrix();
		}
	}

/*****************************
Methods of class KinectViewer:
*****************************/

void KinectViewer::enableStreamerCallback(Misc::CallbackData* cbData,const size_t& streamerIndex)
	{
	GLMotif::ToggleButton::ValueChangedCallbackData* myCbData=static_cast<GLMotif::ToggleButton::ValueChangedCallbackData*>(cbData);
	
	/* Set the streamer's enable flag: */
	streamers[streamerIndex]->enabled=myCbData->set;
	}

void KinectViewer::viewCameraCallback(Misc::CallbackData* cbData,const size_t& streamerIndex)
	{
	/* Move the camera position to the viewer's head position: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	
	/* Rotate the frame to align the projection direction with the viewer's viewing direction: */
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	
	/* Account for the projector's model space transformation: */
	nav*=Geometry::invert(streamers[streamerIndex]->projectorTransform);
	
	Vrui::setNavigationTransformation(nav);
	}

void KinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData,const size_t& streamerIndex)
	{
	/* Capture five second worth of background frames: */
	streamers[streamerIndex]->camera->captureBackground(150);
	}

void KinectViewer::removeBackgroundCallback(Misc::CallbackData* cbData,const size_t& streamerIndex)
	{
	GLMotif::ToggleButton::ValueChangedCallbackData* myCbData=static_cast<GLMotif::ToggleButton::ValueChangedCallbackData*>(cbData);
	
	/* Set the background removal flag: */
	streamers[streamerIndex]->camera->setRemoveBackground(myCbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	myCbData->toggle->setToggle(streamers[streamerIndex]->camera->getRemoveBackground());
	}

GLMotif::PopupMenu* KinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a submenu for each Kinect streamer: */
	for(size_t i=0;i<streamers.size();++i)
		{
		/* Create the submenu's top-level shell: */
		GLMotif::Popup* streamerPopup=new GLMotif::Popup("StreamerPopup",Vrui::getWidgetManager());
		GLMotif::SubMenu* streamerMenu=new GLMotif::SubMenu("StreamerMenu",streamerPopup,false);
		
		/* Create a toggle button to enable / disable the streamer: */
		GLMotif::ToggleButton* enableToggle=new GLMotif::ToggleButton("EnableToggle",streamerMenu,"Enabled");
		enableToggle->setToggle(streamers[i]->enabled);
		enableToggle->getValueChangedCallbacks().add(this,&KinectViewer::enableStreamerCallback,i);
		
		/* Create a button to view from camera's point of view: */
		GLMotif::Button* viewCameraButton=new GLMotif::Button("ViewCameraButton",streamerMenu,"View from Camera");
		viewCameraButton->getSelectCallbacks().add(this,&KinectViewer::viewCameraCallback,i);
		
		/* Create a button to capture background frames: */
		GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",streamerMenu,"Capture Background");
		captureBackgroundButton->getSelectCallbacks().add(this,&KinectViewer::captureBackgroundCallback,i);
		
		/* Create a toggle button to enable/disable background removal: */
		GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",streamerMenu,"Remove Background");
		removeBackgroundToggle->setToggle(streamers[i]->camera->getRemoveBackground());
		removeBackgroundToggle->getValueChangedCallbacks().add(this,&KinectViewer::removeBackgroundCallback,i);
		
		streamerMenu->manageChild();
		
		/* Create a cascade button to show the Kinect streamer's submenu: */
		char streamerName[40];
		snprintf(streamerName,sizeof(streamerName),"StreamerCascade%u",(unsigned int)i);
		char streamerLabel[40];
		snprintf(streamerLabel,sizeof(streamerLabel),"Streamer %u",(unsigned int)(i+1));
		GLMotif::CascadeButton* streamerCascade=new GLMotif::CascadeButton(streamerName,mainMenu,streamerLabel);
		streamerCascade->setPopup(streamerPopup);
		}
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

KinectViewer::KinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 cameraDevice(Vrui::findInputDevice("Camera")),
	 mainMenu(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to given Kinect camera device on the host: */
	#if PLAYBACK
	streamers.push_back(new KinectStreamer("/work/okreylos/3DVideo/Kinect/DepthFrames.dat","/work/okreylos/3DVideo/Kinect/ColorFrames.dat"));
	#else
	
	/* Read the index of the Kinect camera to which to connect: */
	int cameraIndex=0;
	if(argc>=2)
		cameraIndex=atoi(argv[1]);
	
	/* Add a streamer for the selected camera: */
	streamers.push_back(new KinectStreamer(usbContext,cameraIndex));
	
	/* Save frames from the first Kinect streamer: */
	// streamers[0]->frameSaver=new KinectFrameSaver("/work/okreylos/3DVideo/Kinect/DepthFrames2.dat","/work/okreylos/3DVideo/Kinect/ColorFrames2.dat");
	
	/* Add a streamer for another Kinect camera: */
	// streamers.push_back(new KinectStreamer(usbContext,1));
	#endif
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Initialize navigation transformation: */
	viewCameraCallback(0,0);
	}

KinectViewer::~KinectViewer(void)
	{
	delete mainMenu;
	
	/* Delete all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		delete *sIt;
	}

void KinectViewer::frame(void)
	{
	/* Process all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->frame();
	
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
	
	/* Process all streamers: */
	for(std::vector<KinectStreamer*>::const_iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->display(contextData);
	
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
