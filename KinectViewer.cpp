/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
Copyright (c) 2010-2011 Oliver Kreylos

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

/***********************************************************************
Setting PLAYBACK to 1 is just a hack right now; ignore it unless you
understand exactly what it's doing.
***********************************************************************/

#define PLAYBACK 0

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
#include <GLMotif/WidgetManager.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Menu.h>
#include <GLMotif/SubMenu.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/CascadeButton.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/Application.h>
#include <Kinect/USBContext.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/KinectCamera.h>
#if PLAYBACK
#include <Kinect/KinectPlayback.h>
#endif
#include <Kinect/KinectProjector.h>
// #include "MD5MeshAnimator.h"

class KinectViewer:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	class KinectStreamer // Helper class to stream 3D video data from a Kinect camera to a Kinect projector
		{
		/* Elements: */
		public:
		#if PLAYBACK
		KinectPlayback* camera; // Pointer to the fake camera
		#else
		KinectCamera* camera; // Pointer to the camera
		#endif
		Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
		Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
		KinectProjector* projector; // Pointer to the projector
		Vrui::OGTransform projectorTransform; // Transformation from projector space to shared model space
		bool enabled; // Flag whether the streamer is currently processing and rendering 3D video frames
		unsigned short maxDepth; // Maximum depth value for background removal
		
		/* Private methods: */
		void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
		void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
		void showFacadeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void showFromCameraCallback(Misc::CallbackData* cbData);
		void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void backgroundCaptureCompleteCallback(KinectCamera& camera);
		void captureBackgroundCallback(Misc::CallbackData* cbData);
		void saveBackgroundCallback(Misc::CallbackData* cbData);
		void backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
		void backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
		
		/* Constructors and destructors: */
		public:
		#if PLAYBACK
		KinectStreamer(const char* frameFileNamePrefix);
		#else
		KinectStreamer(USBContext& usbContext,int cameraIndex,bool highres); // Creates a streamer for the Kinect camera of the given index in the given USB context
		#endif
		~KinectStreamer(void); // Destroys the streamer
		
		/* Methods: */
		GLMotif::PopupWindow* createStreamerDialog(void); // Creates a dialog box to control parameters of this streamer
		void resetFrameTimer(void); // Resets the streamer's frame timer
		void startStreaming(void); // Starts streaming
		void frame(void); // Called once per Vrui frame to update state
		void display(GLContextData& contextData) const; // Renders the streamer's current state into the given OpenGL context
		};
	
	/* Elements: */
	private:
	USBContext usbContext; // USB device context
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
	std::vector<GLMotif::PopupWindow*> streamerDialogs; // List of created streamer settings dialogs
	Vrui::InputDevice* cameraDevice; // Pointer to the device to which the depth camera is attached
	// MD5MeshAnimator anim; // An animator
	
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void showStreamerDialogCallback(Misc::CallbackData* cbData,const size_t& streamerIndex);
	void streamerDialogCloseCallback(GLMotif::PopupWindow::CloseCallbackData* cbData);
	
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
	if(enabled)
		{
		/* Post the new frame into the color frame triple buffer: */
		colorFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void KinectViewer::KinectStreamer::showFacadeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	enabled=cbData->set;
	}

void KinectViewer::KinectStreamer::showFromCameraCallback(Misc::CallbackData* cbData)
	{
	/* Move the camera position to the viewer's head position: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	
	/* Rotate the frame to align the projection direction with the viewer's viewing direction: */
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	
	/* Account for the projector's model space transformation: */
	nav*=Geometry::invert(projectorTransform);
	
	Vrui::setNavigationTransformation(nav);
	}

void KinectViewer::KinectStreamer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	camera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(camera->getRemoveBackground());
	}

void KinectViewer::KinectStreamer::backgroundCaptureCompleteCallback(KinectCamera& camera)
	{
	/* Apply a max depth value, if one is defined: */
	if(maxDepth<1100)
		camera.setMaxDepth(maxDepth);
	}

void KinectViewer::KinectStreamer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	#if !PLAYBACK
	/* Capture five second worth of background frames: */
	camera->captureBackground(150,new Misc::VoidMethodCall<KinectCamera&,KinectViewer::KinectStreamer>(this,&KinectViewer::KinectStreamer::backgroundCaptureCompleteCallback));
	#endif
	}

void KinectViewer::KinectStreamer::saveBackgroundCallback(Misc::CallbackData* cbData)
	{
	#if !PLAYBACK
	/* Save the current background frame: */
	camera->saveBackground("KinectBackground");
	#endif
	}

void KinectViewer::KinectStreamer::backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	#if !PLAYBACK
	/* Update the max depth value: */
	maxDepth=(unsigned short)Math::floor(cbData->value+0.5);
	camera->setMaxDepth(maxDepth,true);
	#endif
	}

void KinectViewer::KinectStreamer::backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	#if !PLAYBACK
	/* Change the camera's background removal fuzz value: */
	camera->setBackgroundRemovalFuzz(int(Math::floor(cbData->value+0.5)));
	#endif
	}

#if PLAYBACK
KinectViewer::KinectStreamer::KinectStreamer(const char* frameFileNamePrefix)
#else
KinectViewer::KinectStreamer::KinectStreamer(USBContext& context,int cameraIndex,bool highres)
#endif
	:camera(0),
	 projector(0),
	 projectorTransform(Vrui::OGTransform::identity),
	 enabled(true),maxDepth(1100)
	{
	#if PLAYBACK
	
	/* Open the depth and color frame files: */
	std::string depthFrameFileName=frameFileNamePrefix;
	depthFrameFileName.append(".depth");
	std::string colorFrameFileName=frameFileNamePrefix;
	colorFrameFileName.append(".color");
	
	/* Create the fake camera: */
	camera=new KinectPlayback(depthFrameFileName.c_str(),colorFrameFileName.c_str());
	
	/* Create a Kinect projector with the proper calibration matrices: */
	projector=new KinectProjector(camera->getDepthMatrix(),camera->getColorMatrix());
	
	/* Get the fake camera's model space transformation: */
	projectorTransform=camera->getProjectorTransform();
	
	#else
	
	/* Attach to and open the Kinect camera: */
	camera=new KinectCamera(context,cameraIndex);
	camera->open();
	
	/* Get the camera's serial number to load the proper calibration matrices: */
	std::string serialNumber=camera->getSerialNumber();
	
	/* Create a Kinect projector with the proper calibration matrices: */
	std::string calibrationFileName="CameraCalibrationMatrices-";
	calibrationFileName.append(serialNumber);
	if(highres)
		calibrationFileName.append("-high");
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
	
	/* Set the camera's frame size: */
	camera->setFrameSize(KinectCamera::COLOR,highres?KinectCamera::FS_1280_1024:KinectCamera::FS_640_480);
	
	#endif
	}

GLMotif::PopupWindow* KinectViewer::KinectStreamer::createStreamerDialog(void)
	{
	const GLMotif::StyleSheet& ss=*Vrui::getWidgetManager()->getStyleSheet();
	
	std::string dialogTitle="Camera Settings";
	#if !PLAYBACK
	dialogTitle.push_back(' ');
	dialogTitle.append(camera->getSerialNumber());
	#endif
	GLMotif::PopupWindow* streamerDialog=new GLMotif::PopupWindow("StreamerDialog",Vrui::getWidgetManager(),dialogTitle.c_str());
	streamerDialog->setCloseButton(true);
	streamerDialog->setResizableFlags(true,false);
	
	GLMotif::RowColumn* streamerSettings=new GLMotif::RowColumn("StreamerSettings",streamerDialog,false);
	streamerSettings->setOrientation(GLMotif::RowColumn::VERTICAL);
	streamerSettings->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	streamerSettings->setNumMinorWidgets(1);
	
	GLMotif::Margin* showMargin=new GLMotif::Margin("ShowMargin",streamerSettings,false);
	showMargin->setAlignment(GLMotif::Alignment::LEFT);
	
	GLMotif::RowColumn* showBox=new GLMotif::RowColumn("ShowBox",showMargin,false);
	showBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	showBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	showBox->setNumMinorWidgets(1);
	
	GLMotif::ToggleButton* showFacadeToggle=new GLMotif::ToggleButton("ShowFacadeToggle",showBox,"Show Facade");
	showFacadeToggle->setBorderWidth(0.0f);
	showFacadeToggle->setBorderType(GLMotif::Widget::PLAIN);
	showFacadeToggle->setToggle(enabled);
	showFacadeToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::showFacadeCallback);
	
	GLMotif::Button* showFromCameraButton=new GLMotif::Button("ShowFromCameraButton",showBox,"Show From Camera");
	showFromCameraButton->getSelectCallbacks().add(this,&KinectViewer::KinectStreamer::showFromCameraCallback);
	
	showBox->manageChild();
	
	showMargin->manageChild();
	
	#if !PLAYBACK
	GLMotif::Margin* backgroundMargin=new GLMotif::Margin("BackgroundMargin",streamerSettings,false);
	backgroundMargin->setAlignment(GLMotif::Alignment::LEFT);
	
	GLMotif::RowColumn* backgroundBox=new GLMotif::RowColumn("BackgroundBox",backgroundMargin,false);
	backgroundBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	backgroundBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	backgroundBox->setNumMinorWidgets(1);
	
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",backgroundBox,"Remove Background");
	removeBackgroundToggle->setBorderWidth(0.0f);
	removeBackgroundToggle->setBorderType(GLMotif::Widget::PLAIN);
	removeBackgroundToggle->setToggle(camera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::removeBackgroundCallback);
	
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",backgroundBox,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&KinectViewer::KinectStreamer::captureBackgroundCallback);
	
	GLMotif::Button* saveBackgroundButton=new GLMotif::Button("SaveBackgroundButton",backgroundBox,"Save Background");
	saveBackgroundButton->getSelectCallbacks().add(this,&KinectViewer::KinectStreamer::saveBackgroundCallback);
	
	backgroundBox->manageChild();
	
	backgroundMargin->manageChild();
	#endif
	
	GLMotif::RowColumn* sliderBox=new GLMotif::RowColumn("SliderBox",streamerSettings,false);
	sliderBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	sliderBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	sliderBox->setNumMinorWidgets(2);
	
	new GLMotif::Label("BackgroundMaxDepthLabel",sliderBox,"Background Depth Limit");
	
	GLMotif::TextFieldSlider* backgroundMaxDepthSlider=new GLMotif::TextFieldSlider("BackgroundMaxDepthSlider",sliderBox,6,ss.fontHeight*10.0f);
	backgroundMaxDepthSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	backgroundMaxDepthSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	backgroundMaxDepthSlider->setValueRange(0,1100,1);
	backgroundMaxDepthSlider->setValue(maxDepth);
	backgroundMaxDepthSlider->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::backgroundMaxDepthCallback);
	
	#if !PLAYBACK
	new GLMotif::Label("BackgroundRemovalFuzzLabel",sliderBox,"Background Removal Fuzz");
	
	GLMotif::TextFieldSlider* backgroundRemovalFuzzSlider=new GLMotif::TextFieldSlider("BackgroundRemovalFuzzSlider",sliderBox,6,ss.fontHeight*10.0f);
	backgroundRemovalFuzzSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	backgroundRemovalFuzzSlider->setValueType(GLMotif::TextFieldSlider::INT);
	backgroundRemovalFuzzSlider->setValueRange(-100,100,1);
	backgroundRemovalFuzzSlider->setValue(camera->getBackgroundRemovalFuzz());
	backgroundRemovalFuzzSlider->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::backgroundRemovalFuzzCallback);
	#endif
	
	sliderBox->manageChild();
	
	streamerSettings->manageChild();
	
	return streamerDialog;
	}

void KinectViewer::KinectStreamer::resetFrameTimer(void)
	{
	#if PLAYBACK
	camera->resetFrameTimer();
	#endif
	}

void KinectViewer::KinectStreamer::startStreaming(void)
	{
	/* Start streaming: */
	camera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer::KinectStreamer>(this,&KinectViewer::KinectStreamer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,KinectViewer::KinectStreamer>(this,&KinectViewer::KinectStreamer::depthStreamingCallback));
	}

KinectViewer::KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	camera->stopStreaming();
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

void KinectViewer::showStreamerDialogCallback(Misc::CallbackData* cbData,const size_t& streamerIndex)
	{
	GLMotif::ToggleButton::ValueChangedCallbackData* myCbData=static_cast<GLMotif::ToggleButton::ValueChangedCallbackData*>(cbData);
	
	if(myCbData->set)
		{
		/* If the dialog doesn't already exist, create it: */
		if(streamerDialogs[streamerIndex]==0)
			{
			streamerDialogs[streamerIndex]=streamers[streamerIndex]->createStreamerDialog();
			streamerDialogs[streamerIndex]->getCloseCallbacks().add(this,&KinectViewer::streamerDialogCloseCallback);
			}
		
		/* Pop it up: */
		Vrui::popupPrimaryWidget(streamerDialogs[streamerIndex]);
		}
	else
		{
		/* If the dialog exists, pop it down and destroy it: */
		if(streamerDialogs[streamerIndex]!=0)
			{
			Vrui::popdownPrimaryWidget(streamerDialogs[streamerIndex]);
			delete streamerDialogs[streamerIndex];
			streamerDialogs[streamerIndex]=0;
			}
		}
	}

void KinectViewer::streamerDialogCloseCallback(GLMotif::PopupWindow::CloseCallbackData* cbData)
	{
	/* Find the dialog in the list: */
	for(std::vector<GLMotif::PopupWindow*>::iterator sdIt=streamerDialogs.begin();sdIt!=streamerDialogs.end();++sdIt)
		if(*sdIt==cbData->popupWindow)
			{
			/* Delete the dialog: */
			delete *sdIt;
			*sdIt=0;
			}
	}

GLMotif::PopupMenu* KinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a toggle button for each Kinect streamer's control dialog: */
	for(size_t i=0;i<streamers.size();++i)
		{
		GLMotif::ToggleButton* showStreamerDialogToggle=new GLMotif::ToggleButton("ShowStreamerDialogToggle",mainMenu,"Show Streamer Dialog");
		showStreamerDialogToggle->setToggle(false);
		showStreamerDialogToggle->getValueChangedCallbacks().add(this,&KinectViewer::showStreamerDialogCallback,i);
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
	/* Disable camera tracking: */
	cameraDevice=0;
	
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Add a streamer for each camera index or frame file name prefix passed on the command line: */
	bool highres=false;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"high")==0)
				highres=true;
			else if(strcasecmp(argv[i]+1,"low")==0)
				highres=false;
			}
		else
			{
			/* Add a streamer for the selected camera: */
			#if PLAYBACK
			streamers.push_back(new KinectStreamer(argv[i]));
			#else
			int cameraIndex=atoi(argv[i]);
			streamers.push_back(new KinectStreamer(usbContext,cameraIndex,highres));
			#endif
			}
		}
	
	/* Initialize the streamer dialog list: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		streamerDialogs.push_back(0);
	
	/* Reset all streamers' frame timers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->resetFrameTimer();
	
	/* Start streaming on all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->startStreaming();
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	}

KinectViewer::~KinectViewer(void)
	{
	delete mainMenu;
	
	/* Delete all streamer dialogs: */
	for(std::vector<GLMotif::PopupWindow*>::iterator sdIt=streamerDialogs.begin();sdIt!=streamerDialogs.end();++sdIt)
		delete *sdIt;
	
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
