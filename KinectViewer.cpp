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

#include <string.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <Threads/TripleBuffer.h>
#include <USB/Context.h>
#include <Cluster/OpenPipe.h>
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
#include <Vrui/OpenFile.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/Projector.h>
// #include "MD5MeshAnimator.h"

class KinectViewer:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	class KinectStreamer // Helper class to stream 3D video data from a 3D video frame source to a Kinect projector
		{
		/* Elements: */
		public:
		Kinect::FrameSource* source; // Pointer to the 3D video frame source
		Threads::TripleBuffer<Kinect::FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
		Threads::TripleBuffer<Kinect::FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
		Kinect::Projector* projector; // Pointer to the projector
		bool enabled; // Flag whether the streamer is currently processing and rendering 3D video frames
		unsigned short maxDepth; // Maximum depth value for background removal
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
		void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
		void showFacadeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void showFromCameraCallback(Misc::CallbackData* cbData);
		void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void backgroundCaptureCompleteCallback(Kinect::Camera& camera);
		void captureBackgroundCallback(Misc::CallbackData* cbData);
		void saveBackgroundCallback(Misc::CallbackData* cbData);
		void backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
		void backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
		
		/* Constructors and destructors: */
		public:
		KinectStreamer(Kinect::FrameSource* sSource); // Creates a streamer for the given 3D video source
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
	USB::Context usbContext; // USB device context
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
	std::vector<GLMotif::PopupWindow*> streamerDialogs; // List of created streamer settings dialogs
	// Vrui::InputDevice* cameraDevice; // Pointer to the device to which the depth camera is attached
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

void KinectViewer::KinectStreamer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Post the new frame into the color frame triple buffer: */
		colorFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void KinectViewer::KinectStreamer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Post the new frame into the depth frame triple buffer: */
		depthFrames.postNewValue(frameBuffer);
		
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
	nav*=Geometry::invert(projector->getProjectorTransform());
	
	Vrui::setNavigationTransformation(nav);
	}

void KinectViewer::KinectStreamer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Set the background removal flag: */
		camera->setRemoveBackground(cbData->set);
		
		/* Set the toggle button's state to the actual new flag value: */
		cbData->toggle->setToggle(camera->getRemoveBackground());
		}
	else
		{
		/* Reset the toggle button's state: */
		cbData->toggle->setToggle(false);
		}
	}

void KinectViewer::KinectStreamer::backgroundCaptureCompleteCallback(Kinect::Camera& camera)
	{
	/* Apply a max depth value, if one is defined: */
	if(maxDepth<1100)
		camera.setMaxDepth(maxDepth);
	}

void KinectViewer::KinectStreamer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Capture five second worth of background frames: */
		camera->captureBackground(150,Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::backgroundCaptureCompleteCallback));
		}
	}

void KinectViewer::KinectStreamer::saveBackgroundCallback(Misc::CallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Save the current background frame: */
		camera->saveBackground("KinectBackground");
		}
	}

void KinectViewer::KinectStreamer::backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Update the max depth value: */
		maxDepth=(unsigned short)Math::floor(cbData->value+0.5);
		camera->setMaxDepth(maxDepth,true);
		}
	}

void KinectViewer::KinectStreamer::backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Change the camera's background removal fuzz value: */
		camera->setBackgroundRemovalFuzz(int(Math::floor(cbData->value+0.5)));
		}
	}

KinectViewer::KinectStreamer::KinectStreamer(Kinect::FrameSource* sSource)
	:source(sSource),
	 projector(new Kinect::Projector(*source)),
	 enabled(true),maxDepth(1100)
	{
	}

KinectViewer::KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	source->stopStreaming();
	delete projector;
	
	/* Close and disconnect from the 3D video source: */
	delete source;
	}

GLMotif::PopupWindow* KinectViewer::KinectStreamer::createStreamerDialog(void)
	{
	const GLMotif::StyleSheet& ss=*Vrui::getWidgetManager()->getStyleSheet();
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	
	std::string dialogTitle="3D Video Source Settings";
	if(camera!=0)
		{
		dialogTitle.push_back(' ');
		dialogTitle.append(camera->getSerialNumber());
		}
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
	
	if(camera!=0)
		{
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
		
		new GLMotif::Label("BackgroundRemovalFuzzLabel",sliderBox,"Background Removal Fuzz");
		
		GLMotif::TextFieldSlider* backgroundRemovalFuzzSlider=new GLMotif::TextFieldSlider("BackgroundRemovalFuzzSlider",sliderBox,6,ss.fontHeight*10.0f);
		backgroundRemovalFuzzSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
		backgroundRemovalFuzzSlider->setValueType(GLMotif::TextFieldSlider::INT);
		backgroundRemovalFuzzSlider->setValueRange(-100,100,1);
		backgroundRemovalFuzzSlider->setValue(camera->getBackgroundRemovalFuzz());
		backgroundRemovalFuzzSlider->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::backgroundRemovalFuzzCallback);
		
		sliderBox->manageChild();
		}
	
	streamerSettings->manageChild();
	
	return streamerDialog;
	}

void KinectViewer::KinectStreamer::resetFrameTimer(void)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		camera->resetFrameTimer();
	}

void KinectViewer::KinectStreamer::startStreaming(void)
	{
	/* Start streaming: */
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::depthStreamingCallback));
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
		/* Draw the current 3D video frame: */
		projector->draw(contextData);
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
			Vrui::getWidgetManager()->deleteWidget(streamerDialogs[streamerIndex]);
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
			/* Close the dialog: */
			Vrui::getWidgetManager()->deleteWidget(*sdIt);
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
	 mainMenu(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Add a streamer for each camera index or frame file name prefix passed on the command line: */
	bool highres=false;
	bool compressDepth=false;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"high")==0)
				highres=true;
			else if(strcasecmp(argv[i]+1,"low")==0)
				highres=false;
			else if(strcasecmp(argv[i]+1,"compress")==0)
				compressDepth=true;
			else if(strcasecmp(argv[i]+1,"nocompress")==0)
				compressDepth=false;
			else if(strcasecmp(argv[i]+1,"c")==0)
				{
				++i;
				
				/* Open the camera of the given index: */
				int cameraIndex=atoi(argv[i]);
				Kinect::Camera* camera=new Kinect::Camera(usbContext,cameraIndex);
				
				/* Set the camera's frame size and compression flag: */
				camera->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
				camera->setCompressDepthFrames(compressDepth);
				
				/* Add a new streamer for the camera: */
				streamers.push_back(new KinectStreamer(camera));
				}
			else if(strcasecmp(argv[i]+1,"f")==0)
				{
				++i;
				
				/* Open a frame source for the color and depth files of the given name prefix: */
				std::string colorFileName=argv[i];
				colorFileName.append(".color");
				std::string depthFileName=argv[i];
				depthFileName.append(".depth");
				Kinect::FileFrameSource* fileSource=new Kinect::FileFrameSource(Vrui::openFile(colorFileName.c_str()),Vrui::openFile(depthFileName.c_str()));
				
				/* Add a new streamer for the file source: */
				streamers.push_back(new KinectStreamer(fileSource));
				}
			else if(strcasecmp(argv[i]+1,"p")==0)
				{
				i+=2;
				
				/* Open a multiplexed frame source for the given server host name and port number: */
				Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),argv[i-1],atoi(argv[i])));
				
				/* Add a new streamer for each component stream in the multiplexer: */
				for(unsigned int i=0;i<source->getNumStreams();++i)
					streamers.push_back(new KinectStreamer(source->getStream(i)));
				}
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
	#if 0
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
	#endif
	
	/* Process all streamers: */
	for(std::vector<KinectStreamer*>::const_iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->display(contextData);
	
	#if 0
	if(cameraDevice!=0)
		{
		/* Return to navigational space: */
		glPopMatrix();
		}
	#endif
	
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
