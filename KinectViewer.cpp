/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
Copyright (c) 2010-2015 Oliver Kreylos

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

#include "KinectViewer.h"

#include <string.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <USB/Context.h>
#include <Cluster/OpenPipe.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>
#include <Geometry/OrthogonalTransformation.h>
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
#include <GLMotif/Blind.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Button.h>
#include <GLMotif/CascadeButton.h>
#include <Sound/SoundDataFormat.h>
#include <Sound/SoundRecorder.h>
#include <Sound/SoundPlayer.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/OpenFile.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#if KINECT_USE_SHADERPROJECTOR
#include <Kinect/ShaderProjector.h>
#else
#include <Kinect/Projector.h>
#endif
#include <Kinect/FrameSaver.h>

#include "SphereExtractorTool.h"

// #include "MD5MeshAnimator.h"

/*********************************************
Methods of class KinectViewer::KinectStreamer:
*********************************************/

void KinectViewer::KinectStreamer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Forward color frame to the projector: */
		projector->setColorFrame(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		
		{
		Threads::Mutex::Lock sphereExtractorLock(sphereExtractorMutex);
		if(sphereExtractor!=0)
			{
			/* Forward color frame to the sphere extractor: */
			sphereExtractor->setColorFrame(frameBuffer);
			}
		}
		}
	
	if(frameSaver!=0)
		{
		/* Forward the color frame to the frame saver: */
		frameSaver->saveColorFrame(frameBuffer);
		}
	}

void KinectViewer::KinectStreamer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Forward depth frame to the projector: */
		projector->setDepthFrame(frameBuffer);
		
		#if KINECT_USE_SHADERPROJECTOR
		/* Update application state: */
		Vrui::requestUpdate();
		#endif
		
		{
		Threads::Mutex::Lock sphereExtractorLock(sphereExtractorMutex);
		if(sphereExtractor!=0)
			{
			/* Forward depth frame to the sphere extractor: */
			sphereExtractor->setDepthFrame(frameBuffer);
			}
		}
		}
	
	if(frameSaver!=0)
		{
		/* Forward the depth frame to the frame saver: */
		frameSaver->saveDepthFrame(frameBuffer);
		}
	}

#if !KINECT_USE_SHADERPROJECTOR

void KinectViewer::KinectStreamer::meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer)
	{
	if(enabled)
		{
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

#endif

void KinectViewer::KinectStreamer::showStreamerDialogCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	if(cbData->set)
		{
		/* If the dialog doesn't already exist, create it: */
		if(streamerDialog==0)
			{
			streamerDialog=createStreamerDialog();
			streamerDialog->getCloseCallbacks().add(this,&KinectViewer::KinectStreamer::streamerDialogCloseCallback);
			}
		
		/* Pop it up: */
		Vrui::popupPrimaryWidget(streamerDialog);
		}
	else
		{
		/* Pop down and destroy the streamer dialog: */
		if(streamerDialog!=0)
			streamerDialog->close();
		streamerDialog=0;
		}
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
	
	GLMotif::RowColumn* processBox=new GLMotif::RowColumn("ProcessBox",streamerSettings,false);
	processBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	processBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	processBox->setNumMinorWidgets(2);
	
	#if !KINECT_USE_SHADERPROJECTOR
	
	/* Create a toggle button to enable temporal depth frame filtering: */
	GLMotif::ToggleButton* filterDepthFramesToggle=new GLMotif::ToggleButton("FilterDepthFramesToggle",processBox,"Filter Depth Frames");
	filterDepthFramesToggle->setBorderWidth(0.0f);
	filterDepthFramesToggle->setBorderType(GLMotif::Widget::PLAIN);
	filterDepthFramesToggle->setToggle(projector->getFilterDepthFrames());
	filterDepthFramesToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::filterDepthFramesCallback);
	
	new GLMotif::Blind("ProcessBlind",processBox);
	
	#endif
	
	new GLMotif::Label("TriangleDepthRange",processBox,"Triangle Depth Range");
	
	GLMotif::TextFieldSlider* triangleDepthRangeSlider=new GLMotif::TextFieldSlider("TriangleDepthRangeSlider",processBox,6,ss.fontHeight*10.0f);
	triangleDepthRangeSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	triangleDepthRangeSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	triangleDepthRangeSlider->setValueRange(0,1100,1);
	triangleDepthRangeSlider->setValue(projector->getTriangleDepthRange());
	triangleDepthRangeSlider->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::triangleDepthRangeCallback);
	
	processBox->manageChild();
	
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
		
		captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",backgroundBox,"Capture Background");
		captureBackgroundButton->getSelectCallbacks().add(this,&KinectViewer::KinectStreamer::captureBackgroundCallback);
		
		GLMotif::Button* loadBackgroundButton=new GLMotif::Button("LoadBackgroundButton",backgroundBox,"Load Background...");
		application->backgroundSelectionHelper.addLoadCallback(loadBackgroundButton,this,&KinectViewer::KinectStreamer::loadBackgroundOKCallback);
		
		GLMotif::Button* saveBackgroundButton=new GLMotif::Button("SaveBackgroundButton",backgroundBox,"Save Background...");
		application->backgroundSelectionHelper.addSaveCallback(saveBackgroundButton,this,&KinectViewer::KinectStreamer::saveBackgroundOKCallback);
		
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

void KinectViewer::KinectStreamer::showFacadeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	enabled=cbData->set;
	}

void KinectViewer::KinectStreamer::showFromCameraCallback(Misc::CallbackData* cbData)
	{
	#if !KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
	
	/* Move the camera position to the viewer's head position: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	
	/* Rotate the frame to align the projection direction with the viewer's viewing direction: */
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	
	/* Account for the projector's model space transformation: */
	nav*=Geometry::invert(projector->getProjectorTransform());
	
	Vrui::setNavigationTransformation(nav);
	
	#endif
	}

#if !KINECT_USE_SHADERPROJECTOR

void KinectViewer::KinectStreamer::filterDepthFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the projector's depth frame filtering flag: */
	projector->setFilterDepthFrames(cbData->set,false);
	}

#endif

void KinectViewer::KinectStreamer::triangleDepthRangeCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set the projector's triangle depth range: */
	projector->setTriangleDepthRange(DepthPixel(Math::floor(cbData->value+0.5)));
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
	/* Enable the capture button to indicate that capture is complete: */
	// captureBackgroundButton->setEnabled(true);
	
	/* Apply a max depth value, if one is defined: */
	if(maxDepth<1100)
		camera.setMaxDepth(maxDepth);
	}

void KinectViewer::KinectStreamer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Disable the capture button to indicate that a capture is in progress: */
		// captureBackgroundButton->setEnabled(false);
		
		/* Capture five seconds worth of background frames: */
		camera->captureBackground(150,Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::backgroundCaptureCompleteCallback));
		}
	}

void KinectViewer::KinectStreamer::loadBackgroundOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Load the selected background file: */
		IO::FilePtr backgroundFile=cbData->selectedDirectory->openFile(cbData->selectedFileName);
		backgroundFile->setEndianness(Misc::LittleEndian);
		camera->loadBackground(*backgroundFile);
		}
	}

void KinectViewer::KinectStreamer::saveBackgroundOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		try
			{
			/* Save the current background frame: */
			camera->saveBackground(cbData->getSelectedPath().c_str());
			}
		catch(std::runtime_error err)
			{
			Vrui::showErrorMessage("Save Background",Misc::printStdErrMsg("Could not save background due to exception %s",err.what()));
			}
		}
	}

void KinectViewer::KinectStreamer::backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		{
		/* Update the max depth value: */
		maxDepth=DepthPixel(Math::floor(cbData->value+0.5));
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

void KinectViewer::KinectStreamer::streamerDialogCloseCallback(Misc::CallbackData* cbData)
	{
	/* Unselect the toggle button: */
	showStreamerDialogToggle->setToggle(false);
	
	/* Pop down and destroy the streamer dialog: */
	streamerDialog->close();
	streamerDialog=0;
	}

KinectViewer::KinectStreamer::KinectStreamer(KinectViewer* sApplication,Kinect::FrameSource* sSource)
	:application(sApplication),
	 source(sSource),
	 #if KINECT_USE_SHADERPROJECTOR
	 projector(new Kinect::ShaderProjector(*source)),
	 #else
	 projector(new Kinect::Projector(*source)),
	 #endif
	 frameSaver(0),
	 enabled(true),maxDepth(1100),
	 sphereExtractor(0),
	 streamerDialog(0),showStreamerDialogToggle(0)
	{
	}

KinectViewer::KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	source->stopStreaming();
	#if !KINECT_USE_SHADERPROJECTOR
	projector->stopStreaming();
	#endif
	{
	Threads::Mutex::Lock sphereExtractorLock(sphereExtractorMutex);
	if(sphereExtractor!=0)
		sphereExtractor->stopStreaming();
	delete sphereExtractor;
	sphereExtractor=0;
	}
	delete projector;
	delete frameSaver;
	
	/* Close and disconnect from the 3D video source: */
	delete source;
	
	/* Close the streamer dialog: */
	delete streamerDialog;
	}

void KinectViewer::KinectStreamer::setShowStreamerDialogToggle(GLMotif::ToggleButton* newShowStreamerDialogToggle)
	{
	showStreamerDialogToggle=newShowStreamerDialogToggle;
	showStreamerDialogToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::showStreamerDialogCallback);
	}

void KinectViewer::KinectStreamer::resetFrameTimer(void)
	{
	Kinect::Camera* camera=dynamic_cast<Kinect::Camera*>(source);
	if(camera!=0)
		camera->resetFrameTimer();
	Kinect::FileFrameSource* file=dynamic_cast<Kinect::FileFrameSource*>(source);
	if(file!=0)
		file->resetFrameTimer();
	}

void KinectViewer::KinectStreamer::startStreaming(void)
	{
	#if !KINECT_USE_SHADERPROJECTOR
	
	/* Hook this streamer into the projector's mesh callback: */
	projector->startStreaming(Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::meshStreamingCallback));
	
	#endif
	
	/* Hook this streamer into the frame source and start streaming: */
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::depthStreamingCallback));
	}

void KinectViewer::KinectStreamer::setFrameSaver(Kinect::FrameSaver* newFrameSaver)
	{
	/* Destroy the current frame saver: */
	delete frameSaver;
	
	/* Store the new frame saver: */
	frameSaver=newFrameSaver;
	}

void KinectViewer::KinectStreamer::frame(void)
	{
	if(enabled)
		{
		/* Update the projector: */
		projector->updateFrames();
		}
	}

void KinectViewer::KinectStreamer::display(GLContextData& contextData) const
	{
	if(enabled)
		{
		/* Draw the current 3D video frame: */
		projector->glRenderAction(contextData);
		}
	}

/*****************************
Methods of class KinectViewer:
*****************************/

GLMotif::PopupMenu* KinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to reset the navigation transformation: */
	GLMotif::Button* resetNavigationButton=new GLMotif::Button("ResetNavigationButton",mainMenu,"Reset Navigation");
	resetNavigationButton->getSelectCallbacks().add(this,&KinectViewer::resetNavigationCallback);
	
	/* Create a button to go to physical coordinates: */
	GLMotif::Button* goToPhysicalButton=new GLMotif::Button("GoToPhysicalButton",mainMenu,"Go To Physical Space");
	goToPhysicalButton->getSelectCallbacks().add(this,&KinectViewer::goToPhysicalCallback);
	
	/* Create a toggle button for each Kinect streamer's control dialog: */
	for(size_t i=0;i<streamers.size();++i)
		{
		GLMotif::ToggleButton* showStreamerDialogToggle=new GLMotif::ToggleButton("ShowStreamerDialogToggle",mainMenu,"Show Streamer Dialog");
		showStreamerDialogToggle->setToggle(false);
		streamers[i]->setShowStreamerDialogToggle(showStreamerDialogToggle);
		}
	
	/* Create a button to start writing all video streams to files: */
	GLMotif::ToggleButton* saveStreamsButton=new GLMotif::ToggleButton("SaveStreamsButton",mainMenu,"Save Streams...");
	saveStreamsButton->setToggle(false);
	streamSelectionHelper.addSaveCallback(saveStreamsButton,this,&KinectViewer::saveStreamsOKCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

void KinectViewer::resetNavigationCallback(Misc::CallbackData* cbData)
	{
	/* Calculate a bounding box around all projectors' points of interests: */
	Geometry::Box<Vrui::Scalar,3> bbox=Geometry::Box<Vrui::Scalar,3>::empty;
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		{
		/* Calculate the world position of a point 1m in front of the streamer's projector: */
		Kinect::FrameSource::ExtrinsicParameters::Point cp(0.0,0.0,-100.0); // 1m along projection line
		Vrui::Point wp((*sIt)->projector->getProjectorTransform().transform(cp));
		
		#if KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
		
		Vrui::Vector ws(200.0,200.0,200.0);
		
		#else
		
		/* Calculate the projector's influence radius: */
		Vrui::Scalar s((*sIt)->projector->getProjectorTransform().getScaling()*100.0);
		Vrui::Vector ws(s,s,s);
		
		#endif
		/* Add the projector's influence to the bounding box: */
		bbox.addPoint(wp-ws);
		bbox.addPoint(wp+ws);
		}
	
	/* Center the resulting box in the view: */
	Vrui::setNavigationTransformation(Geometry::mid(bbox.min,bbox.max),Math::div2(Geometry::dist(bbox.min,bbox.max)));
	}

void KinectViewer::goToPhysicalCallback(Misc::CallbackData* cbData)
	{
	/* Set the navigation transformation to identity: */
	Vrui::setNavigationTransformation(Vrui::NavTransform::identity);
	}

void KinectViewer::saveStreamsOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	/* Create a sound recorder with default settings: */
	Sound::SoundDataFormat sdf;
	sdf.setStandardSampleFormat(16,true,Sound::SoundDataFormat::LittleEndian);
	sdf.samplesPerFrame=1;
	sdf.framesPerSecond=16000;
	std::string soundFileName=cbData->getSelectedPath();
	soundFileName.append(".wav");
	soundRecorder=new Sound::SoundRecorder(sdf,soundFileName.c_str());
	
	/* Start saving streams on all streamers: */
	for(size_t i=0;i<streamers.size();++i)
		{
		/* Create data file names for this streamer: */
		std::string colorFileName=cbData->getSelectedPath();
		colorFileName.push_back('-');
		colorFileName.append(Misc::ValueCoder<unsigned int>::encode((unsigned int)i+1));
		colorFileName.append(".color");
		std::string depthFileName=cbData->getSelectedPath();
		depthFileName.push_back('-');
		depthFileName.append(Misc::ValueCoder<unsigned int>::encode((unsigned int)i+1));
		depthFileName.append(".depth");
		
		/* Attach a frame saver to the streamer: */
		Kinect::FrameSaver* frameSaver=new Kinect::FrameSaver(streamers[i]->getFrameSource(),colorFileName.c_str(),depthFileName.c_str());
		frameSaver->setTimeStampOffset(Vrui::getApplicationTime());
		streamers[i]->setFrameSaver(frameSaver);
		}
	
	/* Start recording sound: */
	soundRecorder->start();
	}

KinectViewer::KinectViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 backgroundSelectionHelper(KINECT_CAMERA_DEFAULTBACKGROUNDFILENAMEPREFIX,".background",Vrui::openDirectory(KINECT_CONFIG_DIR)),
	 streamSelectionHelper("SavedStreams",".color;.depth",Vrui::openDirectory(".")),
	 soundRecorder(0),soundPlayer(0),
	 mainMenu(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Add a streamer for each camera index or frame file name prefix passed on the command line: */
	bool printHelp=argc==1;
	bool highres=false;
	bool compressDepth=false;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"h")==0)
				printHelp=true;
			else if(strcasecmp(argv[i]+1,"high")==0)
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
				
				if(Vrui::getClusterMultiplexer()==0)
					{
					/* Open the camera of the given index: */
					int cameraIndex=atoi(argv[i]);
					Kinect::Camera* camera=new Kinect::Camera(usbContext,cameraIndex);
					
					/* Set the camera's frame size and compression flag: */
					camera->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
					camera->setCompressDepthFrames(compressDepth);
					
					/* Enable background removal if the camera has a default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a new streamer for the camera: */
					streamers.push_back(new KinectStreamer(this,camera));
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"Ignoring -c "<<argv[i]<<" command line argument: Streaming from local Kinect camera(s) is not supported in cluster environments"<<std::endl;
					}
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
				streamers.push_back(new KinectStreamer(this,fileSource));
				}
			else if(strcasecmp(argv[i]+1,"s")==0)
				{
				++i;
				
				/* Open a sound player: */
				try
					{
					soundPlayer=new Sound::SoundPlayer(argv[i]);
					}
				catch(std::runtime_error err)
					{
					std::cerr<<"Could not open sound file "<<argv[i]<<" due to exception "<<err.what()<<std::endl;
					}
				}
			else if(strcasecmp(argv[i]+1,"p")==0)
				{
				i+=2;
				
				/* Open a multiplexed frame source for the given server host name and port number: */
				Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),argv[i-1],atoi(argv[i])));
				
				/* Add a new streamer for each component stream in the multiplexer: */
				for(unsigned int i=0;i<source->getNumStreams();++i)
					streamers.push_back(new KinectStreamer(this,source->getStream(i)));
				}
			}
		}
	
	if(printHelp)
		{
		std::cout<<"Usage: KinectViewer [option 1] ... [option n]"<<std::endl;
		std::cout<<"  Options:"<<std::endl;
		std::cout<<"  -h"<<std::endl;
		std::cout<<"     Prints this help message"<<std::endl;
		std::cout<<"  -high"<<std::endl;
		std::cout<<"     Sets color frame size for all subsequent Kinect cameras to 1280x1024 @ 15Hz"<<std::endl;
		std::cout<<"  -low"<<std::endl;
		std::cout<<"     Sets color frame size for all subsequent Kinect cameras to 640x480 @ 30Hz"<<std::endl;
		std::cout<<"  -compress"<<std::endl;
		std::cout<<"     Requests compressed depth frames from all subsequent Kinect cameras"<<std::endl;
		std::cout<<"  -nocompress"<<std::endl;
		std::cout<<"     Requests uncompressed depth frames from all subsequent Kinect cameras"<<std::endl;
		std::cout<<"  -c <camera index>"<<std::endl;
		std::cout<<"     Connects to the local Kinect camera of the given index (0: first camera on USB bus)"<<std::endl;
		std::cout<<"  -f <stream file base name>"<<std::endl;
		std::cout<<"     Opens a previously recorded pair of color and depth stream files for playback"<<std::endl;
		std::cout<<"  -s <sound file name>"<<std::endl;
		std::cout<<"     Opens a previously recorded sound file for playback"<<std::endl;
		std::cout<<"  -p <host name of 3D video stream server> <port number of 3D video stream server>"<<std::endl;
		std::cout<<"     Connects to a 3D video streaming server identified by host name and port number"<<std::endl;
		}
	
	if(streamers.empty())
		{
		Vrui::shutdown();
		return;
		}
	
	/* Reset all streamers' frame timers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->resetFrameTimer();
	
	/* Start streaming on all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->startStreaming();
	
	if(soundPlayer!=0)
		soundPlayer->start();
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Initialize the custom tool classes: */
	SphereExtractorTool::initClass();
	
	/* Initialize the navigation transformation: */
	resetNavigationCallback(0);
	}

KinectViewer::~KinectViewer(void)
	{
	delete mainMenu;
	
	/* Delete the sound recorder if it is still active: */
	delete soundRecorder;
	
	/* Delete the sound player: */
	delete soundPlayer;
	
	/* Delete all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		delete *sIt;
	streamers.clear();
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
		KinectViewer app(argc,argv);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
