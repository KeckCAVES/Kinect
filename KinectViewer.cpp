/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
Copyright (c) 2010-2018 Oliver Kreylos

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
#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/MessageLogger.h>
#include <Cluster/OpenPipe.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLContextData.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Blind.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Button.h>
#include <GLMotif/CascadeButton.h>
#include <Sound/SoundDataFormat.h>
#include <Sound/SoundRecorder.h>
#include <Sound/SoundPlayer.h>
#include <Vrui/Vrui.h>
#include <Vrui/Viewer.h>
#include <Vrui/DisplayState.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Config.h>
#include <Kinect/Internal/Config.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>
#include <Kinect/OpenDirectFrameSource.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/ProjectorHeader.h>
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
	
	{
	Threads::Spinlock::Lock frameSaverLock(frameSaverMutex);
	if(frameSaver!=0)
		{
		/* Forward the color frame to the frame saver: */
		frameSaver->saveColorFrame(frameBuffer);
		}
	}
	}

void KinectViewer::KinectStreamer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Forward depth frame to the projector: */
		projector->setDepthFrame(frameBuffer);
		
		#if KINECT_CONFIG_USE_SHADERPROJECTOR
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
	
	{
	Threads::Spinlock::Lock frameSaverLock(frameSaverMutex);
	if(frameSaver!=0)
		{
		/* Forward the depth frame to the frame saver: */
		frameSaver->saveDepthFrame(frameBuffer);
		}
	}
	}

#if !KINECT_CONFIG_USE_SHADERPROJECTOR

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
	Kinect::DirectFrameSource* dfs=dynamic_cast<Kinect::DirectFrameSource*>(source);
	
	std::string dialogTitle="3D Video Source Settings";
	if(dfs!=0)
		{
		dialogTitle.append(" - Camera serial # ");
		dialogTitle.append(dfs->getSerialNumber());
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
	
	#if KINECT_CONFIG_USE_PROJECTOR2
	
	GLMotif::ToggleButton* mapTextureToggle=new GLMotif::ToggleButton("MapTextureToggle",showBox,"Map Color");
	mapTextureToggle->setBorderWidth(0.0f);
	mapTextureToggle->setBorderType(GLMotif::Widget::PLAIN);
	mapTextureToggle->setToggle(true);
	mapTextureToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::mapTextureCallback);
	
	GLMotif::ToggleButton* illuminateToggle=new GLMotif::ToggleButton("IlluminateToggle",showBox,"Illuminate");
	illuminateToggle->setBorderWidth(0.0f);
	illuminateToggle->setBorderType(GLMotif::Widget::PLAIN);
	illuminateToggle->setToggle(false);
	illuminateToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::illuminateCallback);
	
	#endif
	
	showBox->manageChild();
	
	showMargin->manageChild();
	
	if(dfs!=0)
		{
		/* Embed the camera's settings dialog: */
		dfs->buildSettingsDialog(streamerSettings);
		}
	
	GLMotif::RowColumn* processBox=new GLMotif::RowColumn("ProcessBox",streamerSettings,false);
	processBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	processBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	processBox->setNumMinorWidgets(1);
	
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	
	/* Create a toggle button to enable temporal depth frame filtering: */
	GLMotif::ToggleButton* filterDepthFramesToggle=new GLMotif::ToggleButton("FilterDepthFramesToggle",processBox,"Filter Depth Frames");
	filterDepthFramesToggle->setBorderWidth(0.0f);
	filterDepthFramesToggle->setBorderType(GLMotif::Widget::PLAIN);
	filterDepthFramesToggle->setToggle(projector->getFilterDepthFrames());
	filterDepthFramesToggle->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::filterDepthFramesCallback);
	
	#endif
	
	new GLMotif::Label("TriangleDepthRangeLabel",processBox,"Triangle Depth Range");
	
	GLMotif::TextFieldSlider* triangleDepthRangeSlider=new GLMotif::TextFieldSlider("TriangleDepthRangeSlider",processBox,6,ss.fontHeight*10.0f);
	triangleDepthRangeSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	triangleDepthRangeSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	triangleDepthRangeSlider->setValueRange(0,1100,1);
	triangleDepthRangeSlider->setValue(projector->getTriangleDepthRange());
	triangleDepthRangeSlider->getValueChangedCallbacks().add(this,&KinectViewer::KinectStreamer::triangleDepthRangeCallback);
	
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	processBox->setColumnWeight(2,1.0);
	#else
	processBox->setColumnWeight(1,1.0);
	#endif
	processBox->manageChild();
	
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

#if KINECT_CONFIG_USE_PROJECTOR2

void KinectViewer::KinectStreamer::mapTextureCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the projector's texture mapping flag: */
	projector->setMapTexture(cbData->set);
	}

void KinectViewer::KinectStreamer::illuminateCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the projector's texture mapping flag: */
	projector->setIlluminate(cbData->set);
	}

#endif

#if !KINECT_CONFIG_USE_SHADERPROJECTOR

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
	 projector(new Kinect::ProjectorType(*source)),
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
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
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

void KinectViewer::KinectStreamer::startStreaming(const Kinect::FrameSource::Time& timeBase)
	{
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	
	/* Hook this streamer into the projector's mesh callback: */
	projector->startStreaming(Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::meshStreamingCallback));
	
	#endif
	
	/* Hook this streamer into the frame source and start streaming: */
	source->setTimeBase(timeBase);
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectViewer::KinectStreamer::depthStreamingCallback));
	}

void KinectViewer::KinectStreamer::setFrameSaver(Kinect::FrameSaver* newFrameSaver)
	{
	Threads::Spinlock::Lock frameSaverLock(frameSaverMutex);
	
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
	
	/* Create a button to align all Kinects' facades in a vertical column: */
	GLMotif::ToggleButton* alignProjectorsToggle=new GLMotif::ToggleButton("AlignProjectorsToggle",mainMenu,"Align Projectors");
	alignProjectorsToggle->setToggle(false);
	alignProjectorsToggle->getValueChangedCallbacks().add(this,&KinectViewer::alignProjectorsCallback);
	
	/* Create a toggle button to write all video streams to files: */
	saveStreamsToggle=new GLMotif::ToggleButton("SaveStreamsToggle",mainMenu,"Save Streams...");
	saveStreamsToggle->setToggle(false);
	saveStreamsToggle->getValueChangedCallbacks().add(this,&KinectViewer::saveStreamsCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

void KinectViewer::goToPhysicalCallback(Misc::CallbackData* cbData)
	{
	/* Set the navigation transformation to identity: */
	Vrui::setNavigationTransformation(Vrui::NavTransform::identity);
	}

void KinectViewer::alignProjectorsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	if(cbData->set)
		{
		/* Save all projectors' extrinsic parameters and line them up in a column: */
		typedef Kinect::FrameSource::ExtrinsicParameters EP;
		EP extrinsics=EP::translate(EP::Vector(0,EP::Scalar(streamers.size()-1)*Math::div2(EP::Scalar(200)),0));
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			{
			(*sIt)->savedExtrinsics=(*sIt)->projector->getProjectorTransform();
			(*sIt)->projector->setExtrinsicParameters(extrinsics);
			extrinsics.leftMultiply(EP::translate(EP::Vector(0,-200,0)));
			}
		}
	else
		{
		/* Restore all projectors' extrinsic parameters: */
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			(*sIt)->projector->setExtrinsicParameters((*sIt)->savedExtrinsics);
		}
	}

void KinectViewer::saveStreamsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	if(cbData->set)
		{
		if(saveStreamsFileSelectionDialog==0)
			{
			try
				{
				/* Create a file selection dialog to select a file name: */
				Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Streams...",saveStreamsDirectory,"SavedStreams",".color;.depth"));
				saveDialog->getOKCallbacks().add(this,&KinectViewer::saveStreamsOKCallback);
				saveDialog->getCancelCallbacks().add(this,&KinectViewer::saveStreamsCancelCallback);
				
				/* Show the file selection dialog: */
				Vrui::popupPrimaryWidget(saveDialog.getTarget());
				
				/* Remember the file selection dialog: */
				saveStreamsFileSelectionDialog=saveDialog.releaseTarget();
				}
			catch(std::runtime_error err)
				{
				/* Show an error message: */
				Misc::formattedUserError("Save Streams...: Could not select file name due to exception %s",err.what());
				}
			}
		}
	else
		{
		if(saveStreamsFileSelectionDialog!=0)
			{
			/* Close the file selection dialog: */
			saveStreamsFileSelectionDialog->close();
			saveStreamsFileSelectionDialog=0;
			}
		else
			{
			/* Stop saving 3D video streams: */
			for(size_t i=0;i<streamers.size();++i)
				streamers[i]->setFrameSaver(0);
			
			/* Stop recording audio: */
			soundRecorder->stop();
			delete soundRecorder;
			soundRecorder=0;
			}
		}
	}

void KinectViewer::saveStreamsOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	/* Establish a time base for the frame saver: */
	Kinect::FrameSource::Time now;
	
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
		frameSaver->setTimeStampOffset(double(now-timeBase));
		streamers[i]->setFrameSaver(frameSaver);
		}
	
	/* Start recording sound: */
	soundRecorder->start();
	
	/* Close the file selection dialog: */
	saveStreamsFileSelectionDialog->close();
	saveStreamsFileSelectionDialog=0;
	}

void KinectViewer::saveStreamsCancelCallback(GLMotif::FileSelectionDialog::CancelCallbackData* cbData)
	{
	/* Turn the "Save Streams..." toggle button off: */
	saveStreamsToggle->setToggle(false);
	
	/* Close the file selection dialog: */
	saveStreamsFileSelectionDialog->close();
	saveStreamsFileSelectionDialog=0;
	}

KinectViewer::KinectViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 saveStreamsToggle(0),saveStreamsDirectory(Vrui::openDirectory(".")),saveStreamsFileSelectionDialog(0),
	 soundRecorder(0),soundPlayer(0),
	 mainMenu(0)
	{
	/* Add a streamer for each camera index or frame file name prefix passed on the command line: */
	bool printHelp=false;
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
					/* Open the 3D camera of the given index: */
					int cameraIndex=atoi(argv[i]);
					Kinect::DirectFrameSource* camera=Kinect::openDirectFrameSource(cameraIndex);
					std::cout<<"KinectViewer: Connected to 3D camera with serial number "<<camera->getSerialNumber()<<std::endl;
					
					/* Check if it's a first-generation Kinect to apply type-specific settings: */
					Kinect::Camera* kinectV1=dynamic_cast<Kinect::Camera*>(camera);
					if(kinectV1!=0)
						{
						/* Set the color camera's frame size: */
						kinectV1->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
						
						/* Set depth frame compression: */
						kinectV1->setCompressDepthFrames(compressDepth);
						}
					
					/* Enable background removal if the camera has a default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a new streamer for the camera: */
					streamers.push_back(new KinectStreamer(this,camera));
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"Ignoring -c "<<argv[i]<<" command line argument: Streaming from local 3D camera(s) is not supported in cluster environments"<<std::endl;
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
			else
				{
				std::cerr<<"Ignoring unrecognized command line parameter "<<argv[i]<<std::endl;
				printHelp=true;
				}
			}
		else
			{
			std::cerr<<"Ignoring unrecognized command line argument "<<argv[i]<<std::endl;
			printHelp=true;
			}
		}
	
	if(printHelp)
		{
		std::cout<<"Usage: KinectViewer [option 1] ... [option n]"<<std::endl;
		std::cout<<"  Options:"<<std::endl;
		std::cout<<"  -h"<<std::endl;
		std::cout<<"     Prints this help message"<<std::endl;
		std::cout<<"  -high"<<std::endl;
		std::cout<<"     Sets color frame size for all subsequent first-generation Kinect cameras to 1280x1024 @ 15Hz"<<std::endl;
		std::cout<<"  -low"<<std::endl;
		std::cout<<"     Sets color frame size for all subsequent first-generation Kinect cameras to 640x480 @ 30Hz"<<std::endl;
		std::cout<<"  -compress"<<std::endl;
		std::cout<<"     Requests compressed depth frames from all subsequent first-generation Kinect cameras"<<std::endl;
		std::cout<<"  -nocompress"<<std::endl;
		std::cout<<"     Requests uncompressed depth frames from all subsequent first-generation Kinect cameras"<<std::endl;
		std::cout<<"  -c <camera index>"<<std::endl;
		std::cout<<"     Connects to the local 3D camera of the given index (0: first camera on USB bus)"<<std::endl;
		std::cout<<"  -f <stream file base name>"<<std::endl;
		std::cout<<"     Opens a previously recorded pair of color and depth stream files for playback"<<std::endl;
		std::cout<<"  -s <sound file name>"<<std::endl;
		std::cout<<"     Opens a previously recorded sound file for playback"<<std::endl;
		std::cout<<"  -p <host name of 3D video stream server> <port number of 3D video stream server>"<<std::endl;
		std::cout<<"     Connects to a 3D video streaming server identified by host name and port number"<<std::endl;
		}
	
	if(streamers.empty())
		{
		std::cerr<<"No 3D video sources requested; exiting"<<std::endl;
		Vrui::shutdown();
		return;
		}
	
	/* Get a common time base for all streamers: */
	timeBase.set();
	
	/* Start playing back a sound file if requested: */
	if(soundPlayer!=0)
		soundPlayer->start();
	
	/* Start streaming on all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->startStreaming(timeBase);
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Initialize the custom tool classes: */
	SphereExtractorTool::initClass();
	}

KinectViewer::~KinectViewer(void)
	{
	delete mainMenu;
	delete saveStreamsFileSelectionDialog;
	
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

void KinectViewer::resetNavigation(void)
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

VRUI_APPLICATION_RUN(KinectViewer)
