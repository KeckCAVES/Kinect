/***********************************************************************
KinectRecorder - Vislet to capture and save 3D video from one or more
Kinect devices.
Copyright (c) 2011-2013 Oliver Kreylos

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

#include "Vislets/KinectRecorder.h"

#include <string.h>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/CompoundValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <USB/DeviceList.h>
#include <Geometry/GeometryValueCoders.h>
#include <Sound/SoundRecorder.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSaver.h>
#include <Vrui/Vrui.h>
#include <Vrui/VisletManager.h>

/**************************************
Methods of class KinectRecorderFactory:
**************************************/

KinectRecorderFactory::KinectRecorderFactory(Vrui::VisletManager& visletManager)
	:Vrui::VisletFactory("KinectRecorder",visletManager)
	{
	#if 0
	/* Insert class into class hierarchy: */
	Vrui::VisletFactory* visletFactory=visletManager.loadClass("Vislet");
	visletFactory->addChildClass(this);
	addParentClass(visletFactory);
	#endif
	
	/* Load class settings: */
	Misc::ConfigurationFileSection cfs=visletManager.getVisletClassSection(getClassName());
	std::string defaultSaveFileNamePrefix=cfs.retrieveString("./saveFileNamePrefix",".");
	std::string defaultBackgroundFileNamePrefix=cfs.retrieveString("./backgroundFileNamePrefix","");
	
	std::vector<std::string> kinectDevices=cfs.retrieveValue<std::vector<std::string> >("./kinectDevices",std::vector<std::string>());
	for(std::vector<std::string>::iterator kdIt=kinectDevices.begin();kdIt!=kinectDevices.end();++kdIt)
		{
		/* Go to the Kinect device's configuration file section: */
		Misc::ConfigurationFileSection kds=cfs.getSection(kdIt->c_str());
		
		KinectConfig config;
		
		/* Read node index, device serial number, and high-res flag: */
		config.nodeIndex=kds.retrieveValue<int>("./nodeIndex",0);
		config.deviceSerialNumber=kds.retrieveString("./serialNumber");
		config.highResolution=kds.retrieveValue<bool>("./highResolution",false);
		
		/* Read the save file name prefix: */
		config.saveFileNamePrefix=kds.retrieveString("./saveFileNamePrefix",defaultSaveFileNamePrefix);
		
		/* Read background removal settings: */
		config.backgroundFileName=kds.retrieveString("./backgroundFileNamePrefix",defaultBackgroundFileNamePrefix);
		config.captureBackgroundFrames=kds.retrieveValue<unsigned int>("./captureBackgroundFrames",0);
		config.maxDepth=kds.retrieveValue<unsigned int>("./maxDepth",0);
		config.backgroundRemovalFuzz=kds.retrieveValue<int>("./backgroundRemovalFuzz",-1000000);
		
		/* Store the configuration structure: */
		kinectConfigs.push_back(config);
		}
	
	std::vector<std::string> soundDevices=cfs.retrieveValue<std::vector<std::string> >("./soundDevices",std::vector<std::string>());
	for(std::vector<std::string>::iterator sdIt=soundDevices.begin();sdIt!=soundDevices.end();++sdIt)
		{
		/* Go to the sound device's configuration file section: */
		Misc::ConfigurationFileSection sds=cfs.getSection(sdIt->c_str());
		
		SoundConfig config;
		
		/* Read node index and sound device name: */
		config.nodeIndex=sds.retrieveValue<int>("./nodeIndex",0);
		config.soundDeviceName=sds.retrieveString("./soundDeviceName","");
		
		/* Create a sound data format for recording: */
		config.soundFormat.bitsPerSample=sds.retrieveValue<int>("./sampleResolution",config.soundFormat.bitsPerSample);
		config.soundFormat.samplesPerFrame=sds.retrieveValue<int>("./numChannels",config.soundFormat.samplesPerFrame);
		config.soundFormat.framesPerSecond=sds.retrieveValue<int>("./sampleRate",config.soundFormat.framesPerSecond);
		
		/* Read the sound file name: */
		config.soundFileName=sds.retrieveString("./soundFileName");
		
		/* Store the configuration structure: */
		soundConfigs.push_back(config);
		}
	
	/* Set tool class' factory pointer: */
	KinectRecorder::factory=this;
	}

KinectRecorderFactory::~KinectRecorderFactory(void)
	{
	/* Reset tool class' factory pointer: */
	KinectRecorder::factory=0;
	}

Vrui::Vislet* KinectRecorderFactory::createVislet(int numArguments,const char* const arguments[]) const
	{
	return new KinectRecorder(numArguments,arguments);
	}

void KinectRecorderFactory::destroyVislet(Vrui::Vislet* vislet) const
	{
	delete vislet;
	}

extern "C" void resolveKinectRecorderDependencies(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	#if 0
	/* Load base classes: */
	manager.loadClass("Vislet");
	#endif
	}

extern "C" Vrui::VisletFactory* createKinectRecorderFactory(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	/* Get pointer to vislet manager: */
	Vrui::VisletManager* visletManager=static_cast<Vrui::VisletManager*>(&manager);
	
	/* Create factory object and insert it into class hierarchy: */
	KinectRecorderFactory* kinectRecorderFactory=new KinectRecorderFactory(*visletManager);
	
	/* Return factory object: */
	return kinectRecorderFactory;
	}

extern "C" void destroyKinectRecorderFactory(Vrui::VisletFactory* factory)
	{
	delete factory;
	}

/***********************************************
Methods of class KinectRecorder::KinectStreamer:
***********************************************/

KinectRecorder::KinectStreamer::KinectStreamer(USB::Context& usbContext,const KinectRecorderFactory::KinectConfig& config)
	:camera(usbContext,config.deviceSerialNumber.c_str()),frameSaver(0)
	{
	/* Check if there is an existing background frame for the camera: */
	bool removeBackground=false;
	if(!config.backgroundFileName.empty())
		{
		/* Load a background frame file: */
		camera.loadBackground(config.backgroundFileName.c_str());
		removeBackground=true;
		}
	
	/* Check for background capture: */
	if(config.captureBackgroundFrames>0)
		{
		/* Capture background: */
		camera.captureBackground(config.captureBackgroundFrames,false);
		removeBackground=true;
		}
	
	/* Check if there is a maximum depth value: */
	if(config.maxDepth>0)
		{
		/* Set the camera's maximum depth value: */
		camera.setMaxDepth(config.maxDepth);
		removeBackground=true;
		}
	
	/* Enable background removal if requested: */
	camera.setRemoveBackground(removeBackground);
	
	/* Set the background removal fuzz value: */
	if(config.backgroundRemovalFuzz!=-1000000)
		camera.setBackgroundRemovalFuzz(config.backgroundRemovalFuzz);
	
	/* Set the camera's frame size: */
	camera.setFrameSize(Kinect::FrameSource::COLOR,config.highResolution?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
	
	/* Create the frame saver: */
	std::string depthFrameFileName=config.saveFileNamePrefix;
	depthFrameFileName.push_back('-');
	depthFrameFileName.append(config.deviceSerialNumber);
	depthFrameFileName.append(".depth");
	std::string colorFrameFileName=config.saveFileNamePrefix;
	colorFrameFileName.push_back('-');
	colorFrameFileName.append(config.deviceSerialNumber);
	colorFrameFileName.append(".color");
	frameSaver=new Kinect::FrameSaver(camera,colorFrameFileName.c_str(),depthFrameFileName.c_str());
	}

KinectRecorder::KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	camera.stopStreaming();
	
	/* Delete the frame saver: */
	delete frameSaver;
	}

void KinectRecorder::KinectStreamer::startStreaming(void)
	{
	/* Start streaming: */
	camera.startStreaming(Misc::createFunctionCall(frameSaver,&Kinect::FrameSaver::saveColorFrame),Misc::createFunctionCall(frameSaver,&Kinect::FrameSaver::saveDepthFrame));
	}

/***************************************
Static elements of class KinectRecorder:
***************************************/

KinectRecorderFactory* KinectRecorder::factory=0;

/*******************************
Methods of class KinectRecorder:
*******************************/

KinectRecorder::KinectRecorder(int numArguments,const char* const arguments[])
	:soundRecorder(0),
	 firstEnable(true)
	{
	/* Check if this node has any connected Kinect devices: */
	bool haveDevices=false;
	for(std::vector<KinectRecorderFactory::KinectConfig>::const_iterator kcIt=factory->kinectConfigs.begin();kcIt!=factory->kinectConfigs.end()&&!haveDevices;++kcIt)
		haveDevices=kcIt->nodeIndex==Vrui::getNodeIndex();
	
	if(haveDevices)
		{
		/* Enable background USB event handling: */
		usbContext.startEventHandling();
		
		/* Connect to all requested Kinect devices: */
		for(std::vector<KinectRecorderFactory::KinectConfig>::const_iterator kcIt=factory->kinectConfigs.begin();kcIt!=factory->kinectConfigs.end();++kcIt)
			if(kcIt->nodeIndex==Vrui::getNodeIndex())
				{
				/* Create a streamer for the Kinect device of the given serial number: */
				streamers.push_back(new KinectStreamer(usbContext,*kcIt));
				}
		}
	
	/* Create this node's sound recorders: */
	for(std::vector<KinectRecorderFactory::SoundConfig>::const_iterator scIt=factory->soundConfigs.begin();scIt!=factory->soundConfigs.end();++scIt)
		if(scIt->nodeIndex==Vrui::getNodeIndex())
			{
			/* Create the sound recorder: */
			if(!scIt->soundDeviceName.empty())
				soundRecorder=new Sound::SoundRecorder(scIt->soundDeviceName.c_str(),scIt->soundFormat,scIt->soundFileName.c_str());
			else
				soundRecorder=new Sound::SoundRecorder(scIt->soundFormat,scIt->soundFileName.c_str());
			break;
			}
	}

KinectRecorder::~KinectRecorder(void)
	{
	/* Delete all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		delete *sIt;
	
	/* Delete the sound recorder: */
	delete soundRecorder;
	}

Vrui::VisletFactory* KinectRecorder::getFactory(void) const
	{
	return factory;
	}

void KinectRecorder::enable(void)
	{
	/* Call the base class method: */
	Vislet::enable();
	
	if(firstEnable)
		{
		/* Start recording sound: */
		if(soundRecorder!=0)
			soundRecorder->start();
		
		/* Synchronize all cameras' time bases: */
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			(*sIt)->getCamera().resetFrameTimer(Vrui::getApplicationTime());
		
		/* Start recording: */
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			(*sIt)->startStreaming();
		
		firstEnable=false;
		}
	}
