/***********************************************************************
KinectPlayer - Vislet to play back 3D video previously captured from one
or more Kinect devices.
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

#include "Vislets/KinectPlayer.h"

#include <Misc/StandardValueCoders.h>
#include <Misc/CompoundValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <IO/File.h>
#include <Math/Constants.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Sound/SoundPlayer.h>
#include <Kinect/ColorFrameReader.h>
#include <Kinect/DepthFrameReader.h>
#include <Kinect/LossyDepthFrameReader.h>
#include <Vrui/Vrui.h>
#include <Vrui/VisletManager.h>
#include <Vrui/OpenFile.h>

/************************************
Methods of class KinectPlayerFactory:
************************************/

KinectPlayerFactory::KinectPlayerFactory(Vrui::VisletManager& visletManager)
	:Vrui::VisletFactory("KinectPlayer",visletManager)
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
	
	std::vector<std::string> kinectDevices=cfs.retrieveValue<std::vector<std::string> >("./kinectDevices",std::vector<std::string>());
	for(std::vector<std::string>::iterator kdIt=kinectDevices.begin();kdIt!=kinectDevices.end();++kdIt)
		{
		/* Go to the Kinect device's configuration file section: */
		Misc::ConfigurationFileSection kds=cfs.getSection(kdIt->c_str());
		
		KinectConfig config;
		
		/* Read the recorded camera's serial number: */
		config.deviceSerialNumber=kds.retrieveString("./serialNumber");
		
		/* Read the save file name prefix: */
		config.saveFileNamePrefix=kds.retrieveString("./saveFileNamePrefix",defaultSaveFileNamePrefix);
		
		/* Store the configuration structure: */
		kinectConfigs.push_back(config);
		}
	
	std::vector<std::string> soundDevices=cfs.retrieveValue<std::vector<std::string> >("./soundDevices",std::vector<std::string>());
	for(std::vector<std::string>::iterator sdIt=soundDevices.begin();sdIt!=soundDevices.end();++sdIt)
		{
		/* Go to the sound device's configuration file section: */
		Misc::ConfigurationFileSection sds=cfs.getSection(sdIt->c_str());
		
		SoundConfig config;
		
		/* Read node index: */
		config.nodeIndex=sds.retrieveValue<int>("./nodeIndex",0);
		
		/* Read the sound file name: */
		config.soundFileName=sds.retrieveString("./soundFileName");
		
		/* Store the configuration structure: */
		soundConfigs.push_back(config);
		}
	
	/* Set tool class' factory pointer: */
	KinectPlayer::factory=this;
	}

KinectPlayerFactory::~KinectPlayerFactory(void)
	{
	/* Reset tool class' factory pointer: */
	KinectPlayer::factory=0;
	}

Vrui::Vislet* KinectPlayerFactory::createVislet(int numArguments,const char* const arguments[]) const
	{
	return new KinectPlayer(numArguments,arguments);
	}

void KinectPlayerFactory::destroyVislet(Vrui::Vislet* vislet) const
	{
	delete vislet;
	}

extern "C" void resolveKinectPlayerDependencies(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	#if 0
	/* Load base classes: */
	manager.loadClass("Vislet");
	#endif
	}

extern "C" Vrui::VisletFactory* createKinectPlayerFactory(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	/* Get pointer to vislet manager: */
	Vrui::VisletManager* visletManager=static_cast<Vrui::VisletManager*>(&manager);
	
	/* Create factory object and insert it into class hierarchy: */
	KinectPlayerFactory* kinectPlayerFactory=new KinectPlayerFactory(*visletManager);
	
	/* Return factory object: */
	return kinectPlayerFactory;
	}

extern "C" void destroyKinectPlayerFactory(Vrui::VisletFactory* factory)
	{
	delete factory;
	}

/*********************************************
Methods of class KinectPlayer::KinectStreamer:
*********************************************/

void* KinectPlayer::KinectStreamer::colorDecompressorThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read color frames: */
	while(true)
		{
		/* Read the next color frame: */
		Kinect::FrameBuffer nextFrame=colorDecompressor->readNextFrame();
		
		/* Put the new color frame into the queue: */
		{
		Threads::MutexCond::Lock frameQueueLock(frameQueueCond);
		while(numColorFrames==2)
			frameQueueCond.wait(frameQueueLock);
		mostRecentColorFrame=1-mostRecentColorFrame;
		colorFrames[mostRecentColorFrame]=nextFrame;
		++numColorFrames;
		if(numColorFrames==1)
			frameQueueCond.broadcast();
		}
		
		if(nextFrame.timeStamp==Math::Constants<double>::max)
			break;
		}
	
	return 0;
	}

void* KinectPlayer::KinectStreamer::depthDecompressorThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read depth frames: */
	while(true)
		{
		/* Read the next depth frame: */
		Kinect::FrameBuffer nextFrame=depthDecompressor->readNextFrame();
		
		/* Process the next depth frame into a mesh: */
		Kinect::MeshBuffer nextMesh;
		projector.processDepthFrame(nextFrame,nextMesh);
		
		/* Put the new depth frame into the queue: */
		{
		Threads::MutexCond::Lock frameQueueLock(frameQueueCond);
		while(numDepthFrames==2)
			frameQueueCond.wait(frameQueueLock);
		mostRecentDepthFrame=1-mostRecentDepthFrame;
		depthFrames[mostRecentDepthFrame]=nextMesh;
		++numDepthFrames;
		if(numDepthFrames==1)
			frameQueueCond.broadcast();
		}
		
		if(nextMesh.timeStamp>=Math::Constants<double>::max)
			break;
		}
	
	return 0;
	}

KinectPlayer::KinectStreamer::KinectStreamer(const KinectPlayerFactory::KinectConfig& config)
	:colorDecompressor(0),depthDecompressor(0),
	 numColorFrames(0),mostRecentColorFrame(0),
	 numDepthFrames(0),mostRecentDepthFrame(0)
	{
	/* Open the color file: */
	std::string colorFileName=config.saveFileNamePrefix;
	colorFileName.push_back('-');
	colorFileName.append(config.deviceSerialNumber);
	colorFileName.append(".color");
	colorFile=Vrui::openFile(colorFileName.c_str());
	colorFile->setEndianness(Misc::LittleEndian);
	
	/* Open the depth file: */
	std::string depthFileName=config.saveFileNamePrefix;
	depthFileName.push_back('-');
	depthFileName.append(config.deviceSerialNumber);
	depthFileName.append(".depth");
	depthFile=Vrui::openFile(depthFileName.c_str());
	depthFile->setEndianness(Misc::LittleEndian);
	
	/* Read the files' format version numbers: */
	// unsigned int colorFormatVersion=colorFile->read<unsigned int>();
	colorFile->skip<unsigned int>(1);
	unsigned int depthFormatVersion=depthFile->read<unsigned int>();
	
	/* Check if there are per-pixel depth correction coefficients: */
	Kinect::FrameSource::DepthCorrection* depthCorrection=0;
	if(depthFormatVersion>=4)
		{
		/* Read new B-spline based depth correction parameters: */
		depthCorrection=new Kinect::FrameSource::DepthCorrection(*depthFile);
		}
	else
		{
		if(depthFormatVersion>=2&&depthFile->read<char>()!=0)
			{
			/* Skip the depth correction buffer: */
			int size[2];
			depthFile->read<int>(size,2);
			depthFile->skip<float>(size[1]*size[0]*2);
			}
		}
	
	/* Check if the depth stream uses lossy compression: */
	bool depthIsLossy=depthFormatVersion>=3&&depthFile->read<unsigned char>()!=0;
	
	/* Read the color and depth projections from their respective files: */
	Kinect::FrameSource::IntrinsicParameters ips;
	ips.colorProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*colorFile);
	ips.depthProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*depthFile);
	projector.setIntrinsicParameters(ips);
	
	/* Read the camera transformation from the depth file: */
	Kinect::FrameSource::ExtrinsicParameters eps;
	eps=Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::read(*depthFile);
	projector.setExtrinsicParameters(eps);
	
	/* Create the color and depth decompressors: */
	colorDecompressor=new Kinect::ColorFrameReader(*colorFile);
	if(depthIsLossy)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		depthDecompressor=new Kinect::LossyDepthFrameReader(*depthFile);
		#else
		delete colorDecompressor;
		Misc::throwStdErr("KinectPlayer: Lossy depth compression not supported due to lack of Theora library");
		#endif
		}
	else
		depthDecompressor=new Kinect::DepthFrameReader(*depthFile);
	
	/* Set the projector's depth frame size: */
	projector.setDepthFrameSize(depthDecompressor->getSize());
	
	/* Set the projector's depth correction coefficients: */
	projector.setDepthCorrection(depthCorrection);
	
	/* Clean up: */
	delete depthCorrection;
	
	/* Start the color and depth decompression threads: */
	colorDecompressorThread.start(this,&KinectPlayer::KinectStreamer::colorDecompressorThreadMethod);
	depthDecompressorThread.start(this,&KinectPlayer::KinectStreamer::depthDecompressorThreadMethod);
	}

KinectPlayer::KinectStreamer::~KinectStreamer(void)
	{
	/* Shut down the depth and color decompression threads: */
	colorDecompressorThread.cancel();
	depthDecompressorThread.cancel();
	colorDecompressorThread.join();
	depthDecompressorThread.join();
	
	/* Delete the color and depth decompressors: */
	delete colorDecompressor;
	delete depthDecompressor;
	}

void KinectPlayer::KinectStreamer::updateFrames(double currentTimeStamp)
	{
	/* Wait until the next frame is newer than the new time step: */
	Kinect::FrameBuffer currentColorFrame;
	Kinect::MeshBuffer currentDepthFrame;
	while(nextColorFrame.timeStamp<=currentTimeStamp||nextDepthFrame.timeStamp<=currentTimeStamp)
		{
		if(nextColorFrame.timeStamp<=currentTimeStamp)
			{
			currentColorFrame=nextColorFrame;
			{
			Threads::MutexCond::Lock frameQueueLock(frameQueueCond);
			while(numColorFrames==0)
				frameQueueCond.wait(frameQueueLock);
			nextColorFrame=colorFrames[(mostRecentColorFrame-numColorFrames+3)%2];
			if(--numColorFrames==1)
				frameQueueCond.broadcast();
			}
			}
		if(nextDepthFrame.timeStamp<=currentTimeStamp)
			{
			currentDepthFrame=nextDepthFrame;
			{
			Threads::MutexCond::Lock frameQueueLock(frameQueueCond);
			while(numDepthFrames==0)
				frameQueueCond.wait(frameQueueLock);
			nextDepthFrame=depthFrames[(mostRecentDepthFrame-numDepthFrames+3)%2];
			if(--numDepthFrames==1)
				frameQueueCond.broadcast();
			}
			}
		}
	
	/* Update the projector: */
	if(currentColorFrame.timeStamp!=0.0)
		projector.setColorFrame(currentColorFrame);
	if(currentDepthFrame.timeStamp!=0.0)
		projector.setMesh(currentDepthFrame);
	projector.updateFrames();
	}

void KinectPlayer::KinectStreamer::glRenderAction(GLContextData& contextData) const
	{
	/* Draw the camera's facade: */
	projector.glRenderAction(contextData);
	}

/*************************************
Static elements of class KinectPlayer:
*************************************/

KinectPlayerFactory* KinectPlayer::factory=0;

/*****************************
Methods of class KinectPlayer:
*****************************/

KinectPlayer::KinectPlayer(int numArguments,const char* const arguments[])
	:soundPlayer(0),
	 firstEnable(true)
	{
	for(std::vector<KinectPlayerFactory::KinectConfig>::const_iterator kcIt=factory->kinectConfigs.begin();kcIt!=factory->kinectConfigs.end();++kcIt)
		{
		/* Create a streamer for the found camera: */
		streamers.push_back(new KinectStreamer(*kcIt));
		}
	
	for(std::vector<KinectPlayerFactory::SoundConfig>::const_iterator scIt=factory->soundConfigs.begin();scIt!=factory->soundConfigs.end();++scIt)
		if(scIt->nodeIndex==Vrui::getNodeIndex())
			{
			soundPlayer=new Sound::SoundPlayer(scIt->soundFileName.c_str());
			break;
			}
	}

KinectPlayer::~KinectPlayer(void)
	{
	/* Delete all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		delete *sIt;
	
	/* Delete the sound recorder: */
	delete soundPlayer;
	}

Vrui::VisletFactory* KinectPlayer::getFactory(void) const
	{
	return factory;
	}

void KinectPlayer::enable(void)
	{
	/* Call the base class method: */
	Vislet::enable();
	
	if(firstEnable)
		{
		/* Start sound playback: */
		if(soundPlayer!=0)
			soundPlayer->start();
		
		firstEnable=false;
		}
	}

void KinectPlayer::frame(void)
	{
	/* Block until all streamers have frames valid for the current time stamp: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->updateFrames(Vrui::getApplicationTime());
	}

void KinectPlayer::display(GLContextData& contextData) const
	{
	/* Render all streamers: */
	for(std::vector<KinectStreamer*>::const_iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->glRenderAction(contextData);
	}
