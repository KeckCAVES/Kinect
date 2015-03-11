/***********************************************************************
KinectViewer - Vislet to draw 3D reconstructions captured from a Kinect
device in 3D space.
Copyright (c) 2010-2013 Oliver Kreylos

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

#include "Vislets/KinectViewer.h"

#include <string.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/FunctionCalls.h>
#include <Misc/PrintInteger.h>
#include <Cluster/OpenPipe.h>
#include <USB/Context.h>
#include <Math/Constants.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Video/Config.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>
#include <Vrui/VisletManager.h>
#include <Vrui/OpenFile.h>
#include <Kinect/FunctionCalls.h>
#include <Kinect/Camera.h>
#include <Kinect/DepthFrameReader.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Kinect/LossyDepthFrameReader.h>
#endif
#include <Kinect/ColorFrameReader.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/FrameSaver.h>

/************************************
Methods of class KinectViewerFactory:
************************************/

KinectViewerFactory::KinectViewerFactory(Vrui::VisletManager& visletManager)
	:Vrui::VisletFactory("KinectViewer",visletManager)
	{
	#if 0
	/* Insert class into class hierarchy: */
	Vrui::VisletFactory* visletFactory=visletManager.loadClass("Vislet");
	visletFactory->addChildClass(this);
	addParentClass(visletFactory);
	#endif
	
	/* Set tool class' factory pointer: */
	KinectViewer::factory=this;
	}

KinectViewerFactory::~KinectViewerFactory(void)
	{
	/* Reset tool class' factory pointer: */
	KinectViewer::factory=0;
	}

Vrui::Vislet* KinectViewerFactory::createVislet(int numArguments,const char* const arguments[]) const
	{
	return new KinectViewer(numArguments,arguments);
	}

void KinectViewerFactory::destroyVislet(Vrui::Vislet* vislet) const
	{
	delete vislet;
	}

extern "C" void resolveKinectViewerDependencies(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	#if 0
	/* Load base classes: */
	manager.loadClass("Vislet");
	#endif
	}

extern "C" Vrui::VisletFactory* createKinectViewerFactory(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	/* Get pointer to vislet manager: */
	Vrui::VisletManager* visletManager=static_cast<Vrui::VisletManager*>(&manager);
	
	/* Create factory object and insert it into class hierarchy: */
	KinectViewerFactory* kinectViewerFactory=new KinectViewerFactory(*visletManager);
	
	/* Return factory object: */
	return kinectViewerFactory;
	}

extern "C" void destroyKinectViewerFactory(Vrui::VisletFactory* factory)
	{
	delete factory;
	}

/***************************************
Methods of class KinectViewer::Renderer:
***************************************/

KinectViewer::Renderer::~Renderer(void)
	{
	/* Destroy the projector: */
	delete projector;
	}

/*******************************************
Methods of class KinectViewer::LiveRenderer:
*******************************************/

void KinectViewer::LiveRenderer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Forward color frame to the projector: */
	projector->setColorFrame(frameBuffer);
	
	Vrui::requestUpdate();
	
	if(frameSaver!=0)
		{
		/* Time-stamp the incoming frame: */
		Kinect::FrameBuffer saveFrame(frameBuffer);
		saveFrame.timeStamp=timeStamp;
		
		/* Forward the time-stamped frame to the frame saver: */
		frameSaver->saveColorFrame(saveFrame);
		}
	}

void KinectViewer::LiveRenderer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Forward depth frame to the projector: */
	projector->setDepthFrame(frameBuffer);
	
	#if KINECT_USE_SHADERPROJECTOR
	Vrui::requestUpdate();
	#endif
	
	if(frameSaver!=0)
		{
		/* Time-stamp the incoming frame: */
		Kinect::FrameBuffer saveFrame(frameBuffer);
		saveFrame.timeStamp=timeStamp;
		
		/* Forward the time-stamped frame to the frame saver: */
		frameSaver->saveDepthFrame(saveFrame);
		}
	}

#if !KINECT_USE_SHADERPROJECTOR

void KinectViewer::LiveRenderer::meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer)
	{
	Vrui::requestUpdate();
	}

#endif

KinectViewer::LiveRenderer::LiveRenderer(Kinect::FrameSource* sSource)
	:source(sSource),started(false),
	 frameSaver(0),timeStamp(0.0)
	{
	/* Create the projector: */
	#if KINECT_USE_SHADERPROJECTOR
	projector=new Kinect::ShaderProjector(*source);
	#else
	projector=new Kinect::Projector(*source);
	#endif
	}

KinectViewer::LiveRenderer::~LiveRenderer(void)
	{
	if(started)
		{
		/* Stop streaming: */
		source->stopStreaming();
		#if !KINECT_USE_SHADERPROJECTOR
		projector->stopStreaming();
		#endif
		}
	
	/* Destroy the frame source: */
	delete source;
	
	/* Destroy the frame saver: */
	delete frameSaver;
	}

void KinectViewer::LiveRenderer::startStreaming(void)
	{
	#if !KINECT_USE_SHADERPROJECTOR
	
	/* Hook this renderer into the projector's mesh callback: */
	projector->startStreaming(Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::meshStreamingCallback));
	
	#endif
	
	/* Hook this renderer into the frame source and start streaming: */
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::depthStreamingCallback));
	
	started=true;
	}

void KinectViewer::LiveRenderer::frame(double newTimeStamp)
	{
	/* Update the projector: */
	projector->updateFrames();
	
	/* Update the time stamp: */
	timeStamp=newTimeStamp;
	}

void KinectViewer::LiveRenderer::saveStreams(const std::string& saveFileName)
	{
	/* Ignore request if already streaming: */
	if(!started)
		{
		std::string colorFileName=saveFileName;
		colorFileName.append(".color");
		std::string depthFileName=saveFileName;
		depthFileName.append(".depth");
		frameSaver=new Kinect::FrameSaver(*source,colorFileName.c_str(),depthFileName.c_str());
		}
	}

/**********************************************
Methods of class KinectViewer::SynchedRenderer:
**********************************************/

void* KinectViewer::SynchedRenderer::colorReaderThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read color frames: */
	while(true)
		{
		/* Read the next color frame: */
		Kinect::FrameBuffer nextFrame=colorReader->readNextFrame();
		
		/* Put the new color frame into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numColorFrames==numQueueSlots)
			colorFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentColorFrame=(mostRecentColorFrame+1)%numQueueSlots;
		colorFrames[mostRecentColorFrame]=nextFrame;
		if(++numColorFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextFrame.timeStamp>=Math::Constants<double>::max)
			break;
		}
	
	return 0;
	}

void* KinectViewer::SynchedRenderer::depthReaderThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read depth frames: */
	while(true)
		{
		/* Read the next depth frame: */
		Kinect::FrameBuffer nextFrame=depthReader->readNextFrame();
		
		#if KINECT_USE_SHADERPROJECTOR
		
		/* Put the new depth frame into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numDepthFrames==numQueueSlots)
			depthFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentDepthFrame=(mostRecentDepthFrame+1)%numQueueSlots;
		depthFrames[mostRecentDepthFrame]=nextFrame;
		if(++numDepthFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextFrame.timeStamp>=Math::Constants<double>::max)
			break;
		
		#else
		
		/* Process the next depth frame into a mesh: */
		Kinect::MeshBuffer nextMesh;
		projector->processDepthFrame(nextFrame,nextMesh);
		
		/* Put the new mesh into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numDepthFrames==numQueueSlots)
			depthFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentDepthFrame=(mostRecentDepthFrame+1)%numQueueSlots;
		depthFrames[mostRecentDepthFrame]=nextMesh;
		if(++numDepthFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextMesh.timeStamp>=Math::Constants<double>::max)
			break;
		
		#endif
		}
	
	return 0;
	}

KinectViewer::SynchedRenderer::SynchedRenderer(const std::string& fileName)
	:colorReader(0),depthReader(0),
	 started(false),
	 timeStamp(0.0),
	 numColorFrames(0),mostRecentColorFrame(0),
	 numDepthFrames(0),mostRecentDepthFrame(0)
	{
	/* Open the color file: */
	std::string colorFileName=fileName;
	colorFileName.append(".color");
	colorFile=Vrui::openFile(colorFileName.c_str());
	colorFile->setEndianness(Misc::LittleEndian);
	
	/* Open the depth file: */
	std::string depthFileName=fileName;
	depthFileName.append(".depth");
	depthFile=Vrui::openFile(depthFileName.c_str());
	depthFile->setEndianness(Misc::LittleEndian);
	
	/* Read the files' format version numbers: */
	unsigned int colorFormatVersion=colorFile->read<Misc::UInt32>();
	unsigned int depthFormatVersion=depthFile->read<Misc::UInt32>();
	if(colorFormatVersion>1||depthFormatVersion>4)
		Misc::throwStdErr("KinectViewer::SynchedRenderer: Unsupported 3D video file format");
	
	/* Check if there are per-pixel depth correction coefficients: */
	Kinect::FrameSource::DepthCorrection* depthCorrection=0;
	if(depthFormatVersion>=4)
		{
		/* Read new B-spline based depth correction parameters: */
		depthCorrection=new Kinect::FrameSource::DepthCorrection(*depthFile);
		}
	else
		{
		if(depthFormatVersion>=2&&depthFile->read<Misc::UInt8>()!=0)
			{
			/* Skip the depth correction buffer: */
			Misc::SInt32 size[2];
			depthFile->read<Misc::SInt32>(size,2);
			depthFile->skip<Misc::Float32>(size[1]*size[0]*2);
			}
		
		/* Create a dummy depth correction object: */
		int numSegments[2]={1,1};
		depthCorrection=new Kinect::FrameSource::DepthCorrection(0,numSegments);
		}
	
	/* Check if the depth stream uses lossy compression: */
	bool depthIsLossy=depthFormatVersion>=3&&depthFile->read<Misc::UInt8>()!=0;
	
	/* Read the color and depth projections from their respective files: */
	Kinect::FrameSource::IntrinsicParameters ips;
	ips.colorProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*colorFile);
	ips.depthProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*depthFile);
	
	/* Read the camera transformation from the depth file: */
	Kinect::FrameSource::ExtrinsicParameters eps;
	eps=Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::read(*depthFile);
	
	/* Create the color and depth readers: */
	colorReader=new Kinect::ColorFrameReader(*colorFile);
	if(depthIsLossy)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		depthReader=new Kinect::LossyDepthFrameReader(*depthFile);
		#else
		delete colorReader;
		Misc::throwStdErr("KinectViewer::SynchedRenderer: Lossy depth compression not supported due to lack of Theora library");
		#endif
		}
	else
		depthReader=new Kinect::DepthFrameReader(*depthFile);
	
	/* Create and initialize the projector: */
	#if KINECT_USE_SHADERPROJECTOR
	projector=new Kinect::ShaderProjector();
	#else
	projector=new Kinect::Projector();
	#endif
	projector->setDepthFrameSize(depthReader->getSize());
	projector->setDepthCorrection(depthCorrection);
	#if KINECT_USE_SHADERPROJECTOR
	projector->setParameters(ips,eps);
	#else
	projector->setIntrinsicParameters(ips);
	projector->setExtrinsicParameters(eps);
	#endif
	
	/* Clean up: */
	delete depthCorrection;
	}

KinectViewer::SynchedRenderer::~SynchedRenderer(void)
	{
	if(started)
		{
		/* Shut down the depth and color reader threads: */
		colorReaderThread.cancel();
		depthReaderThread.cancel();
		colorReaderThread.join();
		depthReaderThread.join();
		}
	
	/* Delete the color and depth readers: */
	delete colorReader;
	delete depthReader;
	}

void KinectViewer::SynchedRenderer::startStreaming(void)
	{
	/* Start the color and depth reader threads: */
	colorReaderThread.start(this,&KinectViewer::SynchedRenderer::colorReaderThreadMethod);
	depthReaderThread.start(this,&KinectViewer::SynchedRenderer::depthReaderThreadMethod);
	
	started=true;
	}

void KinectViewer::SynchedRenderer::frame(double newTimeStamp)
	{
	timeStamp=newTimeStamp;
	
	/* Wait until the next frame is newer than the new time step: */
	bool newColor=false;
	Kinect::FrameBuffer currentColorFrame;
	bool newDepth=false;
	#if KINECT_USE_SHADERPROJECTOR
	Kinect::FrameBuffer currentDepthFrame;
	#else
	Kinect::MeshBuffer currentDepthFrame;
	#endif
	while(nextColorFrame.timeStamp<=timeStamp||nextDepthFrame.timeStamp<=timeStamp)
		{
		/* Check if both frame queues are empty: */
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numColorFrames==0&&numDepthFrames==0)
			frameQueuesEmptyCond.wait(frameQueueMutex);
		
		/* Advance in the color frame queue: */
		while(numColorFrames>0&&nextColorFrame.timeStamp<=timeStamp)
			{
			newColor=true;
			currentColorFrame=nextColorFrame;
			nextColorFrame=colorFrames[(mostRecentColorFrame-numColorFrames+numQueueSlots+1)%numQueueSlots];
			if(--numColorFrames==numQueueSlots-1)
				colorFrameQueueFullCond.broadcast();
			}
		
		/* Advance in the depth frame queue: */
		while(numDepthFrames>0&&nextDepthFrame.timeStamp<=timeStamp)
			{
			newDepth=true;
			currentDepthFrame=nextDepthFrame;
			nextDepthFrame=depthFrames[(mostRecentDepthFrame-numDepthFrames+numQueueSlots+1)%numQueueSlots];
			if(--numDepthFrames==numQueueSlots-1)
				depthFrameQueueFullCond.broadcast();
			}
		}
	
	/* Update the projector: */
	if(newColor)
		projector->setColorFrame(currentColorFrame);
	if(newDepth)
		{
		#if KINECT_USE_SHADERPROJECTOR
		projector->setDepthFrame(currentDepthFrame);
		#else
		projector->setMesh(currentDepthFrame);
		#endif
		}
	projector->updateFrames();
	}

/*************************************
Static elements of class KinectViewer:
*************************************/

KinectViewerFactory* KinectViewer::factory=0;

/*****************************
Methods of class KinectViewer:
*****************************/

KinectViewer::KinectViewer(int numArguments,const char* const arguments[])
	:usbContext(0),
	 navigational(false),
	 synched(false),
	 firstEnable(true),enabled(false)
	{
	/* Parse the command line: */
	bool highres=false;
	bool compressDepth=false;
	const char* saveFileNameBase=0;
	unsigned int saveFileIndex=0;
	for(int i=0;i<numArguments;++i)
		{
		if(strcasecmp(arguments[i],"-navigational")==0||strcasecmp(arguments[i],"-n")==0)
			navigational=true;
		else if(strcasecmp(arguments[i],"-high")==0)
			highres=true;
		else if(strcasecmp(arguments[i],"-low")==0)
			highres=false;
		else if(strcasecmp(arguments[i],"-compress")==0)
			compressDepth=true;
		else if(strcasecmp(arguments[i],"-nocompress")==0)
			compressDepth=false;
		else if(strcasecmp(arguments[i],"-save")==0)
			{
			++i;
			if(i<numArguments)
				saveFileNameBase=arguments[i];
			else
				std::cerr<<"KinectViewer: Ignoring dangling -save argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-c")==0)
			{
			++i;
			if(i<numArguments)
				{
				if(usbContext==0)
					{
					/* Create a USB context: */
					usbContext=new USB::Context;
					
					/* Enable background USB event handling: */
					usbContext->startEventHandling();
					}
				
				/* Connect to a local Kinect device: */
				if(Vrui::getClusterMultiplexer()==0)
					{
					/* Open the camera of the given index: */
					int cameraIndex=atoi(arguments[i]);
					Kinect::Camera* camera=new Kinect::Camera(*usbContext,cameraIndex);
					
					/* Set the camera's frame size and compression flag: */
					camera->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
					camera->setCompressDepthFrames(compressDepth);
					
					/* Load the camera's default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a renderer for the camera: */
					LiveRenderer* newRenderer=new LiveRenderer(camera);
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"KinectViewer: Ignoring -c "<<arguments[i]<<" argument: Streaming from local Kinect camera(s) is not supported in cluster environments"<<std::endl;
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -c argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-f")==0)
			{
			/* Open a 3D video stream file: */
			++i;
			if(i<numArguments)
				{
				/* Add a synchronized renderer for the given video file: */
				renderers.push_back(new SynchedRenderer(arguments[i]));
				
				synched=true;
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -f argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-p")==0)
			{
			/* Connect to a 3D video streaming server: */
			i+=2;
			if(i<numArguments)
				{
				Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),arguments[i-1],atoi(arguments[i])));
				
				/* Add a renderer for each component stream in the multiplexer: */
				for(unsigned int i=0;i<source->getNumStreams();++i)
					{
					LiveRenderer* newRenderer=new LiveRenderer(source->getStream(i));
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -p argument"<<std::endl;
			}
		}
	}

KinectViewer::~KinectViewer(void)
	{
	/* Delete all renderers: */
	for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		delete *rIt;
	
	/* Close the USB context: */
	delete usbContext;
	}

Vrui::VisletFactory* KinectViewer::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::disable(void)
	{
	/* Don't call the base class method when saving 3D video; we still need to receive frame updates even when invisible: */
	if(!synched)
		Vislet::disable();
	
	enabled=false;
	}

void KinectViewer::enable(void)
	{
	/* Call the base class method: */
	Vislet::enable();
	
	if(firstEnable)
		{
		/* Start streaming on all renderers: */
		for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
			(*rIt)->startStreaming();
		
		firstEnable=false;
		}
	
	enabled=true;
	}

void KinectViewer::frame(void)
	{
	/* Update all renderers: */
	for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->frame(Vrui::getApplicationTime());
	}

void KinectViewer::display(GLContextData& contextData) const
	{
	if(!enabled)
		return;
	
	if(navigational)
		{
		/* Go to navigational coordinates: */
		glPushMatrix();
		glLoadMatrix(Vrui::getDisplayState(contextData).modelviewNavigational);
		}
	
	/* Draw the current 3D video facades of all renderers: */
	for(std::vector<Renderer*>::const_iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->glRenderAction(contextData);
	
	if(navigational)
		{
		/* Go back to physical coordinates: */
		glPopMatrix();
		}
	}
