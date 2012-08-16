/***********************************************************************
KinectViewer - Vislet to draw 3D reconstructions captured from a Kinect
device in 3D space.
Copyright (c) 2010-2012 Oliver Kreylos

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
#include <Misc/FunctionCalls.h>
#include <Cluster/OpenPipe.h>
#include <USB/Context.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/VisletManager.h>
#include <Vrui/OpenFile.h>
#include <Kinect/FunctionCalls.h>
#include <Kinect/Camera.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/Renderer.h>

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

/*************************************
Static elements of class KinectViewer:
*************************************/

KinectViewerFactory* KinectViewer::factory=0;

/*****************************
Methods of class KinectViewer:
*****************************/

void KinectViewer::updateCallback(void)
	{
	/* Update application state: */
	Vrui::requestUpdate();
	}

KinectViewer::KinectViewer(int numArguments,const char* const arguments[])
	:usbContext(0)
	{
	/* Parse the command line: */
	bool highres=false;
	bool compressDepth=false;
	for(int i=0;i<numArguments;++i)
		{
		if(strcasecmp(arguments[i],"-high")==0)
			highres=true;
		else if(strcasecmp(arguments[i],"-low")==0)
			highres=false;
		else if(strcasecmp(arguments[i],"-compress")==0)
			compressDepth=true;
		else if(strcasecmp(arguments[i],"-nocompress")==0)
			compressDepth=false;
		else if(strcasecmp(arguments[i],"-c")==0)
			{
			if(usbContext==0)
				{
				/* Create a USB context: */
				usbContext=new USB::Context;
				
				/* Enable background USB event handling: */
				usbContext->startEventHandling();
				}
			
			/* Connect to a local Kinect device: */
			++i;
			if(Vrui::getClusterMultiplexer()==0)
				{
				/* Open the camera of the given index: */
				int cameraIndex=atoi(arguments[i]);
				Kinect::Camera* camera=new Kinect::Camera(*usbContext,cameraIndex);
				
				/* Set the camera's frame size and compression flag: */
				camera->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
				camera->setCompressDepthFrames(compressDepth);
				
				/* Add a renderer for the camera: */
				renderers.push_back(new Kinect::Renderer(camera));
				}
			else if(Vrui::isMaster())
				{
				/* Can't stream from local camera in cluster mode: */
				std::cerr<<"KinectViewer: Ignoring -c "<<arguments[i]<<" argument: Streaming from local Kinect camera(s) is not supported in cluster environments"<<std::endl;
				}
			}
		else if(strcasecmp(arguments[i],"-f")==0)
			{
			/* Open a 3D video stream file: */
			++i;
			std::string colorFileName=arguments[i];
			colorFileName.append(".color");
			std::string depthFileName=arguments[i];
			depthFileName.append(".depth");
			Kinect::FileFrameSource* fileSource=new Kinect::FileFrameSource(Vrui::openFile(colorFileName.c_str()),Vrui::openFile(depthFileName.c_str()));
			
			/* Add a renderer for the file source: */
			renderers.push_back(new Kinect::Renderer(fileSource));
			}
		else if(strcasecmp(arguments[i],"-p")==0)
			{
			/* Connect to a 3D video streaming server: */
			i+=2;
			Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),arguments[i-1],atoi(arguments[i])));
			
			/* Add a renderer for each component stream in the multiplexer: */
			for(unsigned int i=0;i<source->getNumStreams();++i)
				renderers.push_back(new Kinect::Renderer(source->getStream(i)));
			}
		}
	
	/* Reset all renderers' frame timers: */
	for(std::vector<Kinect::Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->resetFrameTimer();
	
	/* Start streaming on all renderers: */
	for(std::vector<Kinect::Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->startStreaming(Misc::createFunctionCall(this,&KinectViewer::updateCallback));
	}

KinectViewer::~KinectViewer(void)
	{
	/* Delete all renderers: */
	for(std::vector<Kinect::Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		delete *rIt;
	
	/* Close the USB context: */
	delete usbContext;
	}

Vrui::VisletFactory* KinectViewer::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::frame(void)
	{
	/* Update all renderers: */
	for(std::vector<Kinect::Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->frame();
	}

void KinectViewer::display(GLContextData& contextData) const
	{
	/* Draw the current 3D video facades of all renderers: */
	for(std::vector<Kinect::Renderer*>::const_iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->glRenderAction(contextData);
	}
