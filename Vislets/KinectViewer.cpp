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

#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/VisletManager.h>
#include <Kinect/Camera.h>
#include <Kinect/Projector.h>

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

void KinectViewer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Forward the new color frame to the projector: */
	projector->setColorFrame(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void KinectViewer::meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer)
	{
	/* Update application state: */
	Vrui::requestUpdate();
	}

KinectViewer::KinectViewer(int numArguments,const char* const arguments[])
	:source(0),
	 projector(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to first Kinect camera device on the host: */
	source=new Kinect::Camera(usbContext);
	
	/* Create a Kinect projector: */
	projector=new Kinect::Projector(*source);
	
	/* Start streaming: */
	projector->startStreaming(Misc::createFunctionCall(this,&KinectViewer::meshStreamingCallback));
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::colorStreamingCallback),Misc::createFunctionCall(projector,&Kinect::Projector::setDepthFrame));
	}

KinectViewer::~KinectViewer(void)
	{
	/* Stop streaming: */
	source->stopStreaming();
	projector->stopStreaming();
	
	delete projector;
	delete source;
	}

Vrui::VisletFactory* KinectViewer::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::frame(void)
	{
	/* Update the projector: */
	projector->updateFrames();
	}

void KinectViewer::display(GLContextData& contextData) const
	{
	/* Draw the current 3D video facade: */
	projector->glRenderAction(contextData);
	}
