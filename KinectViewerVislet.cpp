/***********************************************************************
KinectViewerVislet - Vislet to draw 3D reconstructions captured from a
Kinect device in 3D space.
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

#include "KinectViewerVislet.h"

#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/VisletManager.h>
#include <Kinect/KinectCamera.h>
#include <Kinect/KinectProjector.h>

/******************************************
Methods of class KinectViewerVisletFactory:
******************************************/

KinectViewerVisletFactory::KinectViewerVisletFactory(Vrui::VisletManager& visletManager)
	:Vrui::VisletFactory("KinectViewerVislet",visletManager),
	 kinectTransform(Vrui::OGTransform::identity),
	 calibrationFileName("CalibrationMatrices.dat")
	{
	#if 0
	/* Insert class into class hierarchy: */
	Vrui::VisletFactory* visletFactory=visletManager.loadClass("Vislet");
	visletFactory->addChildClass(this);
	addParentClass(visletFactory);
	#endif
	
	/* Load class settings: */
	Misc::ConfigurationFileSection cfs=visletManager.getVisletClassSection(getClassName());
	kinectTransform=cfs.retrieveValue<Vrui::OGTransform>("./kinectTransform",kinectTransform);
	calibrationFileName=cfs.retrieveString("./calibrationFileName",calibrationFileName);
	
	/* Set tool class' factory pointer: */
	KinectViewerVislet::factory=this;
	}

KinectViewerVisletFactory::~KinectViewerVisletFactory(void)
	{
	/* Reset tool class' factory pointer: */
	KinectViewerVislet::factory=0;
	}

Vrui::Vislet* KinectViewerVisletFactory::createVislet(int numArguments,const char* const arguments[]) const
	{
	return new KinectViewerVislet(numArguments,arguments);
	}

void KinectViewerVisletFactory::destroyVislet(Vrui::Vislet* vislet) const
	{
	delete vislet;
	}

extern "C" void resolveKinectViewerVisletDependencies(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	#if 0
	/* Load base classes: */
	manager.loadClass("Vislet");
	#endif
	}

extern "C" Vrui::VisletFactory* createKinectViewerVisletFactory(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	/* Get pointer to vislet manager: */
	Vrui::VisletManager* visletManager=static_cast<Vrui::VisletManager*>(&manager);
	
	/* Create factory object and insert it into class hierarchy: */
	KinectViewerVisletFactory* kinectViewerVisletFactory=new KinectViewerVisletFactory(*visletManager);
	
	/* Return factory object: */
	return kinectViewerVisletFactory;
	}

extern "C" void destroyKinectViewerVisletFactory(Vrui::VisletFactory* factory)
	{
	delete factory;
	}

/*******************************************
Static elements of class KinectViewerVislet:
*******************************************/

KinectViewerVisletFactory* KinectViewerVislet::factory=0;

/***********************************
Methods of class KinectViewerVislet:
***********************************/

void KinectViewerVislet::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the depth frame triple buffer: */
	depthFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

void KinectViewerVislet::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Post the new frame into the color frame triple buffer: */
	colorFrames.postNewValue(frameBuffer);
	
	/* Update application state: */
	Vrui::requestUpdate();
	}

KinectViewerVislet::KinectViewerVislet(int numArguments,const char* const arguments[])
	:kinectCamera(0),
	 kinectProjector(0)
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to first Kinect camera device on the host: */
	kinectCamera=new KinectCamera(usbContext);
	
	/* Create a Kinect projector: */
	kinectProjector=new KinectProjector(factory->calibrationFileName.c_str());
	
	/* Start streaming: */
	KinectCamera::StreamingCallback* colorCallback=new Misc::VoidMethodCall<const FrameBuffer&,KinectViewerVislet>(this,&KinectViewerVislet::colorStreamingCallback);
	KinectCamera::StreamingCallback* depthCallback=new Misc::VoidMethodCall<const FrameBuffer&,KinectViewerVislet>(this,&KinectViewerVislet::depthStreamingCallback);
	kinectCamera->startStreaming(colorCallback,depthCallback);
	}

KinectViewerVislet::~KinectViewerVislet(void)
	{
	/* Stop streaming: */
	kinectCamera->stopStreaming();
	
	delete kinectProjector;
	
	/* Disconnect from the Kinect camera device: */
	delete kinectCamera;
	}

Vrui::VisletFactory* KinectViewerVislet::getFactory(void) const
	{
	return factory;
	}

void KinectViewerVislet::frame(void)
	{
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		/* Push the new frame to the Kinect projector: */
		kinectProjector->setDepthFrame(depthFrames.getLockedValue());
		}
	
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		{
		/* Push the new frame to the Kinect projector: */
		kinectProjector->setColorFrame(colorFrames.getLockedValue());
		}
	}

void KinectViewerVislet::display(GLContextData& contextData) const
	{
	/* Move the Kinect camera's reconstruction 3D space into Vrui physical space: */
	glPushMatrix();
	glMultMatrix(factory->kinectTransform);
	
	/* Draw the current depth image: */
	kinectProjector->draw(contextData);
	
	/* Return to physical space: */
	glPopMatrix();
	}
