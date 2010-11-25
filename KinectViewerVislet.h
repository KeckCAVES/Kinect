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

#ifndef KINECTVIEWERVISLET_INCLUDED
#define KINECTVIEWERVISLET_INCLUDED

#include <string>
#include <Threads/TripleBuffer.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Vrui/Geometry.h>
#include <Vrui/Vislet.h>

#include "USBContext.h"
#include "FrameBuffer.h"

/* Forward declarations: */
class KinectCamera;
class KinectProjector;

class KinectViewerVislet;

class KinectViewerVisletFactory:public Vrui::VisletFactory
	{
	friend class KinectViewerVislet;
	
	/* Elements: */
	private:
	Vrui::OGTransform kinectTransform; // Transformation to map the Kinect's 3D reconstruction space into Vrui physical space
	std::string calibrationFileName; // Name of file containing the Kinect's depth and color calibration matrices
	
	/* Constructors and destructors: */
	public:
	KinectViewerVisletFactory(Vrui::VisletManager& visletManager);
	virtual ~KinectViewerVisletFactory(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::Vislet* createVislet(int numVisletArguments,const char* const visletArguments[]) const;
	virtual void destroyVislet(Vrui::Vislet* vislet) const;
	};

class KinectViewerVislet:public Vrui::Vislet
	{
	friend class KinectViewerVisletFactory;
	
	/* Elements: */
	private:
	static KinectViewerVisletFactory* factory; // Pointer to the class' factory object
	
	USBContext usbContext; // USB device context
	KinectCamera* kinectCamera; // Pointer to camera aspect of Kinect device
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	KinectProjector* kinectProjector; // Object to project depth and color frames back into 3D camera space
	
	/* Private methods: */
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback called when a new depth frame was received
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback called when a new color frame was received
	
	/* Constructors and destructors: */
	public:
	KinectViewerVislet(int numArguments,const char* const arguments[]);
	virtual ~KinectViewerVislet(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::VisletFactory* getFactory(void) const;
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
