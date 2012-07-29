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

#ifndef VISLETS_KINECTVIEWER_INCLUDED
#define VISLETS_KINECTVIEWER_INCLUDED

#include <string>
#include <USB/Context.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Vrui/Geometry.h>
#include <Vrui/Vislet.h>

/* Forward declarations: */
namespace Kinect {
class FrameBuffer;
class FrameSource;
class MeshBuffer;
class Projector;
}

class KinectViewer;

class KinectViewerFactory:public Vrui::VisletFactory
	{
	friend class KinectViewer;
	
	/* Constructors and destructors: */
	public:
	KinectViewerFactory(Vrui::VisletManager& visletManager);
	virtual ~KinectViewerFactory(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::Vislet* createVislet(int numVisletArguments,const char* const visletArguments[]) const;
	virtual void destroyVislet(Vrui::Vislet* vislet) const;
	};

class KinectViewer:public Vrui::Vislet
	{
	friend class KinectViewerFactory;
	
	/* Elements: */
	private:
	static KinectViewerFactory* factory; // Pointer to the class' factory object
	
	USB::Context usbContext; // USB device context
	Kinect::FrameSource* source; // Pointer to depth and color frame source
	Kinect::Projector* projector; // Object to project depth and color frames back into 3D camera space
	
	/* Private methods: */
	void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback called when a new color frame was received
	void meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer); // Callback called when a new depth frame was processed by the frame projector
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int numArguments,const char* const arguments[]);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::VisletFactory* getFactory(void) const;
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
