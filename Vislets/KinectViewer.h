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

#ifndef VISLETS_KINECTVIEWER_INCLUDED
#define VISLETS_KINECTVIEWER_INCLUDED

#include <string>
#include <vector>
#include <Threads/Thread.h>
#include <Threads/Mutex.h>
#include <Threads/Cond.h>
#include <IO/File.h>
#include <Vrui/Vislet.h>
#include <Kinect/Config.h>
#if KINECT_USE_SHADERPROJECTOR
#include <Kinect/ShaderProjector.h>
#else
#include <Kinect/Projector.h>
#endif

/* Forward declarations: */

namespace USB {
class Context;
}
namespace Kinect {
class FrameBuffer;
#if !KINECT_USE_SHADERPROJECTOR
class MeshBuffer;
#endif
class FrameSource;
class FrameSaver;
class FrameReader;
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
	
	/* Embedded classes: */
	class Renderer // Base class to render 3D video
		{
		/* Elements: */
		protected:
		#if KINECT_USE_SHADERPROJECTOR
		Kinect::ShaderProjector* projector;
		#else
		Kinect::Projector* projector;
		#endif
		
		/* Constructors and destructors: */
		public:
		Renderer(void)
			:projector(0)
			{
			}
		virtual ~Renderer(void);
		
		/* Methods: */
		virtual void startStreaming(void) =0; // Starts streaming 3D video frames into the projector
		virtual void frame(double newTimeStamp) =0; // Called once per application frame to update renderer state
		void glRenderAction(GLContextData& contextData) const // Draws the renderer's current state into the given OpenGL context
			{
			/* Draw the current 3D video frame: */
			projector->glRenderAction(contextData);
			}
		};
	
	class LiveRenderer:public Renderer // Class to render 3D video from a "live" unsynchronized source
		{
		/* Elements: */
		public:
		Kinect::FrameSource* source;
		bool started; // Flag whether streaming has been started
		Kinect::FrameSaver* frameSaver;
		double timeStamp; // Current timestamp when saving 3D video streams
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the frame source
		void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the frame source
		#if !KINECT_USE_SHADERPROJECTOR
		void meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer); // Callback receiving projected meshes from the projector
		#endif
		
		/* Constructors and destructors: */
		LiveRenderer(Kinect::FrameSource* sSource); // Creates a renderer for the given 3D video source and saves streams from source if save file name is non-empty; adopts source object
		virtual ~LiveRenderer(void);
		
		/* Methods from Renderer: */
		virtual void startStreaming(void);
		virtual void frame(double newTimeStamp);
		
		/* New methods: */
		void saveStreams(const std::string& saveFileName); // Prepares the renderer to save streams to a pair of files of the given name
		};
	
	class SynchedRenderer:public Renderer // Class to render 3D video from a time-synchronized 3D video stream file
		{
		/* Elements: */
		private:
		IO::FilePtr colorFile; // Pointer to the file containing the color stream
		Kinect::FrameReader* colorReader; // Reader for the color stream file
		IO::FilePtr depthFile; // Pointer to the file containing the depth stream
		Kinect::FrameReader* depthReader; // Reader for the depth stream file
		
		Threads::Thread colorReaderThread; // Thread to read color frames from the color stream file
		Threads::Thread depthReaderThread; // Thread to read depth frames from the depth stream file
		bool started; // Flag whether streaming has been started
		
		double timeStamp; // Current display time stamp
		static const int numQueueSlots=3; // Number of frames that can be read ahead from the input files
		Threads::Mutex frameQueueMutex; // Mutex protecting the frame queue state and the condition variables
		Threads::Cond frameQueuesEmptyCond; // Condition variable to wait when both frame queues are empty
		
		Threads::Cond colorFrameQueueFullCond; // Condition variable to wait when the color frame queue is full
		int numColorFrames; // Number of color frames currently in the queue
		Kinect::FrameBuffer colorFrames[numQueueSlots]; // Queue of two color frames whose time stamps bracket the display time stamp
		Kinect::FrameBuffer nextColorFrame; // The next color frame
		int mostRecentColorFrame; // Index of the queue slot containing the most recent color frame
		
		Threads::Cond depthFrameQueueFullCond; // Condition variable to wait when the depth frame queue is full
		int numDepthFrames; // Number of depth frames currently in the queue
		#if KINECT_USE_SHADERPROJECTOR
		Kinect::FrameBuffer depthFrames[numQueueSlots]; // Queue of two depth frames whose time stamps bracket the display time stamp
		#else
		Kinect::MeshBuffer depthFrames[numQueueSlots]; // Queue of two depth frames whose time stamps bracket the display time stamp
		#endif
		int mostRecentDepthFrame; // Index of the queue slot containing the most recent depth frame
		#if KINECT_USE_SHADERPROJECTOR
		Kinect::FrameBuffer nextDepthFrame; // The next depth frame
		#else
		Kinect::MeshBuffer nextDepthFrame; // The next depth frame
		#endif
		
		/* Private methods: */
		void* colorReaderThreadMethod(void);
		void* depthReaderThreadMethod(void);
		
		/* Constructors and destructors: */
		public:
		SynchedRenderer(const std::string& fileName); // Creates a renderer for the given 3D video stream file
		virtual ~SynchedRenderer(void);
		
		/* Methods from Renderer: */
		virtual void startStreaming(void);
		virtual void frame(double newTimeStamp);
		};
	
	/* Elements: */
	private:
	static KinectViewerFactory* factory; // Pointer to the class' factory object
	
	USB::Context* usbContext; // USB device context
	bool navigational; // Flag whether to render 3D video in navigational space
	std::vector<Renderer*> renderers; // List of 3D video stream renderers
	bool synched; // Flag if the vislet has to stay synched to recorded or played video streams even while disabled
	bool firstEnable; // Flag to indicate the first time the vislet is enabled at start-up
	bool enabled; // Flag whether the vislet is enabled; class cannot use the active flag
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int numArguments,const char* const arguments[]);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::VisletFactory* getFactory(void) const;
	virtual void disable(void);
	virtual void enable(void);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
