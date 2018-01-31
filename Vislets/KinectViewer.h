/***********************************************************************
KinectViewer - Vislet to draw 3D reconstructions captured from a Kinect
device in 3D space.
Copyright (c) 2010-2017 Oliver Kreylos

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
#include <Geometry/OrthonormalTransformation.h>
#include <Vrui/Geometry.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/ToolManager.h>
#include <Vrui/Vislet.h>
#include <Kinect/Config.h>
#include <Kinect/FrameSource.h>
#include <Kinect/ProjectorHeader.h>

/* Forward declarations: */

namespace Vrui {
class InputDevice;
}
namespace Kinect {
class FrameBuffer;
#if !KINECT_CONFIG_USE_SHADERPROJECTOR
class MeshBuffer;
#endif
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
	private:
	class Renderer // Base class to render 3D video
		{
		/* Elements: */
		protected:
		Kinect::ProjectorType* projector; // Pointer to the projector of the configured type
		
		/* Constructors and destructors: */
		public:
		Renderer(void)
			:projector(0)
			{
			}
		virtual ~Renderer(void);
		
		/* Methods: */
		Kinect::ProjectorType& getProjector(void) // Returns the renderer's projector
			{
			return *projector;
			}
		void applyPreTransform(const Kinect::FrameSource::ExtrinsicParameters& preTransform); // Applies a pre-transformation to the projector's transformation
		virtual void startStreaming(const Kinect::FrameSource::Time& timeBase) =0; // Starts streaming 3D video frames into the projector
		virtual void frame(double newTimeStamp) =0; // Called once per application frame to update renderer state
		virtual void glRenderAction(GLContextData& contextData) const; // Draws the renderer's current state into the given OpenGL context
		};
	
	class LiveRenderer:public Renderer // Class to render 3D video from a "live" unsynchronized source
		{
		/* Elements: */
		public:
		Kinect::FrameSource* source;
		bool started; // Flag whether streaming has been started
		bool paused; // Flag if the renderer is currently paused
		Kinect::FrameSaver* frameSaver;
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the frame source
		void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the frame source
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		void meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer); // Callback receiving projected meshes from the projector
		#endif
		
		/* Constructors and destructors: */
		LiveRenderer(Kinect::FrameSource* sSource); // Creates a renderer for the given 3D video source and saves streams from source if save file name is non-empty; adopts source object
		virtual ~LiveRenderer(void);
		
		/* Methods from Renderer: */
		virtual void startStreaming(const Kinect::FrameSource::Time& timeBase);
		virtual void frame(double newTimeStamp);
		
		/* New methods: */
		void saveStreams(const std::string& saveFileName); // Prepares the renderer to save streams to a pair of files of the given name
		};
	
	class TrackedRenderer:public LiveRenderer // Class to render 3D video from a "live" source that is attached to a tracked input device
		{
		/* Elements: */
		public:
		Kinect::FrameSource::Time sourceTimeBase; // Time base of the connected frame source
		double latency; // Expected latency of source device
		Vrui::InputDevice* trackingDevice; // Pointer to the tracking device to which the live source is attached
		size_t trackingBufferSize; // Size of ring buffer containing past tracking device positions/orientations
		double* timeStampBuffer; // Ring buffer containing tracking device time stamps
		Vrui::TrackerState* trackingBuffer; // Ring buffer containing past tracking device positions/orientations
		size_t numTrackingBufferEntries; // Number of entries currently in tracking buffer
		size_t tail; // Index behind newest sample in tracking buffer
		double meshTimeStamp; // Time stamp of the triangle mesh currently locked for rendering
		Vrui::TrackerState meshTrackerState; // Tracked device position/orientation to display the triangle mesh currently locked in the projector
		
		/* Constructors and destructors: */
		TrackedRenderer(Kinect::FrameSource* sSource,Vrui::InputDevice* sTrackingDevice); // Creates a renderer for the given 3D video source and tracked input device and saves streams from source if save file name is non-empty; adopts source object
		virtual ~TrackedRenderer(void);
		
		/* Methods from Renderer: */
		virtual void startStreaming(const Kinect::FrameSource::Time& timeBase);
		virtual void frame(double newTimeStamp);
		virtual void glRenderAction(GLContextData& contextData) const;
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
		double timeStampBase; // Application time of first frame, which serves as synchronization point for saved streams
		double colorFrameOffset; // Time offset applied to color frames to fix synchronization
		double depthFrameOffset; // Time offset applied to depth frames to fix synchronization
		
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
		Kinect::FrameBuffer depthFrames[numQueueSlots]; // Queue of two depth frames whose time stamps bracket the display time stamp
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		Kinect::MeshBuffer meshes[numQueueSlots]; // Queue of two meshes whose time stamps bracket the display time stamp
		#endif
		int mostRecentDepthFrame; // Index of the queue slot containing the most recent depth frame
		Kinect::FrameBuffer nextDepthFrame; // The next depth frame
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		Kinect::MeshBuffer nextMesh; // The next mesh
		#endif
		bool newDepth; // Flag if the renderer has a new depth image for the current frame
		
		/* Private methods: */
		void* colorReaderThreadMethod(void);
		void* depthReaderThreadMethod(void);
		
		/* Constructors and destructors: */
		public:
		SynchedRenderer(const std::string& fileName,double sColorFrameOffset,double sDepthFrameOffset); // Creates a renderer for the given 3D video stream file and time offsets
		virtual ~SynchedRenderer(void);
		
		/* Methods from Renderer: */
		virtual void startStreaming(const Kinect::FrameSource::Time& timeBase);
		virtual void frame(double newTimeStamp);
		
		/* New methods: */
		bool haveNewDepth(void) const
			{
			return newDepth;
			}
		};
	
	class TrackedSynchedRenderer:public SynchedRenderer // Class to render 3D video recorded with a tracked camera from a time-synchronized 3D video stream file
		{
		/* Elements: */
		private:
		Vrui::InputDevice* trackingDevice; // Pointer to the tracking device to which the synched source is attached
		Vrui::TrackerState trackingState; // State of the tracking device at the time the current depth frame appeared
		
		/* Constructors and destructors: */
		public:
		TrackedSynchedRenderer(const std::string& fileName,Vrui::InputDevice* sTrackingDevice,double sColorFrameOffset,double sDepthFrameOffset); // Creates a renderer for the given 3D video stream file and tracked input device
		
		/* Methods from Renderer: */
		virtual void frame(double newTimeStamp);
		virtual void glRenderAction(GLContextData& contextData) const;
		};
	
	/* Embedded classes: */
	private:
	class Tool // Mix-in class for tool classes related to KinectViewer vislets
		{
		/* Elements: */
		protected:
		KinectViewer* vislet; // Pointer to the vislet with which this tool is associated
		
		/* Constructors and destructors: */
		public:
		Tool(void) // Default constructor
			:vislet(0)
			{
			}
		
		/* Methods: */
		void setVislet(KinectViewer* newVislet) // Associates the tool with a vislet
			{
			vislet=newVislet;
			}
		};
	
	class PauseViewerTool; // Forward declaration
	typedef Vrui::GenericToolFactory<PauseViewerTool> PauseViewerToolFactory; // Factory class for pausing tools
	
	class PauseViewerTool:public Vrui::Tool,public Tool // Tool class to pause a live 3D video stream
		{
		friend class Vrui::GenericToolFactory<PauseViewerTool>;
		
		/* Elements: */
		private:
		static PauseViewerToolFactory* factory; // Pointer to the tool's factory
		
		/* Constructors and destructors: */
		public:
		static void initClass(void);
		PauseViewerTool(const Vrui::ToolFactory* sFactory,const Vrui::ToolInputAssignment& inputAssignment);
		
		/* Methods: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		};
	
	class MapTextureTool; // Forward declaration
	typedef Vrui::GenericToolFactory<MapTextureTool> MapTextureToolFactory; // Factory class for texture mapping tools
	
	class MapTextureTool:public Vrui::Tool,public Tool // Tool class to switch the texture mapping flag of a projector
		{
		friend class Vrui::GenericToolFactory<MapTextureTool>;
		
		/* Elements: */
		private:
		static MapTextureToolFactory* factory; // Pointer to the tool's factory
		bool mapTexture; // Current texture mapping state
		
		/* Constructors and destructors: */
		public:
		static void initClass(void);
		MapTextureTool(const Vrui::ToolFactory* sFactory,const Vrui::ToolInputAssignment& inputAssignment);
		
		/* Methods: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		};
	
	friend class PauseViewerTool;
	friend class MapTextureTool;
	
	/* Elements: */
	private:
	static KinectViewerFactory* factory; // Pointer to the class' factory object
	
	bool navigational; // Flag whether to render 3D video in navigational space
	std::vector<Renderer*> renderers; // List of 3D video stream renderers
	bool synched; // Flag if the vislet has to stay synched to recorded or played video streams even while disabled
	bool startDisabled; // Flag if the vislet starts in disabled state
	bool firstEnable; // Flag to indicate the first time the vislet is enabled at start-up
	bool enabled; // Flag whether the vislet is enabled; class cannot use the active flag if it needs to stay synched
	bool firstFrame; // Flag to indicate the first time the frame method is called
	bool* windowFlags; // Array of rendering flags for each window defined in the environment
	
	/* Private methods: */
	void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData); // Callback called when a new tool is created
	
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
