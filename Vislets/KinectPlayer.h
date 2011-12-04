/***********************************************************************
KinectPlayer - Vislet to play back 3D video previously captured from one
or more Kinect devices.
Copyright (c) 2011 Oliver Kreylos

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

#ifndef VISLETS_KINECTPLAYER_INCLUDED
#define VISLETS_KINECTPLAYER_INCLUDED

#include <string>
#include <vector>
#include <IO/File.h>
#include <Threads/Thread.h>
#include <Threads/MutexCond.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Sound/SoundDataFormat.h>
#include <Kinect/FrameBuffer.h>
#include <Vrui/Vislet.h>

/* Forward declarations: */
namespace Sound {
class SoundPlayer;
}
class KinectProjector;

class KinectPlayer;

class KinectPlayerFactory:public Vrui::VisletFactory
	{
	friend class KinectPlayer;
	
	/* Embedded classes: */
	private:
	struct KinectConfig // Structure containing configuration data for a Kinect device
		{
		/* Elements: */
		public:
		std::string deviceSerialNumber; // Serial number of Kinect device
		std::string saveFileNamePrefix; // Prefix for recorded camera streams
		};
	
	struct SoundConfig // Structure containing configuration data for sound recording
		{
		/* Elements: */
		public:
		int nodeIndex; // Index of Vrui cluster node on which to play back sound
		std::string soundFileName; // File name for captured sound
		};
	
	/* Elements: */
	private:
	std::vector<KinectConfig> kinectConfigs; // List of Kinect device configuration data structures
	std::vector<SoundConfig> soundConfigs; // List of sound device configuration data structures
	
	/* Constructors and destructors: */
	public:
	KinectPlayerFactory(Vrui::VisletManager& visletManager);
	virtual ~KinectPlayerFactory(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::Vislet* createVislet(int numVisletArguments,const char* const visletArguments[]) const;
	virtual void destroyVislet(Vrui::Vislet* vislet) const;
	};

class KinectPlayer:public Vrui::Vislet
	{
	friend class KinectPlayerFactory;
	
	/* Embedded classes: */
	private:
	class KinectStreamer // Helper class to play back 3D video data from a pair of time-stamped files
		{
		/* Embedded classes: */
		private:
		typedef Geometry::OrthogonalTransformation<double,3> OGTransform; // Type for facade transformations
		
		/* Elements: */
		IO::FilePtr depthFile; // Pointer to the file containing the depth stream
		Threads::Thread depthDecompressorThread; // Thread to decompress depth frames from the depth file
		IO::FilePtr colorFile; // Pointer to the file containing the color stream
		Threads::Thread colorDecompressorThread; // Thread to decompress color frames from the color file
		OGTransform projectorTransform; // Transformation from projector space to physical space
		KinectProjector* projector; // Projector to render a combined depth/color frame
		Threads::MutexCond timeStampCond; // Condition variable to signal a change in the next time stamp value
		double readAheadTimeStamp; // Time stamp up to which to read ahead in the depth and color files
		Threads::MutexCond frameQueueCond; // Condition variable to signal arrival of a new depth or color frame
		FrameBuffer depthFrames[2]; // The two most recently read depth frames
		int numDepthFrames; // Number of depth frames in queue
		int mostRecentDepthFrame; // Index of the most recently read depth frame
		FrameBuffer nextDepthFrame; // The next depth frame
		FrameBuffer colorFrames[2]; // The two most recently read depth frames
		int numColorFrames; // Number of color frames in queue
		int mostRecentColorFrame; // Index of the most recently read depth frame
		FrameBuffer nextColorFrame; // The next color frame
		
		/* Private methods: */
		void* depthDecompressorThreadMethod(void); // Thread method to read depth frames
		void* colorDecompressorThreadMethod(void); // Thread method to read color frames
		
		/* Constructors and destructors: */
		public:
		KinectStreamer(const KinectPlayerFactory::KinectConfig& config); // Creates a streamer from the given configuration structure
		~KinectStreamer(void); // Destroys the streamer
		
		/* Methods: */
		void updateFrames(double currentTimeStamp); // Updates the streamer's frames for display on the given time stamp
		void glRenderAction(GLContextData& contextData) const; // Renders the current frame
		};
	
	/* Elements: */
	private:
	static KinectPlayerFactory* factory; // Pointer to the class' factory object
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers
	Sound::SoundPlayer* soundPlayer; // Pointer to optional sound player
	bool firstFrame; // Flag indicating the first Vrui frame
	
	/* Constructors and destructors: */
	public:
	KinectPlayer(int numArguments,const char* const arguments[]);
	virtual ~KinectPlayer(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::VisletFactory* getFactory(void) const;
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
