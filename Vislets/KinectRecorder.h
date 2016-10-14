/***********************************************************************
KinectRecorder - Vislet to capture and save 3D video from one or more
Kinect devices.
Copyright (c) 2011-2016 Oliver Kreylos

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

#ifndef VISLETS_KINECTRECORDER_INCLUDED
#define VISLETS_KINECTRECORDER_INCLUDED

#include <string>
#include <vector>
#include <Sound/SoundDataFormat.h>
#include <Kinect/Camera.h>
#include <Vrui/Vislet.h>

/* Forward declarations: */
namespace Sound {
class SoundRecorder;
}
namespace Kinect {
class FrameBuffer;
class FrameSaver;
}

class KinectRecorder;

class KinectRecorderFactory:public Vrui::VisletFactory
	{
	friend class KinectRecorder;
	
	/* Embedded classes: */
	private:
	struct KinectConfig // Structure containing configuration data for a Kinect device
		{
		/* Elements: */
		public:
		int nodeIndex; // Index of Vrui cluster node to which the Kinect device is connected
		std::string deviceSerialNumber; // Serial number of Kinect device
		bool highResolution; // Flag whether to capture high-resolution color images
		std::string saveFileNamePrefix; // Prefix for recorded camera streams
		std::string backgroundFileName; // Prefix for pre-recorded background images
		unsigned int captureBackgroundFrames; // Number of background frames to capture for background removal
		unsigned int maxDepth; // Depth cutoff value for background removal
		int backgroundRemovalFuzz; // Fuzz value for background removal
		};
	
	struct SoundConfig // Structure containing configuration data for sound recording
		{
		/* Elements: */
		public:
		int nodeIndex; // Index of Vrui cluster node from which to record sound
		std::string soundDeviceName; // Name of device from which to record sound
		Sound::SoundDataFormat soundFormat; // Sound recording format
		std::string soundFileName; // File name for captured sound
		};
	
	/* Elements: */
	private:
	std::vector<KinectConfig> kinectConfigs; // List of Kinect device configuration data structures
	std::vector<SoundConfig> soundConfigs; // List of sound device configuration data structures
	
	/* Constructors and destructors: */
	public:
	KinectRecorderFactory(Vrui::VisletManager& visletManager);
	virtual ~KinectRecorderFactory(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::Vislet* createVislet(int numVisletArguments,const char* const visletArguments[]) const;
	virtual void destroyVislet(Vrui::Vislet* vislet) const;
	};

class KinectRecorder:public Vrui::Vislet
	{
	friend class KinectRecorderFactory;
	
	/* Embedded classes: */
	private:
	class KinectStreamer // Helper class to stream 3D video data from a Kinect camera to a pair of time-stamped files
		{
		/* Elements: */
		public:
		Kinect::Camera camera; // The Kinect camera from which to receive depth and color streams
		Kinect::FrameSaver* frameSaver; // Pointer to helper object saving depth and color frames received from the Kinect
		
		/* Constructors and destructors: */
		public:
		KinectStreamer(const KinectRecorderFactory::KinectConfig& config); // Creates a streamer for the Kinect camera on the given USB device
		~KinectStreamer(void); // Destroys the streamer
		
		/* Methods: */
		Kinect::Camera& getCamera(void) // Returns a reference to the streamer's camera
			{
			return camera;
			}
		void startStreaming(const Kinect::FrameSource::Time& timeBase); // Begins streaming from the Kinect camera
		};
	
	/* Elements: */
	private:
	static KinectRecorderFactory* factory; // Pointer to the class' factory object
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
	Sound::SoundRecorder* soundRecorder; // Pointer to optional sound recorder
	bool firstEnable; // Flag to indicate the first time the vislet is enabled at start-up
	
	/* Constructors and destructors: */
	public:
	KinectRecorder(int numArguments,const char* const arguments[]);
	virtual ~KinectRecorder(void);
	
	/* Methods from Vrui::Vislet: */
	virtual Vrui::VisletFactory* getFactory(void) const;
	virtual void enable(void);
	};

#endif
