/***********************************************************************
KinectViewer - Simple application to view 3D reconstructions of color
and depth images captured from a Kinect device.
Copyright (c) 2010-2016 Oliver Kreylos

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

#ifndef KINECTVIEWER_INCLUDED
#define KINECTVIEWER_INCLUDED

#include <vector>
#include <Threads/Mutex.h>
#include <Threads/Spinlock.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/FileSelectionHelper.h>
#include <Vrui/Application.h>
#include <Vrui/GenericToolFactory.h>
#include <Kinect/Config.h>
#if KINECT_CONFIG_FRAMESOURCE_EXTRINSIC_PROJECTIVE
#include <Geometry/ProjectiveTransformation.h>
#else
#include <Geometry/OrthogonalTransformation.h>
#endif
#include <Kinect/FrameSource.h>
#include <Kinect/ProjectorType.h>

/* Forward declarations: */
namespace GLMotif {
class PopupWindow;
class PopupMenu;
class Button;
}
namespace Sound {
class SoundRecorder;
class SoundPlayer;
}
namespace Kinect {
class FrameBuffer;
class DirectFrameSource;
#if !KINECT_CONFIG_USE_SHADERPROJECTOR
class MeshBuffer;
#endif
class FrameSaver;
}
class SphereExtractor;
class SphereExtractorTool;

class KinectViewer:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef Kinect::FrameSource::DepthPixel DepthPixel; // Type for depth frame pixels
	
	class KinectStreamer // Helper class to stream 3D video data from a 3D video frame source to a Kinect projector
		{
		/* Elements: */
		public:
		KinectViewer* application; // Pointer to the application object
		Kinect::FrameSource* source; // Pointer to the 3D video frame source
		Kinect::ProjectorType* projector; // Pointer to the projector of configured type
		Kinect::FrameSource::ExtrinsicParameters savedExtrinsics; // Saved extrinsic parameters of projector
		Threads::Spinlock frameSaverMutex; // Mutex protecting changes to the frame saver object
		Kinect::FrameSaver* frameSaver; // Pointer to a frame saver writing received color and depth frames to a pair of files
		bool enabled; // Flag whether the streamer is currently processing and rendering 3D video frames
		DepthPixel maxDepth; // Maximum depth value for background removal
		Threads::Mutex sphereExtractorMutex; // Mutex protecting the sphere extractor object
		SphereExtractor* sphereExtractor; // Pointer to a sphere extractor used during extrinsic calibration
		GLMotif::PopupWindow* streamerDialog; // Pointer to a dialog window to control this streamer
		GLMotif::Button* captureBackgroundButton; // Button to capture a current background image
		GLMotif::ToggleButton* showStreamerDialogToggle; // Pointer to a toggle button to show/hide the control dialog window
		
		/* Private methods: */
		void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the frame source
		void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the frame source
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		void meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer); // Callback receiving projected meshes from the Kinect projector
		#endif
		void showStreamerDialogCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData); // Callback when the control dialog show/hide button is pressed
		GLMotif::PopupWindow* createStreamerDialog(void); // Creates a dialog box to control parameters of this streamer
		void showFacadeCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void showFromCameraCallback(Misc::CallbackData* cbData);
		#if KINECT_CONFIG_USE_PROJECTOR2
		void mapTextureCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		void illuminateCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		#endif
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		void filterDepthFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
		#endif
		void triangleDepthRangeCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
		void streamerDialogCloseCallback(Misc::CallbackData* cbData);
		
		/* Constructors and destructors: */
		public:
		KinectStreamer(KinectViewer* sApplication,Kinect::FrameSource* sSource); // Creates a streamer for the given 3D video source
		~KinectStreamer(void); // Destroys the streamer
		
		/* Methods: */
		Kinect::FrameSource& getFrameSource(void) // Returns a reference to the streamer's frame source
			{
			return *source;
			}
		void setShowStreamerDialogToggle(GLMotif::ToggleButton* newShowStreamerDialogToggle); // Sets the toggle button to show/hide the control dialog
		void startStreaming(const Kinect::FrameSource::Time& timeBase); // Starts streaming with the given time base
		void setFrameSaver(Kinect::FrameSaver* newFrameSaver); // Sets a new frame saver; if !=0, starts saving frames
		void frame(void); // Called once per Vrui frame to update state
		void display(GLContextData& contextData) const; // Renders the streamer's current state into the given OpenGL context
		};
	
	friend class KinectStreamer;
	friend class SphereExtractorTool;
	
	/* Elements: */
	private:
	Kinect::FrameSource::Time timeBase; // A common time base for all streamers
	std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
	GLMotif::ToggleButton* saveStreamsToggle; // Toggle button to save 3D video streams to files
	IO::DirectoryPtr saveStreamsDirectory; // Last directory used/viewed when saving 3D video streams to files
	GLMotif::FileSelectionDialog* saveStreamsFileSelectionDialog; // Pointer to file selection dialog if user is preparing to save 3D video streams
	Sound::SoundRecorder* soundRecorder; // Recorder to save sound from the default sound source while saving 3D video streams
	Sound::SoundPlayer* soundPlayer; // Player to play back sound from a previously saved 3D video stream
	// Vrui::InputDevice* cameraDevice; // Pointer to the device to which the depth camera is attached
	// MD5MeshAnimator anim; // An animator
	
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	void goToPhysicalCallback(Misc::CallbackData* cbData);
	void alignProjectorsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void saveStreamsCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void saveStreamsOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void saveStreamsCancelCallback(GLMotif::FileSelectionDialog::CancelCallbackData* cbData);
	
	/* Constructors and destructors: */
	public:
	KinectViewer(int& argc,char**& argv);
	virtual ~KinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	};

#endif
