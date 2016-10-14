/***********************************************************************
DirectFrameSource - Intermediate class for frame sources that are
directly connected to a camera device.
Copyright (c) 2015-2016 Oliver Kreylos

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

#ifndef KINECT_DIRECTFRAMESOURCE_INCLUDED
#define KINECT_DIRECTFRAMESOURCE_INCLUDED

#include <string>
#include <Misc/SelfDestructPointer.h>
#include <Misc/CallbackData.h>
#include <Misc/CallbackList.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/FileSelectionDialog.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
class ConfigurationFileSection;
}
namespace IO {
class File;
}
namespace GLMotif {
class RowColumn;
class FileSelectionHelper;
}

namespace Kinect {

class DirectFrameSource:public FrameSource
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<DirectFrameSource&> BackgroundCaptureCallback; // Function call type for completion of background capture callback
	
	class CallbackData:public Misc::CallbackData // Base class for camera-related events
		{
		/* Elements: */
		public:
		DirectFrameSource* frameSource; // Pointer to the frame source object that caused the event
		
		/* Constructors and destructors: */
		CallbackData(DirectFrameSource* sFrameSource)
			:
			frameSource(sFrameSource)
			{
			}
		};
	
	class IntrinsicParametersChangedCallbackData:public CallbackData // Class for callback data sent when the camera's intrinsic parameters change in response to user interaction
		{
		/* Constructors and destructors: */
		public:
		IntrinsicParametersChangedCallbackData(DirectFrameSource* sFrameSource)
			:CallbackData(sFrameSource)
			{
			}
		};
	
	/* Elements: */
	private:
	static Misc::SelfDestructPointer<GLMotif::FileSelectionHelper> backgroundSelectionHelper; // Helper object to select background files for loading/saving
	protected:
	DepthPixel* backgroundFrame; // The camera's current background frame
	unsigned int backgroundCaptureNumFrames; // Number of background frames left to capture
	BackgroundCaptureCallback* backgroundCaptureCallback; // Function to call upon completion of background capture
	bool removeBackground; // Flag whether to remove background information during frame processing
	Misc::SInt16 backgroundRemovalFuzz; // Fuzz value for background removal (positive values: more aggressive removal)
	Misc::CallbackList intrinsicParametersChangedCallbacks; // List of callbacks to be called when the camera's intrinsic parameters change
	
	/* Protected methods: */
	void processDepthFrameBackground(FrameBuffer& depthFrame); // Runs a newly-decoded depth frame through background capture and/or removal
	
	/* Private methods: */
	private:
	void removeBackgroundToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData); // Called when user toggles the "remove background" button
	void captureBackgroundCompleteCallback(DirectFrameSource& source,GLMotif::Button* button); // Called when a user-requested background capture finishes
	void captureBackgroundButtonCallback(GLMotif::Button::SelectCallbackData* cbData); // Called when user presses the "capture background" button
	void backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData); // Called when user moves the background max depth slider
	void backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData); // Called when user moves the background removal fuzz slider
	void loadBackgroundCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData); // Called when user requests loading a background file
	void saveBackgroundCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData); // Called when user requests saving the current background image to a file
	
	/* Constructors and destructors: */
	public:
	DirectFrameSource(void);
	virtual ~DirectFrameSource(void);
	
	/* Methods from FrameSource: */
	virtual ExtrinsicParameters getExtrinsicParameters(void);
	
	/* New methods: */
	virtual std::string getSerialNumber(void) =0; // Returns the camera's serial number, unique among all camera device types
	virtual void configure(Misc::ConfigurationFileSection& configFileSection); // Configures the camera device by reading from the given configuration file section
	virtual void buildSettingsDialog(GLMotif::RowColumn* settingsDialog); // Creates a GUI to set runtime-adjustable settings inside the given settings dialog
	Misc::CallbackList& getIntrinsicParametersChangedCallbacks(void) // Returns the list of intrinsic parameter change callbacks
		{
		return intrinsicParametersChangedCallbacks;
		}
	virtual void captureBackground(unsigned int numFrames,bool replace =false,BackgroundCaptureCallback* newBackgroundCaptureCallback =0); // Captures the given number of frames to create a background removal buffer and calls optional callback upon completion
	virtual bool loadDefaultBackground(void); // Loads the default background removal buffer for this camera; returns true if background was loaded
	virtual void loadBackground(const char* fileNamePrefix); // Loads a background removal buffer from a file with the given prefix
	virtual void loadBackground(IO::File& file); // Ditto, from already opened file
	virtual void setMaxDepth(unsigned int newMaxDepth,bool replace =false); // Sets a depth value beyond which all pixels are considered background
	virtual void saveBackground(const char* fileNamePrefix); // Saves the current background frame to a file with the given prefix
	virtual void saveBackground(IO::File& file); // Ditto, into an already opened file
	void setRemoveBackground(bool newRemoveBackground); // Enables or disables background removal
	bool getRemoveBackground(void) const // Returns the current background removal flag
		{
		return removeBackground;
		}
	void setBackgroundRemovalFuzz(int newBackgroundRemovalFuzz); // Sets the fuzz value for background removal
	int getBackgroundRemovalFuzz(void) const // Returns the current background removal fuzz value
		{
		return backgroundRemovalFuzz;
		}
	};

}

#endif
