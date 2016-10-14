/***********************************************************************
CameraRealSense - Class representing an Intel RealSense camera.
Copyright (c) 2016 Oliver Kreylos

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

#ifndef KINECT_CAMERAREALSENSE_INCLUDED
#define KINECT_CAMERAREALSENSE_INCLUDED

#include <Misc/SizedTypes.h>
#include <Threads/Thread.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/DropdownBox.h>
#include <Kinect/DirectFrameSource.h>

/* Forward declarations: */
struct rs_device;
namespace Misc {
template <class TargetParam>
class Autopointer;
}
namespace Kinect {
class LibRealSenseContext;
typedef Misc::Autopointer<LibRealSenseContext> LibRealSenseContextPtr;
}

namespace Kinect {

class CameraRealSense:public DirectFrameSource
	{
	/* Embedded classes: */
	public:
	typedef Misc::UInt16 RSDepthPixel; // Type for depth values received from RealSense depth camera
	
	/* Elements: */
	private:
	LibRealSenseContextPtr context; // Pointer to a librealsense context shared by all active RealSense cameras
	struct rs_device* device; // The low-level librealsense device representing this camera
	unsigned int frameSizes[2][2]; // Frame sizes of the color and depth cameras, respectively
	int frameRates[2]; // Frame rates in Hz of the color and depth cameras, respectively
	bool streamsEnabled[2]; // Flags if the camera's color and depth streams are currently enabled
	RSDepthPixel zRange[2]; // Quantization interval for raw z values returned from RealSense depth camera
	DepthPixel dMax; // Maximum valid depth pixel reported at FrameSource interface
	unsigned int a,b; // Coefficients for the depth quantization formula d=b-(a/z)
	volatile bool runStreamingThread; // Flag to keep the background streaming thread running
	Threads::Thread streamingThread; // Background thread reading frames from the RealSense camera and dispatching streaming callbacks
	StreamingCallback* colorStreamingCallback; // Callback called when a new color frame arrives
	StreamingCallback* depthStreamingCallback; // Callback called when a new depth frame arrives
	
	/* Private methods: */
	void initialize(void); // Initializes the RealSense camera; called from constructors
	void setColorStreamState(bool enable); // Enables or disables the color stream; uses currently configured frame size and frame rate when enabling
	void setDepthStreamState(bool enable); // Enables or disables the depth stream; uses currently configured frame size and frame rate when enabling
	void* streamingThreadMethod(void); // Method implementing the background streaming thread
	void irEmitterEnabledToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void irGainSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void irExposureAutoToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void irExposureSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void qualityMenuValueChangedCallback(GLMotif::DropdownBox::ValueChangedCallbackData* cbData);
	
	/* Constructors and destructors: */
	public:
	static size_t getNumDevices(void); // Returns the number of RealSense cameras on the local host's bus
	CameraRealSense(size_t index =0); // Opens the index-th RealSense camera device
	CameraRealSense(const char* serialNumber); // Opens the RealSense camera with the given serial number
	virtual ~CameraRealSense(void); // Destroys the camera
	
	/* Methods from FrameSource: */
	virtual DepthCorrection* getDepthCorrectionParameters(void);
	virtual IntrinsicParameters getIntrinsicParameters(void);
	virtual const unsigned int* getActualFrameSize(int sensor) const;
	virtual void startStreaming(StreamingCallback* newColorStreamingCallback,StreamingCallback* newDepthStreamingCallback);
	virtual void stopStreaming(void);
	
	/* Methods from DirectFrameSource: */
	virtual std::string getSerialNumber(void);
	virtual void configure(Misc::ConfigurationFileSection& configFileSection);
	virtual void buildSettingsDialog(GLMotif::RowColumn* settingsDialog);
	
	/* New methods: */
	void setFrameSize(int camera,unsigned int newFrameWidth,unsigned int newFrameHeight); // Sets the frame size of the color or depth camera for the next streaming operation
	const unsigned int* getFrameSize(int camera) const // Returns the frame size of the color or depth camera
		{
		return frameSizes[camera];
		}
	void setFrameRate(int camera,int frameRate); // Sets the frame rate in Hz of the color or depth camera for the next streaming operation
	int getFrameRate(int camera) const // Returns the frame rate in Hz of the color or depth camera
		{
		return frameRates[camera];
		}
	void setZRange(RSDepthPixel zMin,RSDepthPixel zMax); // Sets the range of valid z values in mm for depth quantization
	};

}

#endif
