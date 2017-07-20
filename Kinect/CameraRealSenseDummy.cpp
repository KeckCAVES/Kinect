/***********************************************************************
CameraRealSenseDummy - Class to dummy out support for Intel RealSense
cameras.
Copyright (c) 2017 Oliver Kreylos

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

#include <Kinect/CameraRealSense.h>

#include <string>
#include <stdexcept>
#include <Misc/FunctionCalls.h>

namespace Kinect {

/*******************************
Dummy LibRealSenseContext class:
*******************************/

class LibRealSenseContext
	{
	friend class Misc::Autopointer<LibRealSenseContext>;
	
	/* Private methods: */
	void ref(void) // Adds a reference to the context
		{
		}
	void unref(void) // Removes a reference from the context
		{
		}
	};

/********************************
Methods of class CameraRealSense:
********************************/

void CameraRealSense::initialize(void)
	{
	}

void CameraRealSense::setColorStreamState(bool enable)
	{
	}

void CameraRealSense::setDepthStreamState(bool enable)
	{
	}

void* CameraRealSense::streamingThreadMethod(void)
	{
	return 0;
	}

void CameraRealSense::irEmitterEnabledToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	}

void CameraRealSense::irGainSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	}

void CameraRealSense::irExposureAutoToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	}

void CameraRealSense::irExposureSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	}

void CameraRealSense::qualityMenuValueChangedCallback(GLMotif::DropdownBox::ValueChangedCallbackData* cbData)
	{
	}

size_t CameraRealSense::getNumDevices(void)
	{
	/* There are no RealSense cameras: */
	return 0;
	}

CameraRealSense::CameraRealSense(size_t index)
	{
	throw std::runtime_error("Kinect::CameraRealSense: Intel RealSense cameras not supported by Kinect library");
	}

CameraRealSense::CameraRealSense(const char* serialNumber)
	{
	throw std::runtime_error("Kinect::CameraRealSense: Intel RealSense cameras not supported by Kinect library");
	}

CameraRealSense::~CameraRealSense(void)
	{
	}

FrameSource::DepthCorrection* CameraRealSense::getDepthCorrectionParameters(void)
	{
	return 0;
	}

FrameSource::IntrinsicParameters CameraRealSense::getIntrinsicParameters(void)
	{
	return IntrinsicParameters();
	}

const unsigned int* CameraRealSense::getActualFrameSize(int sensor) const
	{
	return frameSizes[sensor];
	}

void CameraRealSense::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	delete newColorStreamingCallback;
	delete newDepthStreamingCallback;
	}

void CameraRealSense::stopStreaming(void)
	{
	}

std::string CameraRealSense::getSerialNumber(void)
	{
	return std::string();
	}

void CameraRealSense::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	DirectFrameSource::configure(configFileSection);
	}

void CameraRealSense::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	}

void CameraRealSense::setFrameSize(int camera,unsigned int newFrameWidth,unsigned int newFrameHeight)
	{
	}

void CameraRealSense::setFrameRate(int camera,int newFrameRate)
	{
	}

void CameraRealSense::setZRange(RSDepthPixel zMin,RSDepthPixel zMax)
	{
	}

}
