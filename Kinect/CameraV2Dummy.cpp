/***********************************************************************
CameraV2Dummy - Class to dummy out support forKinect v2 cameras.
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

#include <Kinect/CameraV2.h>

#include <string>
#include <stdexcept>
#include <Misc/FunctionCalls.h>

namespace Kinect {

/*************************
Methods of class CameraV2:
*************************/

void CameraV2::initialize(void)
	{
	/* Never called */
	}

size_t CameraV2::getNumDevices(void)
	{
	return 0;
	}

CameraV2::CameraV2(libusb_device* sDevice)
	{
	throw std::runtime_error("Kinect::CameraV2: Kinect V2 not supported by Kinect library");
	}

CameraV2::CameraV2(size_t index)
	{
	throw std::runtime_error("Kinect::CameraV2: Kinect V2 not supported by Kinect library");
	}

CameraV2::CameraV2(const char* serialNumber)
	{
	throw std::runtime_error("Kinect::CameraV2: Kinect V2 not supported by Kinect library");
	}

CameraV2::~CameraV2(void)
	{
	}

FrameSource::DepthCorrection* CameraV2::getDepthCorrectionParameters(void)
	{
	return 0;
	}

FrameSource::IntrinsicParameters CameraV2::getIntrinsicParameters(void)
	{
	return IntrinsicParameters();
	}

const unsigned int* CameraV2::getActualFrameSize(int sensor) const
	{
	/* Return the appropriate frame size: */
	return frameSizes[sensor];
	}

void CameraV2::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	delete newColorStreamingCallback;
	delete newDepthStreamingCallback;
	}

void CameraV2::stopStreaming(void)
	{
	}

std::string CameraV2::getSerialNumber(void)
	{
	return std::string();
	}

}
