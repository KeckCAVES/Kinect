/***********************************************************************
CameraV2 - Class representing a Kinect v2 camera.
Copyright (c) 2015-2018 Oliver Kreylos

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

#include <libusb-1.0/libusb.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/MessageLogger.h>
#include <IO/File.h>
#include <IO/Directory.h>
#include <USB/DeviceList.h>
#include <USB/TransferPool.h>
#include <Kinect/Internal/Config.h>
#include <Kinect/Internal/KinectV2CommandDispatcher.h>
#include <Kinect/Internal/KinectV2JpegStreamReader.h>
#include <Kinect/Internal/KinectV2DepthStreamReader.h>

namespace Kinect {

/*************************
Methods of class CameraV2:
*************************/

void CameraV2::initialize(void)
	{
	/* Open the camera device: */
	device.open();
	
	/* Get the camera's serial number: */
	serialNumber="V2-";
	serialNumber.append(device.getSerialNumber());
	
	/* Create a command dispatcher: */
	commandDispatcher=new KinectV2CommandDispatcher(device);
	
	/* Set up the Kinect's USB interfaces: */
	commandDispatcher->initInterfaces();
	
	/* Create color and depth stream readers: */
	colorStreamReader=new KinectV2JpegStreamReader(*this);
	depthStreamReader=new KinectV2DepthStreamReader(*this);
	
	/* Download reconstruction parameter tables: */
	commandDispatcher->downloadTables(depthStreamReader);
	
	/* Calculate the depth calculation parameter tables: */
	depthStreamReader->calcXZTables(commandDispatcher->getDepthCameraParams());
	
	/* Initialize frame sizes: */
	frameSizes[0][0]=1920;
	frameSizes[0][1]=1080;
	frameSizes[1][0]=512;
	frameSizes[1][1]=424;
	}

namespace {

/**************
Helper classes:
**************/

class KinectV2CameraMatcher // Class to match a Kinect v2 camera device
	{
	/* Methods: */
	public:
	bool operator()(const libusb_device_descriptor& dd) const
		{
		/* Match Kinect2-for-Windows devices: */
		return dd.idVendor==0x045eU&&dd.idProduct==0x02c4U;
		}
	};

}

size_t CameraV2::getNumDevices(void)
	{
	/* Get the list of devices on all local USB buses: */
	USB::DeviceList deviceList;
	
	/* Return the number of Kinect v2 cameras: */
	return deviceList.getNumDevices(KinectV2CameraMatcher());
	}

CameraV2::CameraV2(libusb_device* sDevice)
	:device(sDevice),
	 commandDispatcher(0),
	 colorStreamReader(0),colorTransfers(0),
	 depthStreamReader(0),depthTransfers(0)
	{
	/* Initialize the camera: */
	initialize();
	}

CameraV2::CameraV2(size_t index)
	:commandDispatcher(0),
	 colorStreamReader(0),colorTransfers(0),
	 depthStreamReader(0),depthTransfers(0)
	{
	/* Get the index-th Kinect v2 camera device from the context: */
	USB::DeviceList deviceList;
	device=deviceList.getDevice(KinectV2CameraMatcher(),index);
	if(!device.isValid())
		Misc::throwStdErr("Kinect::CameraV2::CameraV2: Fewer than %d Kinect v2 camera devices detected",int(index)+1);
	
	/* Initialize the camera: */
	initialize();
	}

CameraV2::CameraV2(const char* serialNumber)
	:commandDispatcher(0),
	 colorStreamReader(0),colorTransfers(0),
	 depthStreamReader(0),depthTransfers(0)
	{
	/* Enumerate all Kinect v2 cameras on the USB bus: */
	USB::DeviceList deviceList;
	
	/* Search for all Kinect cameras: */
	libusb_device* dev=0;
	KinectV2CameraMatcher kcm;
	for(size_t i=0;dev==0&&i<deviceList.getNumDevices();++i)
		{
		/* Check if the device is a Kinect v2 camera: */
		libusb_device_descriptor dd;
		if(kcm(deviceList.getDeviceDescriptor(i,dd)))
			{
			/* Get the device's serial number: */
			USB::Device tDev(deviceList.getDevice(i));
			if(tDev.getSerialNumber()==serialNumber)
				dev=tDev.getDevice();
			}
		}
	if(dev==0)
		Misc::throwStdErr("Kinect::CameraV2::CameraV2: No Kinect v2 camera device with serial number %s found",serialNumber);
	
	/* Initialize the camera: */
	device=dev;
	initialize();
	}

CameraV2::~CameraV2(void)
	{
	/* Stop streaming, just in case: */
	stopStreaming();
	
	/* Release all resources: */
	delete colorStreamReader;
	delete depthStreamReader;
	delete commandDispatcher;
	}

FrameSource::DepthCorrection* CameraV2::getDepthCorrectionParameters(void)
	{
	return 0;
	}

FrameSource::IntrinsicParameters CameraV2::getIntrinsicParameters(void)
	{
	/* Assemble the name of the intrinsic parameter file: */
	std::string intrinsicParameterFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	intrinsicParameterFileName.push_back('/');
	intrinsicParameterFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX);
	intrinsicParameterFileName.push_back('-');
	intrinsicParameterFileName.append(serialNumber);
	intrinsicParameterFileName.append(".dat");
	
	/* Check if a file of the given name exists and is readable: */
	IntrinsicParameters result;
	if(IO::Directory::getCurrent()->getPathType(intrinsicParameterFileName.c_str())==Misc::PATHTYPE_FILE)
		{
		try
			{
			/* Open the parameter file: */
			IO::FilePtr parameterFile(IO::Directory::getCurrent()->openFile(intrinsicParameterFileName.c_str()));
			parameterFile->setEndianness(Misc::LittleEndian);
			
			/* Read lens distortion correction parameters: */
			Misc::Float64 depthLensDistortionParameters[5];
			parameterFile->read(depthLensDistortionParameters,5);
			for(int i=0;i<3;++i)
				result.depthLensDistortion.setKappa(i,depthLensDistortionParameters[i]);
			for(int i=0;i<2;++i)
				result.depthLensDistortion.setRho(i,depthLensDistortionParameters[3+i]);
			
			/* Read the depth unprojection matrix: */
			Misc::Float64 depthMatrix[16];
			parameterFile->read(depthMatrix,4*4);
			result.depthProjection=IntrinsicParameters::PTransform::fromRowMajor(depthMatrix);
			
			/* Read the color projection matrix: */
			Misc::Float64 colorMatrix[16];
			parameterFile->read(colorMatrix,4*4);
			result.colorProjection=IntrinsicParameters::PTransform::fromRowMajor(colorMatrix);
			}
		catch(std::runtime_error err)
			{
			/* Log an error: */
			Misc::formattedConsoleError("Kinect::CameraV2::getIntrinsicParameters: Could not load intrinsic parameter file %s due to exception %s",intrinsicParameterFileName.c_str(),err.what());
			}
		}
	else
		{
		/* Get depth camera parameters from command dispatcher: */
		const KinectV2CommandDispatcher::DepthCameraParams& dcp=commandDispatcher->getDepthCameraParams();
		
		/* Initialize the lens distortion correction formula: */
		result.depthLensDistortion.setKappa(0,dcp.k1);
		result.depthLensDistortion.setKappa(1,dcp.k2);
		result.depthLensDistortion.setKappa(2,dcp.k3);
		result.depthLensDistortion.setRho(0,dcp.p1);
		result.depthLensDistortion.setRho(1,dcp.p2);
		
		/* Initialize the depth unprojection matrix: */
		FrameSource::IntrinsicParameters::PTransform::Matrix& dum=result.depthProjection.getMatrix();
		dum=FrameSource::IntrinsicParameters::PTransform::Matrix::zero;
		dum(0,0)=-1.0/dcp.sx;
		dum(0,3)=dcp.cx/dcp.sx;
		dum(1,1)=1.0/dcp.sy;
		dum(1,3)=-dcp.cy/dcp.sy;
		dum(2,3)=-1.0;
		dum(3,2)=-1.0/depthStreamReader->getA();
		dum(3,3)=depthStreamReader->getB()/depthStreamReader->getA();
		
		/* Scale the depth unprojection matrix to cm: */
		result.depthProjection.leftMultiply(FrameSource::IntrinsicParameters::PTransform::scale(0.1));
		
		/* Initialize the color projection matrix: */
		result.colorProjection=FrameSource::IntrinsicParameters::PTransform::identity;
		}
	
	/* Set projection parameters for the lens distortion corrector: */
	result.depthLensDistortion.setProjection(result.depthProjection);
	
	return result;
	}

const unsigned int* CameraV2::getActualFrameSize(int sensor) const
	{
	/* Return the appropriate frame size: */
	return frameSizes[sensor];
	}

void CameraV2::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	USB::TransferPool::UserTransferCallback* colorTransferCallback=0;
	if(newColorStreamingCallback!=0)
		{
		/* Set up a bulk transfer pool to receive color data: */
		colorTransfers=new USB::TransferPool(50,0x8000); // Allocate 20 extra buffers to prevent underrun
		
		/* Set up the color image processing pipeline: */
		colorTransferCallback=colorStreamReader->startStreaming(colorTransfers,newColorStreamingCallback);
		}
	
	USB::TransferPool::UserTransferCallback* depthTransferCallback=0;
	if(newDepthStreamingCallback!=0)
		{
		/* Set up an isochronous transfer pool to receive depth data: */
		// depthTransfers=new USB::TransferPool(21,8,device.getMaxIsoPacketSize(0x84)); // Allocate 1 extra buffer to prevent underrun
		depthTransfers=new USB::TransferPool(21,8,33792); // Ignore what libusb says
		
		/* Set up the depth image processing pipeline: */
		depthTransferCallback=depthStreamReader->startStreaming(depthTransfers,newDepthStreamingCallback);
		}
	
	if(newColorStreamingCallback!=0||newDepthStreamingCallback!=0)
		{
		/* Start the Kinect's sensors: */
		commandDispatcher->startSensors();
		}
	
	/* Read color and depth data from the Kinect: */
	if(newColorStreamingCallback!=0)
		colorTransfers->submit(device,0x83,30,colorTransferCallback);
	if(newDepthStreamingCallback!=0)
		depthTransfers->submit(device,0x84,20,depthTransferCallback);
	}

void CameraV2::stopStreaming(void)
	{
	/* Cancel all pending USB transfers: */
	if(colorTransfers!=0)
		colorTransfers->cancel();
	if(depthTransfers!=0)
		depthTransfers->cancel();
	
	if(colorTransfers!=0||depthTransfers!=0)
		{
		/* Stop the Kinect's sensors: */
		commandDispatcher->stopSensors();
		}
	
	/* Shut down the color and depth image processing pipelines: */
	if(colorTransfers!=0)
		colorStreamReader->stopStreaming();
	if(depthTransfers!=0)
		depthStreamReader->stopStreaming();
	
	/* Delete the transfer pools: */
	delete colorTransfers;
	colorTransfers=0;
	delete depthTransfers;
	depthTransfers=0;
	}

std::string CameraV2::getSerialNumber(void)
	{
	return serialNumber;
	}

}
