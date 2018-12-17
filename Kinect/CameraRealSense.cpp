/***********************************************************************
CameraRealSense - Class representing an Intel RealSense camera.
Copyright (c) 2016-2018 Oliver Kreylos

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

#include <string.h>
#include <librealsense/rs.h>
#include <librealsense/rsutil.h>
#include <string>
#include <stdexcept>
#include <Misc/ThrowStdErr.h>
#include <Misc/MessageLogger.h>
#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ArrayValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/Margin.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Label.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Internal/LibRealSenseContext.h>

// DEBUGGING
#include <iostream>

namespace Kinect {

/********************************
Methods of class CameraRealSense:
********************************/

void CameraRealSense::initialize(void)
	{
	/* Set default frame sizes and frame rates: */
	frameSizes[0][0]=640; // 1920;
	frameSizes[0][1]=480; // 1080;
	frameSizes[1][0]=640;
	frameSizes[1][1]=480;
	
	frameRates[0]=30;
	frameRates[1]=30;
	
	/* Initialize depth quantization formula: */
	dMax=FrameSource::invalidDepth-1;
	setZRange(300U,4000U);
	
	/* Disable both streams: */
	streamsEnabled[0]=false;
	streamsEnabled[1]=false;
	}

void CameraRealSense::setColorStreamState(bool enable)
	{
	/* Check if the stream state needs to be changed: */
	if(streamsEnabled[0]!=enable)
		{
		rs_error* error=0;
		if(enable)
			{
			/* Enable the color stream with current parameters: */
			rs_enable_stream(device,RS_STREAM_COLOR,frameSizes[0][0],frameSizes[0][1],RS_FORMAT_RGB8,frameRates[0],&error);
			}
		else
			{
			/* Disable the depth stream: */
			rs_disable_stream(device,RS_STREAM_COLOR,&error);
			}
		
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::setColorStreamState: Error %s while setting stream state",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		
		/* Set the stream state: */
		streamsEnabled[0]=enable;
		}
	}

void CameraRealSense::setDepthStreamState(bool enable)
	{
	/* Check if the stream state needs to be changed: */
	if(streamsEnabled[1]!=enable)
		{
		rs_error* error=0;
		if(enable)
			{
			/* Enable the depth stream with current parameters: */
			rs_enable_stream(device,RS_STREAM_DEPTH,frameSizes[1][0],frameSizes[1][1],RS_FORMAT_Z16,frameRates[1],&error);
			}
		else
			{
			/* Disable the depth stream: */
			rs_disable_stream(device,RS_STREAM_DEPTH,&error);
			}
		
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::setDepthStreamState: Error %s while setting stream state",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		
		/* Set the stream state: */
		streamsEnabled[1]=enable;
		}
	}

namespace {

void handleStreamingError(rs_error* error)
	{
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::streamingThreadMethod: Error %s while receiving frame",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	}

}

void* CameraRealSense::streamingThreadMethod(void)
	{
	try
		{
		while(runStreamingThread)
			{
			/* Wait for the next frame (depth and/or color image) from the RealSense device: */
			rs_error* error=0;
			rs_wait_for_frames(device,&error);
			handleStreamingError(error);
			
			/* Sample the timer: */
			Time now;
			
			/*************************************************************
			This is where we would synchronize clocks to account for
			random OS delays, subtract expected hardware latency, etc. pp.
			*************************************************************/
			
			/* Assign a time stamp to both frames: */
			double timeStamp=double(now-timeBase);
			
			#if 0
			
			// DEBUGGING
			double depthStamp=rs_get_frame_timestamp(device,RS_STREAM_DEPTH,0);
			double colorStamp=rs_get_frame_timestamp(device,RS_STREAM_COLOR,0);
			std::cout<<timeStamp*1000.0<<", "<<depthStamp<<", "<<colorStamp<<", "<<timeStamp*1000.0-depthStamp<<", "<<timeStamp*1000.0-colorStamp<<std::endl;
			
			#endif
			
			if(streamsEnabled[1])
				{
				/* Read the most recent depth frame: */
				const RSDepthPixel* sRowPtr=static_cast<const RSDepthPixel*>(rs_get_frame_data(device,RS_STREAM_DEPTH,&error));
				sRowPtr+=(frameSizes[1][1]-1)*frameSizes[1][0];
				handleStreamingError(error);
				
				/* Allocate a frame buffer and quantize and flip the depth frame: */
				FrameBuffer depthFrame(frameSizes[1][0],frameSizes[1][1],frameSizes[1][1]*frameSizes[1][0]*sizeof(FrameSource::DepthPixel));
				depthFrame.timeStamp=timeStamp;
				FrameSource::DepthPixel* dPtr=depthFrame.getData<FrameSource::DepthPixel>();
				for(unsigned int y=0;y<frameSizes[1][1];++y,sRowPtr-=frameSizes[1][0])
					{
					const RSDepthPixel* sPtr=sRowPtr;
					for(unsigned int x=0;x<frameSizes[1][0];++x,++sPtr,++dPtr)
						{
						if(*sPtr<zRange[0]||*sPtr>zRange[1])
							*dPtr=FrameSource::invalidDepth;
						else
							*dPtr=FrameSource::DepthPixel(b-a/(unsigned int)*sPtr);
						}
					}
				
				/* Let the base class do its frame processing: */
				processDepthFrameBackground(depthFrame);
				
				/* Call the streaming callback: */
				(*depthStreamingCallback)(depthFrame);
				}
			
			if(streamsEnabled[0])
				{
				/* Read the most recent color frame: */
				const FrameSource::ColorPixel* sRowPtr=static_cast<const FrameSource::ColorPixel*>(rs_get_frame_data(device,RS_STREAM_COLOR,&error));
				sRowPtr+=(frameSizes[0][1]-1)*frameSizes[0][0];
				handleStreamingError(error);
				
				/* Allocate a frame buffer and flip the color frame: */
				FrameBuffer colorFrame(frameSizes[0][0],frameSizes[0][1],frameSizes[0][1]*frameSizes[0][0]*sizeof(FrameSource::ColorPixel));
				colorFrame.timeStamp=timeStamp;
				FrameSource::ColorPixel* dRowPtr=colorFrame.getData<FrameSource::ColorPixel>();
				for(unsigned int y=0;y<frameSizes[0][1];++y,sRowPtr-=frameSizes[0][0],dRowPtr+=frameSizes[0][0])
					memcpy(dRowPtr,sRowPtr,frameSizes[0][0]*sizeof(FrameSource::ColorPixel));
				
				/* Call the streaming callback: */
				(*colorStreamingCallback)(colorFrame);
				}
			}
		}
	catch(const std::runtime_error& err)
		{
		Misc::formattedUserError("Kinect::CameraRealSense::streamingThreadMethod: Terminating streaming thread due to exception %s",err.what());
		}
	
	return 0;
	}

void CameraRealSense::irEmitterEnabledToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	rs_set_device_option(device,RS_OPTION_R200_EMITTER_ENABLED,cbData->set?1.0:0.0,0);
	}

void CameraRealSense::irGainSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	rs_set_device_option(device,RS_OPTION_R200_LR_GAIN,cbData->value,0);
	}

void CameraRealSense::irExposureAutoToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	rs_error* error=0;
	rs_set_device_option(device,RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,cbData->set?1.0:0.0,&error);
	if(error==0)
		{
		/* Update the exposure slider: */
		GLMotif::TextFieldSlider* irExposureSlider=dynamic_cast<GLMotif::TextFieldSlider*>(cbData->toggle->getParent()->findChild("IRExposureSlider"));
		if(irExposureSlider!=0)
			{
			/* Disable or enable the exposure slider: */
			irExposureSlider->setEnabled(!cbData->set);
			
			if(!cbData->set)
				{
				/* Query the current exposure value: */
				irExposureSlider->setValue(rs_get_device_option(device,RS_OPTION_R200_LR_EXPOSURE,0));
				}
			}
		}
	else
		rs_free_error(error);
	}

void CameraRealSense::irExposureSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	rs_set_device_option(device,RS_OPTION_R200_LR_EXPOSURE,cbData->value,0);
	}

void CameraRealSense::qualityMenuValueChangedCallback(GLMotif::DropdownBox::ValueChangedCallbackData* cbData)
	{
	rs_apply_depth_control_preset(device,cbData->newSelectedItem);
	}

size_t CameraRealSense::getNumDevices(void)
	{
	/* Acquire a librealsense context: */
	LibRealSenseContextPtr context=LibRealSenseContext::acquireContext();
	
	/* Return the number of RealSense cameras: */
	return context->getNumDevices();
	}

CameraRealSense::CameraRealSense(size_t index)
	:context(LibRealSenseContext::acquireContext()),
	 device(0),
	 runStreamingThread(false),
	 colorStreamingCallback(0),depthStreamingCallback(0)
	{
	/* Check if there are enough connected RealSense cameras: */
	size_t numDevices=size_t(context->getNumDevices());
	if(index>=numDevices)
		Misc::throwStdErr("Kinect::CameraRealSense: Cannot open RealSense camera with index %u; only %u RealSense devices present",(unsigned int)index,(unsigned int)numDevices);
	
	/* Open the camera device: */
	device=context->getDevice(int(index));
	
	/* Initialize the camera: */
	initialize();
	}

CameraRealSense::CameraRealSense(const char* serialNumber)
	:context(LibRealSenseContext::acquireContext()),
	 device(0),
	 runStreamingThread(false),
	 colorStreamingCallback(0),depthStreamingCallback(0)
	{
	/* Check the serial numbers of all RealSense cameras connected to the local context: */
	int numDevices=context->getNumDevices();
	for(int i=0;i<numDevices;++i)
		{
		/* Open the device and query its serial number: */
		rs_device* testDevice=context->getDevice(i);
		rs_error* error=0;
		const char* testSerial=rs_get_device_serial(testDevice,&error);
		if(error!=0)
			rs_free_error(error);
		else if(strcmp(serialNumber,testSerial)==0)
			{
			/* Use this device: */
			device=testDevice;
			break;
			}
		}
	if(device==0)
		Misc::throwStdErr("Kinect::CameraRealSense: No RealSense camera with serial number %s present",serialNumber);
	
	/* Initialize the camera: */
	initialize();
	}

CameraRealSense::~CameraRealSense(void)
	{
	/* Stop streaming, just in case: */
	stopStreaming();
	
	/* Disable the camera's streams (no other way to close a camera): */
	setColorStreamState(false);
	setDepthStreamState(false);
	}

FrameSource::DepthCorrection* CameraRealSense::getDepthCorrectionParameters(void)
	{
	/* Don't have 'em, don't need 'em: */
	return 0;
	}

FrameSource::IntrinsicParameters CameraRealSense::getIntrinsicParameters(void)
	{
	/* We need to enable both streams to calculate intrinsic parameters: */
	setColorStreamState(true);
	setDepthStreamState(true);
	
	/* Query the camera's intrinsic parameters: */
	rs_error* error=0;
	rs_intrinsics colorIntrinsics,depthIntrinsics;
	if(error==0)
		rs_get_stream_intrinsics(device,RS_STREAM_COLOR,&colorIntrinsics,&error);
	if(error==0)
		rs_get_stream_intrinsics(device,RS_STREAM_DEPTH,&depthIntrinsics,&error);
	rs_extrinsics extrinsics; // Still an intrinsic parameter
	if(error==0)
		rs_get_device_extrinsics(device,RS_STREAM_DEPTH,RS_STREAM_COLOR,&extrinsics,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::getIntrinsicParameters: Error %s while querying camera intrinsics",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	
	IntrinsicParameters result;
	
	typedef FrameSource::IntrinsicParameters::PTransform PTransform;
	
	/* Calculate the un-projection matrix from 3D depth image space into 3D camera space: */
	PTransform::Matrix& dum=result.depthProjection.getMatrix();
	dum=PTransform::Matrix::zero;
	dum(0,0)=-1.0/double(depthIntrinsics.fx);
	dum(0,3)=double(depthIntrinsics.ppx)/double(depthIntrinsics.fx);
	dum(1,1)=-1.0/double(depthIntrinsics.fy);
	dum(1,3)=(double(depthIntrinsics.height)-double(depthIntrinsics.ppy))/double(depthIntrinsics.fy);
	dum(2,3)=-1.0;
	dum(3,2)=-1.0/double(a);
	dum(3,3)=double(b)/double(a);
	
	/* Scale the depth unprojection matrix to cm: */
	result.depthProjection.leftMultiply(PTransform::scale(0.1));
	
	/* Calculate the texture projection matrix from 3D camera space into 2D color camera texture space: */
	PTransform::Matrix& cpm=result.colorProjection.getMatrix();
	cpm=PTransform::Matrix::zero;
	cpm(0,0)=double(colorIntrinsics.fx)/double(colorIntrinsics.width);
	cpm(0,2)=double(colorIntrinsics.ppx)/double(colorIntrinsics.width);
	cpm(1,1)=-double(colorIntrinsics.fy)/double(colorIntrinsics.height);
	cpm(1,2)=(double(colorIntrinsics.height)-double(colorIntrinsics.ppy))/double(colorIntrinsics.height);
	cpm(2,3)=1.0;
	cpm(3,2)=1.0;
	
	/* Calculate an affine transformation from 3D depth camera space into 3D color camera space: */
	PTransform extrinsicTransform=PTransform::identity;
	PTransform::Matrix& etm=extrinsicTransform.getMatrix();
	for(int i=0;i<3;++i)
		{
		for(int j=0;j<3;++j)
			etm(i,j)=double(extrinsics.rotation[i+j*3]);
		etm(i,3)=double(extrinsics.translation[i]);
		}
	result.colorProjection*=extrinsicTransform;
	
	/* Scale 3D depth camera space from cm to m: */
	result.colorProjection*=PTransform::scale(PTransform::Scale(-0.01,0.01,-0.01));
	
	/* Concatenate the depth un-projection matrix to transform directly from depth image space to color image space: */
	result.colorProjection*=result.depthProjection;
	
	return result;
	}

const unsigned int* CameraRealSense::getActualFrameSize(int sensor) const
	{
	return frameSizes[sensor];
	}

void CameraRealSense::startStreaming(FrameSource::StreamingCallback* newColorStreamingCallback,FrameSource::StreamingCallback* newDepthStreamingCallback)
	{
	/* Throw an exception if already streaming: */
	if(runStreamingThread)
		throw std::runtime_error("Kinect::CameraRealSense::startStreaming: RealSense device is already streaming");
	
	/* Remember the provided callback functions: */
	colorStreamingCallback=newColorStreamingCallback;
	depthStreamingCallback=newDepthStreamingCallback;
	
	try
		{
		/* Enable the requested camera streams: */
		setColorStreamState(colorStreamingCallback!=0);
		setDepthStreamState(depthStreamingCallback!=0);
		
		/* Start streaming: */
		rs_error* error=0;
		rs_start_device(device,&error);
		if(error!=0)
			{
			/* Throw an exception: */
			std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::startStreaming: Error %s while starting device",rs_get_error_message(error)));
			rs_free_error(error);
			throw exception;
			}
		
		/* Start the background streaming thread: */
		runStreamingThread=true;
		streamingThread.start(this,&CameraRealSense::streamingThreadMethod);
		}
	catch(...)
		{
		/* Clean up: */
		delete colorStreamingCallback;
		colorStreamingCallback=0;
		delete depthStreamingCallback;
		depthStreamingCallback=0;
		
		/* Re-throw the exception: */
		throw;
		}
	}

void CameraRealSense::stopStreaming(void)
	{
	/* Bail out if not actually streaming: */
	if(!runStreamingThread)
		return;
	
	/* Stop the background streaming thread: */
	runStreamingThread=false;
	streamingThread.join();
	
	/* Delete the callback functions: */
	delete colorStreamingCallback;
	colorStreamingCallback=0;
	delete depthStreamingCallback;
	depthStreamingCallback=0;
	
	/* Stop streaming: */
	rs_error* error=0;
	rs_stop_device(device,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::stopStreaming: Error %s while stopping device",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	}

std::string CameraRealSense::getSerialNumber(void)
	{
	/* Combine the RealSense prefix and the device's serial number: */
	std::string result="RS-";
	rs_error* error=0;
	const char* serialNumber=rs_get_device_serial(device,&error);
	if(error!=0)
		{
		/* Throw an exception: */
		std::runtime_error exception(Misc::printStdErrMsg("Kinect::CameraRealSense::getSerialNumber: Error %s while querying device's serial number",rs_get_error_message(error)));
		rs_free_error(error);
		throw exception;
		}
	result.append(serialNumber);
	
	return result;
	}

void CameraRealSense::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Call the base class method: */
	DirectFrameSource::configure(configFileSection);
	
	/* Select the color frame size and frame rate: */
	if(configFileSection.hasTag("./colorFrameRate"))
		setFrameRate(COLOR,configFileSection.retrieveValue<int>("./colorFrameRate"));
	if(configFileSection.hasTag("./colorFrameSize"))
		{
		Misc::FixedArray<unsigned int,2> colorFrameSize=configFileSection.retrieveValue<Misc::FixedArray<unsigned int,2> >("./colorFrameSize");
		setFrameSize(COLOR,colorFrameSize[0],colorFrameSize[1]);
		}
	
	/* Select the depth frame size and frame rate: */
	if(configFileSection.hasTag("./depthFrameRate"))
		setFrameRate(DEPTH,configFileSection.retrieveValue<int>("./depthFrameRate"));
	if(configFileSection.hasTag("./depthFrameSize"))
		{
		Misc::FixedArray<unsigned int,2> depthFrameSize=configFileSection.retrieveValue<Misc::FixedArray<unsigned int,2> >("./depthFrameSize");
		setFrameSize(DEPTH,depthFrameSize[0],depthFrameSize[1]);
		}
	
	/* Configure the Z value range for custom quantization: */
	if(configFileSection.hasTag("./depthValueRange"))
		{
		Misc::FixedArray<RSDepthPixel,2> depthValueRange=configFileSection.retrieveValue<Misc::FixedArray<RSDepthPixel,2> >("./depthValueRange");
		setZRange(depthValueRange[0],depthValueRange[1]);
		}
	
	/* Configure the IR emitter and cameras: */
	if(configFileSection.hasTag("./irEmitterEnabled"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_EMITTER_ENABLED,configFileSection.retrieveValue<bool>("./irEmitterEnabled")?1.0:0.0,0);
		}
	if(configFileSection.hasTag("./irGain"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_GAIN,configFileSection.retrieveValue<double>("./irGain"),0);
		}
	if(configFileSection.hasTag("./irExposureAuto"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,configFileSection.retrieveValue<bool>("./irExposureAuto")?1.0:0.0,0);
		}
	if(configFileSection.hasTag("./irExposure"))
		{
		setDepthStreamState(true);
		rs_set_device_option(device,RS_OPTION_R200_LR_EXPOSURE,configFileSection.retrieveValue<double>("./irExposure"),0);
		}
	if(configFileSection.hasTag("./depthControlPreset"))
		{
		setDepthStreamState(true);
		static const char* presetNames[]=
			{
			"Default",
			"Off",
			"Low",
			"Medium",
			"Optimized",
			"High",
			0
			};
		std::string depthControlPreset=configFileSection.retrieveString("./depthControlPreset");
		int i;
		for(i=0;presetNames[i]!=0;++i)
			if(strcasecmp(depthControlPreset.c_str(),presetNames[i])==0)
				{
				rs_apply_depth_control_preset(device,i);
				break;
				}
		if(presetNames[i]==0)
			Misc::throwStdErr("Kinect::CameraRealSense::configure: Invalid depth control preset \"%s\"",depthControlPreset.c_str());
		}
	}

void CameraRealSense::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	/* Create the base class settings dialog: */
	DirectFrameSource::buildSettingsDialog(settingsDialog);
	
	const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	
	double optionMin,optionMax,optionStep;
	
	/* Create widgets to turn the IR emitter on/off and set IR cameras' gain: */
	GLMotif::RowColumn* irEmitterGainBox=new GLMotif::RowColumn("IREmitterGainBox",settingsDialog,false);
	irEmitterGainBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	irEmitterGainBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	irEmitterGainBox->setNumMinorWidgets(1);
	
	GLMotif::ToggleButton* irEmitterEnabledToggle=new GLMotif::ToggleButton("IREmitterEnabledToggle",irEmitterGainBox,"IR Emitter");
	irEmitterEnabledToggle->setBorderWidth(0.0f);
	irEmitterEnabledToggle->setBorderType(GLMotif::Widget::PLAIN);
	irEmitterEnabledToggle->setToggle(rs_get_device_option(device,RS_OPTION_R200_EMITTER_ENABLED,0)!=0.0);
	irEmitterEnabledToggle->getValueChangedCallbacks().add(this,&CameraRealSense::irEmitterEnabledToggleCallback);
	
	new GLMotif::Label("IRGainLabel",irEmitterGainBox,"IR Gain");
	
	GLMotif::TextFieldSlider* irGainSlider=new GLMotif::TextFieldSlider("IRGainSlider",irEmitterGainBox,5,ss.fontHeight*5.0f);
	irGainSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	irGainSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	rs_get_device_option_range(device,RS_OPTION_R200_LR_GAIN,&optionMin,&optionMax,&optionStep,0);
	irGainSlider->setValueRange(optionMin,optionMax,optionStep);
	irGainSlider->setValue(rs_get_device_option(device,RS_OPTION_R200_LR_GAIN,0));
	irGainSlider->getValueChangedCallbacks().add(this,&CameraRealSense::irGainSliderCallback);
	
	irEmitterGainBox->manageChild();
	
	/* Create widgets to set the IR cameras' exposure: */
	GLMotif::RowColumn* irExposureBox=new GLMotif::RowColumn("IRExposureBox",settingsDialog,false);
	irExposureBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	irExposureBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	irExposureBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("IRExposureLabel",irExposureBox,"IR Exposure");
	
	GLMotif::ToggleButton* irExposureAutoToggle=new GLMotif::ToggleButton("IRExposureAutoToggle",irExposureBox,"Auto");
	irExposureAutoToggle->setBorderWidth(0.0f);
	irExposureAutoToggle->setBorderType(GLMotif::Widget::PLAIN);
	irExposureAutoToggle->setToggle(rs_get_device_option(device,RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,0)!=0.0);
	irExposureAutoToggle->getValueChangedCallbacks().add(this,&CameraRealSense::irExposureAutoToggleCallback);
	
	GLMotif::TextFieldSlider* irExposureSlider=new GLMotif::TextFieldSlider("IRExposureSlider",irExposureBox,5,ss.fontHeight*5.0f);
	irExposureSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	irExposureSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	rs_get_device_option_range(device,RS_OPTION_R200_LR_EXPOSURE,&optionMin,&optionMax,&optionStep,0);
	irExposureSlider->setValueRange(optionMin,optionMax,optionStep);
	irExposureSlider->setValue(rs_get_device_option(device,RS_OPTION_R200_LR_EXPOSURE,0));
	irExposureSlider->getValueChangedCallbacks().add(this,&CameraRealSense::irExposureSliderCallback);
	
	/* Disable the exposure slider if auto exposure is enabled: */
	if(irExposureAutoToggle->getToggle())
		irExposureSlider->setEnabled(false);
	
	irExposureBox->manageChild();
	
	/* Create a drop-down box to select 3D reconstruction quality presets: */
	GLMotif::Margin* qualityMargin=new GLMotif::Margin("QualityMargin",settingsDialog,false);
	qualityMargin->setAlignment(GLMotif::Alignment::LEFT);
	
	GLMotif::RowColumn* qualityBox=new GLMotif::RowColumn("QualityBox",qualityMargin,false);
	qualityBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	qualityBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	qualityBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("QualityLabel",qualityBox,"3D Outlier Removal");
	
	GLMotif::DropdownBox* qualityMenu=new GLMotif::DropdownBox("QualityMenu",qualityBox,false);
	qualityMenu->addItem("Default");
	qualityMenu->addItem("Off");
	qualityMenu->addItem("Low");
	qualityMenu->addItem("Medium");
	qualityMenu->addItem("Optimized");
	qualityMenu->addItem("High");
	qualityMenu->setSelectedItem(0);
	qualityMenu->getValueChangedCallbacks().add(this,&CameraRealSense::qualityMenuValueChangedCallback);
	qualityMenu->manageChild();
	
	qualityBox->manageChild();
	
	qualityMargin->manageChild();
	}

void CameraRealSense::setFrameSize(int camera,unsigned int newFrameWidth,unsigned int newFrameHeight)
	{
	if(frameSizes[camera][0]!=newFrameWidth||frameSizes[camera][1]!=newFrameHeight)
		{
		/* Disable the changed stream if it has already been enabled: */
		if(camera==0)
			setColorStreamState(false);
		else
			setDepthStreamState(false);
		
		/* Update the stream's frame size: */
		frameSizes[camera][0]=newFrameWidth;
		frameSizes[camera][1]=newFrameHeight;
		}
	}

void CameraRealSense::setFrameRate(int camera,int newFrameRate)
	{
	if(frameRates[camera]!=newFrameRate)
		{
		/* Disable the changed stream if it has already been enabled: */
		if(camera==0)
			setColorStreamState(false);
		else
			setDepthStreamState(false);
		
		/* Update the stream's frame rate: */
		frameRates[camera]=newFrameRate;
		}
	}

void CameraRealSense::setZRange(RSDepthPixel zMin,RSDepthPixel zMax)
	{
	/* Check the z value range: */
	if(zMin>=zMax)
		Misc::throwStdErr("Kinect::CameraRealSense::setZRange: Invalid Z value range [%u, %u]",(unsigned int)zMin,(unsigned int)zMax);
	
	/* Store the new z value range: */
	zRange[0]=zMin;
	zRange[1]=zMax;
	
	/* Update the depth quantization formula: */
	a=((unsigned int)dMax*(unsigned int)zRange[0]*(unsigned int)zRange[1])/(unsigned int)(zRange[1]-zRange[0]);
	b=(unsigned int)dMax+((unsigned int)dMax*(unsigned int)zRange[0])/(unsigned int)(zRange[1]-zRange[0]);
	}

}
