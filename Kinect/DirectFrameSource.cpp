/***********************************************************************
DirectFrameSource - Intermediate class for frame sources that are
directly connected to a camera device.
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

#include <Kinect/DirectFrameSource.h>

#include <Misc/SelfDestructArray.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/MessageLogger.h>
#include <Misc/FunctionCalls.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <IO/File.h>
#include <IO/Directory.h>
#include <Math/Math.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/GeometryValueCoders.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/Margin.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/FileSelectionHelper.h>
#include <Kinect/Internal/Config.h>
#include <Kinect/FrameBuffer.h>

namespace Kinect {

/******************************************
Static elements of class DirectFrameSource:
******************************************/

Misc::SelfDestructPointer<GLMotif::FileSelectionHelper> DirectFrameSource::backgroundSelectionHelper;

/**********************************
Methods of class DirectFrameSource:
**********************************/

void DirectFrameSource::processDepthFrameBackground(FrameBuffer& depthFrame)
	{
	/* Check if a background capture is currently active: */
	if(backgroundCaptureNumFrames>0)
		{
		/* Update the background frame's depth values: */
		int width=depthFrame.getSize(0);
		int height=depthFrame.getSize(1);
		DepthPixel* dfPtr=depthFrame.getData<DepthPixel>();
		DepthPixel* bfPtr=backgroundFrame;
		for(int i=height*width;i>0;--i,++dfPtr,++bfPtr)
			{
			if(*bfPtr>*dfPtr)
				*bfPtr=*dfPtr;
			}
		
		/* Check if this was the last captured background frame: */
		--backgroundCaptureNumFrames;
		if(backgroundCaptureNumFrames==0)
			{
			/*****************************************************************
			Open the background frame to increase reliability in high-slope
			areas:
			*****************************************************************/
			
			/* Open the background frame in the y direction: */
			for(int x=0;x<width;++x)
				{
				DepthPixel* bfPtr=backgroundFrame+x;
				DepthPixel last=bfPtr[0];
				bfPtr[0]=Math::min(bfPtr[0],bfPtr[width]);
				bfPtr+=width;
				for(int y=1;y<height-1;++y,bfPtr+=width)
					{
					DepthPixel next=Math::min(last,Math::min(bfPtr[0],bfPtr[width]));
					last=bfPtr[0];
					bfPtr[0]=next;
					}
				bfPtr[0]=Math::min(last,bfPtr[0]);
				}
			
			/* Open the temporary frame in the x direction: */
			DepthPixel* bfPtr=backgroundFrame;
			for(int y=0;y<height;++y)
				{
				DepthPixel last=bfPtr[0];
				bfPtr[0]=Math::min(bfPtr[0],bfPtr[1]);
				++bfPtr;
				for(int x=1;x<width-1;++x,++bfPtr)
					{
					DepthPixel next=Math::min(last,Math::min(bfPtr[0],bfPtr[1]));
					last=bfPtr[0];
					bfPtr[0]=next;
					}
				bfPtr[0]=Math::min(last,bfPtr[0]);
				++bfPtr;
				}
			
			/* Check if there is a callback to be called: */
			if(backgroundCaptureCallback!=0)
				{
				/* Call the callback: */
				(*backgroundCaptureCallback)(*this);
				
				/* Remove the callback object: */
				delete backgroundCaptureCallback;
				backgroundCaptureCallback=0;
				}
			}
		}
	
	/* Check if we're removing background: */
	if(removeBackground)
		{
		/* Remove background pixels: */
		DepthPixel* dfPtr=depthFrame.getData<DepthPixel>();
		DepthPixel* bfPtr=backgroundFrame;
		for(int i=depthFrame.getSize(1)*depthFrame.getSize(0);i>0;--i,++dfPtr,++bfPtr)
			{
			if(*dfPtr+backgroundRemovalFuzz>=*bfPtr)
				*dfPtr=invalidDepth; // Mark the pixel as invalid
			}
		}
	}

void DirectFrameSource::removeBackgroundToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	if(backgroundFrame!=0)
		removeBackground=cbData->set;
	else
		cbData->toggle->setToggle(false);
	}

void DirectFrameSource::captureBackgroundCompleteCallback(DirectFrameSource& source,GLMotif::Button* button)
	{
	/* Re-enable the button: */
	button->setEnabled(true);
	}

void DirectFrameSource::captureBackgroundButtonCallback(GLMotif::Button::SelectCallbackData* cbData)
	{
	/* Disable the "capture background" button until the capture is complete: */
	cbData->button->setEnabled(false);
	
	/* Start a background capture: */
	captureBackground(150,false,Misc::createFunctionCall(this,&DirectFrameSource::captureBackgroundCompleteCallback,cbData->button));
	}

void DirectFrameSource::backgroundMaxDepthCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Create a new background image at the given depth: */
	setMaxDepth(int(Math::floor(cbData->value+0.5)),true);
	}

void DirectFrameSource::backgroundRemovalFuzzCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal fuzz value: */
	backgroundRemovalFuzz=int(Math::floor(cbData->value+0.5));
	}

void DirectFrameSource::loadBackgroundCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Load the background file: */
		IO::FilePtr backgroundFile=cbData->selectedDirectory->openFile(cbData->selectedFileName);
		loadBackground(*backgroundFile);
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Load...: Could not load background from file %s due to exception %s",cbData->selectedFileName,err.what());
		}
	}

void DirectFrameSource::saveBackgroundCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Save the background file: */
		IO::FilePtr backgroundFile=cbData->selectedDirectory->openFile(cbData->selectedFileName,IO::File::WriteOnly);
		saveBackground(*backgroundFile);
		}
	catch(const std::runtime_error& err)
		{
		/* Show an error message: */
		Misc::formattedUserError("Save...: Could not save background to file %s due to exception %s",cbData->selectedFileName,err.what());
		}
	}

DirectFrameSource::DirectFrameSource(void)
	:backgroundFrame(0),
	 backgroundCaptureNumFrames(0),backgroundCaptureCallback(0),
	 removeBackground(false),backgroundRemovalFuzz(3)
	{
	}

DirectFrameSource::~DirectFrameSource(void)
	{
	delete[] backgroundFrame;
	}

FrameSource::ExtrinsicParameters DirectFrameSource::getExtrinsicParameters(void)
	{
	/* Assemble the name of the extrinsic parameter file: */
	std::string extrinsicParameterFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	extrinsicParameterFileName.push_back('/');
	extrinsicParameterFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_EXTRINSICPARAMETERSFILENAMEPREFIX);
	extrinsicParameterFileName.push_back('-');
	extrinsicParameterFileName.append(getSerialNumber());
	extrinsicParameterFileName.append(".txt");
	
	/* Check if a file of the given name exists and is readable: */
	if(IO::Directory::getCurrent()->getPathType(extrinsicParameterFileName.c_str())==Misc::PATHTYPE_FILE)
		{
		try
			{
			/* Open the parameter file: */
			IO::FilePtr parameterFile(IO::Directory::getCurrent()->openFile(extrinsicParameterFileName.c_str()));

			/* Read the camera transformation: */
			std::string transformation;
			while(!parameterFile->eof())
				transformation.push_back(parameterFile->getChar());
			return Misc::ValueCoder<ExtrinsicParameters>::decode(transformation.c_str(),transformation.c_str()+transformation.length(),0);
			}
		catch(const std::runtime_error& err)
			{
			/* Log an error and return a default set of extrinsic parameters: */
			Misc::formattedConsoleError("Kinect::DirectFrameSource::getExtrinsicParameters: Could not load extrinsic parameter file %s due to exception %s",extrinsicParameterFileName.c_str(),err.what());
			
			return ExtrinsicParameters::identity;
			}
		}
	else
		{
		/* Return a default set of extrinsic parameters: */
		return ExtrinsicParameters::identity;
		}
	}

void DirectFrameSource::configure(Misc::ConfigurationFileSection& configFileSection)
	{
	/* Check whether to load a background file: */
	if(configFileSection.hasTag("./loadBackground"))
		{
		try
			{
			/* Load a background file: */
			loadBackground(configFileSection.retrieveString("./loadBackground").c_str());
			}
		catch(const std::runtime_error& err)
			{
			/* Log an error message and carry on: */
			Misc::formattedConsoleError("DirectFrameSource::configure: Unable to load background frame %s due to exception %s",configFileSection.retrieveString("./loadBackground").c_str(),err.what());
			}
		}
	
	/* Check whether to set a maximum depth value: */
	unsigned int maxDepth=configFileSection.retrieveValue<unsigned int>("./maxDepth",0);
	if(maxDepth>0)
		{
		/* Set the maximum depth: */
		setMaxDepth(maxDepth,false);
		}
	
	/* Check whether to capture a background image: */
	unsigned int captureBackgroundFrames=configFileSection.retrieveValue<unsigned int>("./captureBackgroundFrames",0);
	if(captureBackgroundFrames>0)
		{
		/* Request background capture: */
		captureBackground(captureBackgroundFrames,false);
		}
	
	/* Set the background removal fuzz value: */
	int backgroundFuzz=configFileSection.retrieveValue<int>("./backgroundFuzz",getBackgroundRemovalFuzz());
	setBackgroundRemovalFuzz(backgroundFuzz);
	
	/* Enable background removal: */
	setRemoveBackground(configFileSection.retrieveValue<bool>("./removeBackground",getRemoveBackground()));
	}

void DirectFrameSource::buildSettingsDialog(GLMotif::RowColumn* settingsDialog)
	{
	const GLMotif::StyleSheet& ss=*settingsDialog->getStyleSheet();
	
	/* Create a button panel to toggle background removal and creation: */
	GLMotif::Margin* backgroundMargin=new GLMotif::Margin("BackgroundMargin",settingsDialog,false);
	backgroundMargin->setAlignment(GLMotif::Alignment::LEFT);
	
	GLMotif::RowColumn* backgroundBox=new GLMotif::RowColumn("BackgroundBox",backgroundMargin,false);
	backgroundBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	backgroundBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	backgroundBox->setNumMinorWidgets(1);
	
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",backgroundBox,"Remove Background");
	removeBackgroundToggle->setBorderWidth(0.0f);
	removeBackgroundToggle->setBorderType(GLMotif::Widget::PLAIN);
	removeBackgroundToggle->setToggle(removeBackground);
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&DirectFrameSource::removeBackgroundToggleCallback);
	
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",backgroundBox,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&DirectFrameSource::captureBackgroundButtonCallback);
	
	if(!backgroundSelectionHelper.isValid())
		{
		/* Create a new file selection helper: */
		std::string defaultFileName=KINECT_INTERNAL_CONFIG_CAMERA_DEFAULTBACKGROUNDFILENAMEPREFIX;
		defaultFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_BACKGROUNDFILENAMEEXTENSION);
		backgroundSelectionHelper.setTarget(new GLMotif::FileSelectionHelper(settingsDialog->getManager(),defaultFileName.c_str(),KINECT_INTERNAL_CONFIG_CAMERA_BACKGROUNDFILENAMEEXTENSION,IO::Directory::getCurrent()->openDirectory(KINECT_INTERNAL_CONFIG_CONFIGDIR)));
		}
	
	GLMotif::Button* loadBackgroundButton=new GLMotif::Button("LoadBackgroundButton",backgroundBox,"Load...");
	backgroundSelectionHelper->addLoadCallback(loadBackgroundButton,this,&DirectFrameSource::loadBackgroundCallback);
	
	GLMotif::Button* saveBackgroundButton=new GLMotif::Button("SaveBackgroundButton",backgroundBox,"Save...");
	backgroundSelectionHelper->addSaveCallback(saveBackgroundButton,this,&DirectFrameSource::saveBackgroundCallback);
	
	backgroundBox->manageChild();
	
	backgroundMargin->manageChild();
	
	GLMotif::RowColumn* sliderBox=new GLMotif::RowColumn("SliderBox",settingsDialog,false);
	sliderBox->setOrientation(GLMotif::RowColumn::VERTICAL);
	sliderBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	sliderBox->setNumMinorWidgets(2);
	
	new GLMotif::Label("BackgroundMaxDepthLabel",sliderBox,"Background Depth Limit");
	
	GLMotif::TextFieldSlider* backgroundMaxDepthSlider=new GLMotif::TextFieldSlider("BackgroundMaxDepthSlider",sliderBox,6,ss.fontHeight*10.0f);
	backgroundMaxDepthSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	backgroundMaxDepthSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	DepthRange dr=getDepthRange();
	backgroundMaxDepthSlider->setValueRange(dr.getMin(),dr.getMax(),1);
	backgroundMaxDepthSlider->setValue(dr.getMax());
	backgroundMaxDepthSlider->getValueChangedCallbacks().add(this,&DirectFrameSource::backgroundMaxDepthCallback);
	
	new GLMotif::Label("BackgroundRemovalFuzzLabel",sliderBox,"Background Removal Fuzz");
	
	GLMotif::TextFieldSlider* backgroundRemovalFuzzSlider=new GLMotif::TextFieldSlider("BackgroundRemovalFuzzSlider",sliderBox,6,ss.fontHeight*10.0f);
	backgroundRemovalFuzzSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	backgroundRemovalFuzzSlider->setValueType(GLMotif::TextFieldSlider::INT);
	backgroundRemovalFuzzSlider->setValueRange(-100,100,1);
	backgroundRemovalFuzzSlider->setValue(backgroundRemovalFuzz);
	backgroundRemovalFuzzSlider->getValueChangedCallbacks().add(this,&DirectFrameSource::backgroundRemovalFuzzCallback);
	
	sliderBox->manageChild();
	}

void DirectFrameSource::captureBackground(unsigned int numFrames,bool replace,DirectFrameSource::BackgroundCaptureCallback* newBackgroundCaptureCallback)
	{
	/* Remember the background capture callback: */
	delete backgroundCaptureCallback;
	backgroundCaptureCallback=newBackgroundCaptureCallback;
	
	/* Initialize the background frame buffer: */
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(backgroundFrame==0)
		{
		backgroundFrame=new DepthPixel[depthFrameSize[0]*depthFrameSize[1]];
		replace=true;
		}
	
	if(replace)
		{
		/* Initialize the background frame to "empty:" */
		DepthPixel* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				*bfPtr=invalidDepth;
		}
	
	/* Start capturing background frames: */
	backgroundCaptureNumFrames=numFrames;
	}

bool DirectFrameSource::loadDefaultBackground(void)
	{
	/* Compose the default background file name by looking for a serial number-tagged file in the configuration directory: */
	std::string backgroundFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	backgroundFileName.push_back('/');
	backgroundFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_DEFAULTBACKGROUNDFILENAMEPREFIX);
	backgroundFileName.push_back('-');
	backgroundFileName.append(getSerialNumber());
	backgroundFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_BACKGROUNDFILENAMEEXTENSION);
	
	/* Check if the background file exists: */
	if(IO::Directory::getCurrent()->getPathType(backgroundFileName.c_str())==Misc::PATHTYPE_FILE)
		{
		try
			{
			/* Open and read the background file from the current directory: */
			IO::FilePtr backgroundFile=IO::Directory::getCurrent()->openFile(backgroundFileName.c_str());
			backgroundFile->setEndianness(Misc::LittleEndian);
			loadBackground(*backgroundFile);
			
			return true;
			}
		catch(const std::runtime_error& err)
			{
			/* Log an error: */
			Misc::formattedConsoleError("Kinect::DirectFrameSource: Could not load default background file %s due to exception %s",backgroundFileName.c_str(),err.what());
			
			return false;
			}
		}
	else
		return false;
	}

void DirectFrameSource::loadBackground(const char* fileNamePrefix)
	{
	/* Compose the background file name by looking for a serial number-tagged file: */
	std::string backgroundFileName=fileNamePrefix;
	backgroundFileName.push_back('-');
	backgroundFileName.append(getSerialNumber());
	backgroundFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_BACKGROUNDFILENAMEEXTENSION);
	
	/* Open and read the background file from the current directory: */
	IO::FilePtr backgroundFile=IO::Directory::getCurrent()->openFile(backgroundFileName.c_str());
	backgroundFile->setEndianness(Misc::LittleEndian);
	loadBackground(*backgroundFile);
	}

void DirectFrameSource::loadBackground(IO::File& file)
	{
	/* Read the frame header: */
	Misc::UInt32 fileFrameSize[2];
	file.read<Misc::UInt32>(fileFrameSize,2);
	
	/* Check if the file matches the current depth buffer size: */
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(fileFrameSize[0]!=depthFrameSize[0]||fileFrameSize[1]!=depthFrameSize[1])
		Misc::throwStdErr("Kinect::DirectFrameSource::loadBackground: Background frame size mismatch");
	
	/* Create a temporary background frame buffer: */
	Misc::SelfDestructArray<DepthPixel> newBackgroundFrame(depthFrameSize[0]*depthFrameSize[1]);
	
	/* Read the background file: */
	file.read<DepthPixel>(newBackgroundFrame.getArray(),depthFrameSize[0]*depthFrameSize[1]);
	
	/* Install the new background frame: */
	delete[] backgroundFrame;
	backgroundFrame=newBackgroundFrame.releaseTarget();
	}

void DirectFrameSource::setMaxDepth(unsigned int newMaxDepth,bool replace)
	{
	/* Limit the depth value to the valid range: */
	if(newMaxDepth>invalidDepth)
		newMaxDepth=invalidDepth;
	DepthPixel nmd=DepthPixel(newMaxDepth);
	
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	if(backgroundFrame==0)
		{
		/* Create the background frame buffer: */
		backgroundFrame=new DepthPixel[depthFrameSize[0]*depthFrameSize[1]];
		replace=true;
		}
	
	if(replace)
		{
		/* Initialize the background frame to the max depth value */
		DepthPixel* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				*bfPtr=nmd;
		}
	else
		{
		/* Modify the existing background frame buffer: */
		DepthPixel* bfPtr=backgroundFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++bfPtr)
				if(*bfPtr>nmd)
					*bfPtr=nmd;
		}
	}

void DirectFrameSource::saveBackground(const char* fileNamePrefix)
	{
	/* Bail out if there is no background frame: */
	if(backgroundFrame==0)
		return;
	
	/* Construct the full background file name: */
	std::string backgroundFileName=fileNamePrefix;
	backgroundFileName.push_back('-');
	backgroundFileName.append(getSerialNumber());
	backgroundFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_BACKGROUNDFILENAMEEXTENSION);
	
	/* Save the background file: */
	IO::FilePtr backgroundFile=IO::Directory::getCurrent()->openFile(backgroundFileName.c_str());
	backgroundFile->setEndianness(Misc::LittleEndian);
	saveBackground(*backgroundFile);
	}

void DirectFrameSource::saveBackground(IO::File& file)
	{
	/* Bail out if there is no background frame: */
	if(backgroundFrame==0)
		return;
	
	const unsigned int* depthFrameSize=getActualFrameSize(DEPTH);
	for(int i=0;i<2;++i)
		file.write<Misc::UInt32>(depthFrameSize[i]);
	file.write<DepthPixel>(backgroundFrame,depthFrameSize[0]*depthFrameSize[1]);
	}

void DirectFrameSource::setRemoveBackground(bool newRemoveBackground)
	{
	/* Only enable background removal if there is a background frame: */
	removeBackground=newRemoveBackground&&backgroundFrame!=0;
	}

void DirectFrameSource::setBackgroundRemovalFuzz(int newBackgroundRemovalFuzz)
	{
	backgroundRemovalFuzz=Misc::SInt16(newBackgroundRemovalFuzz);
	}

}
