/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
Copyright (c) 2010-2015 Oliver Kreylos

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

#include "RawKinectViewer.h"

#include <stdexcept>
#include <iostream>
#include <Misc/SelfDestructPointer.h>
#include <Misc/FunctionCalls.h>
#include <IO/File.h>
#include <IO/Directory.h>
#include <Geometry/Point.h>
#include <Geometry/Ray.h>
#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/Extensions/GLARBTextureNonPowerOfTwo.h>
#include <GL/GLContextData.h>
#include <Images/RGBImage.h>
#include <Images/WriteImageFile.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Camera.h>

#include "PauseTool.h"
#include "MeasurementTool.h"
#include "TiePointTool.h"
#include "LineTool.h"
#include "DepthCorrectionTool.h"
#include "GridTool.h"
#include "PlaneTool.h"
#include "PointPlaneTool.h"
#include "CalibrationCheckTool.h"

/******************************************
Methods of class RawKinectViewer::DataItem:
******************************************/

RawKinectViewer::DataItem::DataItem(void)
	:colorTextureId(0),colorFrameVersion(0),
	 depthTextureId(0),depthFrameVersion(0)
	{
	/* Allocate texture objects: */
	glGenTextures(1,&depthTextureId);
	glGenTextures(1,&colorTextureId);
	}

RawKinectViewer::DataItem::~DataItem(void)
	{
	/* Destroy texture objects: */
	glDeleteTextures(1,&depthTextureId);
	glDeleteTextures(1,&colorTextureId);
	}

/********************************
Methods of class RawKinectViewer:
********************************/

void RawKinectViewer::mapDepth(unsigned int x,unsigned int y,float depth,GLubyte* result) const
	{
	if(depthPlaneValid)
		{
		/* Color depth pixels by distance to the depth plane: */
		float dist=camDepthPlane.calcDistance(Plane::Point(float(x)+0.5f,float(y)+0.5f,depth));
		if(dist>=0.0f)
			{
			GLubyte col=dist<depthPlaneDistMax?255U-GLubyte((dist*255.0f)/depthPlaneDistMax+0.5f):0U;
			result[0]=col;
			result[1]=col;
			result[2]=255U;
			}
		else
			{
			GLubyte col=-dist<depthPlaneDistMax?255U-GLubyte((-dist*255.0f)/depthPlaneDistMax+0.5f):0U;
			result[0]=255U;
			result[1]=col;
			result[2]=col;
			}
		}
	else
		{
		/* Color depth pixels by depth value: */
		static const GLubyte mapColors[6][3]=
			{
			{255,0,0},
			{255,255,0},
			{0,255,0},
			{0,255,255},
			{0,0,255},
			{255,0,255}
			};
		float d=(depth-depthValueRange[0])*5.0f/(depthValueRange[1]-depthValueRange[0]);
		if(d<=0.0f)
			{
			for(int i=0;i<3;++i)
				result[i]=GLubyte(mapColors[0][i]*0.2f);
			}
		else if(d>=5.0f)
			{
			for(int i=0;i<3;++i)
				result[i]=mapColors[5][i];
			}
		else
			{
			int i0=int(d);
			d-=float(i0);
			for(int i=0;i<3;++i)
				result[i]=GLubyte((mapColors[i0][i]*(1.0f-d)+mapColors[i0+1][i]*d)*(d*0.8f+0.2f));
			}
		}
	}

Vrui::Point RawKinectViewer::calcImagePoint(const Vrui::Ray& physicalRay) const
	{
	/* Transform the ray to navigational space: */
	Vrui::Ray navRay=physicalRay;
	navRay.transform(Vrui::getInverseNavigationTransformation());
	if(navRay.getDirection()[2]!=Vrui::Scalar(0))
		{
		Vrui::Scalar lambda=-navRay.getOrigin()[2]/navRay.getDirection()[2];
		return navRay(lambda);
		}
	else
		return Vrui::Point::origin;
	}

void RawKinectViewer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		#if 0
		
		/* Normalize the color frame: */
		Kinect::FrameBuffer normalizedFrame(frameBuffer.getSize(0),frameBuffer.getSize(1),frameBuffer.getSize(1)*frameBuffer.getSize(0)*sizeof(ColorPixel));
		const ColorPixel* fPtr=static_cast<const ColorPixel*>(frameBuffer.getBuffer());
		ColorPixel* nfPtr=static_cast<ColorPixel*>(normalizedFrame.getBuffer());
		for(int y=0;y<frameBuffer.getSize(1);++y)
			for(int x=0;x<frameBuffer.getSize(0);++x,++fPtr,++nfPtr)
				{
				#if 1
				/* Divide all color components by the largest component: */
				unsigned int max=fPtr->rgb[0];
				for(int i=1;i<3;++i)
					if(max<fPtr->rgb[i])
						max=fPtr->rgb[i];
				for(int i=0;i<3;++i)
					nfPtr->rgb[i]=ColorComponent(((unsigned int)fPtr->rgb[i]*256U)/(max+1));
				#endif
				
				#if 0
				
				/* Classify the unprocessed color: */
				if(fPtr->rgb[0]<32U&&fPtr->rgb[1]<32U&&fPtr->rgb[2]<32U)
					nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=255U;
				else
					nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=0U;
				
				#elif 0
				
				/* Classify the normalized color: */
				if(nfPtr->rgb[0]>=240U&&nfPtr->rgb[1]<80U&&nfPtr->rgb[2]>=80U)
					nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=255U;
				else
					nfPtr->rgb[0]=nfPtr->rgb[1]=nfPtr->rgb[2]=0U;
				
				#endif
				}
		colorFrames.postNewValue(normalizedFrame);
		
		#else
		
		/* Post the new frame into the color frame triple buffer: */
		colorFrames.postNewValue(frameBuffer);
		
		#endif
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		/* Post the new frame into the depth frame triple buffer: */
		depthFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::requestAverageFrame(RawKinectViewer::AverageFrameReadyCallback* callback)
	{
	/* Check if there already is an average frame: */
	if(averageFrameValid)
		{
		/* Just call the callback immediately and forget about it: */
		if(callback!=0)
			{
			(*callback)(0);
			delete callback;
			}
		}
	else
		{
		/* Check if there is already an average frame capture underway: */
		if(averageFrameCounter==0)
			{
			/* Start averaging frames: */
			float* afdPtr=averageFrameDepth;
			float* affPtr=averageFrameForeground;
			for(unsigned int y=0;y<depthFrameSize[1];++y)
				for(unsigned int x=0;x<depthFrameSize[0];++x,++afdPtr,++affPtr)
					{
					*afdPtr=0.0f;
					*affPtr=0.0f;
					}
			averageFrameCounter=averageNumFrames;
			
			/* Show a progress dialog: */
			Vrui::popupPrimaryWidget(averageDepthFrameDialog);
			}
		
		/* Add the callback to the callback list: */
		if(callback!=0)
			averageFrameReadyCallbacks.push_back(callback);
		}
	}

void RawKinectViewer::locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData)
	{
	Vrui::Point pos=cbData->currentTransformation.getOrigin();
	if(pos[0]>=-depthFrameSize[0]&&pos[0]<0.0&&pos[1]>=0.0&&pos[1]<depthFrameSize[1])
		{
		/* Select the pixel under the locator: */
		selectedPixel[0]=int(pos[0]+double(depthFrameSize[0]));
		selectedPixel[1]=int(pos[1]);
		
		/* Start the selected pixel's EKG: */
		selectedPixelCurrentIndex=0;
		const DepthPixel* dfPtr=static_cast<const DepthPixel*>(depthFrames.getLockedValue().getBuffer());
		selectedPixelPulse[0]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
		for(int i=1;i<128;++i)
			selectedPixelPulse[i]=0;
		}
	else
		{
		/* Select an invalid pixel: */
		selectedPixel[0]=selectedPixel[1]=~0x0U;
		}
	}

void RawKinectViewer::resetNavigationCallback(Misc::CallbackData* cbData)
	{
	/* Reset the navigation transformation: */
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(1024),Vrui::Vector(0,1,0));
	}

void RawKinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	/* Capture five seconds worth of background frames: */
	camera->captureBackground(150,true);
	}

void RawKinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	camera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(camera->getRemoveBackground());
	}

void RawKinectViewer::averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	showAverageFrame=cbData->set;
	if(cbData->set)
		{
		/* Request a new average frame: */
		requestAverageFrame(0);
		}
	else
		{
		/* Invalidate the current average frame: */
		averageFrameValid=false;
		depthPlaneValid=false;
		}
	}

void RawKinectViewer::saveAverageFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Open the average frame file: */
		IO::FilePtr frameFile(cbData->selectedDirectory->openFile(cbData->selectedFileName,IO::File::WriteOnly));
		
		/* Write the averaged frame: */
		for(int i=0;i<2;++i)
			frameFile->write<Misc::UInt32>(depthFrameSize[i]);
		float cutoff=float(averageNumFrames)*0.5f;
		float* afdPtr=averageFrameDepth;
		float* affPtr=averageFrameForeground;
		const PixelCorrection* dcPtr=depthCorrection;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++afdPtr,++affPtr,++dcPtr)
				frameFile->write<Misc::Float32>(*affPtr>=cutoff?dcPtr->correct((*afdPtr)/(*affPtr)):2047.0f);
		}
	catch(std::runtime_error err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...",Misc::printStdErrMsg("Could not write depth frame file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what()));
		}
	
	/* Destroy the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void RawKinectViewer::saveAverageFrameCallback(Misc::CallbackData* cbData)
	{
	if(!averageFrameValid)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...","No valid average depth frame to save");
		return;
		}
	
	try
		{
		/* Create a uniquely-named depth image file in the current directory: */
		IO::DirectoryPtr currentDir=Vrui::openDirectory(".");
		std::string depthFrameFileName=currentDir->createNumberedFileName("DepthFrame.dat",4);
		
		/* Create a file selection dialog to select an alternative depth frame file name: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveAverageFrameDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Average Depth Frame...",currentDir,depthFrameFileName.c_str(),".dat"));
		saveAverageFrameDialog->getOKCallbacks().add(this,&RawKinectViewer::saveAverageFrameOKCallback);
		saveAverageFrameDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(saveAverageFrameDialog.releaseTarget());
		}
	catch(std::runtime_error err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Average Depth Frame...",Misc::printStdErrMsg("Could not save average depth frame due to exception %s",err.what()));
		}
	}

void RawKinectViewer::saveColorFrameOKCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData)
	{
	try
		{
		/* Convert the current color frame into an RGB image: */
		const ColorPixel* sPtr=static_cast<const ColorPixel*>(colorFrames.getLockedValue().getBuffer());
		Images::RGBImage colorImage(colorFrameSize[0],colorFrameSize[1]);
		Images::RGBImage::Color* dPtr=colorImage.modifyPixels();
		for(unsigned int y=0;y<colorFrameSize[1];++y)
			for(unsigned int x=0;x<colorFrameSize[0];++x,sPtr+=3,++dPtr)
				for(int i=0;i<3;++i)
					(*dPtr)[i]=sPtr->rgb[i];
		
		/* Write the RGB image to the file: */
		Images::writeImageFile(colorImage,cbData->selectedFileName);
		}
	catch(std::runtime_error err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Color Frame...",Misc::printStdErrMsg("Could not write color frame file %s due to exception %s",cbData->getSelectedPath().c_str(),err.what()));
		}
	
	/* Destroy the file selection dialog: */
	cbData->fileSelectionDialog->close();
	}

void RawKinectViewer::saveColorFrameCallback(Misc::CallbackData* cbData)
	{
	try
		{
		/* Create a uniquely-named color image file in the current directory: */
		IO::DirectoryPtr currentDir=Vrui::openDirectory(".");
		std::string colorFrameFileName=currentDir->createNumberedFileName("ColorFrame.png",4);
		
		/* Create a file selection dialog to select an alternative color frame file name: */
		Misc::SelfDestructPointer<GLMotif::FileSelectionDialog> saveColorFrameDialog(new GLMotif::FileSelectionDialog(Vrui::getWidgetManager(),"Save Color Frame...",currentDir,colorFrameFileName.c_str(),".png"));
		saveColorFrameDialog->getOKCallbacks().add(this,&RawKinectViewer::saveColorFrameOKCallback);
		saveColorFrameDialog->deleteOnCancel();
		
		/* Show the file selection dialog: */
		Vrui::popupPrimaryWidget(saveColorFrameDialog.releaseTarget());
		}
	catch(std::runtime_error err)
		{
		/* Show an error message: */
		Vrui::showErrorMessage("Save Color Frame...",Misc::printStdErrMsg("Could not save color frame due to exception %s",err.what()));
		}
	}

GLMotif::PopupMenu* RawKinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Raw Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to reset navigation: */
	GLMotif::Button* resetNavigationButton=new GLMotif::Button("ResetNavigationButton",mainMenu,"Reset Navigation");
	resetNavigationButton->getSelectCallbacks().add(this,&RawKinectViewer::resetNavigationCallback);
	
	/* Create a button to capture a background frame: */
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",mainMenu,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&RawKinectViewer::captureBackgroundCallback);
	
	/* Create a toggle button to enable/disable background removal: */
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",mainMenu,"Remove Background");
	removeBackgroundToggle->setToggle(camera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&RawKinectViewer::removeBackgroundCallback);
	
	/* Create a toggle button to calculate and show an averaged depth frame: */
	GLMotif::ToggleButton* averageFramesButton=new GLMotif::ToggleButton("AverageFramesButton",mainMenu,"Average Frames");
	averageFramesButton->getValueChangedCallbacks().add(this,&RawKinectViewer::averageFramesCallback);
	
	/* Create a button to save the current averaged depth frame: */
	GLMotif::Button* saveAverageFrameButton=new GLMotif::Button("SaveAverageFrameButton",mainMenu,"Save Average Frame");
	saveAverageFrameButton->getSelectCallbacks().add(this,&RawKinectViewer::saveAverageFrameCallback);
	
	/* Create a button to save the current color frame: */
	GLMotif::Button* saveColorFrameButton=new GLMotif::Button("SaveColorFrameButton",mainMenu,"Save Color Frame");
	saveColorFrameButton->getSelectCallbacks().add(this,&RawKinectViewer::saveColorFrameCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

GLMotif::PopupWindow* RawKinectViewer::createAverageDepthFrameDialog(void)
	{
	/* Create the average depth frame dialog window: */
	GLMotif::PopupWindow* averageDepthFrameDialogPopup=new GLMotif::PopupWindow("AverageDepthFrameDialogPopup",Vrui::getWidgetManager(),"RawKinectViewer");
	
	new GLMotif::Label("AverageDepthFrameLabel",averageDepthFrameDialogPopup,"Capturing average depth frame...");
	
	return averageDepthFrameDialogPopup;
	}

RawKinectViewer::RawKinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 camera(0),
	 colorFrameSize(0),colorFrameVersion(0),
	 depthFrameSize(0),depthCorrection(0),depthPlaneDistMax(10.0),depthFrameVersion(0),
	 paused(false),
	 averageNumFrames(150),averageFrameCounter(0),
	 averageFrameDepth(0),averageFrameForeground(0),
	 averageFrameValid(false),showAverageFrame(false),
	 depthPlaneValid(false),
	 mainMenu(0),averageDepthFrameDialog(0)
	{
	/*********************************************************************
	Register the custom tool classes with the Vrui tool manager:
	*********************************************************************/
	
	PauseTool::initClass(*Vrui::getToolManager());
	MeasurementTool::initClass(*Vrui::getToolManager());
	TiePointTool::initClass(*Vrui::getToolManager());
	LineTool::initClass(*Vrui::getToolManager());
	DepthCorrectionTool::initClass(*Vrui::getToolManager());
	GridTool::initClass(*Vrui::getToolManager());
	PlaneTool::initClass(*Vrui::getToolManager());
	PointPlaneTool::initClass(*Vrui::getToolManager());
	CalibrationCheckTool::initClass(*Vrui::getToolManager());
	
	/* Parse the command line: */
	bool printHelp=false;
	int cameraIndex=0; // Use first Kinect camera device on USB bus
	Kinect::Camera::FrameSize selectedColorFrameSize=Kinect::Camera::FS_640_480;
	Kinect::Camera::FrameSize selectedDepthFrameSize=Kinect::Camera::FS_640_480;
	bool compressDepthFrames=false;
	depthValueRange[0]=300.0f;
	depthValueRange[1]=1100.0f; // float(Kinect::FrameSource::invalidDepth);
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"h")==0)
				printHelp=true;
			else if(strcasecmp(argv[i]+1,"high")==0)
				{
				selectedColorFrameSize=Kinect::Camera::FS_1280_1024;
				// selectedDepthFrameSize=Kinect::Camera::FS_1280_1024;
				}
			else if(strcasecmp(argv[i]+1,"compress")==0)
				compressDepthFrames=true;
			else if(strcasecmp(argv[i]+1,"gridSize")==0)
				{
				GridTool::setGridSize(atoi(argv[i+1]),atoi(argv[i+2]));
				i+=2;
				}
			else if(strcasecmp(argv[i]+1,"tileSize")==0)
				{
				GridTool::setTileSize(atof(argv[i+1]),atof(argv[i+2]));
				i+=2;
				}
			else if(strcasecmp(argv[i]+1,"depthRange")==0)
				{
				for(int j=0;j<2;++j)
					depthValueRange[j]=float(atof(argv[i+1+j]));
				i+=2;
				}
			}
		else
			cameraIndex=atoi(argv[i]);
		}
	
	if(printHelp)
		{
		std::cout<<"Usage: RawKinectViewer [option 1] ... [option n] <camera index>"<<std::endl;
		std::cout<<"  <camera index>"<<std::endl;
		std::cout<<"     Selects the local Kinect camera of the given index (0: first camera on USB bus)"<<std::endl;
		std::cout<<"     Default: 0"<<std::endl;
		std::cout<<"  Options:"<<std::endl;
		std::cout<<"  -h"<<std::endl;
		std::cout<<"     Prints this help message"<<std::endl;
		std::cout<<"  -high"<<std::endl;
		std::cout<<"    Sets color frame size for the selected Kinect camera to 1280x1024 @ 15Hz"<<std::endl;
		std::cout<<"  -compress"<<std::endl;
		std::cout<<"     Requests compressed depth frames from the selected Kinect camera"<<std::endl;
		std::cout<<"  -gridSize <grid width> <grid height>"<<std::endl;
		std::cout<<"     Sets the number of tiles of the semi-transparent calibration grid"<<std::endl;
		std::cout<<"     Default: 7 5"<<std::endl;
		std::cout<<"  -tileSize <tile width> <tile height>"<<std::endl;
		std::cout<<"     Sets the size of each tile of the semi-transparent calibration grid"<<std::endl;
		std::cout<<"     Default: 3.5 3.5 (assumed to be inches)"<<std::endl;
		std::cout<<"  -depthRange <min depth> <max depth>"<<std::endl;
		std::cout<<"     Sets the range of depth values mapped to the full color range"<<std::endl;
		std::cout<<"     Default: 300 1100"<<std::endl;
		}
	
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to the given Kinect camera device on the host: */
	camera=new Kinect::Camera(usbContext,cameraIndex);
	
	/* Set the color camera's frame size: */
	camera->setFrameSize(Kinect::FrameSource::COLOR,selectedColorFrameSize);
	camera->setFrameSize(Kinect::FrameSource::DEPTH,selectedDepthFrameSize);
	
	/* Get the cameras' actual frame sizes: */
	colorFrameSize=camera->getActualFrameSize(Kinect::FrameSource::COLOR);
	depthFrameSize=camera->getActualFrameSize(Kinect::FrameSource::DEPTH);
	
	/* Get the camera's depth correction parameters: */
	Kinect::FrameSource::DepthCorrection* dc=camera->getDepthCorrectionParameters();
	
	/* Evaluate the camera's depth correction parameters into a per-pixel offset array: */
	depthCorrection=dc->getPixelCorrection(depthFrameSize);
	
	/* Clean up: */
	delete dc;
	
	/* Get the camera's intrinsic parameters: */
	intrinsicParameters=camera->getIntrinsicParameters();
	
	/* Allocate the average depth frame buffer: */
	averageFrameDepth=new float[depthFrameSize[0]*depthFrameSize[1]];
	averageFrameForeground=new float[depthFrameSize[0]*depthFrameSize[1]];
	
	/* Set depth frame compression: */
	camera->setCompressDepthFrames(compressDepthFrames);
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	averageDepthFrameDialog=createAverageDepthFrameDialog();
	
	/* Start streaming: */
	camera->startStreaming(Misc::createFunctionCall(this,&RawKinectViewer::colorStreamingCallback),Misc::createFunctionCall(this,&RawKinectViewer::depthStreamingCallback));
	
	/* Select an invalid pixel: */
	selectedPixel[0]=selectedPixel[1]=~0x0U;
	
	/* Initialize navigation transformation: */
	resetNavigationCallback(0);
	}

RawKinectViewer::~RawKinectViewer(void)
	{
	delete mainMenu;
	delete averageDepthFrameDialog;
	delete[] averageFrameDepth;
	delete[] averageFrameForeground;
	
	/* Stop streaming: */
	camera->stopStreaming();
	
	/* Disconnect from the Kinect camera device: */
	delete camera;
	}

void RawKinectViewer::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Call the base class method: */
	Vrui::Application::toolCreationCallback(cbData);
	
	/* Check if the new tool is a locator tool: */
	Vrui::LocatorTool* lt=dynamic_cast<Vrui::LocatorTool*>(cbData->tool);
	if(lt!=0)
		{
		/* Register callbacks with the locator tool: */
		lt->getButtonPressCallbacks().add(this,&RawKinectViewer::locatorButtonPressCallback);
		}
	}

void RawKinectViewer::frame(void)
	{
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		++depthFrameVersion;
		
		if(selectedPixel[0]!=~0x0U&&selectedPixel[1]!=~0x0U)
			{
			/* Update the selected pixel's EKG: */
			++selectedPixelCurrentIndex;
			if(selectedPixelCurrentIndex==128)
				selectedPixelCurrentIndex=0;
			const DepthPixel* dfPtr=static_cast<const DepthPixel*>(depthFrames.getLockedValue().getBuffer());
			selectedPixelPulse[selectedPixelCurrentIndex]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
			}
		
		if(averageFrameCounter>0)
			{
			/* Accumulate the new depth frame into the averaging buffer: */
			const DepthPixel* dfPtr=static_cast<const DepthPixel*>(depthFrames.getLockedValue().getBuffer());
			float* afdPtr=averageFrameDepth;
			float* affPtr=averageFrameForeground;
			for(unsigned int y=0;y<depthFrameSize[1];++y)
				for(unsigned int x=0;x<depthFrameSize[0];++x,++dfPtr,++afdPtr,++affPtr)
					{
					if(*dfPtr!=0x7ffU)
						{
						*afdPtr+=float(*dfPtr);
						*affPtr+=1.0f;
						}
					}
			--averageFrameCounter;
			if(averageFrameCounter==0)
				{
				/* Mark the average frame buffer as valid: */
				averageFrameValid=true;
				
				/* Call all registered callbacks: */
				for(std::vector<AverageFrameReadyCallback*>::iterator afrcIt=averageFrameReadyCallbacks.begin();afrcIt!=averageFrameReadyCallbacks.end();++afrcIt)
					{
					(**afrcIt)(0);
					delete *afrcIt;
					}
				averageFrameReadyCallbacks.clear();
				
				/* Hide the progress dialog: */
				Vrui::popdownPrimaryWidget(averageDepthFrameDialog);
				
				/* Invalidate the average depth frame immediately if it wasn't requested directly by the user: */
				averageFrameValid=showAverageFrame;
				}
			}
		}
	}

void RawKinectViewer::display(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_TEXTURE_BIT);
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	glColor3f(1.0f,1.0f,1.0f);
	
	/* Bind the depth texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(showAverageFrame&&averageFrameValid)
		{
		/* Convert the averaged depth image to RGB: */
		unsigned int width=depthFrameSize[0];
		unsigned int height=depthFrameSize[1];
		GLubyte* byteFrame=new GLubyte[height*width*3];
		const float* afdPtr=averageFrameDepth;
		const float* affPtr=averageFrameForeground;
		float foregroundCutoff=float(averageNumFrames)*0.5f;
		const PixelCorrection* dcPtr=depthCorrection;
		GLubyte* bfPtr=byteFrame;
		for(unsigned int y=0;y<height;++y)
			for(unsigned int x=0;x<width;++x,++afdPtr,++affPtr,++dcPtr,bfPtr+=3)
				{
				if(*affPtr>=foregroundCutoff)
					{
					float d=dcPtr->correct((*afdPtr)/(*affPtr));
					mapDepth(x,y,d,bfPtr);
					}
				else
					bfPtr[0]=bfPtr[1]=bfPtr[2]=GLubyte(0);
				}
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the depth texture image: */
		glTexSubImage2D(GL_TEXTURE_2D,0,0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,byteFrame);
		
		delete[] byteFrame;
		}
	else
		{
		if(dataItem->depthFrameVersion!=depthFrameVersion)
			{
			/* Upload the depth frame into the texture object: */
			const Kinect::FrameBuffer& depthFrame=depthFrames.getLockedValue();
			unsigned int width=depthFrameSize[0];
			unsigned int height=depthFrameSize[1];
			const GLushort* framePtr=static_cast<const GLushort*>(depthFrame.getBuffer());
			
			/* Convert the depth image to unsigned byte: */
			GLubyte* byteFrame=new GLubyte[height*width*3];
			const GLushort* fPtr=framePtr;
			const PixelCorrection* dcPtr=depthCorrection;
			GLubyte* bfPtr=byteFrame;
			for(unsigned int y=0;y<height;++y)
				for(unsigned int x=0;x<width;++x,++fPtr,++dcPtr,bfPtr+=3)
					{
					if(*fPtr!=Kinect::FrameSource::invalidDepth)
						{
						float d=dcPtr->correct(*fPtr);
						mapDepth(x,y,d,bfPtr);
						}
					else
						bfPtr[0]=bfPtr[1]=bfPtr[2]=GLubyte(0);
					}
			
			/* Set up the texture parameters: */
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
			
			/* Upload the depth texture image: */
			glTexSubImage2D(GL_TEXTURE_2D,0,0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,byteFrame);
			
			delete[] byteFrame;
			
			/* Mark the cached depth frame as up-to-date: */
			dataItem->depthFrameVersion=depthFrameVersion;
			}
		}
	
	/* Draw the depth image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(-GLfloat(depthFrameSize[0]),0.0f);
	glTexCoord2f(GLfloat(depthFrameSize[0])/GLfloat(dataItem->depthTextureSize[0]),0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(GLfloat(depthFrameSize[0])/GLfloat(dataItem->depthTextureSize[0]),GLfloat(depthFrameSize[1])/GLfloat(dataItem->depthTextureSize[1]));
	glVertex2f(0.0f,GLfloat(depthFrameSize[1]));
	glTexCoord2f(0.0f,GLfloat(depthFrameSize[1])/GLfloat(dataItem->depthTextureSize[1]));
	glVertex2f(-GLfloat(depthFrameSize[0]),GLfloat(depthFrameSize[1]));
	glEnd();
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		const Kinect::FrameBuffer& colorFrame=colorFrames.getLockedValue();
		unsigned int width=colorFrameSize[0];
		unsigned int height=colorFrameSize[1];
		const GLubyte* framePtr=static_cast<const GLubyte*>(colorFrame.getBuffer());
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the color texture image: */
		glTexSubImage2D(GL_TEXTURE_2D,0,0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	
	/* Draw the color image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(GLfloat(colorFrameSize[0])/GLfloat(dataItem->colorTextureSize[0]),0.0f);
	glVertex2f(GLfloat(colorFrameSize[0]),0.0f);
	glTexCoord2f(GLfloat(colorFrameSize[0])/GLfloat(dataItem->colorTextureSize[0]),GLfloat(colorFrameSize[1])/GLfloat(dataItem->colorTextureSize[1]));
	glVertex2f(GLfloat(colorFrameSize[0]),GLfloat(colorFrameSize[1]));
	glTexCoord2f(0.0f,GLfloat(colorFrameSize[1])/GLfloat(dataItem->colorTextureSize[1]));
	glVertex2f(0.0f,GLfloat(colorFrameSize[1]));
	glEnd();
	
	/* Protect the texture objects: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	if(selectedPixel[0]!=~0x0U&&selectedPixel[1]!=~0x0U)
		{
		/* Draw the selected pixel: */
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		
		glBegin(GL_LINES);
		glColor3f(0.0f,1.0f,0.0f);
		GLfloat spx=GLfloat(selectedPixel[0])-GLfloat(depthFrameSize[0])+0.5f;
		GLfloat spy=GLfloat(selectedPixel[1])+0.5f;
		glVertex3f(spx-5.0f,spy,0.1f);
		glVertex3f(spx+5.0f,spy,0.1f);
		glVertex3f(spx,spy-5.0f,0.1f);
		glVertex3f(spx,spy+5.0f,0.1f);
		glEnd();
		
		/* Draw the selected pixel's EKG: */
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<128;++i)
			glVertex3f(GLfloat(i)*depthFrameSize[0]/128.0f-depthFrameSize[0],GLfloat(selectedPixelPulse[i])*0.25-512.0f,0.1f);
		glEnd();
		}
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void RawKinectViewer::initContext(GLContextData& contextData) const
	{
	/* Create and register the data item: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Check for NPOTD texture support and initialize the padded texture sizes: */
	if(GLARBTextureNonPowerOfTwo::isSupported())
		{
		/* Initialize the extension: */
		GLARBTextureNonPowerOfTwo::initExtension();
		
		/* Use actual image sizes as texture sizes: */
		for(int i=0;i<2;++i)
			{
			dataItem->colorTextureSize[i]=colorFrameSize[i];
			dataItem->depthTextureSize[i]=depthFrameSize[i];
			}
		}
	else
		{
		/* Pad image sizes to the next-larger power of two: */
		for(int i=0;i<2;++i)
			{
			for(dataItem->colorTextureSize[i]=1;dataItem->colorTextureSize[i]<colorFrameSize[i];dataItem->colorTextureSize[i]<<=1)
				;
			for(dataItem->depthTextureSize[i]=1;dataItem->depthTextureSize[i]<depthFrameSize[i];dataItem->depthTextureSize[i]<<=1)
				;
			}
		}
	
	/* Prepare the depth texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,dataItem->depthTextureSize[0],dataItem->depthTextureSize[1],0,GL_RGB,GL_UNSIGNED_BYTE,0);
	
	/* Prepare the color texture: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,dataItem->colorTextureSize[0],dataItem->colorTextureSize[1],0,GL_RGB,GL_UNSIGNED_BYTE,0);
	
	/* Protect the texture images: */
	glBindTexture(GL_TEXTURE_2D,0);
	}

int main(int argc,char* argv[])
	{
	try
		{
		char** appDefaults=0;
		RawKinectViewer app(argc,argv,appDefaults);
		app.run();
		}
	catch(std::runtime_error err)
		{
		std::cerr<<"Caught exception "<<err.what()<<std::endl;
		return 1;
		}
	
	return 0;
	}
