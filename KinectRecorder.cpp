/***********************************************************************
KinectRecorder - Simple utility to save color and depth image streams
from one or more Kinect devices to a set of time-stamped files.
Copyright (c) 2011 Oliver Kreylos

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

#include <string.h>
#include <vector>
#include <string>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <USB/Context.h>
#include <Geometry/GeometryValueCoders.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>
#include <Kinect/FrameSaver.h>

/**************
Helper classes:
**************/
	
class KinectSaver // Helper class to save 3D video data from a Kinect camera to a pair of time-stamped files
	{
	/* Elements: */
	public:
	Kinect::Camera camera; // The camera whose video stream to save
	Kinect::FrameSaver* frameSaver; // Pointer to helper object saving depth and color frames received from the Kinect
	
	/* Constructors and destructors: */
	public:
	KinectSaver(USB::Context& usbContext,int cameraIndex,bool highres,const char* fileNamePrefix); // Creates a video stream saver for the Kinect camera of the given index in the given USB context
	~KinectSaver(void); // Destroys the streamer
	
	/* Methods: */
	Kinect::Camera& getCamera(void) // Returns a pointer to the streamer's camera
		{
		return camera;
		}
	void startStreaming(void); // Begins streaming from the Kinect camera
	};

/****************************
Methods of class KinectSaver:
****************************/

KinectSaver::KinectSaver(USB::Context& context,int cameraIndex,bool highres,const char* fileNamePrefix)
	:camera(context,cameraIndex),frameSaver(0)
	{
	/* Set the camera's frame size: */
	camera.setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
	
	/* Create the frame saver: */
	std::string colorFrameFileName=fileNamePrefix;
	colorFrameFileName.push_back('-');
	colorFrameFileName.append(camera.getSerialNumber());
	colorFrameFileName.append(".color");
	std::string depthFrameFileName=fileNamePrefix;
	depthFrameFileName.push_back('-');
	depthFrameFileName.append(camera.getSerialNumber());
	depthFrameFileName.append(".depth");
	frameSaver=new Kinect::FrameSaver(camera,colorFrameFileName.c_str(),depthFrameFileName.c_str());
	}

KinectSaver::~KinectSaver(void)
	{
	/* Stop streaming: */
	camera.stopStreaming();
	
	/* Delete the frame saver: */
	delete frameSaver;
	}

void KinectSaver::startStreaming(void)
	{
	/* Start streaming: */
	camera.startStreaming(Misc::createFunctionCall(frameSaver,&Kinect::FrameSaver::saveColorFrame),Misc::createFunctionCall(frameSaver,&Kinect::FrameSaver::saveDepthFrame));
	}

/*************
Program state:
*************/
	
USB::Context usbContext; // USB device context
std::vector<KinectSaver*> savers; // List of Kinect video stream savers, each connected to one Kinect camera
const char* backgroundFileNamePrefix=0;

void saveBackground(Kinect::Camera& camera)
	{
	if(backgroundFileNamePrefix!=0)
		{ 
		/* Save the camera's background frame: */
		camera.saveBackground(backgroundFileNamePrefix);
		}
	}

int main(int argc,char* argv[])
	{
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Add a video stream saver for each camera index passed on the command line: */
	const char* fileNamePrefix="KinectRecorder";
	bool highres=false;
	int numBackgroundFrames=150;
	unsigned int maxDepth=0;
	int backgroundRemovalFuzz=5;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"f")==0)
				{
				++i;
				fileNamePrefix=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"bff")==0)
				{
				++i;
				numBackgroundFrames=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"bf")==0)
				{
				++i;
				backgroundFileNamePrefix=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"high")==0)
				highres=true;
			else if(strcasecmp(argv[i]+1,"low")==0)
				highres=false;
			else if(strcasecmp(argv[i]+1,"md")==0)
				{
				++i;
				maxDepth=(unsigned int)(atoi(argv[i]));
				}
			else if(strcasecmp(argv[i]+1,"fuzz")==0)
				{
				++i;
				backgroundRemovalFuzz=atoi(argv[i]);
				}
			}
		else
			{
			/* Add a streamer for the selected camera: */
			int cameraIndex=atoi(argv[i]);
			savers.push_back(new KinectSaver(usbContext,cameraIndex,highres,fileNamePrefix));
			}
		}
	
	if(numBackgroundFrames>0)
		{
		/* Enable background removal on all cameras: */
		for(std::vector<KinectSaver*>::iterator sIt=savers.begin();sIt!=savers.end();++sIt)
			{
			if(backgroundFileNamePrefix!=0)
				(*sIt)->getCamera().captureBackground(numBackgroundFrames,true,Misc::createFunctionCall(saveBackground));
			else
				(*sIt)->getCamera().captureBackground(numBackgroundFrames,true);
			
			if(maxDepth>0)
				(*sIt)->getCamera().setMaxDepth(maxDepth);
			
			(*sIt)->getCamera().setRemoveBackground(true);
			(*sIt)->getCamera().setBackgroundRemovalFuzz(backgroundRemovalFuzz);
			}
		}
	else if(backgroundFileNamePrefix!=0)
		{
		/* Load a background file and enable background removal on all cameras: */
		for(std::vector<KinectSaver*>::iterator sIt=savers.begin();sIt!=savers.end();++sIt)
			{
			(*sIt)->getCamera().loadBackground(backgroundFileNamePrefix);
			(*sIt)->getCamera().setRemoveBackground(true);
			(*sIt)->getCamera().setBackgroundRemovalFuzz(backgroundRemovalFuzz);
			}
		}
	
	/* Synchronize all cameras' time bases: */
	for(std::vector<KinectSaver*>::iterator sIt=savers.begin();sIt!=savers.end();++sIt)
		(*sIt)->getCamera().resetFrameTimer();
	
	/* Start recording: */
	for(std::vector<KinectSaver*>::iterator sIt=savers.begin();sIt!=savers.end();++sIt)
		(*sIt)->startStreaming();
	
	std::cout<<"Streaming... press any key to exit"<<std::endl;
	char key;
	std::cin>>key;
	
	/* Delete all video stream savers: */
	for(std::vector<KinectSaver*>::iterator sIt=savers.begin();sIt!=savers.end();++sIt)
		delete *sIt;
	
	return 0;
	}
