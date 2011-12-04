/***********************************************************************
KinectRecorder - Simple utility to save color and depth image streams
from one or more Kinect devices to a time-stamped file.
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
#include <Geometry/GeometryValueCoders.h>
#include <Kinect/USBContext.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/KinectCamera.h>
#include <Kinect/KinectFrameSaver.h>

/**************
Helper classes:
**************/
	
class KinectStreamer // Helper class to stream 3D video data from a Kinect camera to a time-stamped file
	{
	/* Elements: */
	public:
	KinectCamera* camera; // Pointer to the camera
	std::string serialNumber; // The camera's serial number
	KinectFrameSaver* frameSaver; // Pointer to helper object saving depth and color frames received from the Kinect
	
	/* Private methods: */
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	
	/* Constructors and destructors: */
	public:
	KinectStreamer(USBContext& usbContext,int cameraIndex,bool highres,const char* fileNamePrefix); // Creates a streamer for the Kinect camera of the given index in the given USB context
	~KinectStreamer(void); // Destroys the streamer
	
	/* Methods: */
	KinectCamera* getCamera(void) // Returns a pointer to the streamer's camera
		{
		return camera;
		}
	void startStreaming(void); // Begins streaming from the Kinect camera
	};

/*******************************
Methods of class KinectStreamer:
*******************************/

void KinectStreamer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Save the frame: */
	frameSaver->saveDepthFrame(frameBuffer);
	}

void KinectStreamer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	/* Save the frame: */
	frameSaver->saveColorFrame(frameBuffer);
	}

KinectStreamer::KinectStreamer(USBContext& context,int cameraIndex,bool highres,const char* fileNamePrefix)
	:camera(0),frameSaver(0)
	{
	/* Attach to and open the Kinect camera: */
	camera=new KinectCamera(context,cameraIndex);
	camera->open();
	
	/* Get the camera's serial number to load the proper calibration matrices: */
	serialNumber=camera->getSerialNumber();
	
	/* Create the name of the calibration matrices file: */
	std::string calibrationFileName="CameraCalibrationMatrices-";
	calibrationFileName.append(serialNumber);
	if(highres)
		calibrationFileName.append("-high");
	calibrationFileName.append(".dat");
	
	/* Read the camera's model space transformation: */
	std::string transformFileName="ProjectorTransform-";
	transformFileName.append(serialNumber);
	transformFileName.append(".txt");
	Misc::File transformFile(transformFileName.c_str(),"rt");
	char transform[1024];
	transformFile.gets(transform,sizeof(transform));
	KinectFrameSaver::Transform projectorTransform=Misc::ValueCoder<KinectFrameSaver::Transform>::decode(transform,transform+strlen(transform),0);
	
	/* Set the camera's frame size: */
	camera->setFrameSize(KinectCamera::COLOR,highres?KinectCamera::FS_1280_1024:KinectCamera::FS_640_480);
	
	/* Create the frame saver: */
	std::string depthFrameFileName=fileNamePrefix;
	depthFrameFileName.push_back('-');
	depthFrameFileName.append(serialNumber);
	depthFrameFileName.append(".depth");
	std::string colorFrameFileName=fileNamePrefix;
	colorFrameFileName.push_back('-');
	colorFrameFileName.append(serialNumber);
	colorFrameFileName.append(".color");
	frameSaver=new KinectFrameSaver(*camera,calibrationFileName.c_str(),projectorTransform,depthFrameFileName.c_str(),colorFrameFileName.c_str());
	}

KinectStreamer::~KinectStreamer(void)
	{
	/* Stop streaming: */
	camera->stopStreaming();
	
	/* Close and disconnect from the Kinect camera device: */
	delete camera;
	
	/* Delete the frame saver: */
	delete frameSaver;
	}

void KinectStreamer::startStreaming(void)
	{
	/* Start streaming: */
	camera->startStreaming(Misc::createFunctionCall(this,&KinectStreamer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectStreamer::depthStreamingCallback));
	}

/*************
Program state:
*************/
	
USBContext usbContext; // USB device context
std::vector<KinectStreamer*> streamers; // List of Kinect streamers, each connected to one Kinect camera
const char* backgroundFileNamePrefix=0;

void saveBackground(KinectCamera& camera)
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
	
	/* Add a streamer for each camera index passed on the command line: */
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
			streamers.push_back(new KinectStreamer(usbContext,cameraIndex,highres,fileNamePrefix));
			}
		}
	
	/* Synchronize all cameras' time bases: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->getCamera()->resetFrameTimer();
	
	/* Start recording: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		(*sIt)->startStreaming();
	
	if(numBackgroundFrames>0)
		{
		/* Enable background removal on all cameras: */
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			{
			if(backgroundFileNamePrefix!=0)
				(*sIt)->getCamera()->captureBackground(numBackgroundFrames,true,Misc::createFunctionCall(saveBackground));
			else
				(*sIt)->getCamera()->captureBackground(numBackgroundFrames,true);
			
			if(maxDepth>0)
				(*sIt)->getCamera()->setMaxDepth(maxDepth);
			
			(*sIt)->getCamera()->setRemoveBackground(true);
			(*sIt)->getCamera()->setBackgroundRemovalFuzz(backgroundRemovalFuzz);
			}
		}
	else if(backgroundFileNamePrefix!=0)
		{
		/* Load a background file and enable background removal on all cameras: */
		for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
			{
			(*sIt)->getCamera()->loadBackground(backgroundFileNamePrefix);
			(*sIt)->getCamera()->setRemoveBackground(true);
			(*sIt)->getCamera()->setBackgroundRemovalFuzz(backgroundRemovalFuzz);
			}
		}
	
	std::cout<<"Streaming... press any key to exit"<<std::endl;
	char key;
	std::cin>>key;
	
	/* Delete all streamers: */
	for(std::vector<KinectStreamer*>::iterator sIt=streamers.begin();sIt!=streamers.end();++sIt)
		delete *sIt;
	
	return 0;
	}
