/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
Copyright (c) 2010 Oliver Kreylos

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

#include <stdexcept>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <Threads/TripleBuffer.h>
#include <Math/Math.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/Ray.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <GL/GLContextData.h>
#include <GL/GLObject.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/ToolManager.h>
#include <Vrui/LocatorTool.h>
#include <Vrui/Application.h>

#include "USBContext.h"
#include "FrameBuffer.h"
#include "KinectCamera.h"
#include "FindBlobs.h"

/**************
Helper classes:
**************/

template <>
class BlobProperty<unsigned short> // Blob property accumulator for depth frames; calculates 3D centroid of reprojected depth image pixels
	{
	/* Embedded classes: */
	public:
	typedef unsigned short Pixel;
	typedef Geometry::ProjectiveTransformation<double,3> PTransform;
	typedef PTransform::Point Point;
	
	/* Elements: */
	private:
	static Geometry::ProjectiveTransformation<double,3> projection; // Pixel re-projection matrix
	double pxs,pys,pzs; // Accumulated centroid
	size_t numPixels; // Number of accumulated pixels
	
	/* Constructors and destructors: */
	public:
	static void setProjection(const PTransform& sProjection) // Sets the re-projection matrix
		{
		projection=sProjection;
		}
	BlobProperty(void) // Creates empty accumulator
		:pxs(0.0),pys(0.0),pzs(0.0),numPixels(0)
		{
		}
	
	/* Methods: */
	void addPixel(int x,int y,const Pixel& pixelValue)
		{
		/* Reproject the pixel: */
		Point p=projection.transform(Point(double(x)+0.5,double(y+0.5),double(pixelValue)));
		
		/* Add the reprojected pixel to the plane equation accumulator: */
		pxs+=p[0];
		pys+=p[1];
		pzs+=p[2];
		++numPixels;
		}
	void merge(const BlobProperty& other)
		{
		/* Merge the other blob property's plane equation accumulator: */
		pxs+=other.pxs;
		pys+=other.pys;
		pzs+=other.pzs;
		numPixels+=other.numPixels;
		}
	Point getCentroid(void) const
		{
		Point result;
		result[0]=Point::Scalar(pxs/double(numPixels));
		result[1]=Point::Scalar(pys/double(numPixels));
		result[2]=Point::Scalar(pzs/double(numPixels));
		return result;
		}
	};

/*****************************************************
Static elements of class BlobProperty<unsigned short>:
*****************************************************/

BlobProperty<unsigned short>::PTransform BlobProperty<unsigned short>::projection;

template <>
class PixelComparer<unsigned short> // Pixel comparer for depth frames
	{
	/* Embedded classes: */
	public:
	typedef unsigned short Pixel;
	
	/* Elements: */
	private:
	Pixel minPixelValue; // Minimal similar pixel value
	Pixel maxPixelValue; // Maximal similar pixel value
	
	/* Constructors and destructors: */
	public:
	PixelComparer(const Pixel& sPixelValue,unsigned short sThreshold)
		{
		if(sPixelValue>=sThreshold)
			minPixelValue=sPixelValue-sThreshold;
		else
			minPixelValue=0U;
		if(sPixelValue<=0xffffU-sThreshold)
			maxPixelValue=sPixelValue+sThreshold;
		else
			maxPixelValue=0xffffU;
		}
	
	/* Methods: */
	bool operator()(const Pixel& pixel) const
		{
		return minPixelValue<=pixel&&pixel<=maxPixelValue;
		}
	};

template <>
class PixelComparer<GLColor<GLubyte,3> > // Pixel comparer for color frames
	{
	/* Embedded classes: */
	public:
	typedef GLColor<GLubyte,3> Pixel;
	
	/* Elements: */
	private:
	Pixel minPixelValue; // Minimal similar pixel value, in maximum norm
	Pixel maxPixelValue; // Maximal similar pixel value, in maximum norm
	
	/* Constructors and destructors: */
	public:
	PixelComparer(const Pixel& sPixelValue,unsigned short sThreshold)
		{
		for(int i=0;i<3;++i)
			{
			if(sPixelValue[i]>=sThreshold)
				minPixelValue[i]=sPixelValue[i]-sThreshold;
			else
				minPixelValue[i]=0U;
			if(sPixelValue[i]<=0xffU-sThreshold)
				maxPixelValue[i]=sPixelValue[i]+sThreshold;
			else
				maxPixelValue[i]=0xffU;
			}
		}
	
	/* Methods: */
	bool operator()(const Pixel& pixel) const
		{
		bool result=true;
		for(int i=0;i<3&&result;++i)
			result=minPixelValue[i]<=pixel[i]&&pixel[i]<=maxPixelValue[i];
		return result;
		}
	};

class RawKinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	class PauseTool;
	class TiePointTool;
	typedef Vrui::GenericToolFactory<PauseTool> PauseToolFactory; // Tool class uses the generic factory class
	typedef Vrui::GenericToolFactory<TiePointTool> TiePointToolFactory; // Tool class uses the generic factory class
	
	class PauseTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
		{
		friend class Vrui::GenericToolFactory<PauseTool>;
		
		/* Elements: */
		private:
		static PauseToolFactory* factory; // Pointer to the factory object for this class
		
		/* Constructors and destructors: */
		public:
		PauseTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
		virtual ~PauseTool(void);
		
		/* Methods from class Vrui::Tool: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		};
	
	class TiePointTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
		{
		friend class Vrui::GenericToolFactory<TiePointTool>;
		
		/* Embedded classes: */
		private:
		typedef unsigned short DepthPixel;
		typedef Blob<DepthPixel> DepthBlob;
		typedef GLColor<GLubyte,3> ColorPixel;
		typedef Blob<ColorPixel> ColorBlob;
		
		/* Elements: */
		static TiePointToolFactory* factory; // Pointer to the factory object for this class
		
		bool haveDepthPoint; // Flag whether a depth point has been selected
		DepthBlob depthPoint; // Blob containing the depth point
		bool haveColorPoint; // Flag whether a depth point has been selected
		ColorBlob colorPoint; // Blob containing the depth point
		
		/* Constructors and destructors: */
		public:
		TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
		virtual ~TiePointTool(void);
		
		/* Methods from class Vrui::Tool: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		virtual void display(GLContextData& contextData) const;
		};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint depthTextureId; // ID of texture object holding depth image
		unsigned int depthFrameVersion; // Version number of frame currently texture object
		GLuint colorTextureId; // ID of texture object holding color image
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	friend class PauseTool;
	friend class TiePointTool;
	
	/* Elements: */
	USBContext usbContext; // USB device context
	KinectCamera* kinectCamera; // Pointer to camera aspect of Kinect device
	Threads::TripleBuffer<FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of current depth frame
	Threads::TripleBuffer<FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of current color frame
	bool paused; // Flag whether the video stream display is paused
	unsigned int selectedPixel[2]; // Coordinates of the selected depth image pixel
	unsigned short selectedPixelPulse[128]; // EKG of depth value of selected pixel
	int selectedPixelCurrentIndex; // Index of most recent value in selected pixel's EKG
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData); // Callback when a locator tool's button is pressed
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	GLMotif::PopupMenu* createMainMenu(void); // Creates the program's main menu
	
	/* Constructors and destructors: */
	public:
	RawKinectViewer(int& argc,char**& argv,char**& appDefaults);
	virtual ~RawKinectViewer(void);
	
	/* Methods from Vrui::Application: */
	virtual void toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	
	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

/***************************************************
Static elements of class RawKinectViewer::PauseTool:
***************************************************/

RawKinectViewer::PauseToolFactory* RawKinectViewer::PauseTool::factory=0;

/*******************************************
Methods of class RawKinectViewer::PauseTool:
*******************************************/

RawKinectViewer::PauseTool::PauseTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment)
	{
	}

RawKinectViewer::PauseTool::~PauseTool(void)
	{
	}

const Vrui::ToolFactory* RawKinectViewer::PauseTool::getFactory(void) const
	{
	return factory;
	}

void RawKinectViewer::PauseTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		application->paused=!application->paused;
	}

/******************************************************
Static elements of class RawKinectViewer::TiePointTool:
******************************************************/

RawKinectViewer::TiePointToolFactory* RawKinectViewer::TiePointTool::factory=0;

/**********************************************
Methods of class RawKinectViewer::TiePointTool:
**********************************************/

RawKinectViewer::TiePointTool::TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 haveDepthPoint(false),haveColorPoint(false)
	{
	}

RawKinectViewer::TiePointTool::~TiePointTool(void)
	{
	}

const Vrui::ToolFactory* RawKinectViewer::TiePointTool::getFactory(void) const
	{
	return factory;
	}

void RawKinectViewer::TiePointTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(buttonSlotIndex==0&&cbData->newButtonState)
		{
		/* Intersect the tool's ray with the image plane: */
		Vrui::Ray ray=getButtonDeviceRay(0);
		ray.transform(Vrui::getInverseNavigationTransformation());
		if(ray.getDirection()[2]!=Vrui::Scalar(0))
			{
			Vrui::Scalar lambda=-ray.getOrigin()[2]/ray.getDirection()[2];
			Vrui::Point intersection=ray(lambda);
			int x=int(Math::floor(intersection[0]));
			int y=int(Math::floor(intersection[1]));
			
			/* Check whether the intersection point is inside the depth or the color image: */
			if(x>=-640&&x<0&&y>=0&&y<480) // It's a depth frame point
				{
				x+=640;
				
				/****************************************
				Extract a blob around the selected pixel:
				****************************************/
				
				/* Calculate a low-pass filtered depth value for the selected pixel: */
				const DepthPixel* framePtr=static_cast<const DepthPixel*>(application->depthFrames.getLockedValue().getBuffer());
				double avgDepth=0.0;
				unsigned int numSamples=0;
				for(int dy=-2;dy<=2;++dy)
					if(y+dy>=0&&y+dy<480)
						{
						for(int dx=-2;dx<=2;++dx)
							if(x+dx>=0&&x+dx<640)
								{
								avgDepth+=double(framePtr[(y+dy)*640+(x+dx)]);
								++numSamples;
								}
						}
				DepthPixel avg=DepthPixel(avgDepth/double(numSamples)+0.5);
				
				/* Extract all blobs with the average depth value: */
				PixelComparer<DepthPixel> pc(avg,10); // 10 is just a guess for now
				std::vector<DepthBlob> blobs=findBlobs(application->depthFrames.getLockedValue(),pc);
				
				/* Find the smallest blob containing the selected pixel: */
				unsigned int minSize=~0x0U;
				for(std::vector<DepthBlob>::iterator bIt=blobs.begin();bIt!=blobs.end();++bIt)
					{
					if(bIt->min[0]<=x&&x<bIt->max[0]&&bIt->min[1]<=y&&y<bIt->max[1])
						{
						unsigned int size=(unsigned int)(bIt->max[0]-bIt->min[0])*(unsigned int)(bIt->max[1]-bIt->min[1]);
						if(minSize>size)
							{
							depthPoint=*bIt;
							minSize=size;
							}
						}
					}
				haveDepthPoint=true;
				}
			else if(x>=0&&x<640&&y>=0&&y<480) // It's a color frame point
				{
				/****************************************
				Extract a blob around the selected pixel:
				****************************************/
				
				/* Calculate a low-pass filtered color value for the selected pixel: */
				const ColorPixel* framePtr=static_cast<const ColorPixel*>(application->colorFrames.getLockedValue().getBuffer());
				double avgColor[3];
				for(int i=0;i<3;++i)
					avgColor[i]=0.0;
				unsigned int numSamples=0;
				for(int dy=-2;dy<=2;++dy)
					if(y+dy>=0&&y+dy<480)
						{
						for(int dx=-2;dx<=2;++dx)
							if(x+dx>=0&&x+dx<640)
								{
								for(int i=0;i<3;++i)
									avgColor[i]+=double(framePtr[(y+dy)*640+(x+dx)][i]);
								++numSamples;
								}
						}
				ColorPixel avg;
				for(int i=0;i<3;++i)
					avg[i]=ColorPixel::Scalar(avgColor[i]/double(numSamples)+0.5);
				
				/* Extract all blobs with the average color value: */
				PixelComparer<ColorPixel> pc(avg,25); // 25 is just a guess for now
				std::vector<ColorBlob> blobs=findBlobs(application->colorFrames.getLockedValue(),pc);
				
				/* Find the smallest blob containing the selected pixel: */
				unsigned int minSize=~0x0U;
				for(std::vector<ColorBlob>::iterator bIt=blobs.begin();bIt!=blobs.end();++bIt)
					{
					if(bIt->min[0]<=x&&x<bIt->max[0]&&bIt->min[1]<=y&&y<bIt->max[1])
						{
						unsigned int size=(unsigned int)(bIt->max[0]-bIt->min[0])*(unsigned int)(bIt->max[1]-bIt->min[1]);
						if(minSize>size)
							{
							colorPoint=*bIt;
							minSize=size;
							}
						}
					}
				haveColorPoint=true;
				}
			}
		}
	
	if(buttonSlotIndex==1&&cbData->newButtonState&&haveDepthPoint&&haveColorPoint)
		{
		/* Append a tie point pair to the calibration file: */
		BlobProperty<unsigned short>::Point p=depthPoint.blobProperty.getCentroid();
		std::cout<<p[0]<<','<<p[1]<<','<<p[2]<<',';
		std::cout<<colorPoint.x<<','<<colorPoint.y<<std::endl;
		}
	}

void RawKinectViewer::TiePointTool::display(GLContextData& contextData) const
	{
	if(haveDepthPoint||haveColorPoint)
		{
		glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT);
		glDisable(GL_LIGHTING);
		
		/* Go to navigation coordinates: */
		glPushMatrix();
		const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
		glLoadMatrix(displayState.modelviewNavigational);
		
		if(haveDepthPoint)
			{
			/* Draw the depth blob: */
			glLineWidth(1.0f);
			glColor3f(0.0f,1.0f,0.0f);
			glBegin(GL_LINE_LOOP);
			glVertex3f(depthPoint.min[0]-640,depthPoint.min[1],0.01f);
			glVertex3f(depthPoint.max[0]-640,depthPoint.min[1],0.01f);
			glVertex3f(depthPoint.max[0]-640,depthPoint.max[1],0.01f);
			glVertex3f(depthPoint.min[0]-640,depthPoint.max[1],0.01f);
			glEnd();
			}
		
		if(haveColorPoint)
			{
			/* Draw the color blob: */
			glLineWidth(1.0f);
			glColor3f(0.0f,1.0f,0.0f);
			glBegin(GL_LINE_LOOP);
			glVertex3f(colorPoint.min[0],colorPoint.min[1],0.01f);
			glVertex3f(colorPoint.max[0],colorPoint.min[1],0.01f);
			glVertex3f(colorPoint.max[0],colorPoint.max[1],0.01f);
			glVertex3f(colorPoint.min[0],colorPoint.max[1],0.01f);
			glEnd();
			}
		
		glPopMatrix();
		
		glPopAttrib();
		}
	}

/******************************************
Methods of class RawKinectViewer::DataItem:
******************************************/

RawKinectViewer::DataItem::DataItem(void)
	:depthTextureId(0),depthFrameVersion(0),
	 colorTextureId(0),colorFrameVersion(0)
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

void RawKinectViewer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		/* Post the new frame into the depth frame triple buffer: */
		depthFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	if(!paused)
		{
		/* Post the new frame into the color frame triple buffer: */
		colorFrames.postNewValue(frameBuffer);
		
		/* Update application state: */
		Vrui::requestUpdate();
		}
	}

void RawKinectViewer::locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData)
	{
	Vrui::Point pos=cbData->currentTransformation.getOrigin();
	if(pos[0]>=-640.0&&pos[0]<0.0&&pos[1]>=0.0&&pos[1]<480.0)
		{
		/* Select the pixel under the locator: */
		selectedPixel[0]=int(pos[0]+640.0);
		selectedPixel[1]=int(pos[1]);
		
		/* Start the selected pixel's EKG: */
		selectedPixelCurrentIndex=0;
		const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
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

void RawKinectViewer::captureBackgroundCallback(Misc::CallbackData* cbData)
	{
	/* Capture one second worth of background frames: */
	kinectCamera->captureBackground(150);
	}

void RawKinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	kinectCamera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(kinectCamera->getRemoveBackground());
	}

GLMotif::PopupMenu* RawKinectViewer::createMainMenu(void)
	{
	/* Create a popup shell to hold the main menu: */
	GLMotif::PopupMenu* mainMenuPopup=new GLMotif::PopupMenu("MainMenuPopup",Vrui::getWidgetManager());
	mainMenuPopup->setTitle("Raw Kinect Viewer");
	
	/* Create the main menu itself: */
	GLMotif::Menu* mainMenu=new GLMotif::Menu("MainMenu",mainMenuPopup,false);
	
	/* Create a button to capture a background frame: */
	GLMotif::Button* captureBackgroundButton=new GLMotif::Button("CaptureBackgroundButton",mainMenu,"Capture Background");
	captureBackgroundButton->getSelectCallbacks().add(this,&RawKinectViewer::captureBackgroundCallback);
	
	/* Create a toggle button to enable/disable background removal: */
	GLMotif::ToggleButton* removeBackgroundToggle=new GLMotif::ToggleButton("RemoveBackgroundToggle",mainMenu,"Remove Background");
	removeBackgroundToggle->setToggle(kinectCamera->getRemoveBackground());
	removeBackgroundToggle->getValueChangedCallbacks().add(this,&RawKinectViewer::removeBackgroundCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

RawKinectViewer::RawKinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 kinectCamera(0),
	 depthFrameVersion(0),colorFrameVersion(0),
	 paused(false),
	 mainMenu(0)
	{
	/* Register the custom tool class with the Vrui tool manager: */
	PauseToolFactory* toolFactory1=new PauseToolFactory("PauseTool","Pause",0,*Vrui::getToolManager());
	toolFactory1->setNumButtons(1);
	toolFactory1->setButtonFunction(0,"Pause");
	Vrui::getToolManager()->addClass(toolFactory1,Vrui::ToolManager::defaultToolFactoryDestructor);
	
	TiePointToolFactory* toolFactory2=new TiePointToolFactory("TiePointTool","Tie Points",0,*Vrui::getToolManager());
	toolFactory2->setNumButtons(2);
	toolFactory2->setButtonFunction(0,"Select Point");
	toolFactory2->setButtonFunction(1,"Save Point Pair");
	Vrui::getToolManager()->addClass(toolFactory2,Vrui::ToolManager::defaultToolFactoryDestructor);
	
	/* Open the calibration file: */
	Misc::File calibrationFile("CameraCalibrationMatrices.dat","rb",Misc::File::LittleEndian);
	
	/* Read the depth projection matrix: */
	BlobProperty<unsigned short>::PTransform projection;
	calibrationFile.read(projection.getMatrix().getEntries(),4*4);
	BlobProperty<unsigned short>::setProjection(projection);
	
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to the given Kinect camera device on the host: */
	int cameraIndex=0;
	if(argc>=2)
		cameraIndex=atoi(argv[1]);
	kinectCamera=new KinectCamera(usbContext,cameraIndex);
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Start streaming: */
	kinectCamera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::depthStreamingCallback));
	
	/* Select an invalid pixel: */
	selectedPixel[0]=selectedPixel[1]=~0x0U;
	
	/* Initialize navigation transformation: */
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(640),Vrui::Vector(0,1,0));
	}

RawKinectViewer::~RawKinectViewer(void)
	{
	delete mainMenu;
	
	/* Stop streaming: */
	kinectCamera->stopStreaming();
	
	/* Disconnect from the Kinect camera device: */
	delete kinectCamera;
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
	/* Lock the most recent frame in the depth frame triple buffer: */
	if(depthFrames.lockNewValue())
		{
		++depthFrameVersion;
		
		if(selectedPixel[0]!=~0x0&&selectedPixel[1]!=~0x0)
			{
			/* Update the selected pixel's EKG: */
			++selectedPixelCurrentIndex;
			if(selectedPixelCurrentIndex==128)
				selectedPixelCurrentIndex=0;
			const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
			selectedPixelPulse[selectedPixelCurrentIndex]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
			}
		}
	
	/* Lock the most recent frame in the color frame triple buffer: */
	if(colorFrames.lockNewValue())
		++colorFrameVersion;
	}

void RawKinectViewer::display(GLContextData& contextData) const
	{
	/* Get the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Save and set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT|GL_TEXTURE_BIT);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	
	/* Bind the depth texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(dataItem->depthFrameVersion!=depthFrameVersion)
		{
		/* Upload the depth frame into the texture object: */
		const FrameBuffer& depthFrame=depthFrames.getLockedValue();
		unsigned int width=depthFrame.getSize(0);
		unsigned int height=depthFrame.getSize(1);
		const GLushort* framePtr=static_cast<const GLushort*>(depthFrame.getBuffer());
		
		/* Convert the depth image to unsigned byte: */
		GLubyte* byteFrame=new GLubyte[height*width];
		const GLushort* fPtr=framePtr;
		GLubyte* bfPtr=byteFrame;
		for(unsigned int y=0;y<height;++y)
			for(unsigned int x=0;x<width;++x,++fPtr,++bfPtr)
				*bfPtr=GLubyte(255U-(unsigned int)(*fPtr)*256U/(0x0800U));
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the depth texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE8,width,height,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,byteFrame);
		
		delete[] byteFrame;
		
		/* Mark the cached depth frame as up-to-date: */
		dataItem->depthFrameVersion=depthFrameVersion;
		}
	
	/* Draw the depth image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(-640.0f,0.0f);
	glTexCoord2f(1.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(0.0f,480.0f);
	glTexCoord2f(0.0f,1.0f);
	glVertex2f(-640.0f,480.0f);
	glEnd();
	
	/* Bind the color texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->colorTextureId);
	
	/* Check if the cached color frame needs to be updated: */
	if(dataItem->colorFrameVersion!=colorFrameVersion)
		{
		/* Upload the color frame into the texture object: */
		const FrameBuffer& colorFrame=colorFrames.getLockedValue();
		unsigned int width=colorFrame.getSize(0);
		unsigned int height=colorFrame.getSize(1);
		const GLubyte* framePtr=static_cast<const GLubyte*>(colorFrame.getBuffer());
		
		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		
		/* Upload the color texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGB8,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,framePtr);
		
		/* Mark the cached color frame as up-to-date: */
		dataItem->colorFrameVersion=colorFrameVersion;
		}
	
	/* Draw the color image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(1.0f,0.0f);
	glVertex2f(640.0f,0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(640.0f,480.0f);
	glTexCoord2f(0.0f,1.0f);
	glVertex2f(0.0f,480.0f);
	glEnd();
	
	/* Protect the texture objects: */
	glBindTexture(GL_TEXTURE_2D,0);
	
	if(selectedPixel[0]!=~0x0&&selectedPixel[1]!=~0x0)
		{
		/* Draw the selected pixel: */
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		
		glBegin(GL_LINES);
		glColor3f(0.0f,1.0f,0.0f);
		GLfloat spx=GLfloat(selectedPixel[0])-639.5f;
		GLfloat spy=GLfloat(selectedPixel[1])+0.5f;
		glVertex3f(spx-5.0f,spy,0.1f);
		glVertex3f(spx+5.0f,spy,0.1f);
		glVertex3f(spx,spy-5.0f,0.1f);
		glVertex3f(spx,spy+5.0f,0.1f);
		glEnd();
		
		/* Draw the selected pixel's EKG: */
		glBegin(GL_LINE_STRIP);
		for(int i=0;i<128;++i)
			glVertex3f(GLfloat(i)*640.0f/128.0f-640.0f,GLfloat(selectedPixelPulse[i])*0.25-512.0f,0.1f);
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
