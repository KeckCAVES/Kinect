/***********************************************************************
TiePointTool - Calibration tool for RawKinectViewer.
Copyright (c) 2010-2012 Oliver Kreylos

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

#include "TiePointTool.h"

#include <vector>
#include <iostream>
#include <Math/Math.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>

#include "RawKinectViewer.h"

/**********************************************************************
Static elements of class BlobProperty<Kinect::FrameSource::DepthPixel>:
**********************************************************************/

BlobProperty<Kinect::FrameSource::DepthPixel>::PTransform BlobProperty<Kinect::FrameSource::DepthPixel>::projection;

template <>
class PixelComparer<Kinect::FrameSource::DepthPixel> // Pixel comparer for depth frames
	{
	/* Embedded classes: */
	public:
	typedef Kinect::FrameSource::DepthPixel Pixel;
	
	/* Elements: */
	private:
	Pixel minPixelValue; // Minimal similar pixel value
	Pixel maxPixelValue; // Maximal similar pixel value
	
	/* Constructors and destructors: */
	public:
	PixelComparer(const Pixel& sPixelValue,unsigned int sThreshold)
		{
		if(sPixelValue>=sThreshold)
			minPixelValue=Pixel(sPixelValue-sThreshold);
		else
			minPixelValue=0U;
		if(sPixelValue<=0xffffU-sThreshold)
			maxPixelValue=Pixel(sPixelValue+sThreshold);
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
	PixelComparer(const Pixel& sPixelValue,unsigned int sThreshold)
		{
		for(int i=0;i<3;++i)
			{
			if(sPixelValue[i]>=sThreshold)
				minPixelValue[i]=GLubyte(sPixelValue[i]-sThreshold);
			else
				minPixelValue[i]=0U;
			if(sPixelValue[i]<=0xffU-sThreshold)
				maxPixelValue[i]=GLubyte(sPixelValue[i]+sThreshold);
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

/*************************************
Static elements of class TiePointTool:
*************************************/

TiePointToolFactory* TiePointTool::factory=0;

/*****************************
Methods of class TiePointTool:
*****************************/

TiePointToolFactory* TiePointTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new TiePointToolFactory("TiePointTool","Tie Points",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(2);
	factory->setButtonFunction(0,"Select Point");
	factory->setButtonFunction(1,"Save Point Pair");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

TiePointTool::TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 haveDepthPoint(false),haveColorPoint(false)
	{
	}

TiePointTool::~TiePointTool(void)
	{
	}

const Vrui::ToolFactory* TiePointTool::getFactory(void) const
	{
	return factory;
	}

void TiePointTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(buttonSlotIndex==0&&cbData->newButtonState)
		{
		/* Get the image-space device point: */
		Vrui::Point imagePoint=application->calcImagePoint(getButtonDeviceRay(0));
		int x=int(Math::floor(imagePoint[0]));
		int y=int(Math::floor(imagePoint[1]));
		
		/* Check whether the intersection point is inside the depth or the color image: */
		if(x>=-int(application->depthFrameSize[0])&&x<0&&y>=0&&y<int(application->depthFrameSize[1])) // It's a depth frame point
			{
			x+=int(application->depthFrameSize[0]);
			
			/****************************************
			Extract a blob around the selected pixel:
			****************************************/
			
			/* Calculate a low-pass filtered depth value for the selected pixel: */
			const DepthPixel* framePtr=static_cast<const DepthPixel*>(application->depthFrames.getLockedValue().getBuffer());
			double avgDepth=0.0;
			unsigned int numSamples=0;
			for(int dy=-2;dy<=2;++dy)
				if(y+dy>=0&&y+dy<int(application->depthFrameSize[1]))
					{
					for(int dx=-2;dx<=2;++dx)
						if(x+dx>=0&&x+dx<int(application->depthFrameSize[0]))
							{
							avgDepth+=double(framePtr[(y+dy)*application->depthFrameSize[0]+(x+dx)]);
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
			std::cout<<"Average raw depth value: "<<depthPoint.blobProperty.getRawDepth()<<std::endl;
			}
		else if(x>=0&&x<int(application->colorFrameSize[0])&&y>=0&&y<int(application->colorFrameSize[1])) // It's a color frame point
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
				if(y+dy>=0&&y+dy<int(application->colorFrameSize[1]))
					{
					for(int dx=-2;dx<=2;++dx)
						if(x+dx>=0&&x+dx<int(application->colorFrameSize[0]))
							{
							for(int i=0;i<3;++i)
								avgColor[i]+=double(framePtr[(y+dy)*application->colorFrameSize[0]+(x+dx)][i]);
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
	
	if(buttonSlotIndex==1&&cbData->newButtonState&&haveDepthPoint&&haveColorPoint)
		{
		/* Append a tie point pair to the calibration file: */
		BlobProperty<DepthPixel>::Point p=depthPoint.blobProperty.getCentroid();
		std::cout<<p[0]<<','<<p[1]<<','<<p[2]<<',';
		std::cout<<colorPoint.x<<','<<colorPoint.y<<std::endl;
		}
	}

void TiePointTool::display(GLContextData& contextData) const
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
			glVertex3f(depthPoint.min[0]-GLfloat(application->depthFrameSize[0]),depthPoint.min[1],0.01f);
			glVertex3f(depthPoint.max[0]-GLfloat(application->depthFrameSize[0]),depthPoint.min[1],0.01f);
			glVertex3f(depthPoint.max[0]-GLfloat(application->depthFrameSize[0]),depthPoint.max[1],0.01f);
			glVertex3f(depthPoint.min[0]-GLfloat(application->depthFrameSize[0]),depthPoint.max[1],0.01f);
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
