/***********************************************************************
RawKinectViewer - Simple application to view color and depth images
captured from a Kinect device.
Copyright (c) 2010-2011 Oliver Kreylos

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
#include <Misc/FunctionCalls.h>
#include <Misc/File.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#include <Geometry/Box.h>
#include <Geometry/Ray.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLContextData.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/Button.h>
#include <Vrui/Vrui.h>
#include <Vrui/DisplayState.h>
#include <Vrui/ToolManager.h>
#include <Kinect/KinectCamera.h>

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

/**************************************************
Static elements of class RawKinectViewer::LineTool:
**************************************************/

RawKinectViewer::LineToolFactory* RawKinectViewer::LineTool::factory=0;

/******************************************
Methods of class RawKinectViewer::LineTool:
******************************************/

RawKinectViewer::LineTool::Point RawKinectViewer::LineTool::getPoint(void) const
	{
	/* Intersect the tool's ray with the image plane: */
	Vrui::Ray ray=getButtonDeviceRay(0);
	ray.transform(Vrui::getInverseNavigationTransformation());
	if(ray.getDirection()[2]!=Vrui::Scalar(0))
		{
		Vrui::Scalar lambda=-ray.getOrigin()[2]/ray.getDirection()[2];
		Vrui::Point p=ray(lambda);
		return Point(p[0],p[1]);
		}
	else
		return Point(0.0,0.0);
	}

std::pair<RawKinectViewer::LineTool::LineSet::iterator,int> RawKinectViewer::LineTool::findLine(const RawKinectViewer::LineTool::Point& p)
	{
	std::pair<LineSet::iterator,int> result(lines.end(),-1);
	
	/* Check all line endpoints: */
	double maxDist2=Math::sqr(2.0);
	for(LineSet::iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		{
		double startDist2=Geometry::sqrDist(p,lIt->start);
		if(maxDist2>startDist2)
			{
			result.first=lIt;
			result.second=0;
			maxDist2=startDist2;
			}
		double endDist2=Geometry::sqrDist(p,lIt->end);
		if(maxDist2>endDist2)
			{
			result.first=lIt;
			result.second=1;
			maxDist2=endDist2;
			}
		}
	
	if(result.second>=0)
		return result;
	
	/* Check all lines: */
	double maxDist=1.0;
	for(LineSet::iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		{
		Point::Vector dir=lIt->end-lIt->start;
		Point::Vector norm=Geometry::normal(dir);
		Point::Vector off=p-lIt->start;
		double dist=Math::abs(norm*off)/Geometry::mag(norm);
		if(maxDist>dist)
			{
			double len=dir*off;
			if(len>=0.0&&len<=Geometry::sqr(dir))
				{
				result.first=lIt;
				result.second=2;
				maxDist=dist;
				}
			}
		}
	
	return result;
	}

std::vector<RawKinectViewer::LineTool::LineSet::iterator> RawKinectViewer::LineTool::getIntersectingLines(RawKinectViewer::LineTool::LineSet::iterator base,RawKinectViewer::LineTool::LineSet& lines)
	{
	Point::Vector n0=Geometry::normal(base->end-base->start);
	double o0=n0*Geometry::mid(base->start,base->end);
	
	/* Find all other lines intersecting the selected one: */
	std::vector<std::pair<LineSet::iterator,double> > orderedLines;
	for(LineSet::iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		if(lIt!=base)
			{
			Point::Vector n1=Geometry::normal(lIt->end-lIt->start);
			double o1=n1*Geometry::mid(lIt->start,lIt->end);
			
			/* Check if the lines intersect: */
			if((lIt->start*n0-o0)*(lIt->end*n0-o0)<0.0&&(base->start*n1-o1)*(base->end*n1-o1)<0.0)
				{
				/* Insert the line into the row list: */
				double d1=base->end*n1-o1;
				double d0=base->start*n1-o1;
				double lambda=(0.0-d0)/(d1-d0);
				orderedLines.push_back(std::pair<LineSet::iterator,double>(lIt,lambda));
				for(std::vector<std::pair<LineSet::iterator,double> >::iterator olIt=orderedLines.end()-1;olIt!=orderedLines.begin();--olIt)
					{
					std::vector<std::pair<LineSet::iterator,double> >::iterator olIt2=olIt-1;
					if(olIt2->second>olIt->second)
						std::swap(*olIt,*olIt2);
					}
				}
			}
	
	/* Assemble the result set: */
	std::vector<LineSet::iterator> result;
	for(std::vector<std::pair<LineSet::iterator,double> >::iterator olIt=orderedLines.begin();olIt!=orderedLines.end();++olIt)
		result.push_back(olIt->first);
	return result;
	}

void RawKinectViewer::LineTool::makeCell(const RawKinectViewer::LineTool::Line& l0,const RawKinectViewer::LineTool::Line& l1,RawKinectViewer::LineTool::Point::Vector n[2],double o[2])
	{
	n[0]=Geometry::normal(l0.end-l0.start);
	Point c0=Geometry::mid(l0.start,l0.end);
	o[0]=c0*n[0];
	n[1]=Geometry::normal(l1.end-l1.start);
	Point c1=Geometry::mid(l1.start,l1.end);
	o[1]=c1*n[1];
	if(n[0]*c1<o[0])
		{
		n[0]=-n[0];
		o[0]=-o[0];
		}
	if(n[1]*c0<o[1])
		{
		n[1]=-n[1];
		o[1]=-o[1];
		}
	}

RawKinectViewer::LineTool::Point RawKinectViewer::LineTool::intersectLines(const RawKinectViewer::LineTool::Line& l0,const RawKinectViewer::LineTool::Line& l1)
	{
	Point::Vector n0=Geometry::normal(l0.end-l0.start);
	double o0=Geometry::mid(l0.start,l0.end)*n0;
	double d1=l1.start*n0-o0;
	double d2=l1.end*n0-o0;
	double lambda=(0.0-d1)/(d2-d1);
	return Geometry::affineCombination(l1.start,l1.end,lambda);
	}

bool RawKinectViewer::LineTool::inside(unsigned int x,unsigned int y,const RawKinectViewer::LineTool::Point::Vector ln[4],const double lo[4])
	{
	for(int i=0;i<4;++i)
		if((double(x)+0.5)*ln[i][0]+(double(y)+0.5)*ln[i][1]<lo[i])
			return false;
	return true;
	}

void RawKinectViewer::LineTool::constructGrid(void)
	{
	/* Start with an arbitrary line: */
	LineSet::iterator l0=lines.begin();
	Point::Vector n0=l0->end-l0->start;
	
	/* Find all other lines intersecting the selected one: */
	std::vector<LineSet::iterator> rows=getIntersectingLines(l0,lines);
	
	/* Build the column list by repeating the same process with the middle row: */
	std::vector<LineSet::iterator> columns=getIntersectingLines(rows[rows.size()/2],lines);
	
	/* Calculate the box containing all grid corners: */
	Geometry::Box<double,2> box=Geometry::Box<double,2>::empty;
	for(std::vector<LineSet::iterator>::iterator rIt=rows.begin();rIt!=rows.end();++rIt)
		for(std::vector<LineSet::iterator>::iterator cIt=columns.begin();cIt!=columns.end();++cIt)
			box.addPoint(intersectLines(**rIt,**cIt));
	
	/* Check if the grid is in the color or depth frame: */
	if(box.min[0]>=0.0)
		{
		/* Do something meaningful... */
		}
	else
		{
		/* Offset all rows and columns: */
		for(std::vector<LineSet::iterator>::iterator rIt=rows.begin();rIt!=rows.end();++rIt)
			{
			(*rIt)->start[0]+=double(application->depthFrameSize[0]);
			(*rIt)->end[0]+=double(application->depthFrameSize[0]);
			}
		for(std::vector<LineSet::iterator>::iterator cIt=columns.begin();cIt!=columns.end();++cIt)
			{
			(*cIt)->start[0]+=double(application->depthFrameSize[0]);
			(*cIt)->end[0]+=double(application->depthFrameSize[0]);
			}
		
		/* Iterate through all grid cells: */
		Geometry::PCACalculator<3> oddPca;
		Geometry::PCACalculator<3> evenPca;
		Point::Vector ln[4];
		double lo[4];
		gridSquares.clear();
		float fgCutoff=float(application->averageNumFrames)*0.5f;
		for(unsigned int ri=1;ri<rows.size();++ri)
			{
			makeCell(*rows[ri-1],*rows[ri],ln,lo);
			for(unsigned int ci=1;ci<columns.size();++ci)
				{
				makeCell(*columns[ci-1],*columns[ci],ln+2,lo+2);
				
				/* Process all average depth frame pixels inside the grid cell: */
				Point cellp[4];
				cellp[0]=intersectLines(*rows[ri-1],*columns[ci-1]);
				cellp[1]=intersectLines(*rows[ri-1],*columns[ci]);
				cellp[2]=intersectLines(*rows[ri],*columns[ci]);
				cellp[3]=intersectLines(*rows[ri],*columns[ci-1]);
				Geometry::Box<double,2> box=Geometry::Box<double,2>::empty;
				for(int i=0;i<4;++i)
					box.addPoint(cellp[i]);
				unsigned int x0=box.min[0]<=0.0?0:(unsigned int)(box.min[0]);
				unsigned int x1=box.max[0]>=double(application->depthFrameSize[0]-1)?application->depthFrameSize[0]:(unsigned int)(box.max[0]+1.0);
				unsigned int y0=box.min[1]<=0.0?0:(unsigned int)(box.min[1]);
				unsigned int y1=box.max[1]>=double(application->depthFrameSize[1]-1)?application->depthFrameSize[1]:(unsigned int)(box.max[1]+1.0);
				
				/* Accumulate the cell's foreground points in the PCA calculator: */
				Geometry::PCACalculator<3>& pca=((ri-1)+(ci-1))%2==0?evenPca:oddPca;
				const float* afdRowPtr=application->averageFrameDepth+y0*application->depthFrameSize[0];
				const float* affRowPtr=application->averageFrameForeground+y0*application->depthFrameSize[0];
				for(unsigned int y=y0;y<y1;++y,afdRowPtr+=application->depthFrameSize[0],affRowPtr+=application->depthFrameSize[0])
					{
					const float* afdPtr=afdRowPtr+x0;
					const float* affPtr=affRowPtr+x0;
					for(unsigned int x=x0;x<x1;++x,++afdPtr,++affPtr)
						if(inside(x,y,ln,lo))
							{
							if(*affPtr>=fgCutoff)
								{
								Geometry::Point<double,3> p;
								p[0]=x;
								p[1]=y;
								p[2]=(*afdPtr)/(*affPtr);
								pca.accumulatePoint(p);
								}
							}
					}
				}
			}
		
		/* Calculate the odd and even plane equations: */
		for(int i=0;i<2;++i)
			{
			Geometry::PCACalculator<3>& pca=i==0?evenPca:oddPca;
			
			/* Calculate the grid's plane equation: */
			Geometry::PCACalculator<3>::Point centroid=pca.calcCentroid();
			pca.calcCovariance();
			double evs[3];
			pca.calcEigenvalues(evs);
			Geometry::PCACalculator<3>::Vector normal=pca.calcEigenvector(evs[2]);
			double offset=normal*centroid;
			
			std::cout<<"Plane equation: "<<normal<<", "<<centroid<<", "<<offset<<std::endl;
			std::cout<<"Eigenvalues: "<<evs[0]<<", "<<evs[1]<<", "<<evs[2]<<std::endl;
			}
		}
	haveGrid=true;
	}

RawKinectViewer::LineTool::LineTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 dragging(false),haveGrid(false)
	{
	}

RawKinectViewer::LineTool::~LineTool(void)
	{
	}

const Vrui::ToolFactory* RawKinectViewer::LineTool::getFactory(void) const
	{
	return factory;
	}

void RawKinectViewer::LineTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(buttonSlotIndex==0)
		{
		if(cbData->newButtonState)
			{
			/* Get the interaction point: */
			Point p=getPoint();
			
			/* Check it against all existing lines: */
			std::pair<LineSet::iterator,int> lineHit=findLine(p);
			dragging=true;
			dragMode=lineHit.second;
			if(dragMode>=0)
				{
				/* Drag an existing line: */
				draggedLine=lineHit.first;
				haveGrid=false;
				
				/* Calculate the dragging offsets: */
				startOffset=p-draggedLine->start;
				endOffset=p-draggedLine->end;
				}
			else
				{
				/* Start dragging a new line: */
				current.start=p;
				haveGrid=false;
				}
			}
		else if(dragging)
			{
			if(dragMode<0)
				{
				/* Finish the new line: */
				lines.push_back(current);
				haveGrid=false;
				}
			
			dragging=false;
			}
		}
	else if(buttonSlotIndex==1)
		{
		if(cbData->newButtonState)
			{
			/* Get the interaction point: */
			Point p=getPoint();
			
			/* Check it against all existing lines: */
			std::pair<LineSet::iterator,int> lineHit=findLine(p);
			if(lineHit.first!=lines.end())
				{
				/* Delete the selected line: */
				lines.erase(lineHit.first);
				haveGrid=false;
				}
			}
		}
	else
		{
		constructGrid();
		}
	}

void RawKinectViewer::LineTool::frame(void)
	{
	if(dragging)
		{
		/* Get the interaction point: */
		Point p=getPoint();
		
		switch(dragMode)
			{
			case -1:
				current.end=p;
				break;
			
			case 0:
				draggedLine->start=p-startOffset;
				break;
			
			case 1:
				draggedLine->end=p-endOffset;
				break;
			
			case 2:
				draggedLine->start=p-startOffset;
				draggedLine->end=p-endOffset;
				break;
			}
		}
	}

void RawKinectViewer::LineTool::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0f);
	glPointSize(3.0f);
	glColor3f(0.0f,1.0f,0.0f);
	
	/* Go to navigation coordinates: */
	glPushMatrix();
	const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
	glLoadMatrix(displayState.modelviewNavigational);
	
	/* Draw all existing lines: */
	glBegin(GL_LINES);
	for(LineSet::const_iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		{
		glVertex3d(lIt->start[0],lIt->start[1],0.01);
		glVertex3d(lIt->end[0],lIt->end[1],0.01);
		}
	glEnd();
	glBegin(GL_POINTS);
	for(LineSet::const_iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		{
		glVertex3d(lIt->start[0],lIt->start[1],0.01);
		glVertex3d(lIt->end[0],lIt->end[1],0.01);
		}
	glEnd();
	
	if(dragging&&dragMode==-1)
		{
		/* Draw the current line: */
		glBegin(GL_LINES);
		glVertex3d(current.start[0],current.start[1],0.01);
		glVertex3d(current.end[0],current.end[1],0.01);
		glEnd();
		glBegin(GL_POINTS);
		glVertex3d(current.start[0],current.start[1],0.01);
		glVertex3d(current.end[0],current.end[1],0.01);
		glEnd();
		}
	
	if(haveGrid)
		{
		/* Draw all grid squares: */
		glBegin(GL_QUADS);
		for(std::vector<Point>::const_iterator gsIt=gridSquares.begin();gsIt!=gridSquares.end();++gsIt)
			glVertex3d((*gsIt)[0],(*gsIt)[1],0.01);
		glEnd();
		}
	
	glPopMatrix();
	
	glPopAttrib();
	}

/**************************************************
Static elements of class RawKinectViewer::GridTool:
**************************************************/

RawKinectViewer::GridToolFactory* RawKinectViewer::GridTool::factory=0;
int RawKinectViewer::GridTool::gridSize[2];
double RawKinectViewer::GridTool::tileSize[2];

/******************************************
Methods of class RawKinectViewer::GridTool:
******************************************/

RawKinectViewer::GridTool::Point RawKinectViewer::GridTool::getPoint(void) const
	{
	/* Intersect the tool's ray with the image plane: */
	Vrui::Ray ray=getButtonDeviceRay(0);
	ray.transform(Vrui::getInverseNavigationTransformation());
	if(ray.getDirection()[2]!=Vrui::Scalar(0))
		{
		Vrui::Scalar lambda=-ray.getOrigin()[2]/ray.getDirection()[2];
		Vrui::Point p=ray(lambda);
		return Point(p[0],p[1]);
		}
	else
		return Point(0.0,0.0);
	}

RawKinectViewer::GridTool::Homography RawKinectViewer::GridTool::calcHomography(const RawKinectViewer::GridTool::Point gridPoints[4],const RawKinectViewer::GridTool::Point imagePoints[4])
	{
	/* Create the linear system: */
	Math::Matrix a(9,9,0.0);
	for(int p=0;p<4;++p)
		{
		for(int i=0;i<2;++i)
			{
			for(int j=0;j<2;++j)
				{
				a(p*2+i,i*3+j)=gridPoints[p][j];
				a(p*2+i,6+j)=-imagePoints[p][i]*gridPoints[p][j];
				}
			a(p*2+i,i*3+2)=1.0;
			a(p*2+i,8)=-imagePoints[p][i];
			}
		}
	
	/* Calculate the linear system's null space: */
	Math::Matrix x=a.kernel();
	
	/* Create the result homography: */
	Homography result;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			result.getMatrix()(i,j)=x(i*3+j,0);
	return result;
	}

void RawKinectViewer::GridTool::initHoms(void)
	{
	/* Initialize the depth and image grid homographies: */
	lastDraggedPoints[0]=Point(0.0,0.0);
	lastDraggedPoints[1]=Point(double(gridSize[0]),0.0);
	lastDraggedPoints[2]=Point(0.0,double(gridSize[1]));
	lastDraggedPoints[3]=Point(double(gridSize[0]),double(gridSize[1]));
	Point imagePoints[4];
	imagePoints[0]=Point(-double(application->depthFrameSize[0])+100.0,100.0);
	imagePoints[1]=Point(-100.0,100.0);
	imagePoints[2]=Point(-double(application->depthFrameSize[0])+100.0,double(application->depthFrameSize[1])-100.0);
	imagePoints[3]=Point(-100.0,double(application->depthFrameSize[1])-100.0);
	homs[0]=calcHomography(lastDraggedPoints,imagePoints);
	imagePoints[0]=Point(100.0,100.0);
	imagePoints[1]=Point(double(application->colorFrameSize[0])-100.0,100.0);
	imagePoints[2]=Point(100.0,double(application->colorFrameSize[1])-100.0);
	imagePoints[3]=Point(double(application->colorFrameSize[0])-100.0,double(application->colorFrameSize[1])-100.0);
	homs[1]=calcHomography(lastDraggedPoints,imagePoints);
	}

void RawKinectViewer::GridTool::createTiePoint(void)
	{
	/* Calculate the grid's plane equation in depth image space: */
	Geometry::PCACalculator<3> pca;
	
	const float* afdPtr=application->averageFrameDepth;
	const float* affPtr=application->averageFrameForeground;
	float foregroundCutoff=float(application->averageNumFrames)*0.5f;
	for(unsigned int y=0;y<application->depthFrameSize[1];++y)
		{
		double dy=double(y)+0.5;
		for(unsigned int x=0;x<application->depthFrameSize[0];++x,++afdPtr,++affPtr)
			{
			double dx=double(x)-double(application->depthFrameSize[0])+0.5;
			if(*affPtr>=foregroundCutoff)
				{
				/* Determine the pixel's grid position: */
				Point gp=homs[0].inverseTransform(Point(dx,dy));
				if(gp[0]>=0.0&&gp[0]<double(gridSize[0])&&gp[1]>=0.0&&gp[1]<double(gridSize[1]))
					if((int(gp[0])+int(gp[1]))%2==0)
						{
						double gx=gp[0]-Math::floor(gp[0]);
						double gy=gp[1]-Math::floor(gp[1]);
						if(gx>=0.2&&gx<0.8&&gy>=0.2&&gy<0.8)
							pca.accumulatePoint(Geometry::PCACalculator<3>::Point(dx,dy,*afdPtr/(*affPtr)));
						}
				}
			}
		}
	
	/* Calculate the grid's plane equation: */
	Geometry::PCACalculator<3>::Point centroid=pca.calcCentroid();
	pca.calcCovariance();
	double evs[3];
	pca.calcEigenvalues(evs);
	Geometry::PCACalculator<3>::Vector normal=pca.calcEigenvector(evs[2]);
	
	/* Check for any nans or infs: */
	bool allFinite=true;
	for(int i=0;i<3;++i)
		{
		allFinite=allFinite&&Math::isFinite(normal[i]);
		allFinite=allFinite&&Math::isFinite(centroid[i]);
		}
	
	if(allFinite)
		{
		/* Create and store a tie point: */
		TiePoint newTp;
		newTp.depthHom=homs[0];
		newTp.gridPlane=Plane(normal,centroid);
		newTp.colorHom=homs[1];
		tiePoints.push_back(newTp);
		}
	else
		Vrui::showErrorMessage("GridTool","Could not create tie point due to undefined grid plane equation");
	}

void RawKinectViewer::GridTool::calibrate(void)
	{
	if(tiePoints.size()<2)
		Vrui::showErrorMessage("GridTool","Need at least two tie points to calibrate");
	
	/* Initialize the depth camera's intrinsic parameter matrix: */
	Math::Matrix depthV(6,6,0.0);
	
	/* Process all tie points: */
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Enter the tie point's depth homography into the intrinsic parameter matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(application->depthFrameSize[0]);
		hm*=tpIt->depthHom.getMatrix();
		Homography::Matrix scale(1.0);
		scale(0,0)=1.0/tileSize[0];
		scale(1,1)=1.0/tileSize[1];
		hm*=scale;
		double row[3][6];
		static const int is[3]={0,0,1};
		static const int js[3]={1,0,1};
		for(int r=0;r<3;++r)
			{
			int i=is[r];
			int j=js[r];
			row[r][0]=hm(0,i)*hm(0,j);
			row[r][1]=hm(0,i)*hm(1,j)+hm(1,i)*hm(0,j);
			row[r][2]=hm(0,i)*hm(2,j)+hm(2,i)*hm(0,j);
			row[r][3]=hm(1,i)*hm(1,j);
			row[r][4]=hm(1,i)*hm(2,j)+hm(2,i)*hm(1,j);
			row[r][5]=hm(2,i)*hm(2,j);
			}
		for(int i=0;i<6;++i)
			row[1][i]-=row[2][i];
		for(int r=0;r<2;++r)
			{
			for(unsigned int i=0;i<6;++i)
				for(unsigned int j=0;j<6;++j)
					depthV(i,j)+=row[r][i]*row[r][j];
			}
		}
	
	/* Find the intrinsic parameter linear system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> depthQe=depthV.jacobiIteration();
	unsigned int minEIndex=0;
	double minE=Math::abs(depthQe.second(0));
	for(unsigned int i=1;i<6;++i)
		{
		if(minE>Math::abs(depthQe.second(i)))
			{
			minEIndex=i;
			minE=Math::abs(depthQe.second(i));
			}
		}
	std::cout<<"Smallest eigenvalue of v = "<<depthQe.second(minEIndex)<<std::endl;
	
	/* Calculate the intrinsic parameters: */
	Math::Matrix b=depthQe.first.getColumn(minEIndex);
	std::cout<<b(0)<<", "<<b(1)<<", "<<b(2)<<", "<<b(3)<<", "<<b(4)<<", "<<b(5)<<std::endl;
	double v0=(b(1)*b(2)-b(0)*b(4))/(b(0)*b(3)-Math::sqr(b(1)));
	double lambda=b(5)-(Math::sqr(b(2))+v0*(b(1)*b(2)-b(0)*b(4)))/b(0);
	double alpha=Math::sqrt(lambda/b(0));
	double beta=Math::sqrt(lambda*b(0)/(b(0)*b(3)-Math::sqr(b(1))));
	double gamma=-b(1)*Math::sqr(alpha)*beta/lambda;
	double u0=gamma*v0/beta-b(2)*Math::sqr(alpha)/lambda;
	
	std::cout<<"Intrinsic camera parameters:"<<std::endl;
	std::cout<<alpha<<" "<<gamma<<" "<<u0<<std::endl;
	std::cout<<0.0<<" "<<beta<<" "<<v0<<std::endl;
	std::cout<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;
	
	/* Create the intrinsic camera parameter matrix: */
	Math::Matrix a(3,3,1.0);
	a.set(0,0,alpha);
	a.set(0,1,gamma);
	a.set(0,2,u0);
	a.set(1,1,beta);
	a.set(1,2,v0);
	Math::Matrix aInv=a.inverse();
	
	/* Calculate the full projection matrix: */
	Math::Matrix proj(4,4,0.0);
	proj(0,0)=alpha;
	proj(0,1)=gamma;
	proj(0,2)=u0;
	proj(1,1)=beta;
	proj(1,2)=v0;
	proj(2,3)=1.0;
	proj(3,2)=1.0;
	
	/* Calculate extrinsic parameters for each tie point to get measurements for the depth formula regression: */
	Math::Matrix depthAta(2,2,0.0);
	Math::Matrix depthAtb(2,1,0.0);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Convert the tie point's depth homography to a matrix: */
		Homography::Matrix hm(1.0);
		hm(0,2)=double(application->depthFrameSize[0]);
		hm*=tpIt->depthHom.getMatrix();
		Homography::Matrix scale(1.0);
		scale(0,0)=1.0/tileSize[0];
		scale(1,1)=1.0/tileSize[1];
		hm*=scale;
		Math::Matrix h(3,3);
		for(unsigned int i=0;i<3;++i)
			for(unsigned int j=0;j<3;++j)
				h(i,j)=hm(i,j);
		
		/* Calculate the extrinsic parameters: */
		double lambda=0.5/(aInv*h.getColumn(0)).mag()+0.5/(aInv*h.getColumn(1)).mag();
		Math::Matrix r1=lambda*aInv*h.getColumn(0);
		Math::Matrix r2=lambda*aInv*h.getColumn(1);
		Math::Matrix r3(3,1);
		for(unsigned int i=0;i<3;++i)
			r3.set(i,r1((i+1)%3)*r2((i+2)%3)-r1((i+2)%3)*r2((i+1)%3)); // 'Tis a cross product, in case you're wondering
		Math::Matrix t=lambda*aInv*h.getColumn(2);
		
		/* Create the extrinsic parameter matrix: */
		Math::Matrix rt(3,4);
		rt.setColumn(0,r1);
		rt.setColumn(1,r2);
		rt.setColumn(2,r3);
		rt.setColumn(3,t);
		
		/* Transform the center of the grid to check for inversion: */
		Math::Matrix wgc(4,1);
		wgc(0)=tileSize[0]*double(gridSize[0])*0.5;
		wgc(1)=tileSize[1]*double(gridSize[1])*0.5;
		wgc(2)=0.0;
		wgc(3)=1.0;
		if((rt*wgc)(2)<0.0)
			{
			/* Flip the extrinsic matrix to move the grid to positive z: */
			Math::Matrix flip(3,3,-1.0);
			rt=flip*rt;
			}
		
		/* Transform all world grid points with the extrinsic matrix to get their camera-space z values: */
		for(int y=0;y<gridSize[1];++y)
			for(int x=0;x<gridSize[0];++x)
				{
				/* Create the world point: */
				Math::Matrix wp(4,1);
				wp(0)=tileSize[0]*double(x);
				wp(1)=tileSize[1]*double(y);
				wp(2)=0.0;
				wp(3)=1.0;
				
				/* Get the world point's z coordinate in camera space: */
				double dist=(rt*wp)(2);
				
				/* Get the depth frame value from the grid's plane in depth image space: */
				Point dip=tpIt->depthHom.transform(Point(x,y));
				const Plane::Vector& n=tpIt->gridPlane.getNormal();
				double o=tpIt->gridPlane.getOffset();
				double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
				
				/* Enter the depth / z pair into the depth formula accumulator: */
				depthAta(0,0)+=1.0;
				depthAta(0,1)+=-dist;
				depthAta(1,0)+=-dist;
				depthAta(1,1)+=dist*dist;
				depthAtb(0)+=-dist*depth;
				depthAtb(1)+=dist*dist*depth;
				}
		}
	
	/* Solve the depth formula least-squares system: */
	Math::Matrix depthX=depthAtb.divideFullPivot(depthAta);
	std::cout<<"Depth conversion formula: dist = "<<depthX(0)<<" / ("<<depthX(1)<<" - depth)"<<std::endl;
	
	/* Calculate the full depth unprojection matrix: */
	Math::Matrix depthProj(4,4,0.0);
	depthProj(0,0)=1.0/alpha;
	depthProj(0,1)=-gamma/(alpha*beta);
	depthProj(0,3)=-u0/alpha+v0*gamma/(alpha*beta);
	depthProj(1,1)=1.0/beta;
	depthProj(1,3)=-v0/beta;
	depthProj(2,3)=-1.0;
	depthProj(3,2)=-1.0/depthX(0);
	depthProj(3,3)=depthX(1)/depthX(0);
	
	/* Create the color calibration matrix's linear system: */
	Math::Matrix colorAta(12,12,0.0);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		for(int y=0;y<gridSize[1];++y)
			for(int x=0;x<gridSize[0];++x)
				{
				/* Enter the tie point into the color calibration matrix linear system: */
				Point dip=tpIt->depthHom.transform(Point(x,y));
				const Plane::Vector& n=tpIt->gridPlane.getNormal();
				double o=tpIt->gridPlane.getOffset();
				double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
				Math::Matrix dwp(4,1);
				dwp(0)=dip[0]+double(application->depthFrameSize[0]);
				dwp(1)=dip[1];
				dwp(2)=depth;
				dwp(3)=1.0;
				dwp=depthProj*dwp;
				for(int i=0;i<3;++i)
					dwp(i)/=dwp(3);
				Point cip=tpIt->colorHom.transform(Point(x,y));
				cip[0]/=double(application->colorFrameSize[0]);
				cip[1]/=double(application->colorFrameSize[1]);

				double eq[2][12];
				eq[0][0]=dwp(0);
				eq[0][1]=dwp(1);
				eq[0][2]=dwp(2);
				eq[0][3]=1.0;
				eq[0][4]=0.0;
				eq[0][5]=0.0;
				eq[0][6]=0.0;
				eq[0][7]=0.0;
				eq[0][8]=-cip[0]*dwp(0);
				eq[0][9]=-cip[0]*dwp(1);
				eq[0][10]=-cip[0]*dwp(2);
				eq[0][11]=-cip[0];

				eq[1][0]=0.0;
				eq[1][1]=0.0;
				eq[1][2]=0.0;
				eq[1][3]=0.0;
				eq[1][4]=dwp(0);
				eq[1][5]=dwp(1);
				eq[1][6]=dwp(2);
				eq[1][7]=1.0;
				eq[1][8]=-cip[1]*dwp(0);
				eq[1][9]=-cip[1]*dwp(1);
				eq[1][10]=-cip[1]*dwp(2);
				eq[1][11]=-cip[1];

				for(int row=0;row<2;++row)
					{
					for(unsigned int i=0;i<12;++i)
						for(unsigned int j=0;j<12;++j)
							colorAta(i,j)+=eq[row][i]*eq[row][j];
					}
				}
		}
	
	/* Find the color calibration system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> colorQe=colorAta.jacobiIteration();
	minEIndex=0;
	minE=Math::abs(colorQe.second(0,0));
	for(unsigned int i=1;i<12;++i)
		{
		if(minE>Math::abs(colorQe.second(i,0)))
			{
			minEIndex=i;
			minE=Math::abs(colorQe.second(i,0));
			}
		}
	
	/* Create the normalized color homography: */
	Math::Matrix colorHom(3,4);
	double scale=colorQe.first(11,minEIndex);
	for(int i=0;i<3;++i)
		for(int j=0;j<4;++j)
			colorHom(i,j)=colorQe.first(i*4+j,minEIndex)/scale;
	
	/* Create the full color unprojection matrix by extending the homography: */
	Math::Matrix colorProj(4,4);
	for(unsigned int i=0;i<2;++i)
		for(unsigned int j=0;j<4;++j)
			colorProj(i,j)=colorHom(i,j);
	for(unsigned int j=0;j<4;++j)
		colorProj(2,j)=j==2?1.0:0.0;
	for(unsigned int j=0;j<4;++j)
		colorProj(3,j)=colorHom(2,j);
	
	/* Modify the color unprojection matrix by the depth projection matrix: */
	colorProj*=depthProj;
	
	/* Write the calibration file: */
	std::string calibFileName="CameraCalibrationMatrices-";
	calibFileName.append(application->kinectCamera->getSerialNumber());
	calibFileName.append(".dat");
	std::cout<<"Writing calibration file "<<calibFileName<<std::endl;
	IO::AutoFile calibFile(IO::openFile(calibFileName.c_str(),IO::File::WriteOnly));
	calibFile->setEndianness(IO::File::LittleEndian);
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<double>(depthProj(i,j));
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			calibFile->write<double>(colorProj(i,j));
	}

void RawKinectViewer::GridTool::printWorldPoints(void)
	{
	typedef Geometry::ProjectiveTransformation<double,3> PTransform;
	
	if(tiePoints.empty())
		{
		Vrui::showErrorMessage("GridTool","No tie points to unproject");
		return;
		}
	
	/* Read the calibration file: */
	std::string calibFileName="CameraCalibrationMatrices-";
	calibFileName.append(application->kinectCamera->getSerialNumber());
	calibFileName.append(".dat");
	IO::AutoFile calibFile(IO::openFile(calibFileName.c_str()));
	calibFile->setEndianness(IO::File::LittleEndian);
	double mat[16];
	calibFile->read<double>(mat,16);
	PTransform depthProj=PTransform::fromRowMajor(mat);
	calibFile->read<double>(mat,16);
	PTransform colorProj=PTransform::fromRowMajor(mat);
	
	/* Unproject the grid points of the most recent tie point: */
	const TiePoint& tp=tiePoints.back();
	for(int y=1;y<gridSize[1];++y)
		for(int x=1;x<gridSize[0];++x)
			{
			Point dip=tp.depthHom.transform(Point(x,y));
			const Plane::Vector& n=tp.gridPlane.getNormal();
			double o=tp.gridPlane.getOffset();
			double depth=(o-dip[0]*n[0]-dip[1]*n[1])/n[2];
			PTransform::Point wp=depthProj.transform(PTransform::Point(dip[0]+double(application->depthFrameSize[0]),dip[1],depth));
			std::cout<<wp[0]<<", "<<wp[1]<<", "<<wp[2]<<std::endl;
			}
	}

void RawKinectViewer::GridTool::drawGrid(const RawKinectViewer::GridTool::Homography& hom)
	{
	glBegin(GL_LINES);
	for(int x=0;x<=gridSize[0];++x)
		{
		Point ip1=hom.transform(Point(double(x),0.0));
		glVertex3d(ip1[0],ip1[1],0.01);
		Point ip2=hom.transform(Point(double(x),double(gridSize[1])));
		glVertex3d(ip2[0],ip2[1],0.01);
		}
	for(int y=0;y<=gridSize[1];++y)
		{
		Point ip1=hom.transform(Point(0.0,double(y)));
		glVertex3d(ip1[0],ip1[1],0.01);
		Point ip2=hom.transform(Point(double(gridSize[0]),double(y)));
		glVertex3d(ip2[0],ip2[1],0.01);
		}
	glEnd();
	
	glBegin(GL_POINTS);
	Point ip=hom.transform(Point(0.5,0.5));
	glVertex3d(ip[0],ip[1],0.01);
	#if 0
	for(int y=0;y<gridSize[1];++y)
		for(int x=0;x<gridSize[0];++x)
			if((x+y)%2==0)
				{
				Point ip=hom.transform(Point(double(x)+0.5,double(y)+0.5));
				glVertex3d(ip[0],ip[1],0.01);
				}
	#endif
	glEnd();
	}

RawKinectViewer::GridTool::GridTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 dragging(false),
	 showTiePoints(false)
	{
	}

RawKinectViewer::GridTool::~GridTool(void)
	{
	}

void RawKinectViewer::GridTool::initialize(void)
	{
	#if 0
	/* Open the tie point file: */
	IO::AutoFile tiePointFile(IO::openFile("TiePoints.dat"));
	tiePointFile->setEndianness(IO::File::LittleEndian);
	
	/* Read the grid dimensions: */
	int gridSize[2];
	tiePointFile->read<int>(gridSize,2);
	double tileSize[2];
	tiePointFile->read<double>(tileSize,2);
	
	/* Read all tie points: */
	unsigned int numTiePoints=tiePointFile->read<unsigned int>();
	for(unsigned int i=0;i<numTiePoints;++i)
		{
		TiePoint tp;
		tp.depthHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tp.gridPlane=Misc::Marshaller<Plane>::read(*tiePointFile);
		tp.colorHom=Misc::Marshaller<Homography>::read(*tiePointFile);
		tiePoints.push_back(tp);
		}
	#endif
	
	initHoms();
	}

const Vrui::ToolFactory* RawKinectViewer::GridTool::getFactory(void) const
	{
	return factory;
	}

void RawKinectViewer::GridTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		if(buttonSlotIndex==0)
			{
			/* Get the interaction point: */
			Point p=getPoint();
			
			/* Find the grid point closest to the interaction point: */
			double minDist2=Math::Constants<double>::max;
			for(int y=0;y<=gridSize[1];++y)
				for(int x=0;x<=gridSize[0];++x)
					{
					Point gp=Point(x,y);
					for(int hom=0;hom<2;++hom)
						{
						Point ip=homs[hom].transform(gp);
						double ipDist2=Geometry::sqrDist(p,ip);
						if(minDist2>ipDist2)
							{
							minDist2=ipDist2;
							draggedHom=hom;
							draggedPoint=gp;
							dragOffset=ip-p;
							}
						}
					}
			
			/* Determine which previously dragged point to replace: */
			draggedPointIndex=0;
			double dpDist2=Geometry::sqrDist(lastDraggedPoints[0],draggedPoint);
			for(int i=1;i<4;++i)
				{
				double dist2=Geometry::sqrDist(lastDraggedPoints[i],draggedPoint);
				if(dpDist2>dist2)
					{
					draggedPointIndex=i;
					dpDist2=dist2;
					}
				}
			
			/* Start dragging: */
			lastDraggedPoints[draggedPointIndex]=draggedPoint;
			dragging=true;
			}
		else if(buttonSlotIndex==1)
			{
			createTiePoint();
			}
		else if(buttonSlotIndex==2)
			{
			showTiePoints=!showTiePoints;
			}
		else if(buttonSlotIndex==3)
			{
			/* Write all tie points to a file: */
			IO::AutoFile tiePointFile(IO::openFile("CalibrationTiePoints.dat",IO::File::WriteOnly));
			tiePointFile->setEndianness(IO::File::LittleEndian);
			tiePointFile->write<int>(gridSize,2);
			tiePointFile->write<double>(tileSize,2);
			tiePointFile->write<unsigned int>(application->depthFrameSize,2);
			tiePointFile->write<unsigned int>(application->colorFrameSize,2);
			tiePointFile->write<unsigned int>(tiePoints.size());
			for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
				{
				Misc::Marshaller<Homography>::write(tpIt->depthHom,*tiePointFile);
				Misc::Marshaller<Plane>::write(tpIt->gridPlane,*tiePointFile);
				Misc::Marshaller<Homography>::write(tpIt->colorHom,*tiePointFile);
				}
			
			/* Perform the calibration: */
			calibrate();
			}
		else if(buttonSlotIndex==4)
			{
			/* Print the world positions of the last tie point: */
			printWorldPoints();
			}
		}
	else
		{
		if(buttonSlotIndex==0)
			{
			/* Stop dragging: */
			dragging=false;
			}
		}
	}

void RawKinectViewer::GridTool::frame(void)
	{
	if(dragging)
		{
		/* Get the current interaction point: */
		Point p=getPoint();
		
		/* Recalculate the dragged homography: */
		Point imagePoints[4];
		for(int i=0;i<4;++i)
			imagePoints[i]=homs[draggedHom].transform(lastDraggedPoints[i]);
		imagePoints[draggedPointIndex]=p+dragOffset;
		homs[draggedHom]=calcHomography(lastDraggedPoints,imagePoints);
		}
	}

void RawKinectViewer::GridTool::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glLineWidth(1.0f);
	
	/* Go to navigation coordinates: */
	glPushMatrix();
	const Vrui::DisplayState& displayState=Vrui::getDisplayState(contextData);
	glLoadMatrix(displayState.modelviewNavigational);
	
	if(showTiePoints)
		{
		/* Draw all tie points: */
		glColor3f(0.0f,0.333f,0.0f);
		for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
			{
			drawGrid(tpIt->depthHom);
			drawGrid(tpIt->colorHom);
			}
		}
	
	/* Draw the current depth and color grids: */
	glColor3f(0.0f,1.0f,0.0f);
	for(int i=0;i<2;++i)
		drawGrid(homs[i]);
	
	glPopMatrix();
	
	glPopAttrib();
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
	if(pos[0]>=-depthFrameSize[0]&&pos[0]<0.0&&pos[1]>=0.0&&pos[1]<depthFrameSize[1])
		{
		/* Select the pixel under the locator: */
		selectedPixel[0]=int(pos[0]+double(depthFrameSize[0]));
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
	/* Capture five seconds worth of background frames: */
	kinectCamera->captureBackground(150);
	}

void RawKinectViewer::removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Set the background removal flag: */
	kinectCamera->setRemoveBackground(cbData->set);
	
	/* Set the toggle button's state to the actual new flag value: */
	cbData->toggle->setToggle(kinectCamera->getRemoveBackground());
	}

void RawKinectViewer::averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	if(cbData->set)
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
		
		/* Turn off average frame; will automatically be re-enabled once capture is complete: */
		showAverageFrame=false;
		}
	else
		{
		/* Don't show the average frame any longer: */
		showAverageFrame=false;
		}
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
	
	/* Create a toggle button to calculate and show an averaged depth frame: */
	GLMotif::ToggleButton* averageFramesButton=new GLMotif::ToggleButton("AverageFramesButton",mainMenu,"Average Frames");
	averageFramesButton->getValueChangedCallbacks().add(this,&RawKinectViewer::averageFramesCallback);
	
	/* Finish building the main menu: */
	mainMenu->manageChild();
	
	return mainMenuPopup;
	}

RawKinectViewer::RawKinectViewer(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 kinectCamera(0),
	 depthFrameVersion(0),colorFrameVersion(0),
	 paused(false),
	 averageNumFrames(150),averageFrameCounter(0),averageFrameDepth(0),averageFrameForeground(0),showAverageFrame(false),
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
	
	LineToolFactory* toolFactory3=new LineToolFactory("LineTool","Draw Lines",0,*Vrui::getToolManager());
	toolFactory3->setNumButtons(3);
	toolFactory3->setButtonFunction(0,"Drag / Create Line");
	toolFactory3->setButtonFunction(1,"Delete Line");
	toolFactory3->setButtonFunction(2,"Construct Grid");
	Vrui::getToolManager()->addClass(toolFactory3,Vrui::ToolManager::defaultToolFactoryDestructor);
	
	GridToolFactory* toolFactory4=new GridToolFactory("GridTool","Draw Grids",0,*Vrui::getToolManager());
	toolFactory4->setNumButtons(5);
	toolFactory4->setButtonFunction(0,"Drag Grid Corner");
	toolFactory4->setButtonFunction(1,"Store Grid");
	toolFactory4->setButtonFunction(2,"Toggle Stored Grids");
	toolFactory4->setButtonFunction(3,"Calibrate");
	toolFactory4->setButtonFunction(4,"Unproject Last Grid");
	Vrui::getToolManager()->addClass(toolFactory4,Vrui::ToolManager::defaultToolFactoryDestructor);
	GridTool::gridSize[0]=7;
	GridTool::gridSize[1]=5;
	GridTool::tileSize[0]=3.5*2.54;
	GridTool::tileSize[1]=3.5*2.54;
	
	/* Parse the command line: */
	int cameraIndex=0; // Use first Kinect camera device on USB bus
	KinectCamera::FrameSize selectedColorFrameSize=KinectCamera::FS_640_480;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"high")==0)
				selectedColorFrameSize=KinectCamera::FS_1280_1024;
			else if(strcasecmp(argv[i]+1,"gridSize")==0)
				{
				for(int j=0;j<2;++j)
					{
					++i;
					GridTool::gridSize[j]=atoi(argv[i]);
					}
				}
			else if(strcasecmp(argv[i]+1,"tileSize")==0)
				{
				for(int j=0;j<2;++j)
					{
					++i;
					GridTool::tileSize[j]=atof(argv[i]);
					}
				}
			}
		else
			cameraIndex=atoi(argv[i]);
		}
	
	/* Enable background USB event handling: */
	usbContext.startEventHandling();
	
	/* Connect to the given Kinect camera device on the host: */
	kinectCamera=new KinectCamera(usbContext,cameraIndex);
	
	#if 0
	{
	/* Get the camera's serial number to load the proper calibration matrices: */
	std::string serialNumber=kinectCamera->getSerialNumber();
	
	/* Open the calibration file: */
	Misc::File calibrationFile((std::string("CameraCalibrationMatrices-")+serialNumber+std::string(".dat")).c_str(),"rb",Misc::File::LittleEndian);
	
	/* Read the depth projection matrix: */
	BlobProperty<unsigned short>::PTransform projection;
	calibrationFile.read(projection.getMatrix().getEntries(),4*4);
	BlobProperty<unsigned short>::setProjection(projection);
	}
	#endif
	
	/* Set the color camera's frame size: */
	kinectCamera->setFrameSize(KinectCamera::COLOR,selectedColorFrameSize);
	
	/* Get the cameras' frame sizes: */
	depthFrameSize=kinectCamera->getActualFrameSize(KinectCamera::DEPTH);
	colorFrameSize=kinectCamera->getActualFrameSize(KinectCamera::COLOR);
	
	/* Allocate the average depth frame buffer: */
	averageFrameDepth=new float[depthFrameSize[0]*depthFrameSize[1]];
	averageFrameForeground=new float[depthFrameSize[0]*depthFrameSize[1]];
	
	/* Create the main menu: */
	mainMenu=createMainMenu();
	Vrui::setMainMenu(mainMenu);
	
	/* Start streaming: */
	kinectCamera->startStreaming(new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::colorStreamingCallback),new Misc::VoidMethodCall<const FrameBuffer&,RawKinectViewer>(this,&RawKinectViewer::depthStreamingCallback));
	
	/* Select an invalid pixel: */
	selectedPixel[0]=selectedPixel[1]=~0x0U;
	
	/* Initialize navigation transformation: */
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(1024),Vrui::Vector(0,1,0));
	}

RawKinectViewer::~RawKinectViewer(void)
	{
	delete mainMenu;
	delete[] averageFrameDepth;
	delete[] averageFrameForeground;
	
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
		
		if(selectedPixel[0]!=~0x0U&&selectedPixel[1]!=~0x0U)
			{
			/* Update the selected pixel's EKG: */
			++selectedPixelCurrentIndex;
			if(selectedPixelCurrentIndex==128)
				selectedPixelCurrentIndex=0;
			const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
			selectedPixelPulse[selectedPixelCurrentIndex]=dfPtr[selectedPixel[1]*depthFrames.getLockedValue().getSize(0)+selectedPixel[0]];
			}
		
		if(averageFrameCounter>0)
			{
			/* Accumulate the new depth frame into the averaging buffer: */
			const unsigned short* dfPtr=static_cast<const unsigned short*>(depthFrames.getLockedValue().getBuffer());
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
				showAverageFrame=true;
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
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	glColor3f(1.0f,1.0f,1.0f);
	
	/* Bind the depth texture object: */
	glBindTexture(GL_TEXTURE_2D,dataItem->depthTextureId);
	
	/* Check if the cached depth frame needs to be updated: */
	if(showAverageFrame)
		{
		/* Convert the averaged depth image to unsigned byte: */
		GLubyte* byteFrame=new GLubyte[depthFrameSize[1]*depthFrameSize[0]];
		const float* afdPtr=averageFrameDepth;
		const float* affPtr=averageFrameForeground;
		GLubyte* bfPtr=byteFrame;
		for(unsigned int y=0;y<depthFrameSize[1];++y)
			for(unsigned int x=0;x<depthFrameSize[0];++x,++afdPtr,++affPtr,++bfPtr)
				{
				#if 1
				if(*affPtr>=float(averageNumFrames)*0.5f)
					{
					float grey=255.0f-(34500.0f/(1100.0f-(*afdPtr/(*affPtr)))-50.0f)*255.0f/500.0f;
					if(grey<0.5f)
						*bfPtr=GLubyte(0);
					else if(grey>=254.5f)
						*bfPtr=GLubyte(255);
					else
						*bfPtr=GLubyte(grey+0.5f);
					}
				else
					*bfPtr=GLubyte(0);
				#else
				if(*fPtr<float(averageNumFrames))
					*bfPtr=GLubyte(*fPtr*256.0f/float(averageNumFrames));
				else
					*bfPtr=GLubyte(255);
				#endif
				}

		/* Set up the texture parameters: */
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);

		/* Upload the depth texture image: */
		glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE8,depthFrameSize[0],depthFrameSize[1],0,GL_LUMINANCE,GL_UNSIGNED_BYTE,byteFrame);
		
		delete[] byteFrame;
		}
	else
		{
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
					{
					// *bfPtr=GLubyte(255U-(unsigned int)(*fPtr)*256U/(0x0800U));
					// float grey=(1.0f/float(*fPtr)-0.001f)*1000.0f*255.0f;
					if(*fPtr!=0x7ffU)
						{
						float grey=255.0f-(34500.0f/(1100.0f-float(*fPtr))-50.0f)*255.0f/500.0f;
						if(grey<0.5f)
							*bfPtr=GLubyte(0);
						else if(grey>=254.5f)
							*bfPtr=GLubyte(255);
						else
							*bfPtr=GLubyte(grey+0.5f);
						}
					else
						*bfPtr=GLubyte(0);
					}
			
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
		}
	
	/* Draw the depth image: */
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f,0.0f);
	glVertex2f(-GLfloat(depthFrameSize[0]),0.0f);
	glTexCoord2f(1.0f,0.0f);
	glVertex2f(0.0f,0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(0.0f,GLfloat(depthFrameSize[1]));
	glTexCoord2f(0.0f,1.0f);
	glVertex2f(-GLfloat(depthFrameSize[0]),GLfloat(depthFrameSize[1]));
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
	glVertex2f(GLfloat(colorFrameSize[0]),0.0f);
	glTexCoord2f(1.0f,1.0f);
	glVertex2f(GLfloat(colorFrameSize[0]),GLfloat(colorFrameSize[1]));
	glTexCoord2f(0.0f,1.0f);
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
