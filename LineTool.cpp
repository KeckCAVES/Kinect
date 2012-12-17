/***********************************************************************
LineTool - Calibration tool for RawKinectViewer.
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

#include "LineTool.h"

#include <iostream>
#include <Geometry/Box.h>
#include <Geometry/PCACalculator.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>

#include "RawKinectViewer.h"

/*********************************
Static elements of class LineTool:
*********************************/

LineToolFactory* LineTool::factory=0;

/*************************
Methods of class LineTool:
*************************/

std::pair<LineTool::LineSet::iterator,int> LineTool::findLine(const LineTool::Point& p)
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
		Vector dir=lIt->end-lIt->start;
		Vector norm=Geometry::normal(dir);
		Vector off=p-lIt->start;
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

std::vector<LineTool::LineSet::iterator> LineTool::getIntersectingLines(LineTool::LineSet::iterator base,LineTool::LineSet& lines)
	{
	Vector n0=Geometry::normal(base->end-base->start);
	double o0=n0*Geometry::mid(base->start,base->end);
	
	/* Find all other lines intersecting the selected one: */
	std::vector<std::pair<LineSet::iterator,double> > orderedLines;
	for(LineSet::iterator lIt=lines.begin();lIt!=lines.end();++lIt)
		if(lIt!=base)
			{
			Vector n1=Geometry::normal(lIt->end-lIt->start);
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

void LineTool::makeCell(const LineTool::Line& l0,const LineTool::Line& l1,LineTool::Vector n[2],double o[2])
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

LineTool::Point LineTool::intersectLines(const LineTool::Line& l0,const LineTool::Line& l1)
	{
	Vector n0=Geometry::normal(l0.end-l0.start);
	double o0=Geometry::mid(l0.start,l0.end)*n0;
	double d1=l1.start*n0-o0;
	double d2=l1.end*n0-o0;
	double lambda=(0.0-d1)/(d2-d1);
	return Geometry::affineCombination(l1.start,l1.end,lambda);
	}

bool LineTool::inside(unsigned int x,unsigned int y,const LineTool::Vector ln[4],const double lo[4])
	{
	for(int i=0;i<4;++i)
		if((double(x)+0.5)*ln[i][0]+(double(y)+0.5)*ln[i][1]<lo[i])
			return false;
	return true;
	}

void LineTool::constructGrid(void)
	{
	/* Start with an arbitrary line: */
	LineSet::iterator l0=lines.begin();
	// Vector n0=l0->end-l0->start;
	
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
		Vector ln[4];
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

LineToolFactory* LineTool::initClass(Vrui::ToolManager& toolManager)
	{
	/* Create the tool factory: */
	factory=new LineToolFactory("LineTool","Draw Lines",0,toolManager);
	
	/* Set up the tool class' input layout: */
	factory->setNumButtons(3);
	factory->setButtonFunction(0,"Drag / Create Line");
	factory->setButtonFunction(1,"Delete Line");
	factory->setButtonFunction(2,"Construct Grid");
	
	/* Register and return the class: */
	toolManager.addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	return factory;
	}

LineTool::LineTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 dragging(false),haveGrid(false)
	{
	}

LineTool::~LineTool(void)
	{
	}

const Vrui::ToolFactory* LineTool::getFactory(void) const
	{
	return factory;
	}

void LineTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(buttonSlotIndex==0)
		{
		if(cbData->newButtonState)
			{
			/* Get the interaction point: */
			Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
			
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
			Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
			
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

void LineTool::frame(void)
	{
	if(dragging)
		{
		/* Get the interaction point: */
		Point p(application->calcImagePoint(getButtonDeviceRay(0)).getComponents());
		
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

void LineTool::display(GLContextData& contextData) const
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
