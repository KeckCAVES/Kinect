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

#ifndef LINETOOL_INCLUDED
#define LINETOOL_INCLUDED

#include <utility>
#include <vector>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class RawKinectViewer;

class LineTool;
typedef Vrui::GenericToolFactory<LineTool> LineToolFactory;

class LineTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<LineTool>;

	/* Embedded classes: */
	private:
	typedef Geometry::Point<double,2> Point;
	typedef Geometry::Vector<double,2> Vector;
	
	struct Line
		{
		/* Elements: */
		public:
		Point start;
		Point end;
		};
	
	typedef std::vector<Line> LineSet;
	
	/* Elements: */
	private:
	static LineToolFactory* factory; // Pointer to the factory object for this class
	
	LineSet lines;
	bool dragging;
	int dragMode;
	Line current;
	LineSet::iterator draggedLine;
	Vector startOffset;
	Vector endOffset;
	bool haveGrid;
	std::vector<Point> gridSquares;
	
	/* Private methods: */
	Point getPoint(void) const;
	std::pair<LineSet::iterator,int> findLine(const Point& p);
	static std::vector<LineSet::iterator> getIntersectingLines(LineSet::iterator base,LineSet& lines);
	static void makeCell(const Line& l0,const Line& l1,Vector n[2],double o[2]);
	static Point intersectLines(const Line& l0,const Line& l1);
	static bool inside(unsigned int x,unsigned int y,const Vector ln[4],const double lo[4]);
	void constructGrid(void);
	
	/* Constructors and destructors: */
	public:
	static LineToolFactory* initClass(Vrui::ToolManager& toolManager);
	LineTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~LineTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
