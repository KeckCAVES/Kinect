/***********************************************************************
GridTool - Calibration tool for RawKinectViewer.
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

#ifndef GRIDTOOL_INCLUDED
#define GRIDTOOL_INCLUDED

#include <vector>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Plane.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>

/* Forward declarations: */
class RawKinectViewer;

class GridTool;
typedef Vrui::GenericToolFactory<GridTool> GridToolFactory;

class GridTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<GridTool>;
	
	/* Embedded classes: */
	private:
	typedef Geometry::ProjectiveTransformation<double,2> Homography;
	typedef Homography::Point Point;
	typedef Homography::Vector Vector;
	typedef Geometry::Plane<double,3> Plane;
	typedef Geometry::Point<double,3> Point3;
	typedef Geometry::Vector<double,3> Vector3;
	
	struct TiePoint // Structure to store a calibration tie point
		{
		/* Elements: */
		public:
		Homography depthHom; // Depth homography
		Plane gridPlane; // Plane equation of grid in depth image space
		Homography colorHom; // Color homography
		};
	
	enum DraggingMode // Enumerated type for grid dragging modes
		{
		IDLE,VERTEX,MOVE,ROTATE
		};
	
	/* Elements: */
	private:
	static GridToolFactory* factory; // Pointer to the factory object for this class
	
	static int gridSize[2]; // Number of rows and columns in grid
	static double tileSize[2]; // Size of each grid tile in world space units
	std::vector<TiePoint> tiePoints; // List of already-created tie points
	Homography homs[2]; // Homographies in depth image and color image
	bool lockToPlane; // Flag whether to lock the depth grid to the depth plane
	Plane camDepthPlane; // Camera-space equation of the plane to which the grid is locked
	Plane worldDepthPlane; // World-space equation of the plane to which the grid is locked
	Point lastDraggedPoints[4]; // Last four dragged points in grid coordinates
	DraggingMode draggingMode; // Tool's current dragging mode
	int draggedHom; // Flag which homography is being dragged
	int draggedPointIndex; // Index of dragged point in list of previously dragged points
	Vector dragOffset; // Offset from input device to dragged point
	bool showTiePoints; // Flag whether to draw stored tie points
	
	/* Private methods: */
	Point getPoint(void) const;
	static Homography calcHomography(const Point gridPoints[4],const Point imagePoints[4]);
	void initHoms(void);
	void startDrag(void);
	void dragInPlane(const Point& moveHandle,const Point& rotateHandle);
	void createTiePoint(void);
	void calibrate(void);
	void printWorldPoints(void);
	static void drawGrid(const Homography& hom,bool active);
	
	/* Constructors and destructors: */
	public:
	static GridToolFactory* initClass(Vrui::ToolManager& toolManager);
	static void setGridSize(int newGridSize0,int newGridSize1);
	static void setTileSize(double newTileSize0,double newTileSize1);
	GridTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~GridTool(void);
	
	/* Methods from class Vrui::Tool: */
	virtual void initialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

#endif
