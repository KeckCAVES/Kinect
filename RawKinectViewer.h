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

#ifndef RAWKINECTVIEWER_INCLUDED
#define RAWKINECTVIEWER_INCLUDED

#include <utility>
#include <vector>
#include <Threads/TripleBuffer.h>
#include <USB/Context.h>
#include <Geometry/Point.h>
#include <Geometry/Plane.h>
#include <Geometry/ProjectiveTransformation.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <GL/GLObject.h>
#include <GLMotif/ToggleButton.h>
#include <Vrui/Tool.h>
#include <Vrui/LocatorTool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>
#include <Kinect/FrameBuffer.h>

#include "FindBlobs.h"

/* Forward declarations: */
namespace GLMotif {
class PopupMenu;
}
namespace Kinect {
class Camera;
}

/**************
Helper classes:
**************/

template <>
class BlobProperty<unsigned short> // Blob property accumulator for depth frames; calculates 3D centroid of red depth image pixels
	{
	/* Embedded classes: */
	public:
	typedef unsigned short Pixel;
	typedef Geometry::ProjectiveTransformation<double,3> PTransform;
	typedef PTransform::Point Point;
	
	/* Elements: */
	private:
	static PTransform projection; // Pixel re-projection matrix
	double pxs,pys,pzs; // Accumulated centroid of reprojected pixels
	double rawDepth; // Sum of pixels' raw depth values
	size_t numPixels; // Number of accumulated pixels
	
	/* Constructors and destructors: */
	public:
	static void setProjection(const PTransform& sProjection) // Sets the re-projection matrix
		{
		projection=sProjection;
		}
	BlobProperty(void) // Creates empty accumulator
		:pxs(0.0),pys(0.0),pzs(0.0),rawDepth(0.0),numPixels(0)
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
		
		/* Accumulate the pixel's raw depth value: */
		rawDepth+=double(pixelValue);
		
		++numPixels;
		}
	void merge(const BlobProperty& other)
		{
		/* Merge the other blob property's plane equation accumulator: */
		pxs+=other.pxs;
		pys+=other.pys;
		pzs+=other.pzs;
		rawDepth+=other.rawDepth;
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
	double getRawDepth(void) const
		{
		return rawDepth/double(numPixels);
		}
	};

class RawKinectViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	class PauseTool;
	typedef Vrui::GenericToolFactory<PauseTool> PauseToolFactory; // Tool class uses the generic factory class
	class TiePointTool;
	typedef Vrui::GenericToolFactory<TiePointTool> TiePointToolFactory; // Tool class uses the generic factory class
	class LineTool;
	typedef Vrui::GenericToolFactory<LineTool> LineToolFactory; // Tool class uses the generic factory class
	class GridTool;
	typedef Vrui::GenericToolFactory<GridTool> GridToolFactory; // Tool class uses the generic factory class
	
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
	
	class LineTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
		{
		friend class Vrui::GenericToolFactory<LineTool>;
		
		/* Embedded classes: */
		private:
		typedef Geometry::Point<double,2> Point;
		
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
		Point::Vector startOffset;
		Point::Vector endOffset;
		bool haveGrid;
		std::vector<Point> gridSquares;
		
		/* Private methods: */
		Point getPoint(void) const;
		std::pair<LineSet::iterator,int> findLine(const Point& p);
		static std::vector<LineSet::iterator> getIntersectingLines(LineSet::iterator base,LineSet& lines);
		static void makeCell(const Line& l0,const Line& l1,Point::Vector n[2],double o[2]);
		static Point intersectLines(const Line& l0,const Line& l1);
		static bool inside(unsigned int x,unsigned int y,const Point::Vector ln[4],const double lo[4]);
		void constructGrid(void);
		
		/* Constructors and destructors: */
		public:
		LineTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
		virtual ~LineTool(void);
		
		/* Methods from class Vrui::Tool: */
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		virtual void frame(void);
		virtual void display(GLContextData& contextData) const;
		};
	
	class GridTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
		{
		friend class Vrui::GenericToolFactory<GridTool>;
		friend class RawKinectViewer;
		
		/* Embedded classes: */
		private:
		typedef Geometry::ProjectiveTransformation<double,2> Homography;
		typedef Homography::Point Point;
		typedef Point::Vector Vector;
		typedef Geometry::Plane<double,3> Plane;
		
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
		void createTiePoint(void);
		void calibrate(void);
		void printWorldPoints(void);
		static void drawGrid(const Homography& hom,bool active);
		
		/* Constructors and destructors: */
		public:
		GridTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
		virtual ~GridTool(void);
		
		/* Methods from class Vrui::Tool: */
		virtual void initialize(void);
		virtual const Vrui::ToolFactory* getFactory(void) const;
		virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
		virtual void frame(void);
		virtual void display(GLContextData& contextData) const;
		};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint colorTextureId; // ID of texture object holding color image
		unsigned int colorFrameVersion; // Version number of color currently in texture object
		GLuint depthTextureId; // ID of texture object holding depth image
		unsigned int depthFrameVersion; // Version number of frame currently texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	friend class PauseTool;
	friend class TiePointTool;
	friend class LineTool;
	friend class GridTool;
	
	/* Elements: */
	USB::Context usbContext; // USB device context
	Kinect::Camera* camera; // Pointer to camera aspect of Kinect device
	const unsigned int* colorFrameSize; // Size of color frames in pixels
	Threads::TripleBuffer<Kinect::FrameBuffer> colorFrames; // Triple buffer of color frames received from the camera
	unsigned int colorFrameVersion; // Version number of current color frame
	const unsigned int* depthFrameSize; // Size of depth frames in pixels
	Threads::TripleBuffer<Kinect::FrameBuffer> depthFrames; // Triple buffer of depth frames received from the camera
	unsigned int depthFrameVersion; // Version number of current depth frame
	bool paused; // Flag whether the video stream display is paused
	unsigned int averageNumFrames; // Number of depth frames to average
	unsigned int averageFrameCounter; // Number of depth frames still to accumulate
	float* averageFrameDepth; // Average depth values in depth frame
	float* averageFrameForeground; // Ratio of foreground vs background in depth frame
	bool showAverageFrame; // Flag whether to show the averaged frame
	unsigned int selectedPixel[2]; // Coordinates of the selected depth image pixel
	unsigned short selectedPixelPulse[128]; // EKG of depth value of selected pixel
	int selectedPixelCurrentIndex; // Index of most recent value in selected pixel's EKG
	GLMotif::PopupMenu* mainMenu; // The program's main menu
	
	/* Private methods: */
	void colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving color frames from the Kinect camera
	void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the Kinect camera
	void locatorButtonPressCallback(Vrui::LocatorTool::ButtonPressCallbackData* cbData); // Callback when a locator tool's button is pressed
	void captureBackgroundCallback(Misc::CallbackData* cbData);
	void removeBackgroundCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	void averageFramesCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
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

#endif
