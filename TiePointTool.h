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

#ifndef TIEPOINTTOOL_INCLUDED
#define TIEPOINTTOOL_INCLUDED

#include <GL/gl.h>
#include <GL/GLColor.h>
#include <Geometry/Point.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Vrui/Application.h>
#include <Kinect/FrameSource.h>

#include "FindBlobs.h"

/* Forward declarations: */
class RawKinectViewer;

/*************
Helper classes:
**************/

template <>
class BlobProperty<Kinect::FrameSource::DepthPixel> // Blob property accumulator for depth frames; calculates 3D centroid of read depth image pixels
	{
	/* Embedded classes: */
	public:
	typedef Kinect::FrameSource::DepthPixel Pixel;
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

class TiePointTool;
typedef Vrui::GenericToolFactory<TiePointTool> TiePointToolFactory;

class TiePointTool:public Vrui::Tool,public Vrui::Application::Tool<RawKinectViewer>
	{
	friend class Vrui::GenericToolFactory<TiePointTool>;
	
	/* Embedded classes: */
	private:
	typedef Kinect::FrameSource::DepthPixel DepthPixel;
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
	static TiePointToolFactory* initClass(Vrui::ToolManager& toolManager);
	TiePointTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~TiePointTool(void);

	/* Methods from class Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void display(GLContextData& contextData) const;
	};

#endif
