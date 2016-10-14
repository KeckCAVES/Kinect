/***********************************************************************
KinectViewer - Extrinsic calibration tool for KinectViewer using a large
spherical calibration target.
Copyright (c) 2014-2016 Oliver Kreylos

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

#ifndef SPHEREEXTRACTORTOOL_INCLUDED
#define SPHEREEXTRACTORTOOL_INCLUDED

#include <vector>
#include <iostream>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GLMotif/TextFieldSlider.h>
#include <Vrui/Application.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>

#include "SphereExtractor.h"
#include "KinectViewer.h"

/* Forward declarations: */
namespace GLMotif {
class PopupWindow;
}

class SphereExtractorTool; // Forward declaration
typedef Vrui::GenericToolFactory<SphereExtractorTool> SphereExtractorToolFactory; // SphereExtractorTool class uses the generic factory class

class SphereExtractorTool:public Vrui::Tool,public Vrui::Application::Tool<KinectViewer>,public GLObject
	{
	friend class Vrui::GenericToolFactory<SphereExtractorTool>;
	
	/* Embedded classes: */
	private:
	typedef SphereExtractor::Scalar Scalar;
	typedef SphereExtractor::Sphere Sphere;
	typedef Sphere::Point Point;
	
	struct SphereState // Structure to store spheres during extraction
		{
		/* Elements: */
		public:
		Scalar accPos[3]; // Accumulated sphere center position
		Scalar accWeight; // Current accumulation weight
		
		/* Constructors and destructors: */
		SphereState(void) // Creates a new sphere accumulator
			{
			/* Initialize the accumulator: */
			for(int i=0;i<3;++i)
				accPos[i]=Scalar(0);
			accWeight=Scalar(0);
			}
		
		/* Methods: */
		void addSphere(const Sphere& sphere,Scalar weight) // Adds a sphere to the accumulator
			{
			/* Add the weighted sphere center to the accumulator: */
			for(int i=0;i<3;++i)
				accPos[i]+=sphere.getCenter()[i]*weight;
			accWeight+=weight;
			}
		Point getCenter(void) const // Returns the current accumulated center position
			{
			return Point(accPos[0]/accWeight,accPos[1]/accWeight,accPos[2]/accWeight);
			}
		};
	
	struct StreamerState // Structure to store per-streamer sphere extraction state
		{
		/* Elements: */
		public:
		KinectViewer::KinectStreamer* streamer; // Streamer associated with this sphere extraction state
		SphereExtractor* extractor; // The sphere extractor object
		std::vector<SphereState> sphereStates; // States of all spheres currently tracked by the streamer's sphere extractor
		std::ostream* calibFile; // File to which to write calibration tie points
		std::vector<Point> tiePoints; // List of previously extracted tie points
		
		/* Constructors and destructors: */
		StreamerState(KinectViewer::KinectStreamer* sStreamer)
			:streamer(sStreamer),extractor(streamer->sphereExtractor),
			 calibFile(0)
			{
			}
		};
	
	struct DataItem:public GLObject::DataItem
		{
		/* Elements: */
		public:
		GLuint displayListId; // ID of display list to render extracted spheres
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	private:
	static SphereExtractorToolFactory* factory; // Pointer to the factory object for this class
	int maxBlobMergeDist;
	double sphereRadius;
	unsigned int minWhite;
	unsigned int maxSpread;
	size_t minBlobSize;
	double radiusTolerance;
	double maxResidual;
	GLMotif::PopupWindow* controlDialog; // Dialog to change sphere extraction parameters
	
	volatile bool tracking; // Flag whether the sphere extractor tool is currently tracking spheres
	std::vector<StreamerState> streamerStates; // List of streamer states
	
	/* Private methods: */
	void sphereListCallback(const SphereExtractor::SphereList& spheres,unsigned int streamerIndex);
	void maxBlobMergeDistCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void sphereRadiusCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void minWhiteCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void maxSpreadCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void minBlobSizeCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void radiusToleranceCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void maxResidualCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);

	/* Constructors and destructors: */
	public:
	static void initClass(void);
	SphereExtractorTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment);

	/* Methods from Vrui::Tool: */
	virtual void initialize(void);
	virtual void deinitialize(void);
	virtual const Vrui::ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;

	/* Methods from GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

#endif
