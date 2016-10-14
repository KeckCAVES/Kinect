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

#include "SphereExtractorTool.h"

#include <iomanip>
#include <fstream>
#include <Misc/PrintInteger.h>
#include <Misc/FunctionCalls.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLContextData.h>
#include <GL/GLModels.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Label.h>
#include <Vrui/DisplayState.h>
#include <Kinect/Config.h>
#include <Kinect/ProjectorHeader.h>

/**********************************************
Methods of class SphereExtractorTool::DataItem:
**********************************************/

SphereExtractorTool::DataItem::DataItem(void)
	:displayListId(glGenLists(1))
	{
	}

SphereExtractorTool::DataItem::~DataItem(void)
	{
	glDeleteLists(displayListId,1);
	}

/********************************************
Static elements of class SphereExtractorTool:
********************************************/

SphereExtractorToolFactory* SphereExtractorTool::factory=0;

/************************************
Methods of class SphereExtractorTool:
************************************/

void SphereExtractorTool::sphereListCallback(const SphereExtractor::SphereList& spheres,unsigned int streamerIndex)
	{
	if(tracking)
		{
		/* Add all extracted spheres to their respective accumulators: */
		StreamerState& ss=streamerStates[streamerIndex];
		for(SphereExtractor::SphereList::const_iterator sIt=spheres.begin();sIt!=spheres.end();++sIt)
			{
			/* Check if there is already a sphere at this extracted sphere's approximate position: */
			std::vector<SphereState>::iterator ssIt;
			for(ssIt=ss.sphereStates.begin();ssIt!=ss.sphereStates.end()&&Geometry::sqrDist(ssIt->getCenter(),sIt->getCenter())>Math::sqr(Scalar(sphereRadius*0.5));++ssIt)
				;
			if(ssIt==ss.sphereStates.end())
				{
				/* Create a new sphere state: */
				ss.sphereStates.push_back(SphereState());
				ssIt=ss.sphereStates.end()-1;
				}
			
			/* Accumulate the sphere: */
			Scalar weight=Math::exp(-Math::sqr((sIt->getRadius()-sphereRadius)/radiusTolerance));
			ssIt->addSphere(*sIt,weight);
			}
		}
	}

void SphereExtractorTool::maxBlobMergeDistCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the maximum blob merging distance: */
	maxBlobMergeDist=int(Math::floor(cbData->value+0.5));
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMaxBlobMergeDist(maxBlobMergeDist);
	}

void SphereExtractorTool::sphereRadiusCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the sphere radius: */
	sphereRadius=cbData->value;
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setSphereRadius(sphereRadius);
	}

void SphereExtractorTool::minWhiteCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the minimum white color component value: */
	minWhite=(unsigned int)(Math::floor(cbData->value+0.5));
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
	}

void SphereExtractorTool::maxSpreadCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the maximum color component spread value: */
	maxSpread=(unsigned int)(Math::floor(cbData->value+0.5));
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
	}

void SphereExtractorTool::minBlobSizeCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the minimum blob size: */
	minBlobSize=size_t(Math::floor(cbData->value+0.5));
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
	}

void SphereExtractorTool::radiusToleranceCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the radius tolerance: */
	radiusTolerance=cbData->value;
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
	}

void SphereExtractorTool::maxResidualCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Update the maximum residual: */
	maxResidual=cbData->value;
	
	/* Update all sphere extractors: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
	}

void SphereExtractorTool::initClass(void)
	{
	/* Create a factory object for the sphere extractor tool class: */
	SphereExtractorToolFactory* factory=new SphereExtractorToolFactory("SphereExtractorTool","Extract Spheres",0,*Vrui::getToolManager());
	
	/* Set the sphere extractor tool class' input layout: */
	factory->setNumButtons(1);
	factory->setButtonFunction(0,"Extract Spheres");
	
	/* Register the custom tool class with the Vrui tool manager: */
	Vrui::getToolManager()->addClass(factory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

SphereExtractorTool::SphereExtractorTool(const Vrui::ToolFactory* factory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(factory,inputAssignment),
	 maxBlobMergeDist(1),sphereRadius(4.0*2.54),minWhite(128),maxSpread(32),minBlobSize(1000),radiusTolerance(0.2),maxResidual(0.3),
	 controlDialog(0),
	 tracking(false)
	{
	}

void SphereExtractorTool::initialize(void)
	{
	/* Create sphere extractors for all Kinect streamers: */
	unsigned int streamerIndex=0;
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt,++streamerIndex)
		{
		SphereExtractor* sphereExtractor=new SphereExtractor(*(*ksIt)->source,(*ksIt)->projector->getDepthCorrection());
		sphereExtractor->setSphereRadius(sphereRadius);
		sphereExtractor->setMatchLimits(minWhite,maxSpread,minBlobSize,radiusTolerance,maxResidual);
		sphereExtractor->startStreaming(Misc::createFunctionCall(this,&SphereExtractorTool::sphereListCallback,streamerIndex));
		{
		Threads::Mutex::Lock sphereExtractorLock((*ksIt)->sphereExtractorMutex);
		(*ksIt)->sphereExtractor=sphereExtractor;
		}
		
		/* Create a new streamer state for this Kinect streamer: */
		streamerStates.push_back(*ksIt);
		
		/* Open the calibration file: */
		std::string calibFileName="KinectPoints";
		char number[10];
		calibFileName.append(Misc::print(streamerIndex,number+9));
		calibFileName.append(".csv");
		streamerStates.back().calibFile=new std::ofstream(calibFileName.c_str());
		}
	
	/* Create an extraction control dialog: */
	const GLMotif::StyleSheet& ss=*Vrui::getWidgetManager()->getStyleSheet();
	controlDialog=new GLMotif::PopupWindow("SphereExtractorToolControlDialog",Vrui::getWidgetManager(),"Sphere Extraction Control");
	controlDialog->setResizableFlags(true,false);
	
	GLMotif::RowColumn* extractorSettings=new GLMotif::RowColumn("ExtractorSettings",controlDialog,false);
	extractorSettings->setOrientation(GLMotif::RowColumn::VERTICAL);
	extractorSettings->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	extractorSettings->setNumMinorWidgets(2);
	
	new GLMotif::Label("SphereRadiusLabel",extractorSettings,"Sphere Radius");
	
	GLMotif::TextFieldSlider* sphereRadiusSlider=new GLMotif::TextFieldSlider("SphereRadiusSlider",extractorSettings,8,ss.fontHeight*15.0f);
	sphereRadiusSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	sphereRadiusSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	sphereRadiusSlider->getTextField()->setFieldWidth(8);
	sphereRadiusSlider->getTextField()->setPrecision(4);
	sphereRadiusSlider->getTextField()->setFloatFormat(GLMotif::TextField::SMART);
	sphereRadiusSlider->setValueRange(0.001,1000.0,0.001);
	sphereRadiusSlider->setValue(sphereRadius);
	sphereRadiusSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::sphereRadiusCallback);
	
	new GLMotif::Label("MinWhiteLabel",extractorSettings,"Minimum White Value");
	
	GLMotif::TextFieldSlider* minWhiteSlider=new GLMotif::TextFieldSlider("MinWhiteSlider",extractorSettings,8,ss.fontHeight*15.0f);
	minWhiteSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	minWhiteSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	minWhiteSlider->setValueRange(0,255,1);
	minWhiteSlider->setValue(minWhite);
	minWhiteSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::minWhiteCallback);
	
	new GLMotif::Label("MaxSpreadLabel",extractorSettings,"Maximum White Spread");
	
	GLMotif::TextFieldSlider* maxSpreadSlider=new GLMotif::TextFieldSlider("MaxSpreadSlider",extractorSettings,8,ss.fontHeight*15.0f);
	maxSpreadSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	maxSpreadSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	maxSpreadSlider->setValueRange(0,255,1);
	maxSpreadSlider->setValue(maxSpread);
	maxSpreadSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::maxSpreadCallback);
	
	new GLMotif::Label("MaxBlobMergeDistLabel",extractorSettings,"Blob Merge Distance");
	
	GLMotif::TextFieldSlider* maxBlobMergeDistSlider=new GLMotif::TextFieldSlider("MaxBlobMergeDistSlider",extractorSettings,8,ss.fontHeight*15.0f);
	maxBlobMergeDistSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	maxBlobMergeDistSlider->setValueType(GLMotif::TextFieldSlider::INT);
	maxBlobMergeDistSlider->setValueRange(1,100,1.0);
	maxBlobMergeDistSlider->setValue(maxBlobMergeDist);
	maxBlobMergeDistSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::maxBlobMergeDistCallback);
	
	new GLMotif::Label("MinBlobSizeLabel",extractorSettings,"Minimum Blob Size");
	
	GLMotif::TextFieldSlider* minBlobSizeSlider=new GLMotif::TextFieldSlider("MinBlobSizeSlider",extractorSettings,8,ss.fontHeight*15.0f);
	minBlobSizeSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	minBlobSizeSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	minBlobSizeSlider->setValueRange(100,100000,0.05);
	minBlobSizeSlider->setValue(minBlobSize);
	minBlobSizeSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::minBlobSizeCallback);
	
	new GLMotif::Label("RadiusToleranceLabel",extractorSettings,"Radius Tolerance");
	
	GLMotif::TextFieldSlider* radiusToleranceSlider=new GLMotif::TextFieldSlider("RadiusToleranceSlider",extractorSettings,8,ss.fontHeight*15.0f);
	radiusToleranceSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	radiusToleranceSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	radiusToleranceSlider->getTextField()->setFieldWidth(8);
	radiusToleranceSlider->getTextField()->setPrecision(4);
	radiusToleranceSlider->setValueRange(0.0,1.0,0.01);
	radiusToleranceSlider->setValue(radiusTolerance);
	radiusToleranceSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::radiusToleranceCallback);
	
	new GLMotif::Label("MaxResidualLabel",extractorSettings,"Maximum RMS");
	
	GLMotif::TextFieldSlider* maxResidualSlider=new GLMotif::TextFieldSlider("MaxResidualSlider",extractorSettings,8,ss.fontHeight*15.0f);
	maxResidualSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	maxResidualSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	maxResidualSlider->getTextField()->setFieldWidth(8);
	maxResidualSlider->getTextField()->setPrecision(4);
	maxResidualSlider->setValueRange(0.001,1000.0,0.05);
	maxResidualSlider->setValue(maxResidual);
	maxResidualSlider->getValueChangedCallbacks().add(this,&SphereExtractorTool::maxResidualCallback);
	
	extractorSettings->manageChild();
	
	/* Pop up the control dialog: */
	Vrui::popupPrimaryWidget(controlDialog);
	}

void SphereExtractorTool::deinitialize(void)
	{
	/* Destroy the control dialog: */
	delete controlDialog;
	
	/* Destroy sphere extractors of all Kinect streamers: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		{
		SphereExtractor* sphereExtractor;
		{
		Threads::Mutex::Lock sphereExtractorLock((*ksIt)->sphereExtractorMutex);
		sphereExtractor=(*ksIt)->sphereExtractor;
		(*ksIt)->sphereExtractor=0;
		}
		
		if(sphereExtractor!=0)
			sphereExtractor->stopStreaming();
		delete sphereExtractor;
		}
	
	/* Close all calibration files: */
	for(std::vector<StreamerState>::iterator ssIt=streamerStates.begin();ssIt!=streamerStates.end();++ssIt)
		delete ssIt->calibFile;
	}

const Vrui::ToolFactory* SphereExtractorTool::getFactory(void) const
	{
	return factory;
	}

void SphereExtractorTool::buttonCallback(int,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(cbData->newButtonState)
		{
		/* Start tracking spheres: */
		tracking=true;
		}
	else
		{
		/* Stop tracking spheres: */
		tracking=false;
		
		/* Find the highest-weighted sphere accumulator for each streamer state: */
		for(std::vector<StreamerState>::iterator ssIt=streamerStates.begin();ssIt!=streamerStates.end();++ssIt)
			{
			Scalar maxWeight=Scalar(0);
			Point maxCenter=Point::origin;
			for(std::vector<SphereState>::iterator spsIt=ssIt->sphereStates.begin();spsIt!=ssIt->sphereStates.end();++spsIt)
				{
				/* Check if this sphere is higher weighted: */
				if(maxWeight<spsIt->accWeight)
					{
					/* Store the sphere's center point: */
					maxWeight=spsIt->accWeight;
					maxCenter=spsIt->getCenter();
					}
				}
			
			/* Print the best sphere center: */
			if(maxWeight>Scalar(0))
				{
				std::cout<<"Streamer "<<(ssIt-streamerStates.begin())<<": center "<<maxCenter[0]<<", "<<maxCenter[1]<<", "<<maxCenter[2]<<", weight "<<maxWeight<<std::endl;
				(*ssIt->calibFile)<<maxCenter[0]<<','<<maxCenter[1]<<','<<maxCenter[2]<<std::endl;
				
				/* Store the calibration tie point: */
				ssIt->tiePoints.push_back(maxCenter);
				}
			else
				{
				std::cout<<"Streamer "<<(ssIt-streamerStates.begin())<<": no sphere found"<<std::endl;
				(*ssIt->calibFile)<<"NaN, NaN, NaN"<<std::endl;
				}
			}
		
		/* Reset all sphere accumulators for the next tracking period: */
		for(std::vector<StreamerState>::iterator ssIt=streamerStates.begin();ssIt!=streamerStates.end();++ssIt)
			ssIt->sphereStates.clear();
		}
	}

void SphereExtractorTool::frame(void)
	{
	/* Lock the most recently extracted sphere lists of all Kinect streamers: */
	for(std::vector<KinectViewer::KinectStreamer*>::iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		(*ksIt)->sphereExtractor->lockSpheres();
	}

void SphereExtractorTool::display(GLContextData& contextData) const
	{
	/* Retrieve the context data item: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	
	/* Draw the most recently extracted sphere lists of all Kinect streamers: */
	for(std::vector<KinectViewer::KinectStreamer*>::const_iterator ksIt=application->streamers.begin();ksIt!=application->streamers.end();++ksIt)
		if((*ksIt)->enabled&&!(*ksIt)->sphereExtractor->getSpheres().empty())
			{
			/* Draw the spheres in the projector's coordinate system: */
			glPushMatrix();
			glLoadMatrix(Vrui::getDisplayState(contextData).modelviewNavigational);
			glMultMatrix((*ksIt)->projector->getProjectorTransform());
			
			for(SphereExtractor::SphereList::const_iterator sIt=(*ksIt)->sphereExtractor->getSpheres().begin();sIt!=(*ksIt)->sphereExtractor->getSpheres().end();++sIt)
				{
				glPushMatrix();
				glTranslate(sIt->getCenter()-SphereExtractor::Sphere::Point::origin);
				glScale(sIt->getRadius());
				glCallList(dataItem->displayListId);
				glPopMatrix();
				}
			
			glPopMatrix();
			}
	
	/* Draw all previously-extracted tie points of all Kinect streamers: */
	glPushAttrib(GL_ENABLE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glColor3f(1.0f,0.0f,0.0f);
	
	for(std::vector<StreamerState>::const_iterator ssIt=streamerStates.begin();ssIt!=streamerStates.end();++ssIt)
		if(ssIt->streamer->enabled&&!ssIt->tiePoints.empty())
			{
			/* Draw the tie points in the projector's coordinate system: */
			glPushMatrix();
			glLoadMatrix(Vrui::getDisplayState(contextData).modelviewNavigational);
			glMultMatrix(ssIt->streamer->projector->getProjectorTransform());
			
			glBegin(GL_POINTS);
			for(std::vector<Point>::const_iterator tpIt=ssIt->tiePoints.begin();tpIt!=ssIt->tiePoints.end();++tpIt)
				glVertex(*tpIt);
			glEnd();
			
			glPopMatrix();
			}
	
	glPopAttrib();
	}

void SphereExtractorTool::initContext(GLContextData& contextData) const
	{
	/* Create a new data item and register it with the context data object: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Create the sphere rendering display list: */
	glNewList(dataItem->displayListId,GL_COMPILE);
	
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.0f,0.0f));
	glMaterialSpecular(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,0.5f));
	glMaterialShininess(GLMaterialEnums::FRONT,25.0f);
	
	glDrawSphereIcosahedron(1.0f,6);
	
	glEndList();
	}
