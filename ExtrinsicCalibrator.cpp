/***********************************************************************
ExtrinsicCalibrator - Utility to calibrate a 3D camera with a 6-DOF
tracking system.
Copyright (c) 2016-2017 Oliver Kreylos

This file is part of the Virtual Reality User Interface Library (Vrui).

The Virtual Reality User Interface Library is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Virtual Reality User Interface Library is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Virtual Reality User Interface Library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#include <string.h>
#include <stdlib.h>
#include <utility>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <Misc/FunctionCalls.h>
#include <Misc/MessageLogger.h>
#include <Threads/TripleBuffer.h>
#include <IO/OStream.h>
#include <Cluster/OpenPipe.h>
#include <Math/Matrix.h>
#include <Math/VarianceAccumulator.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Rotation.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/AffineTransformation.h>
#include <Geometry/GeometryValueCoders.h>
#include <GL/gl.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Pager.h>
#include <GLMotif/RowColumn.h>
#include <GLMotif/Button.h>
#include <GLMotif/ToggleButton.h>
#include <GLMotif/TextField.h>
#include <GLMotif/TextFieldSlider.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Vrui/Viewer.h>
#include <Vrui/OpenFile.h>
#include <Vrui/Internal/VRDeviceState.h>
#include <Vrui/Internal/VRDeviceDescriptor.h>
#include <Vrui/Internal/VRDeviceClient.h>
#include <Kinect/Config.h>
#include <Kinect/DirectFrameSource.h>
#include <Kinect/OpenDirectFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/ProjectorType.h>
#include <Kinect/ProjectorHeader.h>
#include <Kinect/DiskExtractor.h>
#include <Kinect/Internal/Config.h>

class ExtrinsicCalibrator:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef Vrui::VRDeviceState::TrackerState TS;
	typedef TS::PositionOrientation PO;
	typedef PO::Scalar Scalar;
	typedef PO::Point Point;
	typedef PO::Vector Vector;
	typedef PO::Rotation Rotation;
	typedef Geometry::AffineTransformation<Scalar,3> ATransform;
	typedef std::pair<Point,Point> TiePoint;
	
	/* Elements: */
	Vrui::VRDeviceClient* deviceClient; // Connection to the VRDeviceDaemon
	std::vector<const Vrui::VRDeviceDescriptor*> controllers; // List of input devices that have buttons
	int buttonIndex; // Index of button to trigger tie point collection; if -1, any button will do
	Vrui::Point diskCenter; // Position of disk center in controller's local coordinate system
	
	Kinect::FrameSource* camera; // 3D video source to calibrate
	std::string cameraSerialNumber; // Camera's serial number if camera is a direct frame source; some dummy string otherwise
	Kinect::DiskExtractor* diskExtractor; // Object to extract disk shapes from a 3D video stream
	Kinect::ProjectorType* projector; // A projeftor to render the 3D video stream
	GLMotif::PopupWindow* configurationDialog; // Dialog window to configure the extrinsic calibrator
	GLMotif::TextField* diskCenterTextFields[3]; // Text fields displaying the calibrated disk center
	GLMotif::TextField* alignmentResidualTextField; // Text field showing the RMS residual of the most recent point set alignment
	
	Threads::TripleBuffer<Vrui::TrackerState*> controllerStates; // Triple buffer of arrays of current controller tracking states
	int previousControllerIndex; // Index of previously active controller
	Threads::TripleBuffer<int> controllerIndex; // Triple buffer containing index of the controller owning the currently pressed button, or -1
	bool calibratingDiskCenter; // Flag whether currently collecting controller transformations for disk center calibration
	std::vector<ATransform> calibTransforms; // Vector of controller transformations collected during calibration for residual calculation
	Math::Matrix calibAta,calibAtb; // Accumulated disk center calibration least-squares linear system
	
	Threads::TripleBuffer<Kinect::DiskExtractor::DiskList> diskList; // Triple buffer of lists of extracted disks
	Point::AffineCombiner tiePointCombiners[2]; // Combiners for controller and disk center tie points
	std::vector<TiePoint> tiePoints; // List of collected calibration tie points
	Vrui::OGTransform cameraTransform; // The current extrinsic camera transformation
	
	/* Private methods: */
	void depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer); // Callback receiving depth frames from the frame source
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	void meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer); // Callback receiving projected meshes from the Kinect projector
	#endif
	void calibrateToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData); // Callback called when the "Calibrate" toggle button changes
	void maxBlobMergeDistSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void minNumPixelsSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void diskRadiusSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void diskRadiusMarginSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void diskFlatnessSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void saveAlignmentCallback(Misc::CallbackData* cbData);
	GLMotif::PopupWindow* createConfigurationDialog(void); // Creates the configuration dialog
	void trackingCallback(Vrui::VRDeviceClient* client); // Called when new tracking data arrives
	void diskExtractionCallback(const Kinect::DiskExtractor::DiskList& disks); // Called when a new list of disks has been extracted
	static Vrui::OGTransform calcOGTransform(const std::vector<TiePoint>& tiePoints); // Calculates the optimal orthogonal alignment transformation for the given list of tie points
	void calcCameraTransform(void); // Calculates the extrinsic camera transformation
	
	/* Constructors and destructors: */
	public:
	ExtrinsicCalibrator(int& argc,char**& argv);
	virtual ~ExtrinsicCalibrator(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	};

/************************************
Methods of class ExtrinsicCalibrator:
************************************/

void ExtrinsicCalibrator::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	/* Forward depth frame to the sphere extractor: */
	diskExtractor->submitFrame(frameBuffer);
	
	/* Forward depth frame to the projector: */
	projector->setDepthFrame(frameBuffer);
	
	#if KINECT_CONFIG_USE_SHADERPROJECTOR
	/* Update application state: */
	Vrui::requestUpdate();
	#endif
	}

#if !KINECT_CONFIG_USE_SHADERPROJECTOR

void ExtrinsicCalibrator::meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer)
	{
	/* Update application state: */
	Vrui::requestUpdate();
	}

#endif

void ExtrinsicCalibrator::calibrateToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Start or stop calibrating the disk center: */
	calibratingDiskCenter=cbData->set;
	
	if(calibratingDiskCenter)
		{
		/* Initialize calibration state: */
		calibTransforms.clear();
		calibAta=Math::Matrix(6,6,0.0);
		calibAtb=Math::Matrix(6,1,0.0);
		}
	else
		{
		/* Solve for the local and global disk center position: */
		Math::Matrix x=calibAtb;
		x.divideFullPivot(calibAta);
		
		/* Evaluate all collected transformations to calculate calibration error: */
		ATransform::Point lp(x(0),x(1),x(2));
		ATransform::Point gp(x(3),x(4),x(5));
		Math::VarianceAccumulator err[3];
		for(std::vector<ATransform>::const_iterator ctIt=calibTransforms.begin();ctIt!=calibTransforms.end();++ctIt)
			{
			Vector d=ctIt->transform(lp)-gp;
			for(int i=0;i<3;++i)
				err[i].addSample(d[i]);
			}
		Misc::formattedConsoleNote("Calibrate Disk Center: Disk center is (%.4f. %.4f, %.4f)",double(x(0)),double(x(1)),double(x(2)));
		Misc::formattedUserNote("Calibrate Disk Center: Calibration standard deviation (%.4f, %.4f, %.4f)",err[0].calcStdDeviation(),err[1].calcStdDeviation(),err[2].calcStdDeviation());
		
		/* Store the calibrated disk center and update the display: */
		for(int i=0;i<3;++i)
			{
			diskCenter[i]=Scalar(x(i));
			diskCenterTextFields[i]->setValue(x(i));
			}
		}
	}

void ExtrinsicCalibrator::maxBlobMergeDistSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	diskExtractor->setMaxBlobMergeDist(int(Math::floor(cbData->value+0.5)));
	}

void ExtrinsicCalibrator::minNumPixelsSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	diskExtractor->setMinNumPixels((unsigned int)(Math::floor(cbData->value+0.5)));
	}

void ExtrinsicCalibrator::diskRadiusSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	diskExtractor->setDiskRadius(cbData->value);
	}

void ExtrinsicCalibrator::diskRadiusMarginSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	diskExtractor->setDiskRadiusMargin(cbData->value);
	}

void ExtrinsicCalibrator::diskFlatnessSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	diskExtractor->setDiskFlatness(cbData->value);
	}

void ExtrinsicCalibrator::saveAlignmentCallback(Misc::CallbackData* cbData)
	{
	/* Open the camera's extrinsic parameter file: */
	std::string extrinsicFileName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	extrinsicFileName.push_back('/');
	extrinsicFileName.append(KINECT_INTERNAL_CONFIG_CAMERA_EXTRINSICPARAMETERSFILENAMEPREFIX);
	extrinsicFileName.push_back('-');
	extrinsicFileName.append(cameraSerialNumber);
	extrinsicFileName.append(".txt");
	IO::OStream extrinsicFile(Vrui::openFile(extrinsicFileName.c_str(),IO::File::WriteOnly));
	
	/* Write the extrinsic calibration transformation: */
	extrinsicFile<<Misc::ValueCoder<Vrui::OGTransform>::encode(cameraTransform)<<std::endl;
	}

GLMotif::PopupWindow* ExtrinsicCalibrator::createConfigurationDialog(void)
	{
	const GLMotif::StyleSheet& ss=*Vrui::getWidgetManager()->getStyleSheet();
	
	GLMotif::PopupWindow* configurationDialogPopup=new GLMotif::PopupWindow("ConfigurationDialogPopup",Vrui::getWidgetManager(),"Configuration");
	
	GLMotif::Pager* configurationPager=new GLMotif::Pager("ConfigurationPager",configurationDialogPopup,false);
	configurationPager->setMarginWidth(ss.size);
	
	/* Create the disk center calibration page: */
	configurationPager->setNextPageName("Disk Center");
	
	GLMotif::Margin* diskCenterDialogMargin=new GLMotif::Margin("DiskCenterDialogMargin",configurationPager,false);
	diskCenterDialogMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::VCENTER));
	
	GLMotif::RowColumn* diskCenterDialog=new GLMotif::RowColumn("DiskCenterDialog",diskCenterDialogMargin,false);
	diskCenterDialog->setOrientation(GLMotif::RowColumn::VERTICAL);
	diskCenterDialog->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	diskCenterDialog->setNumMinorWidgets(1);
	
	GLMotif::RowColumn* diskCenterBox=new GLMotif::RowColumn("DiskCenterBox",diskCenterDialog,false);
	diskCenterBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	diskCenterBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	diskCenterBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("DiskCenterLabel",diskCenterBox,"Disk Center");
	
	for(int i=0;i<3;++i)
		{
		char textFieldName[]="DiskCenterTextField0";
		textFieldName[19]='0'+i;
		diskCenterTextFields[i]=new GLMotif::TextField(textFieldName,diskCenterBox,8);
		diskCenterTextFields[i]->setPrecision(4);
		diskCenterTextFields[i]->setFloatFormat(GLMotif::TextField::FIXED);
		diskCenterTextFields[i]->setValue(diskCenter[i]);
		
		diskCenterBox->setColumnWeight(i+1,1.0f);
		}
	
	diskCenterBox->manageChild();
	
	GLMotif::Margin* calibrateMargin=new GLMotif::Margin("CalibrateMargin",diskCenterDialog,false);
	calibrateMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::RIGHT));
	
	GLMotif::ToggleButton* calibrateToggle=new GLMotif::ToggleButton("CalibrateToggle",calibrateMargin,"Calibrate Disk Center");
	calibrateToggle->setToggle(false);
	calibrateToggle->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::calibrateToggleCallback);
	
	calibrateMargin->manageChild();
	
	diskCenterDialog->manageChild();
	
	diskCenterDialogMargin->manageChild();
	
	/* Create the disk extractor configuration page: */
	configurationPager->setNextPageName("Disk Extractor");
	
	GLMotif::Margin* diskExtractorDialogMargin=new GLMotif::Margin("DiskExtractorDialogMargin",configurationPager,false);
	diskExtractorDialogMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::VCENTER));
	
	GLMotif::RowColumn* diskExtractorDialog=new GLMotif::RowColumn("DiskExtractorDialog",diskExtractorDialogMargin,false);
	diskExtractorDialog->setOrientation(GLMotif::RowColumn::VERTICAL);
	diskExtractorDialog->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	diskExtractorDialog->setNumMinorWidgets(2);
	
	new GLMotif::Label("MaxBlobMergeDistLabel",diskExtractorDialog,"Max Blob Merge Dist");
	
	GLMotif::TextFieldSlider* maxBlobMergeDistSlider=new GLMotif::TextFieldSlider("MaxBlobMergeDistSlider",diskExtractorDialog,6,ss.fontHeight*10.0f);
	maxBlobMergeDistSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	maxBlobMergeDistSlider->setValueType(GLMotif::TextFieldSlider::INT);
	maxBlobMergeDistSlider->setValueRange(0,100,1);
	maxBlobMergeDistSlider->setValue(diskExtractor->getMaxBlobMergeDist());
	maxBlobMergeDistSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::maxBlobMergeDistSliderCallback);
	
	new GLMotif::Label("MinNumPixelsLabel",diskExtractorDialog,"Min Num Pixels");
	
	GLMotif::TextFieldSlider* minNumPixelsSlider=new GLMotif::TextFieldSlider("MinNumPixelsSlider",diskExtractorDialog,6,ss.fontHeight*10.0f);
	minNumPixelsSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	minNumPixelsSlider->setValueType(GLMotif::TextFieldSlider::UINT);
	minNumPixelsSlider->setValueRange(1,50000,0.01);
	minNumPixelsSlider->setValue(diskExtractor->getMinNumPixels());
	minNumPixelsSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::minNumPixelsSliderCallback);
	
	new GLMotif::Label("DiskRadiusLabel",diskExtractorDialog,"Disk Radius");
	
	GLMotif::TextFieldSlider* diskRadiusSlider=new GLMotif::TextFieldSlider("DiskRadiusSlider",diskExtractorDialog,6,ss.fontHeight*10.0f);
	diskRadiusSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	diskRadiusSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	diskRadiusSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	diskRadiusSlider->getTextField()->setPrecision(2);
	diskRadiusSlider->setValueRange(1.0,100.0,0.01);
	diskRadiusSlider->setValue(diskExtractor->getDiskRadius());
	diskRadiusSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::diskRadiusSliderCallback);
	
	new GLMotif::Label("DiskRadiusMarginLabel",diskExtractorDialog,"Disk Radius Margin");
	
	GLMotif::TextFieldSlider* diskRadiusMarginSlider=new GLMotif::TextFieldSlider("DiskRadiusMarginSlider",diskExtractorDialog,6,ss.fontHeight*10.0f);
	diskRadiusMarginSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	diskRadiusMarginSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	diskRadiusMarginSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	diskRadiusMarginSlider->getTextField()->setPrecision(2);
	diskRadiusMarginSlider->setValueRange(1.0,2.0,0.01);
	diskRadiusMarginSlider->setValue(diskExtractor->getDiskRadiusMargin());
	diskRadiusMarginSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::diskRadiusMarginSliderCallback);
	
	new GLMotif::Label("DiskFlatnessLabel",diskExtractorDialog,"Disk Flatness");
	
	GLMotif::TextFieldSlider* diskFlatnessSlider=new GLMotif::TextFieldSlider("DiskFlatnessSlider",diskExtractorDialog,6,ss.fontHeight*10.0f);
	diskFlatnessSlider->setSliderMapping(GLMotif::TextFieldSlider::LINEAR);
	diskFlatnessSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	diskFlatnessSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	diskFlatnessSlider->getTextField()->setPrecision(2);
	diskFlatnessSlider->setValueRange(0.0,50.0,0.01);
	diskFlatnessSlider->setValue(diskExtractor->getDiskFlatness());
	diskFlatnessSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::diskFlatnessSliderCallback);
	
	diskExtractorDialog->manageChild();
	
	diskExtractorDialogMargin->manageChild();
	
	/* Create the extrinsic calibration page: */
	configurationPager->setNextPageName("Extrinsic Calibration");
	
	GLMotif::Margin* extrinsicCalibrationDialogMargin=new GLMotif::Margin("ExtrinsicCalibrationDialogMargin",configurationPager,false);
	extrinsicCalibrationDialogMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::VCENTER));
	
	GLMotif::RowColumn* extrinsicCalibrationDialog=new GLMotif::RowColumn("ExtrinsicCalibrationDialog",extrinsicCalibrationDialogMargin,false);
	extrinsicCalibrationDialog->setOrientation(GLMotif::RowColumn::VERTICAL);
	extrinsicCalibrationDialog->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	extrinsicCalibrationDialog->setNumMinorWidgets(1);
	
	GLMotif::RowColumn* alignmentResidualBox=new GLMotif::RowColumn("AlignmentResidualBox",extrinsicCalibrationDialog,false);
	alignmentResidualBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	alignmentResidualBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	alignmentResidualBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("AlignmentResidualLabel",alignmentResidualBox,"Alignment RMS Residual");
	
	alignmentResidualTextField=new GLMotif::TextField("AlignmentResidualTextField",alignmentResidualBox,8);
	alignmentResidualTextField->setPrecision(4);
	alignmentResidualTextField->setFloatFormat(GLMotif::TextField::FIXED);
	alignmentResidualTextField->setString("");
	
	alignmentResidualBox->manageChild();
	
	GLMotif::Margin* saveAlignmentMargin=new GLMotif::Margin("SaveAlignmentMargin",extrinsicCalibrationDialog,false);
	saveAlignmentMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::RIGHT));
	
	GLMotif::Button* saveAlignmentButton=new GLMotif::Button("SaveAlignmentButton",saveAlignmentMargin,"Save Alignment");
	saveAlignmentButton->getSelectCallbacks().add(this,&ExtrinsicCalibrator::saveAlignmentCallback);
	
	saveAlignmentMargin->manageChild();
	
	extrinsicCalibrationDialog->manageChild();
	
	extrinsicCalibrationDialogMargin->manageChild();
	
	configurationPager->setCurrentChildIndex(0);
	configurationPager->manageChild();
	
	return configurationDialogPopup;
	}

void ExtrinsicCalibrator::trackingCallback(Vrui::VRDeviceClient* client)
	{
	/* Lock and retrieve the most recent input device states: */
	deviceClient->lockState();
	const Vrui::VRDeviceState& state=deviceClient->getState();
	
	/* Extract all controller's current tracking states into a new triple buffer slot: */
	Vrui::TrackerState* tss=controllerStates.startNewValue();
	for(unsigned int i=0;i<controllers.size();++i)
		tss[i]=state.getTrackerState(controllers[i]->trackerIndex).positionOrientation;
	
	/* Check if the active controller changed: */
	int newControllerIndex=previousControllerIndex;
	if(newControllerIndex==-1)
		{
		if(buttonIndex>=0)
			{
			/* Check if the specified button is pressed: */
			if(state.getButtonState(buttonIndex))
				{
				/* Find the controller belonging to the specified button: */
				for(unsigned int i=0;i<controllers.size();++i)
					for(int j=0;j<controllers[i]->numButtons;++j)
						{
						if(controllers[i]->buttonIndices[j]==buttonIndex)
							newControllerIndex=i;
						}
				}
			}
		else
			{
			/* Check if any controller buttons are pressed: */
			for(unsigned int i=0;i<controllers.size();++i)
				for(int j=0;j<controllers[i]->numButtons;++j)
					{
					if(state.getButtonState(controllers[i]->buttonIndices[j]))
						newControllerIndex=i;
					}
			}
		}
	else
		{
		/* Check if the previously active controller is still active: */
		newControllerIndex=-1;
		for(int j=0;j<controllers[previousControllerIndex]->numButtons;++j)
			{
			if(state.getButtonState(controllers[previousControllerIndex]->buttonIndices[j]))
				newControllerIndex=previousControllerIndex;
			}
		}
	if(previousControllerIndex!=newControllerIndex)
		{
		controllerIndex.postNewValue(newControllerIndex);
		previousControllerIndex=newControllerIndex;
		}
	
	/* Release input device state lock: */
	deviceClient->unlockState();
	
	/* Post the new controller states and wake up the main thread: */
	controllerStates.postNewValue();
	Vrui::requestUpdate();
	}

void ExtrinsicCalibrator::diskExtractionCallback(const Kinect::DiskExtractor::DiskList& disks)
	{
	/* Store the new disk list in the triple buffer: */
	Kinect::DiskExtractor::DiskList& newList=diskList.startNewValue();
	newList=disks;
	diskList.postNewValue();
	
	Vrui::requestUpdate();
	}

Vrui::OGTransform ExtrinsicCalibrator::calcOGTransform(const std::vector<ExtrinsicCalibrator::TiePoint>& tiePoints)
	{
	/* Calculate both point sets' centroids: */
	Point::AffineCombiner cc0;
	Point::AffineCombiner cc1;
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		cc0.addPoint(tpIt->first);
		cc1.addPoint(tpIt->second);
		}
	Point c0=cc0.getPoint();
	Point c1=cc1.getPoint();
	
	/* Calculate both point sets' inner products: */
	double ip0=0.0;
	double ip1=0.0;
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		Vector d0=tpIt->first-c0;
		Vector d1=tpIt->second-c1;
		ip0+=Math::sqr(d0[0])+Math::sqr(d0[1])+Math::sqr(d0[2]);
		ip1+=Math::sqr(d1[0])+Math::sqr(d1[1])+Math::sqr(d1[2]);
		}
	
	/* Calculate the normalizing scaling factors: */
	double scale0=Math::sqrt(ip0);
	double scale1=Math::sqrt(ip1);
	
	/* Move both point sets to their centroids and scale them to uniform size: */
	Vrui::OGTransform centroidTransform0=Vrui::OGTransform::translateToOriginFrom(c0);
	centroidTransform0.leftMultiply(Vrui::OGTransform::scale(1.0/scale0));
	Vrui::OGTransform centroidTransform1=Vrui::OGTransform::translateToOriginFrom(c1);
	centroidTransform1.leftMultiply(Vrui::OGTransform::scale(1.0/scale1));
	
	/* Calculate the inner product between the two scaled and centroid-aligned point sets: */
	double m[3][3];
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			m[i][j]=0.0;
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Normalize both tie point components: */
		Point p0=centroidTransform0.transform(tpIt->first);
		Point p1=centroidTransform1.transform(tpIt->second);
		
		/* Add their inner product to the matrix: */
		for(int i=0;i<3;++i)
			for(int j=0;j<3;++j)
				m[i][j]+=p0[i]*p1[j];
		}
	
	/* Calculate the coefficients of the quaternion-based characteristic polynomial of the quaternion key matrix: */
	double q4=1.0;
	double q3=0.0;
	double q2=0.0;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			q2-=2.0*Math::sqr(m[i][j]);
	double q1=8.0*(m[0][0]*m[1][2]*m[2][1]+m[1][1]*m[2][0]*m[0][2]+m[2][2]*m[0][1]*m[1][0])
	         -8.0*(m[0][0]*m[1][1]*m[2][2]+m[1][2]*m[2][0]*m[0][1]+m[2][1]*m[1][0]*m[0][2]);
	double qd0=Math::sqr(Math::sqr(m[0][1])+Math::sqr(m[0][2])-Math::sqr(m[1][0])-Math::sqr(m[2][0]));
	double qd1=(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])-2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]))
	          *(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])+2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]));
	double qd2=(-(m[0][2]+m[2][0])*(m[1][2]-m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]+m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]+m[2][2]));
	double qd3=(-(m[0][2]+m[2][0])*(m[1][2]+m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]-m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd4=((m[0][1]+m[1][0])*(m[1][2]+m[2][1])+(m[0][2]+m[2][0])*(m[0][0]-m[1][1]+m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]-m[2][1])+(m[0][2]+m[2][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd5=((m[0][1]+m[1][0])*(m[1][2]-m[2][1])+(m[0][2]-m[2][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]+m[2][1])+(m[0][2]-m[2][0])*(m[0][0]+m[1][1]-m[2][2]));
	double q0=qd0+qd1+qd2+qd3+qd4+qd5;
	
	/* Calculate the optimal rotation: */
	double lambda=Math::mid(ip0,ip1);
	double lambda0;
	do
		{
		lambda0=lambda;
		double poly=(((q4*lambda+q3)*lambda+q2)*lambda+q1)*lambda+q0;
		double dPoly=((4.0*q4*lambda+3.0*q3)*lambda+2.0*q2)*lambda+q1;
		lambda-=poly/dPoly;
		}
	while(Math::abs(lambda-lambda0)<1.0e-10);
	
	/* Find the eigenvector corresponding to the largest eigenvalue: */
	Math::Matrix k(4,4);
	k(0,0)=m[0][0]+m[1][1]+m[2][2];
	k(0,1)=m[1][2]-m[2][1];
	k(0,2)=m[2][0]-m[0][2];
	k(0,3)=m[0][1]-m[1][0];
	k(1,0)=m[1][2]-m[2][1];
	k(1,1)=m[0][0]-m[1][1]-m[2][2];
	k(1,2)=m[0][1]+m[1][0];
	k(1,3)=m[2][0]+m[0][2];
	k(2,0)=m[2][0]-m[0][2];
	k(2,1)=m[0][1]+m[1][0];
	k(2,2)=-m[0][0]+m[1][1]-m[2][2];
	k(2,3)=m[1][2]+m[2][1];
	k(3,0)=m[0][1]-m[1][0];
	k(3,1)=m[2][0]+m[0][2];
	k(3,2)=m[1][2]+m[2][1];
	k(3,3)=-m[0][0]-m[1][1]+m[2][2];
	
	/* Calculate the optimal rotation: */
	std::pair<Math::Matrix,Math::Matrix> jacobi=k.jacobiIteration();
	double maxE=jacobi.second(0);
	int maxEIndex=0;
	for(int i=1;i<4;++i)
		if(maxE<jacobi.second(i))
			{
			maxE=jacobi.second(i);
			maxEIndex=i;
			}
	Vrui::Rotation rotation=Vrui::Rotation::fromQuaternion(jacobi.first(1,maxEIndex),jacobi.first(2,maxEIndex),jacobi.first(3,maxEIndex),jacobi.first(0,maxEIndex));
	
	/* Assemble and return the optimal alignment transformation: */
	Vrui::OGTransform result=Geometry::invert(centroidTransform1);
	result*=Vrui::OGTransform::rotate(rotation);
	result*=centroidTransform0;
	result.renormalize();
	return result;
	}

void ExtrinsicCalibrator::calcCameraTransform(void)
	{
	/* DEBUGGING: Save tie points to a file */
	{
	std::ofstream points0File("KinectPoints.csv");
	std::ofstream points1File("TrackerPoints.csv");
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		points0File<<tpIt->first[0]<<','<<tpIt->first[1]<<','<<tpIt->first[2]<<std::endl;
		points1File<<tpIt->second[0]<<','<<tpIt->second[1]<<','<<tpIt->second[2]<<std::endl;
		}
	}
	
	/* Calculate the alignment transformation: */
	cameraTransform=calcOGTransform(tiePoints);
	
	/* Calculate the residual: */
	double rmsd=0.0;
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		rmsd+=Geometry::sqrDist(cameraTransform.transform(tpIt->first),Vrui::Point(tpIt->second));
	alignmentResidualTextField->setValue(Math::sqrt(rmsd/double(tiePoints.size())));
	// std::cout<<"Best distance: "<<Math::sqrt(rmsd/double(tiePoints.size()))<<std::endl;
	}

ExtrinsicCalibrator::ExtrinsicCalibrator(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 deviceClient(0),buttonIndex(-1),
	 diskCenter(Point::origin),
	 configurationDialog(0),
	 previousControllerIndex(-1),
	 calibratingDiskCenter(false),
	 cameraTransform(Vrui::OGTransform::identity)
	{
	/* Parse command line: */
	const char* serverName="localhost:8555";
	Kinect::MultiplexedFrameSource* remoteSource=0;
	int cameraIndex=0;
	for(int i=1;i<argc;++i)
		{
		if(argv[i][0]=='-')
			{
			if(strcasecmp(argv[i]+1,"server")==0)
				{
				++i;
				if(i<argc)
					serverName=argv[i];
				}
			else if(strcasecmp(argv[i]+1,"remote")==0)
				{
				i+=2;
				if(i<argc)
					{
					/* Open a connection to a remote Kinect server: */
					remoteSource=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),argv[i-1],atoi(argv[i])));
					}
				}
			else if(strcasecmp(argv[i]+1,"camera")==0)
				{
				++i;
				if(i<argc)
					cameraIndex=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"button")==0)
				{
				++i;
				if(i<argc)
					buttonIndex=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"diskCenter")==0)
				{
				for(int j=0;j<3;++j)
					{
					++i;
					diskCenter[j]=Vrui::Scalar(atof(argv[i]));
					}
				std::cout<<"Disk center position: "<<diskCenter[0]<<", "<<diskCenter[1]<<", "<<diskCenter[2]<<std::endl;
				}
			}
		}
	
	/* Split the server name into hostname:port: */
	const char* colonPtr=0;
	for(const char* cPtr=serverName;*cPtr!='\0';++cPtr)
		if(*cPtr==':')
			colonPtr=cPtr;
	int portNumber=8555;
	if(colonPtr!=0)
		portNumber=atoi(colonPtr+1);
	std::string hostName=colonPtr!=0?std::string(serverName,colonPtr):std::string(serverName);
	
	/* Initialize device client: */
	deviceClient=new Vrui::VRDeviceClient(hostName.c_str(),portNumber);
	
	/* Query a list of virtual devices that have buttons: */
	for(int i=0;i<deviceClient->getNumVirtualDevices();++i)
		{
		/* Store the device as a controller if it has position and direction tracking and at least one button: */
		const Vrui::VRDeviceDescriptor* device=&deviceClient->getVirtualDevice(i);
		if((device->trackType&Vrui::VRDeviceDescriptor::TRACK_POS)&&(device->trackType&Vrui::VRDeviceDescriptor::TRACK_ORIENT)&&device->numButtons>0)
			controllers.push_back(device);
		}
	
	/* Open the requested 3D video source: */
	if(remoteSource!=0)
		{
		camera=remoteSource->getStream(cameraIndex);
		cameraSerialNumber="RemoteCamera";
		}
	else
		{
		Kinect::DirectFrameSource* directCamera=Kinect::openDirectFrameSource(cameraIndex);
		camera=directCamera;
		cameraSerialNumber=directCamera->getSerialNumber();
		}
	
	/* Create a disk extractor for the 3D video source: */
	diskExtractor=new Kinect::DiskExtractor(camera->getActualFrameSize(Kinect::FrameSource::DEPTH),camera->getDepthCorrectionParameters(),camera->getIntrinsicParameters());
	diskExtractor->setMaxBlobMergeDist(2);
	diskExtractor->setMinNumPixels(300);
	diskExtractor->setDiskRadius(6.0);
	diskExtractor->setDiskRadiusMargin(1.10);
	diskExtractor->setDiskFlatness(2.5);
	
	/* Create a projector for the 3D video source: */
	projector=new Kinect::ProjectorType(*camera);
	
	/* Initialize interaction state: */
	for(int i=0;i<3;++i)
		controllerStates.getBuffer(i)=new Vrui::TrackerState[controllers.size()];
	for(int i=0;i<3;++i)
		controllerIndex.getBuffer(i)=-1;
	
	/* Create the configuration dialog: */
	configurationDialog=createConfigurationDialog();
	Vrui::popupPrimaryWidget(configurationDialog);
	
	/* Activate the device client and start streaming: */
	deviceClient->activate();
	deviceClient->startStream(Misc::createFunctionCall(this,&ExtrinsicCalibrator::trackingCallback));
	
	/* Start streaming from the 3D video source and extracting disks: */
	diskExtractor->startStreaming(Misc::createFunctionCall(this,&ExtrinsicCalibrator::diskExtractionCallback));
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	projector->startStreaming(Misc::createFunctionCall(this,&ExtrinsicCalibrator::meshStreamingCallback));
	#endif
	camera->startStreaming(Misc::createFunctionCall(projector,&Kinect::ProjectorType::setColorFrame),Misc::createFunctionCall(this,&ExtrinsicCalibrator::depthStreamingCallback));
	}

ExtrinsicCalibrator::~ExtrinsicCalibrator(void)
	{
	// std::cout<<"Extrinsic transformation: "<<Misc::ValueCoder<Vrui::OGTransform>::encode(cameraTransform)<<std::endl;
	
	/* Stop streaming from the 3D video source: */
	camera->stopStreaming();
	diskExtractor->stopStreaming();
	
	/* Stop streaming and deactivate the device client: */
	deviceClient->stopStream();
	deviceClient->deactivate();
	
	delete configurationDialog;
	
	/* Clean up: */
	delete diskExtractor;
	delete projector;
	delete camera;
	for(int i=0;i<3;++i)
		delete[] controllerStates.getBuffer(i);
	delete deviceClient;
	}

void ExtrinsicCalibrator::frame(void)
	{
	/* Check if the pressed button changed: */
	if(controllerIndex.lockNewValue()&&!calibratingDiskCenter)
		{
		if(controllerIndex.getLockedValue()>=0) // Button has just been pressed
			{
			/* Reset the tie point accumulators: */
			for(int i=0;i<2;++i)
				tiePointCombiners[i].reset();
			}
		else // Button has just been released
			{
			/* Store the accumulated tie point: */
			TiePoint tp;
			tp.first=tiePointCombiners[0].getPoint();
			tp.second=tiePointCombiners[1].getPoint();
			bool finite=true;
			for(int i=0;i<3;++i)
				finite=finite&&Math::isFinite(tp.first[i])&&Math::isFinite(tp.second[i]);
			if(finite)
				tiePoints.push_back(tp);
			
			/* Run calibration if there are enough tie points: */
			if(tiePoints.size()>=3)
				calcCameraTransform();
			}
		}
	
	/* Check if there is new tracking data for the active controller: */
	if(controllerStates.lockNewValue()&&controllerIndex.getLockedValue()>=0)
		{
		if(calibratingDiskCenter)
			{
			/* Convert the controller's new orthonormal transformation to an affine transformation: */
			ATransform ct;
			controllerStates.getLockedValue()[controllerIndex.getLockedValue()].writeMatrix(ct.getMatrix());
			
			/* Store the affine transformation for later: */
			calibTransforms.push_back(ct);
			
			/* Enter the affine transformation into the least-squares linear system: */
			for(int i=0;i<3;++i)
				{
				/* Set up the equation: */
				double eq[7];
				for(int j=0;j<3;++j)
					{
					eq[j]=ct.getMatrix()(i,j);
					eq[3+j]=i==j?-1.0:0.0;
					}
				eq[6]=-ct.getMatrix()(i,3);
				
				/* Enter the equation into the least-squares system: */
				for(int j=0;j<6;++j)
					{
					for(int k=0;k<6;++k)
						calibAta(j,k)+=eq[j]*eq[k];
					calibAtb(j)+=eq[j]*eq[6];
					}
				}
			}
		else
			{
			/* Add the controller position to the tie point combiner: */
			tiePointCombiners[1].addPoint(controllerStates.getLockedValue()[controllerIndex.getLockedValue()].transform(diskCenter));
			}
		}
	
	/* Check if there are new extracted disks: */
	if(diskList.lockNewValue()&&diskList.getLockedValue().size()>0&&controllerIndex.getLockedValue()>=0)
		{
		if(!calibratingDiskCenter)
			{
			#if 0
			// DEBUGGING
			const Kinect::DiskExtractor::Disk& disk=diskList.getLockedValue().front();
			std::cout<<disk.radius<<", "<<disk.flatness<<", "<<disk.numPixels<<std::endl;
			#endif
			
			/* Add the first disk center position to the tie point combiner: */
			tiePointCombiners[0].addPoint(diskList.getLockedValue().front().center);
			}
		}
	
	/* Update the projector: */
	projector->updateFrames();
	}

void ExtrinsicCalibrator::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glLineWidth(1.0f);
	glPointSize(3.0f);
	
	/* Draw the current disk center positions driven by all controllers: */
	glBegin(GL_POINTS);
	glColor3f(1.0f,1.0f,1.0f);
	for(unsigned int i=0;i<controllers.size();++i)
		glVertex(controllerStates.getLockedValue()[i].transform(diskCenter));
	glEnd();
	
	/* Draw all extracted disks: */
	glPushMatrix();
	glMultMatrix(cameraTransform);
	
	const Kinect::DiskExtractor::DiskList& dl=diskList.getLockedValue();
	for(Kinect::DiskExtractor::DiskList::const_iterator dlIt=dl.begin();dlIt!=dl.end();++dlIt)
		{
		glPushMatrix();
		glTranslate(dlIt->center-Kinect::DiskExtractor::Point::origin);
		glRotate(Vrui::Rotation::rotateFromTo(Vrui::Vector(0,0,1),Vrui::Vector(dlIt->normal)));
		
		glBegin(GL_LINE_LOOP);
		glColor3f(1.0f,1.0f,1.0f);
		for(int i=0;i<64;++i)
			{
			Vrui::Scalar angle=Vrui::Scalar(i)*Vrui::Scalar(2)*Math::Constants<Vrui::Scalar>::pi/Vrui::Scalar(64);
			glVertex3d(Math::cos(angle)*dlIt->radius,Math::sin(angle)*dlIt->radius,0.0);
			}
		glEnd();
		
		glBegin(GL_POLYGON);
		glColor3f(1.0f,0.0f,0.0f);
		for(int i=0;i<64;++i)
			{
			Vrui::Scalar angle=Vrui::Scalar(i)*Vrui::Scalar(2)*Math::Constants<Vrui::Scalar>::pi/Vrui::Scalar(64);
			glVertex3d(Math::cos(angle)*dlIt->radius,Math::sin(angle)*dlIt->radius,0.0);
			}
		glEnd();
		
		glPopMatrix();
		}
	
	/* Draw the current 3D video facade: */
	glMultMatrix(Geometry::invert(projector->getProjectorTransform()));
	projector->glRenderAction(contextData);
	
	glPopMatrix();
	
	/* Draw all collected tie points: */
	glBegin(GL_LINES);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		glColor3f(0.0f,1.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->first));
		glColor3f(1.0f,0.0f,1.0f);
		glVertex(tpIt->second);
		}
	glEnd();
	glBegin(GL_POINTS);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		glColor3f(0.0f,1.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->first));
		glColor3f(1.0f,0.0f,1.0f);
		glVertex(tpIt->second);
		}
	glEnd();
	
	glPopAttrib();
	}

void ExtrinsicCalibrator::resetNavigation(void)
	{
	/* Move the camera position to the viewer's head position: */
	Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
	
	/* Rotate the frame to align the projection direction with the viewer's viewing direction: */
	nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateX(Math::rad(Vrui::Scalar(90))));
	
	/* Account for the projector's model space transformation: */
	nav*=Geometry::invert(cameraTransform);
	
	Vrui::setNavigationTransformation(nav);
	}

VRUI_APPLICATION_RUN(ExtrinsicCalibrator)
