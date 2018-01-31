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
#include <IO/File.h>
#include <IO/OStream.h>
#include <Cluster/OpenPipe.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Random.h>
#include <Math/Matrix.h>
#include <Math/VarianceAccumulator.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Rotation.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/AffineTransformation.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/AlignPoints.h>
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
#include <Vrui/ObjectSnapperTool.h>
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

/* Flag to enable the experimental full calibration routine: */
#define RUN_FULL_CALIBRATION 1

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
	typedef std::pair<PO,Point> FullCalibTiePoint;
	typedef Geometry::OrthogonalTransformation<double,3> CameraTransform; // Type for camera transformations
	
	#if RUN_FULL_CALIBRATION
	struct Calibration // Structure to hold (preliminary) calibration results
		{
		/* Elements: */
		public:
		Point diskCenter; // Disk center in controller's local coordinates
		CameraTransform cameraTransform; // Transformation from camera space to tracker space
		double rms; // RMS error for all provided tie points
		double linf; // L-infinity error for all provided tie points
		};
	#endif
	
	/* Elements: */
	Vrui::VRDeviceClient* deviceClient; // Connection to the VRDeviceDaemon
	std::vector<const Vrui::VRDeviceDescriptor*> controllers; // List of input devices that have buttons
	int trackerIndex; // Index of tracker used to collect tie points; if -1, the tracker attached to the most-recently pressed button will be used
	int buttonIndex; // Index of button to trigger tie point collection; if -1, any button will do
	Vrui::Point diskCenter; // Position of disk center in controller's local coordinate system
	
	Kinect::FrameSource* camera; // 3D video source to calibrate
	std::string cameraSerialNumber; // Camera's serial number if camera is a direct frame source; some dummy string otherwise
	Kinect::DiskExtractor* diskExtractor; // Object to extract disk shapes from a 3D video stream
	Kinect::ProjectorType* projector; // A projeftor to render the 3D video stream
	GLMotif::PopupWindow* configurationDialog; // Dialog window to configure the extrinsic calibrator
	GLMotif::TextField* diskCenterTextFields[3]; // Text fields displaying the calibrated disk center
	GLMotif::TextField* alignmentResidualTextField; // Text field showing the RMS residual of the most recent point set alignment
	
	Threads::TripleBuffer<PO*> controllerStates; // Triple buffer of arrays of current controller tracking states
	int previousControllerIndex; // Index of previously active controller
	Threads::TripleBuffer<int> controllerIndex; // Triple buffer containing index of the controller owning the currently pressed button, or -1
	int lastActiveControllerIndex; // Index of the controller that was last used to collect a tie point, or -1
	bool calibratingDiskCenter; // Flag whether currently collecting controller transformations for disk center calibration
	std::vector<ATransform> calibTransforms; // Vector of controller transformations collected during calibration for residual calculation
	Math::Matrix calibAta,calibAtb; // Accumulated disk center calibration least-squares linear system
	
	bool movingCamera; // Flag whether the camera instead of the calibration disk is attached to the tracked controller
	Threads::TripleBuffer<Kinect::DiskExtractor::DiskList> diskList; // Triple buffer of lists of extracted disks
	Point::AffineCombiner tiePointCombiners[2]; // Combiners for controller and disk center tie points
	std::vector<TiePoint> tiePoints; // List of collected calibration tie points
	bool haveCalibration; // True after the first calibration has been calculated
	CameraTransform cameraTransform; // The current extrinsic camera transformation
	
	#if RUN_FULL_CALIBRATION
	std::vector<FullCalibTiePoint> fullCalibTiePoints; // List of collected tie points (controller transformation and disk center) for full calibration
	unsigned int fullCalibNumSamples; // Number of samples accumulated into the full calibration system
	unsigned int numAlignIterations; // Number of iterations for non-linear point set alignment
	unsigned int numRansacIterations; // Number of RANSAC iterations, duh
	double ransacMaxInlier; // Maximum inlier distance for RANSAC
	#endif
	
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
	void movingCameraToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData);
	#if RUN_FULL_CALIBRATION
	void ransacMaxInlierSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	#endif
	void saveAlignmentCallback(Misc::CallbackData* cbData);
	GLMotif::PopupWindow* createConfigurationDialog(void); // Creates the configuration dialog
	void trackingCallback(Vrui::VRDeviceClient* client); // Called when new tracking data arrives
	void diskExtractionCallback(const Kinect::DiskExtractor::DiskList& disks); // Called when a new list of disks has been extracted
	void objectSnapCallback(Vrui::ObjectSnapperToolFactory::SnapRequest& snapRequest); // Handles a snap request from an object snapper tool
	static Vrui::OGTransform calcOGTransform(const std::vector<TiePoint>& tiePoints); // Calculates the optimal orthogonal alignment transformation for the given list of tie points
	#if RUN_FULL_CALIBRATION
	Calibration fullCalibration(const std::vector<FullCalibTiePoint>& tiePoints); // Calculates full calibration
	#endif
	bool calcCameraTransform(void); // Calculates the extrinsic camera transformation; returns true if calibration was successful
	
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

void ExtrinsicCalibrator::movingCameraToggleCallback(GLMotif::ToggleButton::ValueChangedCallbackData* cbData)
	{
	/* Update the calibration mode: */
	movingCamera=cbData->set;
	
	/* Reset all calibration data: */
	#if RUN_FULL_CALIBRATION
	fullCalibTiePoints.clear();
	fullCalibNumSamples=0;
	#else
	tiePoints.clear();
	#endif
	haveCalibration=false;
	cameraTransform=CameraTransform::identity;
	}

#if RUN_FULL_CALIBRATION

void ExtrinsicCalibrator::ransacMaxInlierSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData)
	{
	/* Set the maximum inlier distance for RANSAC: */
	ransacMaxInlier=cbData->value;
	
	/* Re-run camera calibration if there are enough tie points: */
	if(tiePoints.size()>=5)
		{
		/* If this was the first calibration, reset the view: */
		if(calcCameraTransform()&&!haveCalibration)
			{
			resetNavigation();
			haveCalibration=true;
			}
		}
	}

#endif

void ExtrinsicCalibrator::saveAlignmentCallback(Misc::CallbackData* cbData)
	{
	#if RUN_FULL_CALIBRATION
	
	/* Run a full-circuit RANSAC optimization: */
	numAlignIterations=1000;
	numRansacIterations=10000;
	calcCameraTransform();
	numAlignIterations=0;
	numRansacIterations=1000;
	
	#endif
	
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
	
	GLMotif::Margin* movingCameraMargin=new GLMotif::Margin("MovingCameraMargin",extrinsicCalibrationDialog,false);
	movingCameraMargin->setAlignment(GLMotif::Alignment(GLMotif::Alignment::RIGHT));
	
	GLMotif::ToggleButton* movingCameraToggle=new GLMotif::ToggleButton("MovingCameraToggle",movingCameraMargin,"Moving Camera");
	movingCameraToggle->setBorderType(GLMotif::Widget::PLAIN);
	movingCameraToggle->setToggle(movingCamera);
	movingCameraToggle->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::movingCameraToggleCallback);
	
	movingCameraMargin->manageChild();
	
	#if RUN_FULL_CALIBRATION
	
	GLMotif::RowColumn* ransacMaxInlierBox=new GLMotif::RowColumn("RansacMaxInlierBox",extrinsicCalibrationDialog,false);
	ransacMaxInlierBox->setOrientation(GLMotif::RowColumn::HORIZONTAL);
	ransacMaxInlierBox->setPacking(GLMotif::RowColumn::PACK_TIGHT);
	ransacMaxInlierBox->setNumMinorWidgets(1);
	
	new GLMotif::Label("RansacMaxInlierLabel",ransacMaxInlierBox,"Max Inlier Distance");
	
	GLMotif::TextFieldSlider* ransacMaxInlierSlider=new GLMotif::TextFieldSlider("RansacMaxInlierSlider",ransacMaxInlierBox,6,ss.fontHeight*10.0f);
	ransacMaxInlierSlider->setSliderMapping(GLMotif::TextFieldSlider::EXP10);
	ransacMaxInlierSlider->setValueType(GLMotif::TextFieldSlider::FLOAT);
	ransacMaxInlierSlider->getTextField()->setFloatFormat(GLMotif::TextField::FIXED);
	ransacMaxInlierSlider->getTextField()->setPrecision(3);
	ransacMaxInlierSlider->setValueRange(0.001,0.1,0.01);
	ransacMaxInlierSlider->setValue(ransacMaxInlier);
	ransacMaxInlierSlider->getValueChangedCallbacks().add(this,&ExtrinsicCalibrator::ransacMaxInlierSliderCallback);
	
	ransacMaxInlierBox->manageChild();
	
	#endif
	
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
	
	/* Extract all controllers' current tracking states into a new triple buffer slot: */
	PO* tss=controllerStates.startNewValue();
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
	if(newControllerIndex>=0)
		lastActiveControllerIndex=newControllerIndex;
	
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
	
	/* Wake up the main thread: */
	Vrui::requestUpdate();
	}

void ExtrinsicCalibrator::objectSnapCallback(Vrui::ObjectSnapperToolFactory::SnapRequest& snapRequest)
	{
	/* Snap the requested ray or point against all currently extracted disks: */
	if(snapRequest.rayBased)
		{
		/* Intersect the snap ray with all current disks: */
		const Kinect::DiskExtractor::DiskList& disks=diskList.getLockedValue();
		for(Kinect::DiskExtractor::DiskList::const_iterator dIt=disks.begin();dIt!=disks.end();++dIt)
			{
			/* Intersect the ray with the disk's plane: */
			Vrui::Scalar divisor=dIt->normal*snapRequest.snapRay.getDirection();
			if(divisor!=Vrui::Scalar(0))
				{
				Vrui::Scalar lambda=((dIt->center-snapRequest.snapRay.getOrigin())*dIt->normal)/divisor;
				if(lambda>=Vrui::Scalar(0)&&lambda<snapRequest.snapRayMax)
					{
					/* Check if the intersection point lies within the disk: */
					Vrui::Point p=snapRequest.snapRay(lambda);
					if(Geometry::sqrDist(p,dIt->center)<=Math::sqr(dIt->radius))
						{
						/* Store the hit: */
						snapRequest.snapped=true;
						snapRequest.snapRayMax=lambda;
						snapRequest.snapResult=Vrui::ONTransform(dIt->center-Vrui::Point::origin,Vrui::Rotation::rotateFromTo(Vrui::Vector(0,1,0),dIt->normal));
						}
					}
				}
			}
		}
	else
		{
		/* Check the snap point against all current disks: */
		const Kinect::DiskExtractor::DiskList& disks=diskList.getLockedValue();
		for(Kinect::DiskExtractor::DiskList::const_iterator dIt=disks.begin();dIt!=disks.end();++dIt)
			{
			Vrui::Vector sc=snapRequest.snapPosition-dIt->center;
			Vrui::Scalar scn=sc*dIt->normal;
			Vrui::Scalar dist2=Math::sqr(scn);
			if(dist2<Math::sqr(snapRequest.snapRadius))
				{
				sc-=dIt->normal*scn;
				dist2+=Geometry::sqr(sc);
				if(dist2<Math::sqr(snapRequest.snapRadius))
					{
					/* Store the hit: */
					snapRequest.snapped=true;
					snapRequest.snapRadius=Math::sqrt(dist2);
					snapRequest.snapResult=Vrui::ONTransform(dIt->center-Vrui::Point::origin,Vrui::Rotation::rotateFromTo(Vrui::Vector(0,1,0),dIt->normal));
					}
				}
			}
		}
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

#if RUN_FULL_CALIBRATION

ExtrinsicCalibrator::Calibration ExtrinsicCalibrator::fullCalibration(const std::vector<ExtrinsicCalibrator::FullCalibTiePoint>& tiePoints)
	{
	/* Create the least-squares system: */
	Math::Matrix ata(15,15,0.0);
	Math::Matrix atb(15,1,0.0);
	for(std::vector<FullCalibTiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		/* Convert the tracker transformation to a matrix: */
		ATransform::Matrix tracker;
		tpIt->first.writeMatrix(tracker);
		
		/* Add the three equations one at a time: */
		double lhs[15];
		double rhs;
		for(int eq=0;eq<3;++eq)
			{
			/* Add coefficients depending on disk center: */
			for(int j=0;j<3;++j)
				lhs[j]=double(tracker(eq,j));
			
			/* Add coefficients depending on camera-to-tracker transformation: */
			for(int j=0;j<3;++j)
				lhs[3+eq*4+j]=-double(tpIt->second[j])*0.01; // Scale from cm to meters to condition the least-squares matrix
			lhs[3+eq*4+3]=-1.0;
			
			/* Zero out the rest of the left hand side: */
			for(int j=0;j<4;++j)
				{
				lhs[3+((eq+1)%3)*4+j]=0.0;
				lhs[3+((eq+2)%3)*4+j]=0.0;
				}
			
			/* Assign right-hand side: */
			rhs=-double(tracker(eq,3));
			
			/* Add the equation to the least-squares matrix: */
			for(int i=0;i<15;++i)
				{
				/* Add the left-hand side: */
				for(int j=0;j<15;++j)
					ata(i,j)+=lhs[i]*lhs[j];
				
				/* Add the right-hand side: */
				atb(i)+=lhs[i]*rhs;
				}
			}
		}
	
	try
		{
		/* Solve the least-squares system using Gaussian elimination: */
		Math::Matrix sol=atb;
		sol.divideFullPivot(ata);
		
		Calibration result;
		
		/* Extract the disk center in controller's local coordinates: */
		result.diskCenter=Point(sol(0),sol(1),sol(2));
		
		/* Use the calculated disk center to run an orthogonal point alignment algorithm: */
		std::vector<Point> p0s;
		std::vector<Point> p1s;
		for(std::vector<FullCalibTiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
			{
			p0s.push_back(tpIt->second);
			p1s.push_back(tpIt->first.transform(result.diskCenter));
			}
		Geometry::AlignResult<CameraTransform> ar=Geometry::alignPointsOGTransform(p0s,p1s,numAlignIterations);
		result.cameraTransform=ar.transform;
		result.rms=ar.rms;
		result.linf=ar.linf;
		
		return result;
		}
	catch(Math::Matrix::RankDeficientError err)
		{
		/* System was under-determined; return bogus result: */
		Calibration result;
		result.diskCenter=Point::origin;
		result.rms=Math::Constants<Scalar>::infinity;
		result.linf=Math::Constants<Scalar>::infinity;
		
		return result;
		}
	}

#endif

bool ExtrinsicCalibrator::calcCameraTransform(void)
	{
	#if RUN_FULL_CALIBRATION
	
	// std::cout<<"Calibration: "<<fullCalibTiePoints.size()<<" tie points";
	
	/* Run RANSAC to find the best affine camera transformation: */
	double bestRms=Math::Constants<double>::infinity;
	// double bestLinf=Math::Constants<double>::infinity;
	CameraTransform bestTransform;
	Point bestDiskCenter;
	// double bestPercent=0.0;
	double maxInlier2=Math::sqr(ransacMaxInlier);
	double minPercent=75.0;
	for(unsigned int ransac=0;ransac<numRansacIterations;++ransac)
		{
		/* Pick a random subset of five tie points: */
		std::vector<FullCalibTiePoint> rtps;
		for(int i=0;i<5;++i)
			rtps.push_back(fullCalibTiePoints[Math::randUniformCO(0,fullCalibTiePoints.size())]); // Let's ignore for now that we may get duplicates
		
		/* Calibrate based on the minimal subset: */
		Calibration cal1=fullCalibration(rtps);
		
		/* Find all inliers based on the minimal calibration: */
		std::vector<FullCalibTiePoint> tps;
		for(std::vector<FullCalibTiePoint>::iterator tpIt=fullCalibTiePoints.begin();tpIt!=fullCalibTiePoints.end();++tpIt)
			{
			/* Calculate this tie point's approximation error: */
			Point trackerPos=tpIt->first.transform(cal1.diskCenter);
			Point kinectPos=cal1.cameraTransform.transform(tpIt->second);
			double dist2=Geometry::sqrDist(trackerPos,kinectPos);
			if(dist2<=maxInlier2)
				tps.push_back(*tpIt);
			}
		
		/* Check if the inlier set is large enough: */
		if(double(tps.size())/double(fullCalibTiePoints.size())*100.0>=minPercent)
			{
			/* Run calibration based on all inliers: */
			Calibration cal2=fullCalibration(tps);
			
			/* Check if this calibration is better than the current best: */
			if(bestRms>cal2.rms)
				{
				/* Keep it: */
				bestRms=cal2.rms;
				// bestLinf=cal2.linf;
				bestTransform=cal2.cameraTransform;
				bestDiskCenter=cal2.diskCenter;
				// bestPercent=double(tps.size())/double(fullCalibTiePoints.size())*100.0;
				}
			}
		}
	
	/* Check if RANSAC came up with a calibration solution: */
	if(Math::isFinite(bestRms))
		{
		/* Store the best disk center and camera transformation: */
		diskCenter=bestDiskCenter;
		cameraTransform=bestTransform;
		
		// std::cout<<", error RMS="<<bestRms<<", Linf="<<bestLinf<<", "<<bestPercent<<"% inliers"<<std::endl;
		
		/* Update the GUI: */
		for(int i=0;i<3;++i)
			diskCenterTextFields[i]->setValue(diskCenter[i]);
		alignmentResidualTextField->setValue(bestRms);
		
		// DEBUGGING
		/* Print the camera transformation estimate: */
		// std::cout<<"Camera transform: "<<Misc::ValueCoder<CameraTransform>::encode(cameraTransform)<<std::endl;
		
		return true;
		}
	else
		return false;
	
	#else
	
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
	
	return true;
	
	#endif
	}

ExtrinsicCalibrator::ExtrinsicCalibrator(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 deviceClient(0),trackerIndex(-1),buttonIndex(-1),
	 diskCenter(Point::origin),
	 configurationDialog(0),
	 previousControllerIndex(-1),lastActiveControllerIndex(-1),
	 calibratingDiskCenter(false),
	 movingCamera(false),
	 haveCalibration(false),cameraTransform(Vrui::OGTransform::identity)
	{
	/* Parse command line: */
	const char* serverName="localhost:8555";
	Kinect::MultiplexedFrameSource* remoteSource=0;
	int cameraIndex=0;
	unsigned int projectorTriangleDepthRange=30;
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
			else if(strcasecmp(argv[i]+1,"tracker")==0)
				{
				++i;
				if(i<argc)
					trackerIndex=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"button")==0)
				{
				++i;
				if(i<argc)
					buttonIndex=atoi(argv[i]);
				}
			else if(strcasecmp(argv[i]+1,"diskCenter")==0||strcasecmp(argv[i]+1,"dc")==0)
				{
				for(int j=0;j<3;++j)
					{
					++i;
					diskCenter[j]=Vrui::Scalar(atof(argv[i]));
					}
				std::cout<<"Disk center position: "<<diskCenter[0]<<", "<<diskCenter[1]<<", "<<diskCenter[2]<<std::endl;
				}
			else if(strcasecmp(argv[i]+1,"triangleDepthRange")==0||strcasecmp(argv[i]+1,"tdr")==0)
				{
				++i;
				projectorTriangleDepthRange=atoi(argv[i]);
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
		if((device->trackType&Vrui::VRDeviceDescriptor::TRACK_POS)&&(device->trackType&Vrui::VRDeviceDescriptor::TRACK_ORIENT))
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
	diskExtractor->setMinNumPixels(250);
	diskExtractor->setDiskRadius(6.0);
	diskExtractor->setDiskRadiusMargin(1.10);
	diskExtractor->setDiskFlatness(1.0);
	
	/* Create a projector for the 3D video source: */
	projector=new Kinect::ProjectorType(*camera);
	projector->setTriangleDepthRange(projectorTriangleDepthRange);
	
	/* Initialize interaction state: */
	for(int i=0;i<3;++i)
		controllerStates.getBuffer(i)=new PO[controllers.size()];
	for(int i=0;i<3;++i)
		controllerIndex.getBuffer(i)=-1;
	
	#if RUN_FULL_CALIBRATION
	
	/* Initialize the full calibration least-squares system: */
	fullCalibNumSamples=0;
	numAlignIterations=0;
	numRansacIterations=2000;
	ransacMaxInlier=0.015;
	
	#endif
	
	/* Create the configuration dialog: */
	configurationDialog=createConfigurationDialog();
	Vrui::popupPrimaryWidget(configurationDialog);
	
	/* Register a snapping callback: */
	Vrui::ObjectSnapperTool::addSnapCallback(Misc::createFunctionCall(this,&ExtrinsicCalibrator::objectSnapCallback));
	
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
	#if RUN_FULL_CALIBRATION
	
	/* Dump full calibration tie points to a binary file: */
	{
	IO::FilePtr tiePointFile=Vrui::openFile("FullCalibTiePoints.dat",IO::File::WriteOnly);
	tiePointFile->setEndianness(Misc::LittleEndian);
	tiePointFile->write<unsigned int>(fullCalibTiePoints.size());
	for(std::vector<FullCalibTiePoint>::iterator tpIt=fullCalibTiePoints.begin();tpIt!=fullCalibTiePoints.end();++tpIt)
		{
		/* Write the tracker transformation: */
		tiePointFile->write<Scalar>(tpIt->first.getTranslation().getComponents(),3);
		tiePointFile->write<Scalar>(tpIt->first.getRotation().getQuaternion(),4);
		
		/* Write the disk position: */
		tiePointFile->write<Scalar>(tpIt->second.getComponents(),3);
		}
	}
	
	#endif
	
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
			#if RUN_FULL_CALIBRATION
			if(tiePoints.size()>=5)
				{
				/* If this was the first calibration, reset the view: */
				if(calcCameraTransform()&&!haveCalibration)
					{
					resetNavigation();
					haveCalibration=true;
					}
				}
			#else
			if(tiePoints.size()>=3)
				{
				/* If this was the first calibration, reset the view: */
				if(calcCameraTransform()&&!haveCalibration)
					{
					resetNavigation();
					haveCalibration=true;
					}
				}
			#endif
			}
		}
	
	/* Determine the index of the tracker to use: */
	int useTrackerIndex=trackerIndex>=0?trackerIndex:controllerIndex.getLockedValue();
	
	/* Check if there is new tracking data for the active controller and a button is pressed: */
	if(controllerStates.lockNewValue()&&controllerIndex.getLockedValue()>=0)
		{
		if(calibratingDiskCenter)
			{
			/* Convert the controller's new orthonormal transformation to an affine transformation: */
			ATransform ct;
			controllerStates.getLockedValue()[useTrackerIndex].writeMatrix(ct.getMatrix());
			
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
			tiePointCombiners[1].addPoint(controllerStates.getLockedValue()[useTrackerIndex].transform(diskCenter));
			}
		}
	
	/* Check if there are new extracted disks and a button is pressed: */
	if(!calibratingDiskCenter&&diskList.lockNewValue()&&diskList.getLockedValue().size()==1&&controllerIndex.getLockedValue()>=0)
		{
		const Kinect::DiskExtractor::Disk& disk=diskList.getLockedValue().front();
		
		// DEBUGGING
		// std::cout<<disk.radius<<", "<<disk.flatness<<", "<<disk.numPixels<<std::endl;
		
		/* Check if there is a real disk center position: */
		bool finite=true;
		for(int i=0;i<3;++i)
			finite=finite&&Math::isFinite(disk.center[i]);
		if(finite)
			{
			/* Add the first disk center position to the tie point combiner: */
			tiePointCombiners[0].addPoint(disk.center);
			
			#if RUN_FULL_CALIBRATION
			
			/* Add a (tracker state, disk center) tie point for full calibration: */
			PO trackerPo=controllerStates.getLockedValue()[useTrackerIndex];
			if(movingCamera)
				{
				/* Store the inverse tracker transformation instead: */
				trackerPo=Geometry::invert(trackerPo);
				}
			fullCalibTiePoints.push_back(FullCalibTiePoint(trackerPo,disk.center));
			}
		
		#endif
		}
	
	/* Update the projector: */
	projector->updateFrames();
	}

void ExtrinsicCalibrator::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glLineWidth(3.0f);
	glPointSize(3.0f);
	
	if(movingCamera)
		{
		/* Draw the current disk center in world coordinates: */
		glBegin(GL_POINTS);
		glColor3f(0.0f,1.0f,1.0f);
		glVertex(diskCenter);
		glEnd();
		glBegin(GL_LINES);
		float crossSize=diskExtractor->getDiskRadius()*cameraTransform.getScaling();
		Point dc=diskCenter;
		glVertex(dc-Vector(crossSize,0,0));
		glVertex(dc+Vector(crossSize,0,0));
		glVertex(dc-Vector(0,crossSize,0));
		glVertex(dc+Vector(0,crossSize,0));
		glVertex(dc-Vector(0,0,crossSize));
		glVertex(dc+Vector(0,0,crossSize));
		glEnd();
		}
	else
		{
		/* Draw the current disk center positions driven by all controllers: */
		glBegin(GL_POINTS);
		glColor3f(0.0f,1.0f,1.0f);
		for(unsigned int i=0;i<controllers.size();++i)
			glVertex(controllerStates.getLockedValue()[i].transform(diskCenter));
		glEnd();
		glBegin(GL_LINES);
		float crossSize=diskExtractor->getDiskRadius()*cameraTransform.getScaling();
		for(unsigned int i=0;i<controllers.size();++i)
			{
			Point cc=controllerStates.getLockedValue()[i].transform(diskCenter);
			glVertex(cc-Vector(crossSize,0,0));
			glVertex(cc+Vector(crossSize,0,0));
			glVertex(cc-Vector(0,crossSize,0));
			glVertex(cc+Vector(0,crossSize,0));
			glVertex(cc-Vector(0,0,crossSize));
			glVertex(cc+Vector(0,0,crossSize));
			}
		glEnd();
		}
	
	/* Determine the index of the tracker to use: */
	int useTrackerIndex=trackerIndex>=0?trackerIndex:lastActiveControllerIndex;
	
	/* Draw all extracted disks: */
	glPushMatrix();
	if(movingCamera&&haveCalibration)
		glMultMatrix(controllerStates.getLockedValue()[useTrackerIndex]);
	glMultMatrix(cameraTransform);
	
	glLineWidth(1.0f);
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
	
	#if RUN_FULL_CALIBRATION
	
	/* Draw all collected tie points using the current disk center estimate: */
	glBegin(GL_LINES);
	for(std::vector<FullCalibTiePoint>::const_iterator tpIt=fullCalibTiePoints.begin();tpIt!=fullCalibTiePoints.end();++tpIt)
		{
		glColor3f(0.0f,1.0f,1.0f);
		glVertex(tpIt->first.transform(diskCenter));
		glColor3f(1.0f,0.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->second));
		}
	glEnd();
	glBegin(GL_POINTS);
	for(std::vector<FullCalibTiePoint>::const_iterator tpIt=fullCalibTiePoints.begin();tpIt!=fullCalibTiePoints.end();++tpIt)
		{
		glColor3f(0.0f,1.0f,1.0f);
		glVertex(tpIt->first.transform(diskCenter));
		glColor3f(1.0f,0.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->second));
		}
	glEnd();
	
	#else
	
	/* Draw all collected tie points: */
	glBegin(GL_LINES);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		glColor3f(1.0f,0.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->first));
		glColor3f(0.0f,1.0f,1.0f);
		glVertex(tpIt->second);
		}
	glEnd();
	glBegin(GL_POINTS);
	for(std::vector<TiePoint>::const_iterator tpIt=tiePoints.begin();tpIt!=tiePoints.end();++tpIt)
		{
		glColor3f(1.0f,0.0f,0.0f);
		glVertex(cameraTransform.transform(tpIt->first));
		glColor3f(0.0f,1.0f,1.0f);
		glVertex(tpIt->second);
		}
	glEnd();
	
	#endif
	
	glPopAttrib();
	}

void ExtrinsicCalibrator::resetNavigation(void)
	{
	if(movingCamera&&haveCalibration)
		{
		/* Move the current disk center estimate to the center of the environment: */
		Vrui::setNavigationTransformation(diskCenter,Vrui::Scalar(1),Vrui::Vector(0,1,0));
		}
	else
		{
		/* Move the camera position to the viewer's head position: */
		Vrui::NavTransform nav=Vrui::NavTransform::translateFromOriginTo(Vrui::getMainViewer()->getHeadPosition());
		
		/* Rotate the frame to align the projection direction with the viewer's viewing direction: */
		nav*=Vrui::NavTransform::rotate(Vrui::Rotation::rotateFromTo(Vrui::Vector(0,0,-1),Vrui::getMainViewer()->getViewDirection()));
		
		/* Account for the projector's model space transformation: */
		nav*=Geometry::invert(cameraTransform);
		
		Vrui::setNavigationTransformation(nav);
		}
	}

VRUI_APPLICATION_RUN(ExtrinsicCalibrator)
