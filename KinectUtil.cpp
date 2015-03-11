/***********************************************************************
KinectUtil - Utility program to detect, list, and configure Kinect
devices.
Copyright (c) 2011-2015 Oliver Kreylos

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

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <USB/Context.h>
#include <USB/VendorProductId.h>
#include <USB/DeviceList.h>
#include <USB/Device.h>
#include <Math/Math.h>
#include <Math/Matrix.h>
#include <Kinect/Camera.h>

USB::Context usbContext;

/**************
Helper classes:
**************/

class KinectMatcher // Class to match a Kinect camera device
	{
	/* Methods: */
	public:
	bool operator()(const libusb_device_descriptor& dd) const
		{
		/* Match Kinect-for-Xbox and Kinect-for-Windows devices: */
		return dd.idVendor==0x045eU&&(dd.idProduct==0x02aeU||dd.idProduct==0x02bfU);
		}
	};

enum KinectModel // Enumerated type for Kinect models
	{
	KINECT_FOR_XBOX_1414,KINECT_FOR_XBOX_1473,KINECT_FOR_WINDOWS_1517
	};

void list(void)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the number of Kinect camera devices: */
	size_t numKinects=deviceList.getNumDevices(KinectMatcher());
	
	/* Print information about all Kinect camera devices: */
	unsigned int* busNumbers=new unsigned int[numKinects];
	unsigned int maxBusNumber=0;
	for(size_t i=0;i<numKinects;++i)
		{
		/* Get the i-th Kinect device: */
		USB::Device kinect=deviceList.getDevice(KinectMatcher(),i);
		
		/* Determine the Kinect's model number: */
		KinectModel model;
		libusb_device_descriptor dd=kinect.getDeviceDescriptor();
		if(dd.idProduct==0x02bfU)
			{
			/* Kinect for Windows: */
			model=KINECT_FOR_WINDOWS_1517;
			}
		else if(dd.bcdDevice>0x010bU)
			{
			/* Kinect for Xbox 1473: */
			model=KINECT_FOR_XBOX_1473;
			}
		else
			{
			/* Kinect for Xbox 1414: */
			model=KINECT_FOR_XBOX_1414;
			}
		
		kinect.open();
		busNumbers[i]=kinect.getBusNumber();
		if(maxBusNumber<busNumbers[i]+1)
			maxBusNumber=busNumbers[i]+1;
		std::cout<<"Kinect "<<i<<": ";
		switch(model)
			{
			case KINECT_FOR_XBOX_1414:
				std::cout<<"Kinect for Xbox 1414   ";
				break;
			
			case KINECT_FOR_XBOX_1473:
				std::cout<<"Kinect for Xbox 1473   ";
				break;
			
			case KINECT_FOR_WINDOWS_1517:
				std::cout<<"Kinect for Windows 1517";
				break;
			}
		std::cout<<", USB address ";
		std::cout<<std::setfill('0')<<std::setw(3)<<kinect.getBusNumber()<<":"<<std::setfill('0')<<std::setw(3)<<kinect.getAddress();
		std::cout<<", device serial number ";
		if(model==KINECT_FOR_XBOX_1414)
			{
			/* Get serial number from Kinect camera device: */
			std::cout<<kinect.getSerialNumber();
			}
		else
			{
			/* Get the Kinect camera device's parent device, i.e., the Kinect's internal USB hub: */
			libusb_device* hub=deviceList.getParent(kinect.getDevice());
			
			/* Find the Kinect audio device connected to the same hub: */
			USB::VendorProductId kinectAudioId(0x045eU,model==KINECT_FOR_WINDOWS_1517?0x02beU:0x02adU);
			libusb_device* audioDev=0;
			for(size_t i=0;audioDev==0&&i<deviceList.getNumDevices();++i)
				{
				if(deviceList.getVendorProductId(i)==kinectAudioId&&deviceList.getParent(deviceList.getDevice(i))==hub)
					audioDev=deviceList.getDevice(i);
				}
			if(audioDev!=0)
				{
				/* Get serial number from corresponding Kinect audio device: */
				USB::Device audio(audioDev);
				std::cout<<audio.getSerialNumber();
				}
			else
				std::cout<<"(no serial number found)";
			}
		std::cout<<std::endl;
		}
	
	/* Check for bus conflicts: */
	for(unsigned int busNumber=0;busNumber<maxBusNumber;++busNumber)
		{
		/* Count the number of Kinect devices on this USB bus: */
		size_t numKinectsOnBus=0;
		for(size_t i=0;i<numKinects;++i)
			if(busNumbers[i]==busNumber)
				++numKinectsOnBus;
		
		/* Check for conflicts: */
		if(numKinectsOnBus>1)
			{
			size_t i=0;
			while(busNumbers[i]!=busNumber)
				++i;
			std::cout<<"Warning: USB bus "<<busNumber<<" is shared by Kinect devices "<<i;
			for(size_t j=1;j<numKinectsOnBus-1;++j)
				{
				while(busNumbers[i]!=busNumber)
					++i;
				std::cout<<", "<<i;
				}
			while(busNumbers[i]!=busNumber)
				++i;
			std::cout<<" and "<<i<<"."<<std::endl;
			std::cout<<"This will not work unless bus "<<busNumber<<" is a super-speed USB 3 bus."<<std::endl;
			}
		}
	}

void resetAll(void)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the number of Kinect camera devices: */
	size_t numKinects=deviceList.getNumDevices(KinectMatcher());
	
	/* Reset all Kinect devices: */
	for(size_t i=0;i<numKinects;++i)
		{
		std::cout<<"Resetting Kinect "<<i<<"..."<<std::flush;
		USB::Device kinect=deviceList.getDevice(KinectMatcher(),i);
		kinect.open();
		kinect.reset();
		std::cout<<" done"<<std::endl;
		}
	}

bool reset(unsigned int index)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the index-th Kinect device: */
	USB::Device kinect=deviceList.getDevice(KinectMatcher(),index);
	if(!kinect.isValid())
		return false;
		
	std::cout<<"Resetting Kinect "<<index<<"..."<<std::flush;
	kinect.open();
	kinect.reset();
	std::cout<<" done"<<std::endl;
	
	return true;
	}

void downloadCalibration(unsigned int index)
	{
	/* Open a Kinect camera: */
	Kinect::Camera camera(usbContext,index);
	std::cout<<"Downloading factory calibration data for Kinect "<<camera.getSerialNumber()<<"..."<<std::endl;
	
	/* Retrieve the factory calibration data: */
	Kinect::Camera::CalibrationParameters cal;
	camera.getCalibrationParameters(cal);
	
	/* Calculate the depth-to-distance conversion formula: */
	double numerator=10.0*(4.0*cal.dcmosEmitterDist*cal.referenceDistance)/cal.referencePixelSize;
	double denominator=4.0*cal.dcmosEmitterDist/cal.referencePixelSize+4.0*cal.constantShift+1.5;
	std::cout<<"Depth conversion formula: dist[mm] = "<<numerator<<" / ("<<denominator<<" - depth)"<<std::endl;
	
	/* Calculate the depth pixel unprojection matrix: */
	double scale=2.0*cal.referencePixelSize/cal.referenceDistance;
	Math::Matrix depthMatrix(4,4,0.0);
	depthMatrix(0,0)=scale;
	depthMatrix(0,3)=-scale*640.0/2.0;
	depthMatrix(1,1)=scale;
	depthMatrix(1,3)=-scale*480.0/2.0;
	depthMatrix(2,3)=-1.0;
	depthMatrix(3,2)=-1.0/numerator;
	depthMatrix(3,3)=denominator/numerator;
	
	{
	std::cout<<std::endl<<"Depth unprojection matrix:"<<std::endl;
	std::streamsize oldPrec=std::cout.precision(8);
	std::cout.setf(std::ios::fixed);
	for(int i=0;i<4;++i)
		{
		std::cout<<"    ";
		for(int j=0;j<4;++j)
			std::cout<<' '<<std::setw(11)<<depthMatrix(i,j);
		std::cout<<std::endl;
		}
	std::cout.unsetf(std::ios::fixed);
	std::cout.precision(oldPrec);
	}
	
	/* Calculate the depth-to-color mapping: */
	double colorShiftA=cal.dcmosRcmosDist/(cal.referencePixelSize*2.0)+0.375;
	double colorShiftB=cal.dcmosRcmosDist*cal.referenceDistance*10.0/(cal.referencePixelSize*2.0);
	std::cout<<"Depth-to-color pixel shift formula: Shift = "<<colorShiftA<<" - "<<colorShiftB<<"/dist[mm]"<<std::endl;
	
	/* Formula to go directly from depth to displacement: */
	double dispA=colorShiftA-colorShiftB*denominator/numerator;
	double dispB=colorShiftB/numerator;
	std::cout<<"Or: Shift = "<<dispA<<" + "<<dispB<<"*depth"<<std::endl;
	
	/* Tabulate the bivariate pixel mapping polynomial using forward differencing: */
	double* colorx=new double[640*480];
	double* colory=new double[640*480];
	double ax=cal.ax;
	double bx=cal.bx;
	double cx=cal.cx;
	double dx=cal.dx;
	
	double ay=cal.ay;
	double by=cal.by;
	double cy=cal.cy;
	double dy=cal.dy;
	
	double dx0=(cal.dxStart<<13)>>4;
	double dy0=(cal.dyStart<<13)>>4;
	
	double dxdx0=(cal.dxdxStart<<11)>>3;
	double dxdy0=(cal.dxdyStart<<11)>>3;
	double dydx0=(cal.dydxStart<<11)>>3;
	double dydy0=(cal.dydyStart<<11)>>3;
	
	double dxdxdx0=(cal.dxdxdxStart<<5)<<3;
	double dydxdx0=(cal.dydxdxStart<<5)<<3;
	double dxdxdy0=(cal.dxdxdyStart<<5)<<3;
	double dydxdy0=(cal.dydxdyStart<<5)<<3;
	double dydydx0=(cal.dydydxStart<<5)<<3;
	double dydydy0=(cal.dydydyStart<<5)<<3;
	
	for(int y=0;y<480;++y)
		{
		dxdxdx0+=cx;
		dxdx0  +=dydxdx0/256.0;
		dydxdx0+=dx;
		
		dx0    +=dydx0/64.0;
		dydx0  +=dydydx0/256.0;
		dydydx0+=bx;
		
		dxdxdy0+=cy;
		
		dxdy0  +=dydxdy0/256.0;
		dydxdy0+=dy;
		
		dy0    +=dydy0/64.0;
		dydy0  +=dydydy0/256.0;
		dydydy0+=by;
		
		double coldxdxdy=dxdxdy0;
		double coldxdy=dxdy0;
		double coldy=dy0;
		
		double coldxdxdx=dxdxdx0;
		double coldxdx=dxdx0;
		double coldx=dx0;
		
		for(int x=0;x<640;++x)
			{
			colorx[y*640+x]=double(x)+coldx/131072.0+1.0;
			colory[y*640+x]=double(y)+coldy/131072.0+1.0;
			
			#if 0
			if(x%40==20&&y%40==20)
				std::cout<<x<<", "<<y<<": "<<colorx[y*640+x]<<", "<<colory[y*640+x]<<std::endl;
			#endif
			
			coldx+=coldxdx/64.0;
			coldxdx+=coldxdxdx/256.0;
			coldxdxdx+=ax;
			
			coldy+=coldxdy/64.0;
			coldxdy+=coldxdxdy/256.0;
			coldxdxdy+=ay;
			}
		}
	
	/* Calculate the texture projection matrix by sampling the pixel mapping polynomial: */
	int minDepth=Math::ceil(denominator-numerator/500.0); // 500mm is minimum viewing distance
	int maxDepth=Math::ceil(denominator-numerator/3000.0); // 3000mm is maximum reasonable viewing distance
	std::cout<<"Calculating best-fit color projection matrix for depth value range "<<minDepth<<" - "<<maxDepth<<"..."<<std::endl;
	
	Math::Matrix colorSystem(12,12,0.0);
	Math::Matrix depthPoint(4,1,0.0);
	depthPoint(3)=1.0;
	for(int y=20;y<480;y+=40)
		{
		depthPoint(1)=(double(y)+0.5)/480.0;
		for(int x=20;x<640;x+=40)
			{
			depthPoint(0)=(double(x)+0.5)/640.0;
			for(int depth=minDepth;depth<=maxDepth;depth+=20)
				{
				/* Transform the depth point to color image space using the non-linear transformation: */
				// int xdisp=x+Math::floor(dispA+dispB*double(depth)+0.5);
				// if(xdisp>=0&&xdisp<640)
					{
					// double colX=double(xdisp)/640.0;
					// double colY=double(y)/480.0;
					// double colX=colorx[(479-y)*640+xdisp]/640.0;
					// double colY=(479.0-colory[(479-y)*640+xdisp])/480.0;
					double colX=(colorx[(479-y)*640+x]+dispA+dispB*double(depth))/640.0;
					double colY=(479.0-colory[(479-y)*640+x])/480.0;
					
					/* Calculate the 3D point based on the depth unprojection matrix: */
					depthPoint(2)=double(depth)/double(maxDepth);
					Math::Matrix worldPoint=depthMatrix*depthPoint;
					
					/* Make the world point affine: */
					for(int i=0;i<3;++i)
						worldPoint(i)/=worldPoint(3);
					
					/* Enter the world point / color point pair into the color projection system: */
					double eq[2][12];
					eq[0][0]=depthPoint(0);
					eq[0][1]=depthPoint(1);
					eq[0][2]=depthPoint(2);
					eq[0][3]=1.0;
					eq[0][4]=0.0;
					eq[0][5]=0.0;
					eq[0][6]=0.0;
					eq[0][7]=0.0;
					eq[0][8]=-colX*depthPoint(0);
					eq[0][9]=-colX*depthPoint(1);
					eq[0][10]=-colX*depthPoint(2);
					eq[0][11]=-colX;

					eq[1][0]=0.0;
					eq[1][1]=0.0;
					eq[1][2]=0.0;
					eq[1][3]=0.0;
					eq[1][4]=depthPoint(0);
					eq[1][5]=depthPoint(1);
					eq[1][6]=depthPoint(2);
					eq[1][7]=1.0;
					eq[1][8]=-colY*depthPoint(0);
					eq[1][9]=-colY*depthPoint(1);
					eq[1][10]=-colY*depthPoint(2);
					eq[1][11]=-colY;
					
					for(int row=0;row<2;++row)
						for(unsigned int i=0;i<12;++i)
							for(unsigned int j=0;j<12;++j)
								colorSystem(i,j)+=eq[row][i]*eq[row][j];
					}
				}
			}
		}
	
	/* Find the linear system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> qe=colorSystem.jacobiIteration();
	unsigned int minEIndex=0;
	double minE=Math::abs(qe.second(0,0));
	for(unsigned int i=1;i<12;++i)
		{
		if(minE>Math::abs(qe.second(i,0)))
			{
			minEIndex=i;
			minE=Math::abs(qe.second(i,0));
			}
		}
	std::cout<<"Smallest eigenvalue of color system = "<<minE<<std::endl;
	
	/* Create the normalized homography and extend it to a 4x4 matrix: */
	Math::Matrix colorMatrix(4,4);
	double cmScale=qe.first(11,minEIndex);
	for(int i=0;i<2;++i)
		for(int j=0;j<4;++j)
			colorMatrix(i,j)=qe.first(i*4+j,minEIndex)/cmScale;
	for(int j=0;j<4;++j)
		colorMatrix(2,j)=j==2?1.0:0.0;
	for(int j=0;j<4;++j)
		colorMatrix(3,j)=qe.first(2*4+j,minEIndex)/cmScale;
	
	/* Un-normalize the color matrix: */
	for(int i=0;i<4;++i)
		{
		colorMatrix(i,0)/=640.0;
		colorMatrix(i,1)/=480.0;
		colorMatrix(i,2)/=double(maxDepth);
		}
	
	/* Calculate the approximation error: */
	double rms=0.0;
	unsigned int numPoints=0;
	depthPoint(3)=1.0;
	for(int y=20;y<480;y+=40)
		{
		depthPoint(1)=double(y)+0.5;
		for(int x=20;x<640;x+=40)
			{
			depthPoint(0)=double(x)+0.5;
			for(int depth=minDepth;depth<=maxDepth;depth+=20)
				{
				/* Transform the depth point to color image space using the non-linear transformation: */
				// int xdisp=x+Math::floor(dispA+dispB*double(depth)+0.5);
				// if(xdisp>=0&&xdisp<640)
					{
					// double colX=double(xdisp)/640.0;
					// double colY=double(y)/480.0;
					// double colX=colorx[y*640+xdisp]/640.0;
					// double colY=(colory[y*640+xdisp]-50.0)/480.0;
					double colX=(colorx[y*640+x]+dispA+dispB*double(depth))/640.0;
					double colY=(colory[y*640+x]-0.0)/480.0;
					
					/* Transform the depth image point to color space using the color matrix: */
					depthPoint(2)=double(depth);
					Math::Matrix colorPoint=colorMatrix*depthPoint;
					
					/* Calculate the approximation error: */
					double dist=Math::sqr(colorPoint(0)/colorPoint(3)-colX)+Math::sqr(colorPoint(1)/colorPoint(3)-colY);
					rms+=dist;
					++numPoints;
					}
				}
			}
		}
	
	/* Print the RMS: */
	std::cout<<"Color matrix approximation RMS = "<<Math::sqrt(rms/double(numPoints))<<std::endl;
	
	/* Make the color matrix compatible with Kinect::Projector's way of doing things: */
	// colorMatrix*=depthMatrix;
	
	{
	std::cout<<"Optimal homography from world space to color image space:"<<std::endl;
	std::streamsize oldPrec=std::cout.precision(8);
	std::cout.setf(std::ios::fixed);
	for(int i=0;i<4;++i)
		{
		std::cout<<"    ";
		for(int j=0;j<4;++j)
			std::cout<<' '<<std::setw(11)<<colorMatrix(i,j);
		std::cout<<std::endl;
		}
	std::cout.unsetf(std::ios::fixed);
	std::cout.precision(oldPrec);
	}
	
	delete[] colorx;
	delete[] colory;
	
	/* Convert the depth matrix from mm to cm: */
	Math::Matrix scaleMatrix(4,4,1.0);
	for(int i=0;i<3;++i)
		scaleMatrix(i,i)=0.1;
	depthMatrix=scaleMatrix*depthMatrix;
	
	/* Write the depth and color matrices to an intrinsic parameter file: */
	std::string calibFileName=KINECT_CONFIG_DIR;
	calibFileName.push_back('/');
	calibFileName.append(KINECT_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX);
	calibFileName.push_back('-');
	calibFileName.append(camera.getSerialNumber());
	calibFileName.append(".dat");
	if(!Misc::doesPathExist(calibFileName.c_str()))
		{
		std::cout<<"Writing full intrinsic camera parameters to "<<calibFileName<<std::endl;
		
		IO::FilePtr calibFile=IO::openFile(calibFileName.c_str(),IO::File::WriteOnly);
		calibFile->setEndianness(Misc::LittleEndian);
		for(int i=0;i<4;++i)
			for(int j=0;j<4;++j)
				calibFile->write<double>(depthMatrix(i,j));
		for(int i=0;i<4;++i)
			for(int j=0;j<4;++j)
				calibFile->write<double>(colorMatrix(i,j));
		}
	else
		std::cout<<"Intrinsic camera parameter file "<<calibFileName<<" already exists"<<std::endl;
	}

bool setLed(unsigned int index,unsigned int ledState)
	{
	/* Get the list of all USB devices: */
	USB::DeviceList deviceList(usbContext);
	
	/* Get the index-th Kinect motor device: */
	USB::Device motor=deviceList.getDevice(0x045e,0x02b0,index);
	if(!motor.isValid())
		return false;
	
	/* Open and prepare the motor device: */
	motor.open();
	// motor.setConfiguration(1);
	// motor.claimInterface(0);
	
	/* Write an LED control message: */
	motor.writeControl(0x40,0x06,ledState,0x0000,0,0);
	
	return true;
	}

int main(int argc,char* argv[])
	{
	/* Initialize the USB context: */
	usbContext.setDebugLevel(3);
	usbContext.startEventHandling();
	
	/* Parse the command line: */
	if(argc<2)
		{
		std::cout<<"Missing command. Usage:"<<std::endl;
		std::cout<<"KinectUtil ( list | ( reset [ all | <index> ] ) | ( getCalib <index> ) | ( setLED [ <index> ] <LED state 0...7>) )"<<std::endl;
		return 1;
		}
	if(strcasecmp(argv[1],"list")==0)
		{
		/* List all Kinect devices: */
		list();
		}
	else if(strcasecmp(argv[1],"reset")==0)
		{
		if(argc>2&&strcasecmp(argv[2],"all")==0)
			{
			/* Reset all Kinect devices: */
			resetAll();
			}
		else
			{
			/* Reset the given Kinect device: */
			int index=0;
			if(argc>2)
				index=atoi(argv[2]);
			if(!reset(index))
				{
				std::cerr<<"Kinect "<<index<<" does not exist."<<std::endl;
				return 1;
				}
			}
		}
	else if(strcasecmp(argv[1],"getCalib")==0)
		{
		/* Download factory calibration data from the indicated Kinect device: */
		downloadCalibration(argc>2?atoi(argv[2]):0);
		}
	else if(strcasecmp(argv[1],"setLED")==0)
		{
		/* Set the LED state of the indicated Kinect device: */
		unsigned int index=0;
		unsigned int ledState=0;
		if(argc>3)
			{
			index=atoi(argv[2]);
			ledState=atoi(argv[3]);
			}
		else if(argc==3)
			ledState=atoi(argv[2]);
		else
			{
			std::cerr<<"No LED state provided"<<std::endl;
			return 1;
			}
		if(!setLed(index,ledState))
			{
			std::cerr<<"Kinect "<<index<<" does not exist."<<std::endl;
			return 1;
			}
		}
	
	return 0;
	}
