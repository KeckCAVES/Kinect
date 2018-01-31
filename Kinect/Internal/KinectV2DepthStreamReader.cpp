/***********************************************************************
KinectV2DepthStreamReader - Class to extract depth images from raw gated
IR images read from a stream of USB transfer buffers.
Copyright (c) 2015-2017 Oliver Kreylos

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

#include <Kinect/Internal/KinectV2DepthStreamReader.h>

#include <string.h>
#include <libusb-1.0/libusb.h>
#include <Misc/FunctionCalls.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>
#include <Kinect/LensDistortion.h>
#include <Kinect/CameraV2.h>

// DEBUGGING
// #include <iostream>

namespace Kinect {

/******************************************
Methods of class KinectV2DepthStreamReader:
******************************************/

void* KinectV2DepthStreamReader::phaseThreadMethod(int exposure)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next wake-up call: */
		unsigned int nextFrameNumber;
		{
		Threads::MutexCond::Lock phaseThreadLock(phaseThreadConds[exposure]);
		while(phaseFrameNumbers[exposure]==frameNumber)
			phaseThreadConds[exposure].wait(phaseThreadLock);
		nextFrameNumber=frameNumber;
		phaseFrameTimeStamp=frameTimeStamp;
		}
		
		// DEBUGGING
		// std::cout<<"Phase "<<nextFrameNumber<<':'<<exposure<<": "<<std::flush;
		// Realtime::TimePointMonotonic start;
		// std::cout<<"Phase "<<exposure<<": Frame "<<nextFrameNumber<<" at time "<<phaseFrameTimeStamp<<std::endl;
		
		/* Process the image triplet: */
		const IRPixel* i0Ptr=inputBuffers[exposure*3+0];
		const IRPixel* i1Ptr=inputBuffers[exposure*3+1];
		const IRPixel* i2Ptr=inputBuffers[exposure*3+2];
		const float* ttPtr=trigonometryTables[exposure];
		float* piPtr=phaseImages[exposure];
		for(unsigned int y=0;y<424;++y)
			for(unsigned int x=0;x<512;++x,++i0Ptr,++i1Ptr,++i2Ptr,ttPtr+=6,piPtr+=2)
				{
				/* Check for pixel saturation: */
				if(*i0Ptr!=32767&&*i1Ptr!=32767&&*i2Ptr!=32767)
					{
					/* Calculate the phase vector normally: */
					float x=ttPtr[0]*float(*i0Ptr)+ttPtr[1]*float(*i1Ptr)+ttPtr[2]*float(*i2Ptr);
					float y=ttPtr[3]*float(*i0Ptr)+ttPtr[4]*float(*i1Ptr)+ttPtr[5]*float(*i2Ptr);
					
					/* Calculate the pixel's phase angle in the [0, 2pi) range: */
					if(y>=0.0f)
						{
						if(x>=0.0f)
							{
							if(y<=x)
								{
								if(x>0.0f)
									{
									/* 0-45 degrees: */
									piPtr[0]=arctanTable[int(y*8192.0f/x)];
									}
								else
									piPtr[0]=0.0f; // NaN, actually
								}
							else
								{
								/* 45-90 degrees: */
								piPtr[0]=0.5f*Math::Constants<float>::pi-arctanTable[int(x*8192.0f/y)];
								}
							}
						else // x<0.0f
							{
							if(-x<=y)
								{
								/* 90-135 degrees: */
								piPtr[0]=0.5f*Math::Constants<float>::pi+arctanTable[int(x*-8192.0f/y)];
								}
							else
								{
								/* 135-180 degrees: */
								piPtr[0]=1.0f*Math::Constants<float>::pi-arctanTable[int(y*-8192.0f/x)];
								}
							}
						}
					else // y<0.0f
						{
						if(x<=0.0f)
							{
							if(x<=y)
								{
								/* 180-225 degrees: */
								piPtr[0]=1.0f*Math::Constants<float>::pi+arctanTable[int(y*8192.0f/x)];
								}
							else
								{
								/* 225-270 degrees: */
								piPtr[0]=1.5f*Math::Constants<float>::pi-arctanTable[int(x*8192.0f/y)];
								}
							}
						else
							{
							if(x<=-y)
								{
								/* 270-315 degrees: */
								piPtr[0]=1.5f*Math::Constants<float>::pi+arctanTable[int(x*-8192.0f/y)];
								}
							else
								{
								/* 315-360 degrees: */
								piPtr[0]=2.0f*Math::Constants<float>::pi-arctanTable[int(y*-8192.0f/x)];
								}
							}
						}
					
					/* Calculate the pixel's squared magnitude: */
					piPtr[1]=Math::sqrt(x*x+y*y)*magnitudeFactors[exposure];
					}
				else
					piPtr[1]=piPtr[0]=0.0f;
				}
		
		// DEBUGGING
		// std::cout<<double(start.setAndDiff())*1000.0<<"ms"<<std::endl;
		
		/* Mark the phase image as complete: */
		phaseFrameNumbers[exposure]=nextFrameNumber;
		
		/* Wake up the depth calculation thread if all three phase images are complete: */
		if(phaseFrameNumbers[0]==nextFrameNumber&&phaseFrameNumbers[1]==nextFrameNumber&&phaseFrameNumbers[2]==nextFrameNumber)
			{
			Threads::MutexCond::Lock depthThreadLock(depthThreadCond);
			depthThreadCond.signal();
			}
		}
	
	return 0;
	}

void* KinectV2DepthStreamReader::depthThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	while(true)
		{
		/* Wait for the next wake-up call: */
		unsigned int nextFrameNumber;
		double nextFrameTimeStamp;
		{
		Threads::MutexCond::Lock depthThreadLock(depthThreadCond);
		while(depthFrameNumber==phaseFrameNumbers[0]||depthFrameNumber==phaseFrameNumbers[1]||depthFrameNumber==phaseFrameNumbers[2])
			depthThreadCond.wait(depthThreadLock);
		nextFrameNumber=phaseFrameNumbers[0];
		nextFrameTimeStamp=phaseFrameTimeStamp;
		}
		
		// DEBUGGING
		// std::cout<<nextFrameNumber<<std::endl;
		// Realtime::TimePointMonotonic start;
		// std::cout<<"Depth: Frame "<<nextFrameNumber<<" at time "<<nextFrameTimeStamp<<std::endl;
		
		const float* pi0Ptr=phaseImages[0];
		const float* pi1Ptr=phaseImages[1];
		const float* pi2Ptr=phaseImages[2];
		const float* xPtr=xTable;
		const float* zPtr=zTable;
		float* diPtr=depthImage;
		const float twoPi=2.0f*Math::Constants<float>::pi;
		for(int y=0;y<424;++y)
			for(int x=0;x<512;++x,pi0Ptr+=2,pi1Ptr+=2,pi2Ptr+=2,++xPtr,++zPtr,++diPtr)
				{
				*diPtr=0.0f;
				
				float magSum=pi0Ptr[1]+pi1Ptr[1]+pi2Ptr[1];
				float magMin=Math::min(pi0Ptr[1],Math::min(pi1Ptr[1],pi2Ptr[1]));
				if(magMin>=magThreshold1&&magSum>=magThreshold2)
					{
					/* Convert phase angles to wave distances: */
					float t0=pi0Ptr[0]*3.0f/twoPi;
					float t1=pi1Ptr[0]*15.0f/twoPi;
					float t2=pi2Ptr[0]*2.0f/twoPi;
					
					float t5=Math::floor((t1-t0)*0.333333f+0.5f)*3.0f+t0;
					float t3=t5-t2;
					float f1=t3>=0.0f?2.0f:-2.0f;
					float f2=t3>=0.0f?0.5f:-0.5f;
					t3*=f2;
					t3=(t3-int(t3))*f1; // t3 is always >=0
					
					float t6=t5;
					float t7=t1;
					// if(Math::abs(t3-1.0f)<0.5f) // Good optimization; slight changes around edges
					if(0.5f<Math::abs(t3)&&Math::abs(t3)<1.5f)
						{
						t6+=15.0f;
						t7+=15.0f;
						}
					
					float t8=(Math::floor((t6-t2)*0.5f+0.5f)*2.0f+t2)*0.5f;
					
					t6/=3.0f;
					t7/=15.0f;
					
					float t9=(t6+t7+t8);
					float t10=t9/3.0f;
					
					t6*=twoPi;
					t7*=twoPi;
					t8*=twoPi;
					
        	float t6p=t8*0.551318f-t6*0.826977f;
        	float t7p=t6*0.110264f-t7*0.551318f;
        	float t8p=t7*0.826977f-t8*0.110264f;
					
					t6=t6p;
					t7=t7p;
					t8=t8p;
					
					float norm=t6*t6+t7*t7+t8*t8;
					#if 0
					float mask=t9>=0.0f?1.0f:0.0f;
					t10*=mask;
					#else
					if(t9<0.0f)
						t10=0.0f;
					#endif
					
					#if 0 // Use when confidenceSlope>0.0f
					float irX=confidenceTable[Math::clamp(int(magMin),304,871)-304];
					#else // Use when confidenceSlope<=0.0f
					float irX=confidenceTable[Math::clamp(int(Math::max(pi0Ptr[1],Math::max(pi1Ptr[1],pi2Ptr[1]))),304,871)-304];
					#endif
					float t11=irX>=norm?t10:0.0f;
					
					#if 0
					if(norm>maxConfidence*maxConfidence)
						t10=0.0f;
					#endif
					
					float phase=t11;
					if(phase>0.0f)
						{
						phase+=phaseOffset;
						
						float depthLinear=*zPtr*phase;
						float maxDepth=phase*unambiguousDistance*2.0f;
						
						float xFactor=*xPtr*90.0f/(maxDepth*maxDepth*8192.0f);
						float denominator=(1.0f-depthLinear*xFactor);
						if(denominator>0.0f)
							*diPtr=depthLinear/denominator;
						}
					}
				}
		
		/* Run a spatial low-pass filter over the depth image: */
		for(int y=0;y<424;++y)
			{
			float* diPtr=depthImage+y*512;
			float last=diPtr[0];
			if(Math::abs(diPtr[0]-diPtr[1])<filterDistanceThreshold)
				diPtr[0]=diPtr[0]*0.667f+diPtr[1]*0.333f;
			++diPtr;
			for(int x=1;x<512-1;++x,++diPtr)
				{
				float sum=diPtr[0]+diPtr[0];
				float weight=2.0f;
				if(Math::abs(diPtr[0]-last)<filterDistanceThreshold)
					{
					sum+=last;
					weight+=1.0f;
					}
				if(Math::abs(diPtr[0]-diPtr[1])<filterDistanceThreshold)
					{
					sum+=diPtr[1];
					weight+=1.0f;
					}
				last=diPtr[0];
				diPtr[0]=sum/weight;
				}
			if(Math::abs(diPtr[0]-last)<filterDistanceThreshold)
				diPtr[0]=last*0.333f+diPtr[0]*0.667f;
			}
		for(int x=0;x<512;++x)
			{
			float* diPtr=depthImage+x;
			float last=diPtr[0];
			if(Math::abs(diPtr[0]-diPtr[512])<filterDistanceThreshold)
				diPtr[0]=diPtr[0]*0.667f+diPtr[512]*0.333f;
			diPtr+=512;
			for(int y=1;y<424-1;++y,diPtr+=512)
				{
				float sum=diPtr[0]+diPtr[0];
				float weight=2.0f;
				if(Math::abs(diPtr[0]-last)<filterDistanceThreshold)
					{
					sum+=last;
					weight+=1.0f;
					}
				if(Math::abs(diPtr[0]-diPtr[512])<filterDistanceThreshold)
					{
					sum+=diPtr[512];
					weight+=1.0f;
					}
				last=diPtr[0];
				diPtr[0]=sum/weight;
				}
			if(Math::abs(diPtr[0]-last)<filterDistanceThreshold)
				diPtr[0]=last*0.333f+diPtr[0]*0.667f;
			}
		
		/* Quantize the depth image: */
		FrameBuffer depthFrame(512,424,424*512*sizeof(FrameSource::DepthPixel));
		depthFrame.timeStamp=nextFrameTimeStamp;
		diPtr=depthImage;
		FrameSource::DepthPixel* fPtr=depthFrame.getData<FrameSource::DepthPixel>();
		for(int y=0;y<424;++y)
			for(int x=0;x<512;++x,++diPtr,++fPtr)
				{
				if(*diPtr<zMin||*diPtr>zMax)
					*fPtr=FrameSource::invalidDepth;
				else
					*fPtr=FrameSource::DepthPixel(B-A/(*diPtr));
				}
		
		// DEBUGGING
		// std::cout<<double(start.setAndDiff())*1000.0<<"ms"<<std::endl;
		
		/* Mark the depth image as complete: */
		depthFrameNumber=nextFrameNumber;
		
		/* Let the camera do its processing: */
		camera.processDepthFrameBackground(depthFrame);
		
		/* Call the callback: */
		(*imageReadyCallback)(depthFrame);
		}
	
	return 0;
	}

KinectV2DepthStreamReader::KinectV2DepthStreamReader(CameraV2& sCamera)
	:camera(sCamera),
	 transferPool(0),
	 decompressTable(0),
	 inputBufferBlock(0),
	 frameStart(true),frameNumber(0),currentImage(0),nextRow(0),frameValid(true),
	 rawImageReadyCallback(0),
	 arctanTable(0),
	 confidenceTable(0),xTable(0),zTable(0),
	 depthImage(0),depthFrameNumber(0),
	 imageReadyCallback(0)
	{
	for(int i=0;i<10;++i)
		inputBuffers[i]=0;
	for(int exposure=0;exposure<3;++exposure)
		{
		trigonometryTables[exposure]=0;
		phaseImages[exposure]=0;
		}
	
	/* Allocate and initialize the uncompression look-up table (upper half of table are negative values): */
	decompressTable=new IRPixel[2048];
	for(IRPixel i=0;i<2048;++i)
		{
		IRPixel v=i>=1024?i-1024:i;
		if(v>=256)
			v=(v<<1)-256;
		if(v>=512)
			v=(v<<1)-512;
		if(v>=1024)
			v=(v<<1)-1024;
		if(v>=2048)
			v=(v<<1)-2048;
		if(v>=4096)
			v=(v<<1)-4096;
		if(v>=8192)
			v=(v<<1)-8192;
		decompressTable[i]=i>=1024?-v:v;
		}
	decompressTable[1024]=IRPixel(32767);
	
	/* Allocate the input buffers: */
	inputBufferBlock=new IRPixel[10*424*512];
	for(unsigned int i=0;i<10;++i)
		inputBuffers[i]=inputBufferBlock+i*424*512;
	
	/* Create the trigonometry coefficient tables: */
	for(int exposure=0;exposure<3;++exposure)
		trigonometryTables[exposure]=new float[424*512*6];
	
	/* Create the arctangent table: */
	arctanTable=new float[8193];
	for(int i=0;i<8192;++i)
		arctanTable[i]=Math::atan((float(i)+0.5f)/8192.0f);
	arctanTable[8192]=arctanTable[8191];
	
	/* Initialize the magnitude multipliers: */
	magnitudeFactors[0]=1.322581f*0.6666667f;
	magnitudeFactors[1]=1.0f*0.6666667f;
	magnitudeFactors[2]=1.612903f*0.6666667f;
	
	/* Allocate the phase images: */
	for(int exposure=0;exposure<3;++exposure)
		{
		phaseImages[exposure]=new float[424*512*2];
		phaseFrameNumbers[exposure]=0;
		}
	
	/* Initialize the magnitude thresholds: */
	magThreshold1=3.0f;
	magThreshold2=10.0f;
	
	/* Initialize the dealiasing confidence check parameters: */
	confidenceSlope=-0.5330578f*0.301030f*3.321928f;
	confidenceOffset=0.7694894f*3.321928f;

	minConfidence=0.3490659f;
	maxConfidence=0.6108653f;
	
	/* Initialize the dealiasing confidence table: */
	confidenceTable=new float[871-304+1];
	for(int i=304;i<=871;++i)
		{
		float irX=float(i)+0.5f;
		irX=Math::exp(Math::log(irX)*confidenceSlope+confidenceOffset);
		irX=Math::clamp(irX,minConfidence,maxConfidence);
		irX*=irX;
		confidenceTable[i-304]=irX;
		}
	
	/* Initialize phase-to-depth calculation parameters: */
	phaseOffset=0.0f;
	unambiguousDistance=6250.0f/3.0f; // Magic number
	
	/* Allocate the x and z tables: */
	xTable=new float[424*512];
	zTable=new float[424*512];
	
	/* Allocate the linear depth image: */
	depthImage=new float[424*512];
	
	/* Set the filter threshold: */
	filterDistanceThreshold=50.0f;
	
	/* Initialize the quantization parameters: */
	dMax=2047U;
	setZRange(500.0f,5000.0f);
	}

KinectV2DepthStreamReader::~KinectV2DepthStreamReader(void)
	{
	/* Stop streaming if necessary: */
	if(transferPool!=0)
		stopStreaming();
	
	/* Delete the input buffer and uncompression look-up table: */
	delete[] inputBufferBlock;
	delete[] decompressTable;
	
	/* Delete the raw image callback: */
	delete rawImageReadyCallback;
	
	/* Delete the trigonometry coefficient tables: */
	for(int exposure=0;exposure<3;++exposure)
		delete[] trigonometryTables[exposure];
	
	/* Delete the phase images: */
	for(int exposure=0;exposure<3;++exposure)
		delete[] phaseImages[exposure];
	
	/* Delete the depth calculation tables: */
	delete[] confidenceTable;
	delete[] xTable;
	delete[] zTable;
	
	/* Delete the depth image: */
	delete[] depthImage;
	}

void KinectV2DepthStreamReader::loadP0Tables(IO::FilePtr file)
	{
	file->setEndianness(Misc::LittleEndian);
	
	/* Skip the file header: */
	file->skip<Misc::UInt32>(8);
	
	/* Allocate a temporary table: */
	Misc::UInt16* p0Table=new Misc::UInt16[424*512];
	
	/* Load the three tables: */
	for(int exposure=0;exposure<3;++exposure)
		{
		/* Load the raw p0 table: */
		file->skip<Misc::UInt16>(1);
		file->read(p0Table,424*512);
		file->skip<Misc::UInt16>(1);
		
		/* Calculate the trigonometry table: */
		const Misc::UInt16* ptPtr=p0Table;
		float* ttPtr=trigonometryTables[exposure];
		for(unsigned int y=0;y<424;++y)
			for(unsigned int x=0;x<512;++x,++ptPtr,ttPtr+=6)
				{
				/* Convert the per-pixel phase offset to radians: */
				float p=-2.0f*Math::Constants<float>::pi*float(*ptPtr)/65536.0f;
				
				/* Calculate the per-image phase angles: */
				float p0=p; // First image in a triplet is at base angle
				float p1=p+2.0f*Math::Constants<float>::pi/3.0f; // Second image is 120 degrees ahead
				float p2=p+4.0f*Math::Constants<float>::pi/3.0f; // Third image is 240 degrees ahead
				
				/* Calculate the per-image phase angle cosines and sines: */
				ttPtr[0]=Math::cos(p0);
				ttPtr[1]=Math::cos(p1);
				ttPtr[2]=Math::cos(p2);
				
				ttPtr[3]=-Math::sin(p0);
				ttPtr[4]=-Math::sin(p1);
				ttPtr[5]=-Math::sin(p2);
				}
		}
	
	/* Clean up: */
	delete[] p0Table;
	}

void KinectV2DepthStreamReader::calcXZTables(const KinectV2CommandDispatcher::DepthCameraParams& depthCameraParams)
	{
	/* Get depth camera parameters: */
	double fx=depthCameraParams.sx;
	double cx=depthCameraParams.cx;
	double fy=depthCameraParams.sy;
	double cy=depthCameraParams.cy;
	LensDistortion ld;
	ld.setKappa(0,depthCameraParams.k1);
	ld.setKappa(1,depthCameraParams.k2);
	ld.setKappa(2,depthCameraParams.k3);
	ld.setRho(0,depthCameraParams.p1);
	ld.setRho(1,depthCameraParams.p2);
	
	/* Calculate X and Z tables: */
	float* xTablePtr=xTable;
	float* zTablePtr=zTable;
	for(int y=0;y<424;++y)
		{
		/* Calculate the distorted pixel position in normalized projection space: */
		LensDistortion::Point dp;
		dp[1]=(double(y)+0.5-cy)/fy;
		for(int x=0;x<512;++x,++xTablePtr,++zTablePtr)
			{
			dp[0]=(double(x)+0.5-cx)/fx;
			
			/* Undistort the pixel position: */
			LensDistortion::Point up=ld.undistort(dp);
			
			/* Calculate the X and Z table entries: */
			*xTablePtr=8192.0f*float(up[0]); // Correction factor based on x position to account for distance from lens to IR emitter
			*zTablePtr=float(unambiguousDistance/Math::sqrt(1.0+up.sqr()));
			}
		}
	}

void KinectV2DepthStreamReader::setDMax(unsigned int newDMax)
	{
	/* Set the new maximum depth value: */
	dMax=newDMax;
	
	/* Update the z value range with the current values to recalculate the conversion parameters: */
	setZRange(zMin,zMax);
	}

void KinectV2DepthStreamReader::setZRange(float newZMin,float newZMax)
	{
	/* Set the new z value range: */
	zMin=newZMin;
	zMax=newZMax;
	
	/* Calculate the quantization formula coefficients: */
	A=(float(dMax)*zMax*zMin)/(zMax-zMin);
	B=float(dMax)+(float(dMax)*zMin)/(zMax-zMin);
	}

void KinectV2DepthStreamReader::postTransfer(USB::TransferPool::Transfer* newTransfer,USB::TransferPool* newTransferPool)
	{
	#if 0
	/* Bail out if the new transfer pool doesn't match the established transfer pool: */
	if(newTransferPool!=transferPool)
		return;
	#endif
	
	/* Process all isochronous packets contained in this transfer: */
	const Misc::UInt8* packetDataPtr=newTransfer->getTransfer().buffer;
	const libusb_iso_packet_descriptor* packetDescriptorPtr=newTransfer->getTransfer().iso_packet_desc;
	int numIsoPackets=newTransfer->getTransfer().num_iso_packets;
	for(int i=0;i<numIsoPackets;++i,++packetDescriptorPtr)
		{
		// DEBUGGING
		// std::cout<<packetDescriptorPtr->actual_length<<std::endl;
		
		/* Check if the packet is valid: */
		if(packetDescriptorPtr->status==LIBUSB_TRANSFER_COMPLETED
		   &&(packetDescriptorPtr->actual_length==33792
		      ||packetDescriptorPtr->actual_length==28312))
			{
			/* Check if this is the first packet in a new frame: */
			if(frameStart)
				{
				/* Sample the real-time clock: */
				FrameSource::Time now;
				
				/*************************************************************
				This is where we would synchronize clocks to account for
				random OS delays, subtract expected hardware latency, etc. pp.
				*************************************************************/
				
				/* Calculate the time stamp for the new frame: */
				nextFrameTimeStamp=double(now-camera.timeBase);
				
				/* Subtract approximate depth image capture latency: */
				nextFrameTimeStamp-=0.030;
				
				frameStart=false;
				}
			
			/* Calculate the number of complete pixel rows contained in this packet: */
			unsigned int numPixelRows=packetDescriptorPtr->actual_length==33792?48:40;
			
			/* Check for buffer overrun (potential result of dropped packets): */
			if(nextRow+numPixelRows<=424)
				{
				/* Process all complete rows of pixels: */
				const Misc::UInt8* pdPtr=packetDataPtr;
				for(unsigned int row=0;row<numPixelRows;++row)
					{
					/* Find the starting pixel of the next row (vertical half-images are mirrored): */
					IRPixel* rowPtr=inputBuffers[currentImage]+(nextRow<212?423-nextRow:nextRow-212)*512;
					
					/* Process the pixel row in four stripes to de-interlace the pixel columns: */
					for(unsigned int stripe=0;stripe<4;++stripe)
						{
						IRPixel* pixelPtr=rowPtr+stripe;
						
						/* Process all chunks of eight 11-bit pixels in this stripe: */
						for(unsigned int x=0;x<128;x+=8,pdPtr+=11,pixelPtr+=32)
							{
							/* Unpack eight signed 11-bit pixels from 11 bytes and uncompress into eight signed 16-bit pixels: */
							pixelPtr[0]=decompressTable[IRPixel(pdPtr[0])|(IRPixel(pdPtr[1]&0x07U)<<8)];
							pixelPtr[4]=decompressTable[(IRPixel(pdPtr[1]&0xf8U)>>3)|(IRPixel(pdPtr[2]&0x3fU)<<5)];
							pixelPtr[8]=decompressTable[(IRPixel(pdPtr[2]&0xc0U)>>6)|(IRPixel(pdPtr[3])<<2)|(IRPixel(pdPtr[4]&0x01U)<<10)];
							pixelPtr[12]=decompressTable[(IRPixel(pdPtr[4]&0xfeU)>>1)|(IRPixel(pdPtr[5]&0x0fU)<<7)];
							pixelPtr[16]=decompressTable[(IRPixel(pdPtr[5]&0xf0U)>>4)|(IRPixel(pdPtr[6]&0x7fU)<<4)];
							pixelPtr[20]=decompressTable[(IRPixel(pdPtr[6]&0x80U)>>7)|(IRPixel(pdPtr[7])<<1)|(IRPixel(pdPtr[8]&0x03U)<<9)];
							pixelPtr[24]=decompressTable[(IRPixel(pdPtr[8]&0xfcU)>>2)|(IRPixel(pdPtr[9]&0x1fU)<<6)];
							pixelPtr[28]=decompressTable[(IRPixel(pdPtr[9]&0xe0U)>>5)|(IRPixel(pdPtr[10]&0xffU)<<3)];
							}
						}
					
					/* Move to the next row: */
					++nextRow;
					}
				}
			else
				frameValid=false;
			
			/* Check if this was an end-of-frame packet: */
			if(packetDescriptorPtr->actual_length==28312)
				{
				struct FrameFooter // Structure to represent the footer of a raw gated IR image
					{
					/* Elements: */
					public:
					Misc::UInt32 reserved1[2]; // Two reserved values, {0, 9}
					Misc::UInt32 timeStamp; // Some time stamp?
					Misc::UInt32 frameNumber; // Number of the frame containing this image
					Misc::UInt32 imageIndex; // Index of this image in the frame containing it
					Misc::UInt32 frameSize; // Size of frame in bytes, always 0x48e00
					Misc::UInt32 flags; // Flag array? Bits 16 and 18 determine gated vs background exposure; rest is 0x3331
					Misc::UInt32 timeStamp2[2]; // Another time stamp, as a double? Changes between image indices 8 and 9, oddly
					Misc::UInt32 frameNumber2; // Another frame number? Changes between image indices 8 and 9, increments by 38 or 39
					Misc::UInt32 reserved2[2]; // Two reserved values, {0x80008000, 0} normally, but sometimes {0xc3ff8000, 0}
					Misc::UInt32 unknown; // Some value fluctuating between 491 and 528
					Misc::UInt32 flags2; // Flag array? Bit 1 indicates background exposure
					Misc::UInt32 reserved3; // Reserved value, 0x4bba0014
					Misc::UInt32 unknown2[2]; // Some floating-point value
					Misc::UInt32 reserved4[21]; // Twenty-one reserved values, {0xd3, 0x1ff, 0, 0, 0, 0, 0, 0, 0, 0, 0x9347633e, 0x3c2b5645,
					                            // 0xd456fcb6, 0x19104628, 0xb257cae3, 0x5d35f2df, 0x1dee74e0, 0x76b519c, 0x921a2543,
					                            // 0x51a08a4e, 0xe2d2369d}
					} footer;
				
				/* Check the image footer for correctness: */
				memcpy(&footer,packetDataPtr+(packetDescriptorPtr->actual_length-sizeof(footer)),sizeof(footer));
				if(footer.reserved1[0]==0&&footer.reserved1[1]==9&&footer.reserved2[1]==0&&footer.reserved3==0x4bba0014U)
					{
					// DEBUGGING
					// std::cout<<"Read image "<<footer.imageIndex<<" of frame "<<footer.frameNumber<<std::endl;
					
					/* Assign the frame number: */
					frameNumber=footer.frameNumber;
					
					// DEBUGGING
					// std::cout<<"Transfer: Frame "<<frameNumber<<" with time stamp "<<nextFrameTimeStamp<<std::endl;
					
					/* Check if the image we just extracted is in the right input slot: */
					if(currentImage!=footer.imageIndex)
						{
						currentImage=footer.imageIndex;
						frameValid=false;
						}
					}
				else
					{
					// DEBUGGING
					// std::cout<<"Invalid frame "<<footer.reserved1[0]<<", "<<footer.reserved1[1]<<", "<<footer.reserved2[0]<<", "<<footer.reserved2[1];
					// std::cout<<", "<<footer.frameNumber<<", "<<footer.imageIndex<<", "<<footer.frameSize;
					// std::cout<<", "<<footer.reserved3<<std::endl;
					
					frameValid=false;
					}
				
				if(frameValid)
					{
					/* Call the raw IR image callback: */
					if(rawImageReadyCallback!=0)
						{
						RawImage ri;
						ri.image=inputBuffers[currentImage];
						ri.imageIndex=currentImage;
						(*rawImageReadyCallback)(ri);
						}
					
					/* Finish the current image: */
					++currentImage;
					if(currentImage==10)
						{
						// DEBUGGING
						// std::cout<<"Finished depth frame "<<frameNumber<<std::endl;
						
						/* Wrap around after reading all 10 raw IR images comprising a frame: */
						currentImage=0;
						frameStart=true;
						}
					else if(currentImage%3==0)
						{
						/* Wake up the respective image processing thread after a complete image triplet is read: */
						Threads::MutexCond::Lock phaseThreadLock(phaseThreadConds[(currentImage/3)-1]);
						frameTimeStamp=nextFrameTimeStamp;
						phaseThreadConds[(currentImage/3)-1].signal();
						}
					}
				else
					{
					/* Try to recover from processing failure: */
					if(currentImage==9)
						{
						/* Start over from the beginning: */
						currentImage=0;
						frameStart=true;
						frameValid=true;
						}
					}
				
				nextRow=0;
				}
			}
		else if(packetDescriptorPtr->status!=LIBUSB_TRANSFER_COMPLETED||packetDescriptorPtr->actual_length!=0)
			frameValid=false;
		
		/* Move to the next packet in the transfer: */
		packetDataPtr+=packetDescriptorPtr->length;
		}
	
	transferPool->release(newTransfer);
	}

void KinectV2DepthStreamReader::setRawImageReadyCallback(KinectV2DepthStreamReader::RawImageReadyCallback* newRawImageReadyCallback)
	{
	/* Install the new callback: */
	RawImageReadyCallback* previousCallback=rawImageReadyCallback;
	rawImageReadyCallback=newRawImageReadyCallback;
	
	/* Delete the previous callback: */
	delete previousCallback;
	}

USB::TransferPool::UserTransferCallback*  KinectV2DepthStreamReader::startStreaming(USB::TransferPool* newTransferPool,KinectV2DepthStreamReader::ImageReadyCallback* newImageReadyCallback)
	{
	/* Remember the source transfer pool: */
	transferPool=newTransferPool;
	
	/* Install the callback function: */
	delete imageReadyCallback;
	imageReadyCallback=newImageReadyCallback;
	
	/* Start the phase calculation threads: */
	for(int exposure=0;exposure<3;++exposure)
		phaseThreads[exposure].start(this,&KinectV2DepthStreamReader::phaseThreadMethod,exposure);
	
	/* Start the depth calculation thread: */
	depthThread.start(this,&KinectV2DepthStreamReader::depthThreadMethod);
	
	/* Create and return a transfer callback: */
	return Misc::createFunctionCall(this,&KinectV2DepthStreamReader::postTransfer,newTransferPool);
	}

void KinectV2DepthStreamReader::stopStreaming(void)
	{
	/* Shut down the decoding threads: */
	for(int exposure=0;exposure<3;++exposure)
		phaseThreads[exposure].cancel();
	depthThread.cancel();
	for(int exposure=0;exposure<3;++exposure)
		phaseThreads[exposure].join();
	depthThread.join();
	
	/* Forget the assigned transfer pool: */
	transferPool=0;
	
	/* Delete the callback function: */
	delete imageReadyCallback;
	imageReadyCallback=0;
	}

}
