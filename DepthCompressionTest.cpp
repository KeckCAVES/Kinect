/***********************************************************************
DepthCompressionTest - Utility to check the results of compressing a
depth frame file.
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

#include <iostream>
#include <Misc/Timer.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/DepthFrameWriter.h>
#include <Kinect/DepthFrameReader.h>

int main(int argc,char* argv[])
	{
	/* Open the uncompressed depth stream file: */
	IO::FilePtr depthFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/DepthFrames.dat"));
	depthFrameFile->setEndianness(Misc::LittleEndian);
	
	/* Read the file header: */
	unsigned int size[2];
	depthFrameFile->read(size,2);
	
	/* Create a background frame: */
	unsigned short* backgroundFrame=new unsigned short[size[1]*size[0]];
	unsigned short* bfPtr=backgroundFrame;
	for(unsigned int y=0;y<size[1];++y)
		for(unsigned int x=0;x<size[0];++x,++bfPtr)
			*bfPtr=0x07ffU;
	unsigned int numCaptureFrames=150;
	
	/* Create the depth frame writer and reader: */
	IO::FilePtr compressedDepthFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/CompressedDepthFrames.dat",IO::File::ReadWrite));
	Kinect::DepthFrameWriter depthFrameWriter(*compressedDepthFrameFile,size);
	compressedDepthFrameFile->flush();
	Kinect::DepthFrameReader depthFrameReader(*compressedDepthFrameFile);
	
	/* Process all frames from the two depth frame files: */
	double totalTime=0.0;
	double maxTime=0.0;
	unsigned int numFrames=0;
	while(!depthFrameFile->eof())
		{
		/* Read the next uncompressed depth frame: */
		Kinect::FrameBuffer frame0(size[0],size[1],size[1]*size[0]*sizeof(unsigned short));
		frame0.timeStamp=depthFrameFile->read<double>();
		unsigned short* frameBuffer0=frame0.getData<unsigned short>();
		depthFrameFile->read(frameBuffer0,size[1]*size[0]);
		
		if(numCaptureFrames>0)
			{
			/* Add the depth frame to the background frame: */
			unsigned short* bfPtr=backgroundFrame;
			const unsigned short* dfPtr=frameBuffer0;
			for(unsigned int y=0;y<size[1];++y)
				for(unsigned int x=0;x<size[0];++x,++bfPtr,++dfPtr)
					if(*bfPtr>*dfPtr-2)
						*bfPtr=*dfPtr-2;
			--numCaptureFrames;
			}
		else
			{
			/* Remove background from the depth frame: */
			const unsigned short* bfPtr=backgroundFrame;
			unsigned short* dfPtr=frameBuffer0;
			for(unsigned int y=0;y<size[1];++y)
				for(unsigned int x=0;x<size[0];++x,++bfPtr,++dfPtr)
					if(*dfPtr>=*bfPtr)
						*dfPtr=0x07ffU;
			}
		
		/* Write the compressed depth frame: */
		depthFrameWriter.writeFrame(frame0);
		compressedDepthFrameFile->flush();
		
		/* Read the next compressed depth frame: */
		Misc::Timer uncompressTime;
		Kinect::FrameBuffer frame1=depthFrameReader.readNextFrame();
		uncompressTime.elapse();
		double time=uncompressTime.getTime();
		totalTime+=time;
		if(maxTime<time)
			maxTime=time;
		++numFrames;
		
		/* Compare the two frames: */
		unsigned int numPixels=size[0]*size[1];
		const unsigned short* f0Ptr=frameBuffer0;
		const unsigned short* f1Ptr=frame1.getData<unsigned short>();
		while(numPixels>0)
			{
			if(*f0Ptr!=*f1Ptr)
				std::cerr<<"Difference in frame "<<numFrames-1<<", "<<numPixels<<" pixels left"<<std::endl;
			++f0Ptr;
			++f1Ptr;
			--numPixels;
			}
		}
	std::cout<<"Total decompression time: "<<totalTime*1000.0<<" ms, "<<numFrames<<" frames"<<std::endl;
	std::cout<<"Maximum decompression time: "<<maxTime*1000.0<<" ms"<<std::endl;
	std::cout<<"Decompression frame rate: "<<double(numFrames)/totalTime<<" Hz"<<std::endl;
	
	return 0;
	}
