/***********************************************************************
ColorCompressionTest - Utility to experiment with methods to compress
color frame streams.
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

#include <iostream>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Misc/Timer.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/ColorFrameWriter.h>
#include <Kinect/ColorFrameReader.h>

int main(int argc,char* argv[])
	{
	/* Open the uncompressed color stream file: */
	IO::FilePtr colorFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/ColorFrames.dat"));
	colorFrameFile->setEndianness(Misc::LittleEndian);
	
	/* Read the file header: */
	unsigned int size[2];
	colorFrameFile->read(size,2);
	
	/* Create the color frame writer and reader: */
	IO::FilePtr compressedColorFrameFile(IO::openFile("/work/okreylos/3DVideo/Kinect/CompressedColorFrames.dat",IO::File::ReadWrite));
	Kinect::ColorFrameWriter colorFrameWriter(*compressedColorFrameFile,size);
	compressedColorFrameFile->flush();
	Kinect::ColorFrameReader colorFrameReader(*compressedColorFrameFile);
	
	/* Process all frames from the two color frame files: */
	double totalTime=0.0;
	double maxTime=0.0;
	unsigned int numFrames=0;
	while(!colorFrameFile->eof())
		{
		/* Read the next uncompressed color frame: */
		Kinect::FrameBuffer frame0(size[0],size[1],size[1]*size[0]*3*sizeof(unsigned char));
		frame0.timeStamp=colorFrameFile->read<double>();
		unsigned char* frameBuffer0=static_cast<unsigned char*>(frame0.getBuffer());
		colorFrameFile->read(frameBuffer0,size[1]*size[0]*3);
	
		/* Write the compressed color frame: */
		colorFrameWriter.writeFrame(frame0);
		compressedColorFrameFile->flush();
		
		/* Read the next compressed color frame: */
		Misc::Timer uncompressTime;
		Kinect::FrameBuffer frame1=colorFrameReader.readNextFrame();
		uncompressTime.elapse();
		double time=uncompressTime.getTime();
		totalTime+=time;
		if(maxTime<time)
			maxTime=time;
		++numFrames;
		}
	std::cout<<"Total decompression time: "<<totalTime*1000.0<<" ms, "<<numFrames<<" frames"<<std::endl;
	std::cout<<"Maximum decompression time: "<<maxTime*1000.0<<" ms"<<std::endl;
	std::cout<<"Decompression frame rate: "<<double(numFrames)/totalTime<<" Hz"<<std::endl;
	
	return 0;
	}
