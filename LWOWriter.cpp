/***********************************************************************
LWOWriter - Utility to convert a stream of depth and color frames into
a sequence of 3D triangle meshes in Lightwave Object file format.
Copyright (c) 2012-2015 Oliver Kreylos

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

#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <Misc/SizedTypes.h>
#include <Misc/FileNameExtensions.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Constants.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Images/RGBImage.h>
#include <Images/WriteImageFile.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/Projector.h>

#include "IFFChunkWriter.h"

void writeFrames(const Kinect::FrameSource::IntrinsicParameters& ip,const Kinect::FrameBuffer& color,const Kinect::MeshBuffer& mesh,const char* lwoFileName)
	{
	/* Create the texture file name: */
	std::string textureFileName(lwoFileName,Misc::getExtension(lwoFileName));
	textureFileName.append("-color.png");
	
	/* Write the color frame as a texture image: */
	{
	Images::RGBImage texImage(color.getSize(0),color.getSize(1));
	Images::RGBImage::Color* tiPtr=texImage.modifyPixels();
	const unsigned char* cfPtr=reinterpret_cast<const unsigned char*>(color.getBuffer());
	for(int y=0;y<color.getSize(1);++y)
		for(int x=0;x<color.getSize(0);++x,++tiPtr,cfPtr+=3)
			*tiPtr=Images::RGBImage::Color(cfPtr);
	
	Images::writeImageFile(texImage,textureFileName.c_str());
	}
	
	/* Open the LWO file: */
	IO::FilePtr lwoFile=IO::openFile(lwoFileName,IO::File::WriteOnly);
	lwoFile->setEndianness(Misc::BigEndian);
	
	/* Create the LWO file structure via the FORM chunk: */
	{
	IFFChunkWriter form(lwoFile,"FORM");
	form.write<char>("LWO2",4);
	
	/* Create the TAGS chunk: */
	{
	IFFChunkWriter tags(&form,"TAGS");
	tags.writeString("ColorImage");
	tags.writeChunk();
	}
	
	/* Create the LAYR chunk: */
	{
	IFFChunkWriter layr(&form,"LAYR");
	layr.write<Misc::UInt16>(0U);
	layr.write<Misc::UInt16>(0x0U);
	for(int i=0;i<3;++i)
		layr.write<Misc::Float32>(0.0f);
	layr.writeString("DepthImage");
	layr.writeChunk();
	}
	
	/* Create an index map for all vertices to omit unused vertices: */
	unsigned int* indices=new unsigned int[mesh.numVertices];
	for(unsigned int i=0;i<mesh.numVertices;++i)
		indices[i]=~0x0U;
	unsigned int numUsedVertices=0;
	
	/* Create the PNTS, BBOX and VMAP chunks in one go: */
	{
	typedef Kinect::FrameSource::IntrinsicParameters::PTransform PTransform;
	typedef PTransform::Point Point;
	typedef Geometry::Box<Point::Scalar,3> Box;
	
	IFFChunkWriter bbox(&form,"BBOX");
	IFFChunkWriter pnts(&form,"PNTS");
	IFFChunkWriter vmap(&form,"VMAP");
	
	/* Write the VMAP header: */
	vmap.write<char>("TXUV",4);
	vmap.write<Misc::UInt16>(2U);
	vmap.writeString("ColorImageUV");
	
	/* Process all triangle vertices: */
	Box pBox=Box::empty;
	const Kinect::MeshBuffer::Vertex* vertices=mesh.getVertices();
	const Kinect::MeshBuffer::Index* tiPtr=mesh.getTriangleIndices();
	for(unsigned int i=0;i<mesh.numTriangles*3;++i,++tiPtr)
		{
		/* Check if the triangle vertex doesn't already have an index: */
		if(indices[*tiPtr]==~0x0U)
			{
			/* Assign an index to the triangle vertex: */
			indices[*tiPtr]=numUsedVertices;
			
			/* Transform the mesh vertex to camera space using the depth projection matrix: */
			Point dp(vertices[*tiPtr].position.getXyzw());
			Point cp=ip.depthProjection.transform(dp);
			
			/* Transform the depth-space point to texture space using the color projection matrix: */
			Point tp=ip.colorProjection.transform(dp);
			
			/* Add the point to the bounding box: */
			pBox.addPoint(cp);
			
			/* Store the point and its texture coordinates: */
			pnts.writePoint(cp);
			vmap.writeVarIndex(numUsedVertices);
			for(int i=0;i<2;++i)
				vmap.write<Misc::Float32>(tp[i]);
			
			++numUsedVertices;
			}
		}
	
	/* Write the bounding box: */
	bbox.writeBox(pBox);
	
	/* Write the BBOX, PNTS, and VMAP chunks: */
	bbox.writeChunk();
	pnts.writeChunk();
	vmap.writeChunk();
	}
	
	/* Create the POLS chunk: */
	{
	IFFChunkWriter pols(&form,"POLS");
	pols.write<char>("FACE",4);
	const Kinect::MeshBuffer::Index* tiPtr=mesh.getTriangleIndices();
	for(unsigned int triangleIndex=0;triangleIndex<mesh.numTriangles;++triangleIndex,tiPtr+=3)
		{
		pols.write<Misc::UInt16>(3U);
		for(int i=0;i<3;++i)
			pols.writeVarIndex(indices[tiPtr[2-i]]);
		}
	pols.writeChunk();
	}
	
	/* Delete the vertex index map: */
	delete[] indices;
	
	/* Create the PTAG chunk: */
	{
	IFFChunkWriter ptag(&form,"PTAG");
	ptag.write<char>("SURF",4);
	for(unsigned int triangleIndex=0;triangleIndex<mesh.numTriangles;++triangleIndex)
		{
		ptag.writeVarIndex(triangleIndex);
		ptag.write<Misc::UInt16>(0U);
		}
	ptag.writeChunk();
	}
	
	/* Create the CLIP chunk: */
	{
	IFFChunkWriter clip(&form,"CLIP");
	clip.write<Misc::UInt32>(1U);
	
	/* Create the STIL chunk: */
	{
	IFFChunkWriter stil(&clip,"STIL",true);
	stil.writeString(textureFileName.c_str());
	stil.writeChunk();
	}
	
	clip.writeChunk();
	}
	
	/* Create the SURF chunk: */
	{
	IFFChunkWriter surf(&form,"SURF");
	surf.writeString("ColorImage");
	surf.writeString("");
	
	/* Create the SIDE subchunk: */
	{
	IFFChunkWriter side(&surf,"SIDE",true);
	side.write<Misc::UInt16>(3U);
	side.writeChunk();
	}
	
	/* Create the SMAN subchunk: */
	{
	IFFChunkWriter sman(&surf,"SMAN",true);
	sman.write<Misc::Float32>(Math::rad(90.0f));
	sman.writeChunk();
	}
	
	/* Create the COLR subchunk: */
	{
	IFFChunkWriter colr(&surf,"COLR",true);
	colr.writeColor(1.0f,1.0f,1.0f);
	colr.writeVarIndex(0U);
	colr.writeChunk();
	}
	
	/* Create the DIFF subchunk: */
	{
	IFFChunkWriter diff(&surf,"DIFF",true);
	diff.write<Misc::Float32>(1.0f);
	diff.writeVarIndex(0U);
	diff.writeChunk();
	}
	
	/* Create the LUMI subchunk: */
	{
	IFFChunkWriter lumi(&surf,"LUMI",true);
	lumi.write<Misc::Float32>(0.0f);
	lumi.writeVarIndex(0U);
	lumi.writeChunk();
	}
	
	/* Create the BLOK subchunk: */
	{
	IFFChunkWriter blok(&surf,"BLOK",true);
	
	/* Create the IMAP subchunk: */
	{
	IFFChunkWriter imap(&blok,"IMAP",true);
	imap.writeString("1");
	
	/* Create the CHAN subchunk: */
	{
	IFFChunkWriter chan(&imap,"CHAN",true);
	chan.write<char>("COLR",4);
	chan.writeChunk();
	}
	
	imap.writeChunk();
	}
	
	/* Create the PROJ subchunk: */
	{
	IFFChunkWriter proj(&blok,"PROJ",true);
	proj.write<Misc::UInt16>(5U);
	proj.writeChunk();
	}
	
	/* Create the IMAG subchunk: */
	{
	IFFChunkWriter imag(&blok,"IMAG",true);
	imag.writeVarIndex(1U);
	imag.writeChunk();
	}
	
	/* Create the VMAP subchunk: */
	{
	IFFChunkWriter vmap(&blok,"VMAP",true);
	vmap.writeString("ColorImageUV");
	vmap.writeChunk();
	}
	
	blok.writeChunk();
	}
	
	/* Write the SURF chunk: */
	surf.writeChunk();
	}
	
	/* Write the FORM chunk: */
	form.writeChunk();
	}
	}

int main(int argc,char* argv[])
	{
	/* Open the requested 3D video stream: */
	std::string colorFileName=argv[1];
	colorFileName.append(".color");
	std::string depthFileName=argv[1];
	depthFileName.append(".depth");
	Kinect::FileFrameSource frameSource(colorFileName.c_str(),depthFileName.c_str());
	
	/* Get the 3D video stream's intrinsic parameters: */
	Kinect::FrameSource::IntrinsicParameters ip=frameSource.getIntrinsicParameters();
	
	/* Create a 3D video projector: */
	Kinect::Projector projector(frameSource);
	projector.setFilterDepthFrames(false,true);
	
	/* Read pairs of frames from the source and export them to a sequence of Lightwave Object files: */
	unsigned int minIndex=argc>3?atoi(argv[3]):0;
	unsigned int maxIndex=argc>4?atoi(argv[4]):Math::Constants<unsigned int>::max;
	unsigned int frameIndex=0;
	std::cout<<"Processing frame      0"<<std::flush;
	while(true)
		{
		std::cout<<"\b\b\b\b\b\b"<<std::setw(6)<<frameIndex<<std::flush;
		/* Read the next pair of frames: */
		Kinect::FrameBuffer color=frameSource.readNextColorFrame();
		Kinect::FrameBuffer depth=frameSource.readNextDepthFrame();
		
		/* Bail out if either frame is invalid: */
		if(color.timeStamp==Math::Constants<double>::max||depth.timeStamp==Math::Constants<double>::max)
			break;
		
		if(frameIndex>=minIndex&&frameIndex<maxIndex)
			{
			/* Convert the depth frame to a mesh: */
			Kinect::MeshBuffer mesh;
			projector.processDepthFrame(depth,mesh);
			
			/* Export the frame pair to an LWO file: */
			char lwoFileName[1024];
			snprintf(lwoFileName,sizeof(lwoFileName),argv[2],frameIndex);
			writeFrames(ip,color,mesh,lwoFileName);
			}
		
		++frameIndex;
		}
	std::cout<<std::endl;
	
	return 0;
	}
