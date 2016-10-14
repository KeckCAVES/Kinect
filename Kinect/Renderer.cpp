/***********************************************************************
Renderer - Helper class to receive a 3D video stream from a frame
source, and render it into an OpenGL context using a projector.
Copyright (c) 2012-2016 Oliver Kreylos

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
**********************************************************************/

#include <Kinect/Renderer.h>

#include <Misc/FunctionCalls.h>
#include <Kinect/FunctionCalls.h>
#include <Kinect/Camera.h>
#include <Kinect/ProjectorHeader.h>

namespace Kinect {

/*************************
Methods of class Renderer:
*************************/

void Renderer::colorStreamingCallback(const FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Forward color frame to the projector: */
		projector->setColorFrame(frameBuffer);
		
		/* Notify interested parties: */
		if(streamingCallback!=0)
			(*streamingCallback)();
		}
	}

void Renderer::depthStreamingCallback(const FrameBuffer& frameBuffer)
	{
	if(enabled)
		{
		/* Forward depth frame to the projector: */
		projector->setDepthFrame(frameBuffer);
		
		#if KINECT_CONFIG_USE_SHADERPROJECTOR
		
		/* Notify interested parties: */
		if(streamingCallback!=0)
			(*streamingCallback)();
		
		#endif
		}
	}

#if !KINECT_CONFIG_USE_SHADERPROJECTOR

void Renderer::meshStreamingCallback(const MeshBuffer& meshBuffer)
	{
	if(enabled)
		{
		/* Notify interested parties: */
		if(streamingCallback!=0)
			(*streamingCallback)();
		}
	}

#endif

Renderer::Renderer(FrameSource* sSource)
	:source(sSource),
	 projector(new ProjectorType(*source)),
	 streamingCallback(0),
	 enabled(true)
	{
	}

Renderer::~Renderer(void)
	{
	/* Stop streaming: */
	source->stopStreaming();
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	projector->stopStreaming();
	#endif
	
	/* Destroy the projector and frame source: */
	delete projector;
	delete source;
	
	delete streamingCallback;
	}

void Renderer::setTimeBase(const FrameSource::Time& newTimeBase)
	{
	/* Pass call through to the frame source: */
	source->setTimeBase(newTimeBase);
	}

void Renderer::startStreaming(StreamingCallback* newStreamingCallback)
	{
	/* Delete the old streaming callback and install the new one: */
	delete streamingCallback;
	streamingCallback=newStreamingCallback;
	
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	
	/* Hook this renderer into the projector's mesh callback: */
	projector->startStreaming(Misc::createFunctionCall(this,&Renderer::meshStreamingCallback));
	
	#endif
	
	/* Hook this renderer into the frame source and start streaming: */
	source->startStreaming(Misc::createFunctionCall(this,&Renderer::colorStreamingCallback),Misc::createFunctionCall(this,&Renderer::depthStreamingCallback));
	}

void Renderer::frame(void)
	{
	if(enabled)
		{
		/* Update the projector: */
		projector->updateFrames();
		}
	}

void Renderer::glRenderAction(GLContextData& contextData) const
	{
	if(enabled)
		{
		/* Draw the current 3D video frame: */
		projector->glRenderAction(contextData);
		}
	}

}
