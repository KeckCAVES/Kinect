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

#ifndef KINECT_RENDERER_INCLUDED
#define KINECT_RENDERER_INCLUDED

#include <Kinect/Config.h>
#include <Kinect/FrameSource.h>
#include <Kinect/ProjectorType.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}
class GLContextData;
namespace Kinect {
class FrameBuffer;
#if !KINECT_CONFIG_USE_SHADERPROJECTOR
class MeshBuffer;
#endif
}

namespace Kinect {

class Renderer
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<void> StreamingCallback; // Function call type for streaming callbacks
	
	/* Elements: */
	private:
	FrameSource* source; // Pointer to the 3D video frame source
	ProjectorType* projector; // Pointer to the projector of configured type
	StreamingCallback* streamingCallback; // Function to be called when the state of the projector has changed
	bool enabled; // Flag whether the renderer is currently enabled, i.e., receiving and rendering 3D video frames
	
	/* Private methods: */
	void colorStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving color frames from the frame source
	void depthStreamingCallback(const FrameBuffer& frameBuffer); // Callback receiving depth frames from the frame source
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	void meshStreamingCallback(const MeshBuffer& meshBuffer); // Callback receiving projected meshes from the projector
	#endif
	
	/* Constructors and destructors: */
	public:
	Renderer(FrameSource* sSource); // Creates a renderer for the given 3D video source; adopts source object
	~Renderer(void); // Destroys the renderer
	
	/* Methods: */
	FrameSource& getSource(void) // Returns a reference to the renderer's frame source
		{
		return *source;
		}
	ProjectorType& getProjector(void) // Returns a reference to the renderer's projector
		{
		return *projector;
		}
	void setTimeBase(const FrameSource::Time& newTimeBase); // Sets the time base of the connected frame source
	void startStreaming(StreamingCallback* newStreamingCallback); // Starts streaming 3D video frames from the frame source into the projector for rendering; calls given callback every time the projector has new data; adopts callback object
	void frame(void); // Called once per application frame to update renderer state
	void glRenderAction(GLContextData& contextData) const; // Draws the renderer's current state into the given OpenGL context
	};

}

#endif
