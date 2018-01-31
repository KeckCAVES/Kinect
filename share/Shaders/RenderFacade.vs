/***********************************************************************
RenderFacade.vs - Vertex shader to render a 3D video facade.
Copyright (c) 2013-2017 Oliver Kreylos

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

#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable

uniform usampler2DRect depthSampler; // Sampler for depth image texture
uniform sampler2DRect depthCorrectionSampler; // Sampler for per-pixel depth correction texture
uniform mat4 depthProjection; // Projection from depth image space to clip space
uniform mat4 colorProjection; // Projection from depth image space to color image space

varying uint rawDepth; // Pixel's raw depth value for validity and triangle range check

void main()
	{
	/* Get the pixel's raw depth value: */
	rawDepth=texture2DRect(depthSampler,gl_Vertex.xy).r;
	
	/* Get the pixel's depth correction coefficients: */
	vec2 depthCorrection=texture2DRect(depthCorrectionSampler,gl_Vertex.xy).rg;
	
	/* Calculate the pixel's corrected depth value: */
	float depth=float(rawDepth)*depthCorrection.x+depthCorrection.y;
	
	/* Create the pixel's position in depth image space: */
	vec4 diPixel=vec4(gl_Vertex.xy,depth,1.0);
	
	/* Project the pixel from depth image space to color image space: */
	gl_TexCoord[0]=colorProjection*diPixel;
	
	/* Project the pixel from depth image space to clip space: */
	gl_Position=depthProjection*diPixel;
	}
