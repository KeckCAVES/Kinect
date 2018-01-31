/***********************************************************************
RenderFacade.vs - Vertex shader to render a 3D video facade using a zig-
zagging triangle strip.
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

#version 120
#extension GL_ARB_geometry_shader4: enable

uniform float triangleDepthRange; // Maximum depth range for generated triangles

varying in uint rawDepth[]; // Raw pixel depth values for validity and triangle range checks

void main()
	{
	/* Calculate the triangle's depth range: */
	uint depthMin=min(rawDepth[0],min(rawDepth[1],rawDepth[2]));
	uint depthMax=max(rawDepth[0],max(rawDepth[1],rawDepth[2]));
	
	/* Check if the triangle has all-valid vertices and depth range lower than threshold: */
	if(depthMax<2046U&&float(depthMax-depthMin)<=triangleDepthRange)
		{
		/* Emit the triangle: */
		gl_TexCoord[0]=gl_TexCoordIn[0][0];
		gl_Position=gl_PositionIn[0];
		EmitVertex();
		
		gl_TexCoord[0]=gl_TexCoordIn[1][0];
		gl_Position=gl_PositionIn[1];
		EmitVertex();
		
		gl_TexCoord[0]=gl_TexCoordIn[2][0];
		gl_Position=gl_PositionIn[2];
		EmitVertex();
		
		EndPrimitive();
		}
	}
