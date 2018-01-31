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

#version 120
#extension GL_ARB_geometry_shader4: enable

uniform float triangleDepthRange; // Maximum depth range for generated triangles

varying in uint rawDepth[]; // Raw pixel depth values for validity and triangle range checks

#if 0

void main()
	{
	uint quadMin=min(min(rawDepth[0],rawDepth[1]),min(rawDepth[2],rawDepth[3]));
	uint quadMax=max(max(rawDepth[0],rawDepth[1]),max(rawDepth[2],rawDepth[3]));
	if(quadMax<2046U&&float(quadMax-quadMin)<=triangleDepthRange)
		{
		/* Emit the quad: */
		gl_TexCoord[0]=gl_TexCoordIn[1][0];
		gl_Position=gl_PositionIn[1];
		EmitVertex();
		
		gl_TexCoord[0]=gl_TexCoordIn[2][0];
		gl_Position=gl_PositionIn[2];
		EmitVertex();
		
		gl_TexCoord[0]=gl_TexCoordIn[0][0];
		gl_Position=gl_PositionIn[0];
		EmitVertex();
		
		gl_TexCoord[0]=gl_TexCoordIn[3][0];
		gl_Position=gl_PositionIn[3];
		EmitVertex();
		
		EndPrimitive();
		}
	}

#else

void main()
	{
	/* Determine the quad's vertex validity case: */
	int caseIndex=0x0;
	if(rawDepth[0]<2046U)
		caseIndex|=0x1;
	if(rawDepth[1]<2046U)
		caseIndex|=0x2;
	if(rawDepth[2]<2046U)
		caseIndex|=0x4;
	if(rawDepth[3]<2046U)
		caseIndex|=0x8;
	
	/* Generate output triangles based on vertex validity case: */
	if(caseIndex==0x7) // Lower-left triangle
		{
		float depthRange=float(max(rawDepth[0],max(rawDepth[1],rawDepth[2]))-min(rawDepth[0],min(rawDepth[1],rawDepth[2])));
		if(depthRange<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[1][0];
			gl_Position=gl_PositionIn[1];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			EndPrimitive();
			}
		}
	else if(caseIndex==0xb) // Upper-left triangle
		{
		float depthRange=float(max(rawDepth[0],max(rawDepth[1],rawDepth[3]))-min(rawDepth[0],min(rawDepth[1],rawDepth[3])));
		if(depthRange<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[1][0];
			gl_Position=gl_PositionIn[1];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[3][0];
			gl_Position=gl_PositionIn[3];
			EmitVertex();
			
			EndPrimitive();
			}
		}
	else if(caseIndex==0xd) // Upper-right triangle
		{
		float depthRange=float(max(rawDepth[0],max(rawDepth[2],rawDepth[3]))-min(rawDepth[0],min(rawDepth[2],rawDepth[3])));
		if(depthRange<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[3][0];
			gl_Position=gl_PositionIn[3];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			EndPrimitive();
			}
		}
	else if(caseIndex==0xe) // Lower-right triangle
		{
		float depthRange=float(max(rawDepth[1],max(rawDepth[2],rawDepth[3]))-min(rawDepth[1],min(rawDepth[2],rawDepth[3])));
		if(depthRange<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[3][0];
			gl_Position=gl_PositionIn[3];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[1][0];
			gl_Position=gl_PositionIn[1];
			EmitVertex();
			
			EndPrimitive();
			}
		}
	else if(caseIndex==0xf) // Full quad
		{
		/* Calculate the depth range of the first two candidate triangles: */
		float depthRangeLL=float(max(rawDepth[0],max(rawDepth[1],rawDepth[2]))-min(rawDepth[0],min(rawDepth[1],rawDepth[2])));
		float depthRangeUR=float(max(rawDepth[0],max(rawDepth[2],rawDepth[3]))-min(rawDepth[0],min(rawDepth[2],rawDepth[3])));
		if(depthRangeLL<=triangleDepthRange&&depthRangeUR<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[1][0];
			gl_Position=gl_PositionIn[1];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[3][0];
			gl_Position=gl_PositionIn[3];
			EmitVertex();
			
			EndPrimitive();
			}
		else if(depthRangeLL<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[1][0];
			gl_Position=gl_PositionIn[1];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			EndPrimitive();
			}
		else if(depthRangeUR<=triangleDepthRange)
			{
			gl_TexCoord[0]=gl_TexCoordIn[3][0];
			gl_Position=gl_PositionIn[3];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[0][0];
			gl_Position=gl_PositionIn[0];
			EmitVertex();
			
			gl_TexCoord[0]=gl_TexCoordIn[2][0];
			gl_Position=gl_PositionIn[2];
			EmitVertex();
			
			EndPrimitive();
			}
		}
	}

#endif
