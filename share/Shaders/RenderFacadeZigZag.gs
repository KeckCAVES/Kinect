/***********************************************************************
RenderFacade.vs - Vertex shader to render a 3D video facade using a zig-
zagging triangle strip.
Copyright (c) 2013 Oliver Kreylos
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
