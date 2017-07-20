/***********************************************************************
MeshRenderer.vs - Vertex shader to render a 3D video facade using a
triangle index buffer and per-pixel depth correction texture.
Copyright (c) 2016 Oliver Kreylos
***********************************************************************/

#extension GL_ARB_texture_rectangle : enable
#extension GL_EXT_gpu_shader4 : enable

uniform usampler2DRect depthSampler; // Sampler for depth image texture
uniform sampler2DRect depthCorrectionSampler; // Sampler for per-pixel depth correction texture
uniform mat4 depthProjection; // Projection from depth image space to clip space
uniform mat4 colorProjection; // Projection from depth image space to color image space

void main()
	{
	/* Get the pixel's raw depth value: */
	float rawDepth=float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy).r);
	
	/* Get the pixel's depth correction coefficients: */
	vec2 depthCorrection=texture2DRect(depthCorrectionSampler,gl_MultiTexCoord0.xy).rg;
	
	/* Calculate the pixel's corrected depth value: */
	float depth=rawDepth*depthCorrection.x+depthCorrection.y;
	
	/* Create the pixel's position in depth image space: */
	vec4 diPixel=vec4(gl_Vertex.xy,depth,1.0);
	
	/* Project the pixel from depth image space to color image space: */
	gl_TexCoord[0]=colorProjection*diPixel;
	
	/* Project the pixel from depth image space to clip space: */
	gl_Position=depthProjection*diPixel;
	}
