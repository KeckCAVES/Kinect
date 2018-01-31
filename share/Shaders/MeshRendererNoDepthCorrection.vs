/***********************************************************************
MeshRendererNoDepthCorrection.vs - Vertex shader to render a 3D video
facade using a triangle index buffer.
Copyright (c) 2016-2017 Oliver Kreylos

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
uniform mat4 depthProjection; // Projection from depth image space to eye space
uniform mat4 inverseTransposedDepthProjection; // Inverse transposed projection from depth image space into eye space for illumination
uniform mat4 colorProjection; // Projection from depth image space to color image space

void main()
	{
	/* Get the pixel's raw depth value: */
	float rawDepth=float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy).r);
	
	/* Create the pixel's position in depth image space: */
	vec4 diPixel=vec4(gl_Vertex.xy,rawDepth,1.0);
	
	/* Project the pixel from depth image space to color image space: */
	gl_TexCoord[0]=colorProjection*diPixel;
	
	/* Calculate the pixel's normal vector in depth image space: */
	vec3 diNormal=vec3(float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy+vec2(1.0,0.0)).r)-float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy-vec2(1.0,0.0)).r),
	                   float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy+vec2(0.0,1.0)).r)-float(texture2DRect(depthSampler,gl_MultiTexCoord0.xy-vec2(0.0,1.0)).r),
	                   -2.0);
	
	/* Calculate the pixel's normal plane in depth image space: */
	vec4 diPlane=vec4(diNormal,-dot(diPixel.xyz,diNormal));
	
	/* Project the pixel to eye space: */
	vec4 eyeVertex=depthProjection*diPixel;
	
	/* Compute the light direction (works both for directional and point lights): */
	vec3 lightDir=gl_LightSource[0].position.xyz*eyeVertex.w-eyeVertex.xyz*gl_LightSource[0].position.w;
	float lightDist=length(lightDir);
	lightDir=normalize(lightDir);
	
	/* Transform the normal plane to eye space: */
	vec3 eyeNormal=normalize((inverseTransposedDepthProjection*diPlane).xyz);
	
	/* Calculate global ambient light term: */
	vec4 color=gl_LightModel.ambient*gl_FrontMaterial.ambient;
	
	for(int lightIndex=0;lightIndex<8;++lightIndex)
		{
		/* Calculate per-source ambient light term: */
		vec4 sourceColor=gl_LightSource[lightIndex].ambient*gl_FrontMaterial.ambient;
		
		/* Compute the diffuse lighting angle: */
		float nl=dot(eyeNormal,lightDir);
		if(nl>0.0)
			{
			/* Calculate per-source diffuse light term: */
			sourceColor+=(gl_LightSource[lightIndex].diffuse*gl_FrontMaterial.diffuse)*nl;
			
			/* Compute the eye direction: */
			vec3 eyeDir=normalize(-eyeVertex.xyz);
			
			/* Compute the specular lighting angle: */
			float nhv=max(dot(eyeNormal,normalize(eyeDir+lightDir)),0.0);
			
			/* Calculate per-source specular lighting term: */
			sourceColor+=(gl_LightSource[lightIndex].specular*gl_FrontMaterial.specular)*pow(nhv,gl_FrontMaterial.shininess);
			}
		
		/* Attenuate the per-source light terms and accumulate the light source's contribution: */
		float att=(gl_LightSource[lightIndex].quadraticAttenuation*lightDist+gl_LightSource[lightIndex].linearAttenuation)*lightDist+gl_LightSource[lightIndex].constantAttenuation;
		color+=sourceColor*(1.0/att);
		}
	
	/* Assign final vertex color: */
	gl_FrontColor=color;
	
	/* Project the pixel from depth image space to clip space: */
	gl_Position=gl_ProjectionMatrix*eyeVertex;
	}
