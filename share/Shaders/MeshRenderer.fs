/***********************************************************************
MeshRenderer.fs - Vertex shader to render a 3D video facade using a
triangle index buffer and per-pixel depth correction texture.
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

uniform bool mapTexture; // Flag whether to map textures or show shaded surfaces
uniform sampler2D colorSampler; // Sampler for color image texture

void main()
	{
	if(mapTexture)
		{
		/* Get the fragment color from the color image texture: */
		gl_FragColor=texture2DProj(colorSampler,gl_TexCoord[0]);
		}
	else
		{
		/* Get the fragment color from the interpolated vertex color: */
		gl_FragColor=gl_Color;
		}
	}
