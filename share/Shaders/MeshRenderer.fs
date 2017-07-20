/***********************************************************************
MeshRenderer.fs - Vertex shader to render a 3D video facade using a
triangle index buffer and per-pixel depth correction texture.
Copyright (c) 2016 Oliver Kreylos
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
