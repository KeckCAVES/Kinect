/***********************************************************************
RenderFacade.fs - Fragment shader to render a 3D video facade.
Copyright (c) 2013 Oliver Kreylos
***********************************************************************/

uniform sampler2D colorSampler; // Sampler for color image texture

void main()
	{
	/* Get the fragment color from the color image texture: */
	gl_FragColor=texture2DProj(colorSampler,gl_TexCoord[0]);
	}
