/***********************************************************************
MD5MeshAnimator - Quick 'n' dirty class to render an animated MD5 model.
Copyright (c) 2010 Oliver Kreylos

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

#ifndef MD5MESHANIMATOR_INCLUDED
#define MD5MESHANIMATOR_INCLUDED

#include <SceneGraph/Internal/Doom3FileManager.h>
#include <SceneGraph/Internal/Doom3TextureManager.h>
#include <SceneGraph/Internal/Doom3MaterialManager.h>
#include <SceneGraph/Internal/Doom3MD5Mesh.h>
#include <SceneGraph/Internal/Doom3MD5Anim.h>

class MD5MeshAnimator
	{
	/* Elements: */
	private:
	SceneGraph::Doom3FileManager fileManager; // The file manager
	SceneGraph::Doom3TextureManager textureManager; // The texture manager
	SceneGraph::Doom3MaterialManager materialManager; // The material manager
	SceneGraph::Doom3MD5Mesh* mesh; // Mesh being viewed
	SceneGraph::Doom3MD5Mesh::JointID bodyJointID; // ID of mesh's body joint (if it has one)
	int numAnims; // Number of animation sequences to play
	SceneGraph::Doom3MD5Anim** anims; // Array of animation sequences to apply to the mesh
	
	/* Animation state: */
	double animStartTime; // Start time for animation
	int currentAnimIndex;
	int currentFrameIndex;
	double nextFrameTime;
	
	/* Constructors and destructors: */
	public:
	MD5MeshAnimator(void);
	~MD5MeshAnimator(void);
	
	/* Methods: */
	void frame(void);
	void glRenderAction(GLContextData& contextData) const;
	};

#endif
