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

#include "MD5MeshAnimator.h"

#include <Geometry/OrthogonalTransformation.h>
#include <GL/gl.h>
#include <GL/GLMaterial.h>
#include <GL/GLTransformationWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/OpenFile.h>

/********************************
Methods of class MD5MeshAnimator:
********************************/

MD5MeshAnimator::MD5MeshAnimator(void)
	:textureManager(fileManager),
	 materialManager(textureManager),
	 mesh(0),
	 numAnims(0),anims(0)
	{
	/* Find all pak???.pk4 files in the base/ directory: */
	fileManager.addPakFiles(Vrui::openDirectory("/work/okreylos/GameData/Doom3/base/"),"pak");
	
	/* Load the mesh: */
	mesh=new SceneGraph::Doom3MD5Mesh(fileManager,materialManager,"models/md5/monsters/hellknight/hellknight.md5mesh");
	bodyJointID=mesh->findJoint("Body");
	
	/* Load all requested materials: */
	materialManager.loadMaterials(fileManager);
	
	/* Load the animations: */
	numAnims=1;
	anims=new SceneGraph::Doom3MD5Anim*[numAnims];
	anims[0]=new SceneGraph::Doom3MD5Anim(fileManager,"models/md5/monsters/hellknight/idle2.md5anim");
	// anims[1]=new SceneGraph::Doom3MD5Anim(fileManager,"models/md5/monsters/hellknight/roar1.md5anim");
	
	if(numAnims>0)
		{
		animStartTime=Vrui::getApplicationTime();
		currentAnimIndex=0;
		currentFrameIndex=0;
		nextFrameTime=animStartTime+double(anims[currentAnimIndex]->getFrameTime());
		}
	}

MD5MeshAnimator::~MD5MeshAnimator(void)
	{
	/* Delete the mesh and animations: */
	delete mesh;
	if(anims!=0)
		{
		for(int i=0;i<numAnims;++i)
			delete anims[i];
		delete[] anims;
		}
	}

void MD5MeshAnimator::frame(void)
	{
	if(numAnims>0)
		{
		double animTime=Vrui::getApplicationTime()-animStartTime;
		if(animTime>=nextFrameTime)
			{
			/* Go to the next animation frame: */
			++currentFrameIndex;
			if(currentFrameIndex==anims[currentAnimIndex]->getNumFrames())
				{
				currentFrameIndex=0;
				++currentAnimIndex;
				if(currentAnimIndex==numAnims)
					currentAnimIndex=0;
				}
			nextFrameTime+=double(anims[currentAnimIndex]->getFrameTime());
			
			/* Apply the animation to the mesh: */
			anims[currentAnimIndex]->animateMesh(mesh,currentFrameIndex);
			}
		
		Vrui::requestUpdate();
		}
	
	/* Re-pose the mesh: */
	mesh->updatePose();
	}

void MD5MeshAnimator::glRenderAction(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	
	if(numAnims>0&&bodyJointID.isValid())
		{
		/* Move the body joint to the origin: */
		glPushMatrix();
		glMultMatrix(Vrui::NavTransform::translateToOriginFrom(mesh->getJointTransform(bodyJointID).getOrigin()));
		}
	
	/* Draw the mesh's surface: */
	glMaterial(GLMaterialEnums::FRONT_AND_BACK,GLMaterial(GLMaterial::Color(1.0f,1.0f,1.0f),GLMaterial::Color(0.4f,0.4f,0.4f),25.0f));
	glDisable(GL_COLOR_MATERIAL);
	mesh->drawSurface(contextData,false);
	
	if(numAnims>0&&bodyJointID.isValid())
		glPopMatrix();
	
	glPopAttrib();
	}
