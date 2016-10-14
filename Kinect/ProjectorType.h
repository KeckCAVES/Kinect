/***********************************************************************
ProjectorType - Type declarations for the build-time selected type of 3D
mesh projector.
Copyright (c) 2016 Oliver Kreylos

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

#ifndef KINECT_PROJECTORTYPE_INCLUDED
#define KINECT_PROJECTORTYPE_INCLUDED

#include <Kinect/Config.h>

#if KINECT_CONFIG_USE_SHADERPROJECTOR

namespace Kinect {
class ShaderProjector;
typedef ShaderProjector ProjectorType;
}

#elif KINECT_CONFIG_USE_PROJECTOR2

namespace Kinect {
class Projector2;
typedef Projector2 ProjectorType;
}

#else

namespace Kinect {
class Projector;
typedef Projector ProjectorType;
}

#endif

#endif
