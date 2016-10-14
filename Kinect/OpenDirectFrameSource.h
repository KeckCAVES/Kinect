/***********************************************************************
OpenDirectFrameSource - Helper functions to open a 3D camera by index or
serial number without having to know its type.
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

#ifndef KINECT_OPENDIRECTFRAMESOURCE_INCLUDED
#define KINECT_OPENDIRECTFRAMESOURCE_INCLUDED

/* Forward declarations: */
namespace Kinect {
class DirectFrameSource;
}

namespace Kinect {

DirectFrameSource* openDirectFrameSource(unsigned int index); // Opens supported 3D camera of the given index on the local host's bus; throws exception if index larger than total number of cameras
DirectFrameSource* openDirectFrameSource(const char* serialNumber); // Opens supported 3D camera of the given serial number on the local host's bus; throws exception if camera not found

}

#endif
