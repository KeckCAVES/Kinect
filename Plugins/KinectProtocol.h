/***********************************************************************
KinectProtocol - Common interface between a Kinect server and a Kinect
client in the Vrui collaboration infrastructure.
Copyright (c) 2010-2011 Oliver Kreylos

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

#ifndef KINECTPROTOCOL_INCLUDED
#define KINECTPROTOCOL_INCLUDED

#include <Collaboration/Protocol.h>

struct KinectProtocol:public Collaboration::Protocol
	{
	/* Elements: */
	public:
	static const char* protocolName; // Network name of Kinect protocol
	static const unsigned int protocolVersion; // Version number of specific Kinect protocol implementation
	};

#endif
