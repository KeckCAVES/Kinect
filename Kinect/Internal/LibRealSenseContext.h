/***********************************************************************
LibRealSenseContext - Class to share librealsense context objects
between multiple RealSense cameras (as required by the API).
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

#ifndef KINECT_INTERNAL_LIBREALSENSECONTEXT_INCLUDED
#define KINECT_INTERNAL_LIBREALSENSECONTEXT_INCLUDED

#include <Misc/Autopointer.h>
#include <Threads/Mutex.h>

/* Forward declarations: */
struct rs_context;
struct rs_device;

namespace Kinect {

class LibRealSenseContext;
typedef Misc::Autopointer<LibRealSenseContext> LibRealSenseContextPtr; // Type for pointers to the singleton librealsense context object

class LibRealSenseContext
	{
	friend class Misc::Autopointer<LibRealSenseContext>;
	
	/* Elements: */
	private:
	static LibRealSenseContext theContext; // The singleton librealsense context
	Threads::Mutex refMutex; // Mutex protecting the ref() and unref() methods to guarantee that a call to acquireContext returns a valid context
	unsigned int refCount; // Reference counter for the singleton librealsense context
	struct rs_context* context; // USB context handle from the librealsense library
	
	/* Private methods: */
	void ref(void); // Adds a reference to the context
	void unref(void); // Removes a reference from the context
	
	/* Constructors and destructors: */
	LibRealSenseContext(void); // Creates an uninitialized USB context
	LibRealSenseContext(const LibRealSenseContext& source); // Prohibit copy constructor
	LibRealSenseContext& operator=(const LibRealSenseContext& source); // Prohibit assignment operator
	
	/* Methods: */
	public:
	static LibRealSenseContextPtr acquireContext(void); // Returns a pointer to the singleton librealsense context; throws exception if context can not be initialized
	int getNumDevices(void) const; // Returns the number of RealSense devices in the context
	struct rs_device* getDevice(int index); // Returns the RealSense device of the given index in the context
	};

}

#endif
