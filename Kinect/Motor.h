/***********************************************************************
Motor - Wrapper class to represent the motor and accelerometer interface
aspect of the Kinect sensor.
Copyright (c) 2010-2015 Oliver Kreylos

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

#ifndef KINECT_MOTOR_INCLUDED
#define KINECT_MOTOR_INCLUDED

#include <USB/Device.h>

namespace Kinect {

class Motor:public USB::Device
	{
	/* Embedded classes: */
	public:
	enum LEDState // Enumerated type for Kinect's LED states
		{
		LED_OFF=0x0,
		LED_GREEN,LED_RED,LED_YELLOW,
		LED_BLINK_YELLOW,LED_BLINK_GREEN,
		LED_RED_YELLOW,LED_RED_GREEN
		};
	
	/* Constructors and destructors: */
	public:
	Motor(size_t index =0); // Opens the index-th Kinect motor device on the host's USB subsystem
	
	/* Methods: */
	void setLED(LEDState newLEDState); // Sets the state of the Kinect's LED
	void setPitch(int pitch); // Sets the motor's pitch angle
	void readAccelerometers(float accels[3]); // Writes the Kinect's accelerometer readings into the given array
	};

}

#endif
