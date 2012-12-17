/***********************************************************************
KinectServerMain - Main program for Kinect 3D video streamer.
Copyright (c) 2010-2012 Oliver Kreylos

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

#include <string.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <Misc/ConfigurationFile.h>
#include <USB/Context.h>

#include "KinectServer.h"

volatile bool runServerLoop=true;

void termSignalHandler(int)
	{
	runServerLoop=false;
	}

int main(void)
	{
	/* Ignore SIGPIPE and leave handling of pipe errors to TCP sockets: */
	struct sigaction sigPipeAction;
	memset(&sigPipeAction,0,sizeof(struct sigaction));
	sigPipeAction.sa_handler=SIG_IGN;
	sigemptyset(&sigPipeAction.sa_mask);
	sigPipeAction.sa_flags=0x0;
	sigaction(SIGPIPE,&sigPipeAction,0);
	
	/* Reroute SIG_INT signals to cleanly shut down the Kinect server: */
	struct sigaction sigIntAction;
	memset(&sigIntAction,0,sizeof(struct sigaction));
	sigIntAction.sa_handler=termSignalHandler;
	if(sigaction(SIGINT,&sigIntAction,0)!=0)
		std::cerr<<"KinectServerMain: Cannot intercept SIG_INT signals. Server won't shut down cleanly."<<std::endl;
	
	/* Open the server's configuration file: */
	std::string serverConfigName=KINECT_CONFIG_DIR;
	serverConfigName.push_back('/');
	serverConfigName.append(KINECTSERVER_CONFIGURATIONFILENAME);
	Misc::ConfigurationFile serverConfig(serverConfigName.c_str());
	
	/* Open the USB context: */
	USB::Context usbContext;
	
	/* Create a Kinect server object: */
	Misc::ConfigurationFileSection serverSection=serverConfig.getSection("KinectServer");
	KinectServer server(usbContext,serverSection);
	
	/* Process all USB events: */
	while(runServerLoop)
		usbContext.processEvents();
	
	/* Start background handling of USB events while shutting down the cameras: */
	usbContext.startEventHandling();
	
	return 0;
	}
