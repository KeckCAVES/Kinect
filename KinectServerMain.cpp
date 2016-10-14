/***********************************************************************
KinectServerMain - Main program for Kinect 3D video streamer.
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

#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <Misc/ConfigurationFile.h>
#include <IO/OpenFile.h>
#include <Kinect/Internal/Config.h>

#include "KinectServer.h"

int shutdownPipeFds[2];

void termSignalHandler(int)
	{
	/* Write something to the shutdown pipe: */
	char shutdown=1;
	if(write(shutdownPipeFds[1],&shutdown,sizeof(char))!=ssize_t(sizeof(char)))
		;
	}

int main(void)
	{
	IO::Directory::setCurrent(IO::openDirectory("."));
	
	/* Open the shutdown pipe: */
	if(pipe(shutdownPipeFds)!=0)
		{
		std::cerr<<"KinectServerMain: Cannot create shutdown pipe."<<std::endl;
		return 1;
		}
	
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
	std::string serverConfigName=KINECT_INTERNAL_CONFIG_CONFIGDIR;
	serverConfigName.push_back('/');
	serverConfigName.append(KINECT_INTERNAL_CONFIG_KINECTSERVER_CONFIGURATIONFILENAME);
	Misc::ConfigurationFile serverConfig(serverConfigName.c_str());
	
	/* Create a Kinect server object: */
	Misc::ConfigurationFileSection serverSection=serverConfig.getSection("KinectServer");
	KinectServer* server=new KinectServer(serverSection);
	
	/* Wait for a shutdown event: */
	char shutdown;
	while(true)
		{
		ssize_t readSize=read(shutdownPipeFds[0],&shutdown,sizeof(char));
		if(readSize==1&&shutdown==1)
			break;
		}
	
	/* Shut down the server: */
	delete server;
	
	/* Close the shutdown pipe: */
	close(shutdownPipeFds[0]);
	close(shutdownPipeFds[1]);
	
	return 0;
	}
