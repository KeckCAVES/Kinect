/***********************************************************************
KinectClientTest - Test application to receive and display streaming
3D video data from a remote server.
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

#include <stdlib.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>

#include "KinectClient.h"

class KinectClientTest:public Vrui::Application
	{
	/* Elements: */
	private:
	KinectClient* client; // The streaming 3D video client
	
	/* Constructors and destructors: */
	public:
	KinectClientTest(int& argc,char**& argv,char**& appDefaults);
	virtual ~KinectClientTest(void);
	
	/* Methods from Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	};

/*********************************
Methods of class KinectClientTest:
*********************************/

KinectClientTest::KinectClientTest(int& argc,char**& argv,char**& appDefaults)
	:Vrui::Application(argc,argv,appDefaults),
	 client(0)
	{
	/* Connect to the Kinect server: */
	client=new KinectClient(argv[1],atoi(argv[2]),Vrui::getClusterMultiplexer());
	}

KinectClientTest::~KinectClientTest(void)
	{
	delete client;
	}

void KinectClientTest::frame(void)
	{
	client->frame();
	}

void KinectClientTest::display(GLContextData& contextData) const
	{
	client->glRenderAction(contextData);
	}

int main(int argc,char* argv[])
	{
	char** appDefaults=0;
	KinectClientTest app(argc,argv,appDefaults);
	app.run();
	
	return 0;
	}
