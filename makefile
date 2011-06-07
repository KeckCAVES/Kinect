########################################################################
# Makefile for Kinect 3D Video Capture Project.
# Copyright (c) 2010-2011 Oliver Kreylos
#
# This file is part of the WhyTools Build Environment.
# 
# The WhyTools Build Environment is free software; you can redistribute
# it and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
# 
# The WhyTools Build Environment is distributed in the hope that it will
# be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the WhyTools Build Environment; if not, write to the Free
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

# Root directory of the Vrui software installation. This must match the
# same setting in Vrui's makefile. By default the directories match; if
# a different version of Vrui was installed, or Vrui's installation
# directory was adjusted, the directory must be adjusted here as well.
VRUIDIR = $(HOME)/Vrui-2.1

#
# Everything underneath here should not need to be changed
#

# Check if Vrui's collaboration infrastructure is installed:
ifneq ($(strip $(wildcard $(VRUIDIR)/include/Collaboration/*)),)
  HAVE_COLLABORATION = 1
else
  HAVE_COLLABORATION = 0
endif

# Set up additional flags for the C++ compiler:
CFLAGS = -I.

# Set up destination directories for compilation products:
OBJDIRBASE = o
LIBDIRBASE = lib
BINDIRBASE = bin

# Create debug or fully optimized versions of the software:
ifdef DEBUG
  # Include the debug version of the Vrui application makefile fragment:
  include $(VRUIDIR)/share/Vrui.debug.makeinclude
  # Enable debugging and disable optimization:
  CFLAGS += -g3 -O0
  # Set destination directories for created objects:
  OBJDIR = $(OBJDIRBASE)/debug
  LIBDIR = $(LIBDIRBASE)/debug
  BINDIR = $(BINDIRBASE)/debug
else
  # Include the release version of the Vrui application makefile fragment:
  include $(VRUIDIR)/share/Vrui.makeinclude
  # Disable debugging and enable optimization:
  CFLAGS += -g0 -O3 -DNDEBUG
  # Set destination directories for created objects:
  OBJDIR = $(OBJDIRBASE)
  LIBDIR = $(LIBDIRBASE)
  BINDIR = $(BINDIRBASE)
endif

# Set up installation destinations:
PLUGININSTALLDIR = $(VRUI_LIBDIR)/CollaborationPlugins

# Pattern rule to compile C++ sources:
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(OBJDIR)/$(*D)
	@echo Compiling $<...
	@g++ -c -o $@ $(VRUI_CFLAGS) $(CFLAGS) $<

# Pattern rule to compile C++ sources as position-independent code:
$(OBJDIR)/pic/%.o: %.cpp
	@mkdir -p $(OBJDIR)/pic/$(*D)
	@echo Compiling $<...
	@g++ -c -o $@ $(VRUI_CFLAGS) $(VRUI_PLUGINCFLAGS) $(CFLAGS) $<

# Pattern rule to link executables:
$(BINDIR)/%: $(OBJDIR)/%.o
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS)

# Rule to build all Kinect 3D Video Capture Project components:
ALL = $(LIBDIR)/libKinect.a \
      $(BINDIR)/USBTest \
      $(BINDIR)/RawKinectViewer \
      $(BINDIR)/AlignPoints \
      $(BINDIR)/KinectRecorder \
      $(BINDIR)/KinectViewer \
      $(BINDIR)/KinectServer \
      $(BINDIR)/KinectClientTest \
      $(LIBDIR)/libKinectViewerVislet.$(VRUI_PLUGINFILEEXT)
ifneq ($(HAVE_COLLABORATION),0)
  ALL += $(LIBDIR)/libKinectServer.$(VRUI_PLUGINFILEEXT) \
         $(LIBDIR)/libKinectClient.$(VRUI_PLUGINFILEEXT)
endif

.PHONY: all
all: $(ALL)

# Rule to remove build results:
clean:
	-rm -f $(OBJDIR)/Kinect/*.o $(OBJDIR)/*.o
	-rm -f $(OBJDIR)/pic/Kinect/*.o $(OBJDIR)/pic/*.o
	-rm -f $(ALL)

# Rule to clean the source directory for packaging:
distclean:
	-rm -rf $(OBJDIRBASE)
	-rm -rf $(LIBDIRBASE)
	-rm -rf $(BINDIRBASE)

#
# Library containing basic USB and Kinect classes:
#

LIBKINECT_SOURCES = $(wildcard Kinect/*.cpp)

$(LIBDIR)/libKinect.a: $(LIBKINECT_SOURCES:%.cpp=$(OBJDIR)/%.o)
	@mkdir -p $(LIBDIR)
	@-rm -f $@
	@echo Linking $@...
	@ar crs $@ $^

#
# Simple test program for libusb-1.0; can control motor and LEDs and
# send commands to camera:
#

USBTEST_SOURCES = USBTest.cpp

$(BINDIR)/USBTest: VRUI_LINKFLAGS += -lusb-1.0
$(BINDIR)/USBTest: $(USBTEST_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                   $(LIBDIR)/libKinect.a
.PHONY: USBTest
USBTest: $(BINDIR)/USBTest

#
# Viewer for raw depth and color image streams:
#

RAWKINECTVIEWER_SOURCES = RawKinectViewer.cpp

$(BINDIR)/RawKinectViewer: VRUI_LINKFLAGS += -lusb-1.0
$(BINDIR)/RawKinectViewer: $(RAWKINECTVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                           $(LIBDIR)/libKinect.a
.PHONY: RawKinectViewer
RawKinectViewer: $(BINDIR)/RawKinectViewer

#
# 3D space alignment program, based on two files containing 3D tie
# points:
#

ALIGNPOINTS_SOURCES = AlignPoints.cpp

$(BINDIR)/AlignPoints: $(ALIGNPOINTS_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: AlignPoints
AlignPoints: $(BINDIR)/AlignPoints

#
# Utility to record color and depth streams from one or more Kinect
# devices to pairs of compressed files:
#

KINECTRECORDER_SOURCES = KinectRecorder.cpp

$(BINDIR)/KinectRecorder: VRUI_LINKFLAGS += -lusb-1.0
$(BINDIR)/KinectRecorder: $(KINECTRECORDER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                          $(LIBDIR)/libKinect.a

.PHONY: KinectRecorder
KinectRecorder: $(BINDIR)/KinectRecorder

#
# Viewer for 3D image streams from one or more Kinect devices:
#

KINECTVIEWER_SOURCES = KinectViewer.cpp

$(BINDIR)/KinectViewer: VRUI_LINKFLAGS += -lusb-1.0
$(BINDIR)/KinectViewer: $(KINECTVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                        $(LIBDIR)/libKinect.a
.PHONY: KinectViewer
KinectViewer: $(BINDIR)/KinectViewer

#
# Server for real-time 3D video streaming protocol:
#

KINECTSERVER_SOURCES = KinectServer.cpp \
                       KinectServerMain.cpp

$(BINDIR)/KinectServer: CFLAGS += -DVERBOSE
$(BINDIR)/KinectServer: VRUI_LINKFLAGS += -lusb-1.0
$(BINDIR)/KinectServer: $(KINECTSERVER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                        $(LIBDIR)/libKinect.a

.PHONY: KinectServer
KinectServer: $(BINDIR)/KinectServer

#
# Client application for real-time 3D video streaming protocol:
#

KINECTCLIENTTEST_SOURCES = KinectClient.cpp \
                           KinectClientTest.cpp

$(BINDIR)/KinectClientTest: $(KINECTCLIENTTEST_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                            $(LIBDIR)/libKinect.a

.PHONY: KinectClientTest
KinectClientTest: $(BINDIR)/KinectClientTest

#
# Utility to convert frames from multiple colocated depth image streams
# into a 3D volume to create a watertight mesh based on a space carving
# approach:
#

SPACECARVER_SOURCES = SpaceCarver.cpp

$(BINDIR)/SpaceCarver: $(SPACECARVER_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: SpaceCarver
SpaceCarver: $(BINDIR)/SpaceCarver

#
# Vislet to render 3D video in otherwise unaware Vrui applications:
#

KINECTVIEWERVISLET_SOURCES = Kinect/USBContext.cpp \
                             Kinect/USBConfigDescriptor.cpp \
                             Kinect/USBDevice.cpp \
                             Kinect/USBDeviceList.cpp \
                             Kinect/KinectCamera.cpp \
                             Kinect/KinectProjector.cpp \
                             KinectViewerVislet.cpp

$(LIBDIR)/libKinectViewerVislet.$(VRUI_PLUGINFILEEXT): VRUI_LINKFLAGS += -lusb-1.0
$(LIBDIR)/libKinectViewerVislet.$(VRUI_PLUGINFILEEXT): $(KINECTVIEWERVISLET_SOURCES:%.cpp=$(OBJDIR)/pic/%.o)
	@mkdir -p $(LIBDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) $(VRUI_PLUGINLINKFLAGS)

#
# Server plug-in to transmit 3D video in Vrui's collaboration
# infrastructure:
#

KINECTSERVERPLUGIN_SOURCES = KinectPipe.cpp \
                             KinectServerPlugin.cpp

$(LIBDIR)/libKinectServer.$(VRUI_PLUGINFILEEXT): $(KINECTSERVERPLUGIN_SOURCES:%.cpp=$(OBJDIR)/pic/%.o)
	@mkdir -p $(LIBDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) $(VRUI_PLUGINLINKFLAGS)

#
# Client plug-in to transmit 3D video in Vrui's collaboration
# infrastructure:
#

KINECTCLIENTPLUGIN_SOURCES = Kinect/ColorFrameReader.cpp \
                             Kinect/HilbertCurve.cpp \
                             Kinect/DepthFrameReader.cpp \
                             Kinect/KinectProjector.cpp \
                             KinectPipe.cpp \
                             KinectClient.cpp \
                             KinectClientPlugin.cpp

$(LIBDIR)/libKinectClient.$(VRUI_PLUGINFILEEXT): $(KINECTCLIENTPLUGIN_SOURCES:%.cpp=$(OBJDIR)/pic/%.o)
	@mkdir -p $(LIBDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) $(VRUI_PLUGINLINKFLAGS)

#
# Installation rule to place vislets and plugins into their proper
# destination directories:
#

install:
# Install all vislet plugins in VRUI_VISLETSDIR:
	@echo Installing vislet plugins...
	@install -d $(VRUI_VISLETSDIR)
	@install $(LIBDIR)/libKinectViewerVislet.$(VRUI_PLUGINFILEEXT) $(VRUI_VISLETSDIR)
ifneq ($(HAVE_COLLABORATION),0)
  # Install all protocol plugins in PLUGININSTALLDIR:
	@echo Installing protocol plugins...
	@install -d $(PLUGININSTALLDIR)
	@install $(LIBDIR)/libKinectServer.$(VRUI_PLUGINFILEEXT) $(LIBDIR)/libKinectClient.$(VRUI_PLUGINFILEEXT) $(PLUGININSTALLDIR)
endif
