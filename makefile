########################################################################
# Makefile for Kinect hacking project.
# Copyright (c) 2010 Oliver Kreylos
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
VRUIDIR = $(HOME)/Vrui-2.0

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

# Pattern rule to compile C++ sources:
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(OBJDIR)/$(*D)
	@echo Compiling $<...
	@g++ -c -o $@ $(VRUI_CFLAGS) $(CFLAGS) $<

# Rule to build all Kinect components:
ALL = $(LIBDIR)/libKinect.a \
      $(BINDIR)/USBTest \
      $(BINDIR)/CalibrateCameras \
      $(BINDIR)/RawKinectViewer \
      $(BINDIR)/AlignPoints \
      $(BINDIR)/KinectViewer
.PHONY: all
all: $(ALL)

# Rule to remove build results:
clean:
	-rm -f $(OBJDIR)/*.o
	-rm -f $(ALL)

# Rule to clean the source directory for packaging:
distclean:
	-rm -rf $(OBJDIRBASE)
	-rm -rf $(LIBDIRBASE)
	-rm -rf $(BINDIRBASE)

#
# Library containing basic USB and Kinect classes:
#

LIBKINECT_SOURCES = USBContext.cpp \
                    USBDevice.cpp \
                    USBDeviceList.cpp \
                    USBConfigDescriptor.cpp \
                    KinectMotor.cpp \
                    KinectCamera.cpp

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

$(BINDIR)/USBTest: $(USBTEST_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                   $(LIBDIR)/libKinect.a
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) -lusb-1.0
.PHONY: USBTest
USBTest: $(BINDIR)/USBTest

#
# Camera calibration program, based on set of tie points:
#

CALIBRATECAMERAS_SOURCES = CalibrateCameras.cpp

$(BINDIR)/CalibrateCameras: $(CALIBRATECAMERAS_SOURCES:%.cpp=$(OBJDIR)/%.o)
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS)
.PHONY: CalibrateCameras
CalibrateCameras: $(BINDIR)/CalibrateCameras

#
# Viewer for raw depth and color image streams:
#

RAWKINECTVIEWER_SOURCES = RawKinectViewer.cpp

$(BINDIR)/RawKinectViewer: $(RAWKINECTVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                           $(LIBDIR)/libKinect.a
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) -lusb-1.0
.PHONY: RawKinectViewer
RawKinectViewer: $(BINDIR)/RawKinectViewer

#
# 3D space alignment program, based on two files containing 3D tie
# points:
#

ALIGNPOINTS_SOURCES = AlignPoints.cpp

$(BINDIR)/AlignPoints: $(ALIGNPOINTS_SOURCES:%.cpp=$(OBJDIR)/%.o)
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS)
.PHONY: AlignPoints
AlignPoints: $(BINDIR)/AlignPoints

#
# Viewer for 3D image streams:
#

KINECTVIEWER_SOURCES = KinectFrameSaver.cpp \
                       KinectPlayback.cpp \
                       KinectProjector.cpp \
                       KinectViewer.cpp

$(BINDIR)/KinectViewer: $(KINECTVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o) \
                        $(LIBDIR)/libKinect.a
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) -lusb-1.0
.PHONY: KinectViewer
KinectViewer: $(BINDIR)/KinectViewer
