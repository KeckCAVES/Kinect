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
# the installation directory was adjusted during Vrui's installation, it
# must be adjusted here as well.
VRUIDIR = $(HOME)/Vrui-2.0

# Set up additional flags for the C++ compiler:
CFLAGS = 

# Set up destination directories for compilation products:
OBJDIRBASE = o
BINDIRBASE = bin

# Create debug or fully optimized versions of the software:
ifdef DEBUG
  # Include the debug version of the Vrui application makefile fragment:
  include $(VRUIDIR)/etc/Vrui.debug.makeinclude
  # Enable debugging and disable optimization:
  CFLAGS += -g3 -O0
  # Set destination directories for created objects:
  OBJDIR = $(OBJDIRBASE)/debug
  BINDIR = $(BINDIRBASE)/debug
else
  # Include the release version of the Vrui application makefile fragment:
  include $(VRUIDIR)/etc/Vrui.makeinclude
  # Disable debugging and enable optimization:
  CFLAGS += -g0 -O3 -DNDEBUG
  # Set destination directories for created objects:
  OBJDIR = $(OBJDIRBASE)
  BINDIR = $(BINDIRBASE)
endif

# Pattern rule to compile C++ sources:
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(OBJDIR)/$(*D)
	@echo Compiling $<...
	@g++ -c -o $@ $(VRUI_CFLAGS) $(CFLAGS) $<

# Rule to build all Kinect components:
ALL = $(BINDIR)/CalibrateCameras \
      $(BINDIR)/KinectViewer
.PHONY: all
all: $(ALL)

# Rule to remove build results:
clean:
	-rm -f $(OBJDIR)/*.o
	-rm -f $(ALL)
	-rmdir $(BINDIR)

# Rule to clean the source directory for packaging:
distclean:
	-rm -rf $(OBJDIRBASE)
	-rm -rf $(BINDIRBASE)

CALIBRATECAMERAS_SOURCES = CalibrateCameras.cpp

$(BINDIR)/CalibrateCameras: $(CALIBRATECAMERAS_SOURCES:%.cpp=$(OBJDIR)/%.o)
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS)
.PHONY: CalibrateCameras
CalibrateCameras: $(BINDIR)/CalibrateCameras

KINECTVIEWER_SOURCES = USBContext.cpp \
                       USBDevice.cpp \
                       USBDeviceList.cpp \
                       KinectMotor.cpp \
                       KinectCamera.cpp \
                       MD5Animator.cpp \
                       KinectViewer.cpp

$(BINDIR)/KinectViewer: $(KINECTVIEWER_SOURCES:%.cpp=$(OBJDIR)/%.o)
	@mkdir -p $(BINDIR)
	@echo Linking $@...
	@g++ -o $@ $^ $(VRUI_LINKFLAGS) -lusb-1.0
.PHONY: KinectViewer
KinectViewer: $(BINDIR)/KinectViewer
