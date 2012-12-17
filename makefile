########################################################################
# Makefile for Kinect 3D Video Capture Project.
# Copyright (c) 2010-2012 Oliver Kreylos
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

# Directory containing the Vrui build system. The directory below
# matches the default Vrui installation; if Vrui's installation
# directory was changed during Vrui's installation, the directory below
# must be adapted.
VRUI_MAKEDIR := $(HOME)/Vrui-2.6/share/make
ifdef DEBUG
  VRUI_MAKEDIR := $(VRUI_MAKEDIR)/debug
endif

########################################################################
# Everything below here should not have to be changed
########################################################################

# Define the root of the toolkit source tree
PACKAGEROOT := $(shell pwd)

# Specify version of created dynamic shared libraries
KINECT_VERSION = 2004
MAJORLIBVERSION = 2
MINORLIBVERSION = 4
KINECT_NAME := Kinect-$(MAJORLIBVERSION).$(MINORLIBVERSION)

# Check if Vrui's collaboration infrastructure is installed
ifneq ($(wildcard $(VRUI_MAKEDIR)/Packages.Collaboration),)
  HAVE_COLLABORATION = 1
else
  HAVE_COLLABORATION = 0
endif

# Root directory for Kinect configuration data underneath Vrui's
# configuration directory:
KINECTCONFIGDIREXT = $(KINECT_NAME)

# Include definitions for the system environment and system-provided
# packages
include $(VRUI_MAKEDIR)/SystemDefinitions
include $(VRUI_MAKEDIR)/Packages.System
include $(VRUI_MAKEDIR)/Configuration.Vrui
include $(VRUI_MAKEDIR)/Packages.Vrui
ifneq ($(HAVE_COLLABORATION),0)
  include $(VRUI_MAKEDIR)/Configuration.Collaboration
  include $(VRUI_MAKEDIR)/Packages.Collaboration
endif
include $(PACKAGEROOT)/BuildRoot/Packages.Kinect

# Override the include file and library search directories:
EXTRACINCLUDEFLAGS += -I$(PACKAGEROOT)
EXTRALINKDIRFLAGS += -L$(PACKAGEROOT)/$(MYLIBEXT)

# Set destination directory for libraries and plugins:
LIBDESTDIR := $(PACKAGEROOT)/$(MYLIBEXT)
VISLETDESTDIR := $(LIBDESTDIR)/Vislets
PLUGINDESTDIR := $(LIBDESTDIR)/Plugins

########################################################################
# Specify additional compiler and linker flags
########################################################################

CFLAGS += -Wall -pedantic

# Set flags to distinguish between static and shared libraries
ifdef STATIC_LINK
  LIBRARYNAME = $(LIBDESTDIR)/$(1).$(LDEXT).a
  OBJDIREXT = Static
else
  CFLAGS += $(CDSOFLAGS)
  LIBRARYNAME=$(LIBDESTDIR)/$(call FULLDSONAME,$(1))
endif
VISLETNAME = $(VISLETDESTDIR)/lib$(1).$(PLUGINFILEEXT)
PLUGINNAME = $(PLUGINDESTDIR)/lib$(1).$(PLUGINFILEEXT)

########################################################################
# List packages used by this project
# (Supported packages can be found in
# $(VRUI_MAKEDIR)/BuildRoot/Packages)
########################################################################

PACKAGES = MYTHREADS MYMISC

########################################################################
# Specify all final targets
########################################################################

LIBRARIES = 
PLUGINS = 
VISLETS = 
EXECUTABLES = 

#
# The Kinect driver library:
#

LIBRARY_NAMES = libKinect

LIBRARIES += $(LIBRARY_NAMES:%=$(call LIBRARYNAME,%))

#
# The Kinect executables:
#

EXECUTABLES += $(EXEDIR)/KinectUtil \
               $(EXEDIR)/RawKinectViewer \
               $(EXEDIR)/AlignPoints \
               $(EXEDIR)/KinectServer \
               $(EXEDIR)/KinectViewer

#
# The Kinect vislets:
#

VISLET_NAMES = KinectViewer \
               KinectRecorder \
               KinectPlayer

VISLETS += $(VISLET_NAMES:%=$(call VISLETNAME,%))

#
# The Kinect collaboration infrastructure plug-ins:
#

COLLABORATIONPLUGIN_NAMES = KinectServer \
                            KinectClient

ifneq ($(HAVE_COLLABORATION),0)
  PLUGINS += $(COLLABORATIONPLUGIN_NAMES:%=$(call PLUGINNAME,%))
endif

# Set the name of the make configuration file:
MAKECONFIGFILE = share/Configuration.Kinect

ALL = $(LIBRARIES) $(EXECUTABLES) $(PLUGINS) $(VISLETS) $(MAKECONFIGFILE)

.PHONY: all
all: $(ALL)

# Make all components depend on Kinect driver library:
$(VISLETS) $(PLUGINS) $(EXECUTABLES): $(call LIBRARYNAME,libKinect)

########################################################################
# Pseudo-target to print configuration options
########################################################################

.PHONY: config
config: Configure-End

.PHONY: Configure-Begin
Configure-Begin:
	@echo "---- Configured Kinect options: ----"
	@echo "Installation directory: $(VRUI_PACKAGEROOT)"
	@echo "Calibration data directory: $(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)"
	@echo "Vislet plug-in directory: $(PLUGININSTALLDIR)/$(VRVISLETSDIREXT)"
ifneq ($(HAVE_COLLABORATION),0)
	@echo "Vrui collaboration infrastructure detected"
	@echo "Collaboration protocol plug-in directory: $(PLUGININSTALLDIR)/$(COLLABORATIONPLUGINSDIREXT)"
endif
ifeq ($(SYSTEM_HAVE_LIBUSB1),0)
	@echo "ERROR: Vrui was not built with libusb-1 support. Please install missing development package(s) and rebuild Vrui"
	@exit 1
endif

.PHONY: Configure-End
Configure-End: Configure-Begin
Configure-End:
	@echo "--------"

$(wildcard *.cpp Kinect/*.cpp): config

########################################################################
# Specify other actions to be performed on a `make clean'
########################################################################

.PHONY: extraclean
extraclean:
	-rm -f $(LIBRARY_NAMES:%=$(LIBDESTDIR)/$(call DSONAME,%))
	-rm -f $(LIBRARY_NAMES:%=$(LIBDESTDIR)/$(call LINKDSONAME,%))
	-rm -f $(VISLET_NAMES:%=$(call VISLETNAME,%))
	-rm -f $(PLUGIN_NAMES:%=$(call PLUGINNAME,%))

.PHONY: extrasqueakyclean
extrasqueakyclean:
	-rm -f $(ALL)
	-rm -rf $(PACKAGEROOT)/$(LIBEXT)

# Include basic makefile
include $(VRUI_MAKEDIR)/BasicMakefile

########################################################################
# Specify additional per-package compiler flags
########################################################################

# Define location for configuration files:
CFLAGS += -DKINECT_CONFIG_DIR='"$(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)"'

########################################################################
# Specify build rules for dynamic shared objects
########################################################################

LIBKINECT_HEADERS = $(wildcard Kinect/*.h)

LIBKINECT_SOURCES = $(wildcard Kinect/*.cpp)

# Define names for camera calibration files:
$(OBJDIR)/Kinect/Camera.o: CFLAGS += -DKINECT_CAMERA_DEPTHCORRECTIONFILENAMEPREFIX='"DepthCorrection"'
$(OBJDIR)/Kinect/Camera.o: CFLAGS += -DKINECT_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX='"IntrinsicParameters"'
$(OBJDIR)/Kinect/Camera.o: CFLAGS += -DKINECT_CAMERA_EXTRINSICPARAMETERSFILENAMEPREFIX='"ExtrinsicParameters"'

$(call LIBRARYNAME,libKinect): PACKAGES += $(MYKINECT_DEPENDS)
$(call LIBRARYNAME,libKinect): EXTRACINCLUDEFLAGS += $(MYKINECT_INCLUDE)
$(call LIBRARYNAME,libKinect): CFLAGS += $(MYKINECT_CFLAGS)
$(call LIBRARYNAME,libKinect): $(LIBKINECT_SOURCES:%.cpp=$(OBJDIR)/%.o)
.PHONY: libKinect
libKinect: $(call LIBRARYNAME,libKinect)

########################################################################
# Specify build rules for executables
########################################################################

#
# Utility to list Kinect devices on all USB buses and send commands to
# them:
#

$(EXEDIR)/KinectUtil: PACKAGES += MYKINECT
$(EXEDIR)/KinectUtil: $(OBJDIR)/KinectUtil.o
.PHONY: KinectUtil
KinectUtil: $(EXEDIR)/KinectUtil

#
# Viewer for raw depth and color image streams, with ability to
# internally and externally calibrate Kinect cameras:
#

$(OBJDIR)/DepthCorrectionTool.o: CFLAGS += -DKINECT_CAMERA_DEPTHCORRECTIONFILENAMEPREFIX='"DepthCorrection"'
$(OBJDIR)/GridTool.o: CFLAGS += -DKINECT_CAMERA_INTRINSICPARAMETERSFILENAMEPREFIX='"IntrinsicParameters"'

$(EXEDIR)/RawKinectViewer: PACKAGES += MYVRUI MYKINECT
$(EXEDIR)/RawKinectViewer: $(OBJDIR)/PauseTool.o \
                           $(OBJDIR)/TiePointTool.o \
                           $(OBJDIR)/LineTool.o \
                           $(OBJDIR)/DepthCorrectionTool.o \
                           $(OBJDIR)/GridTool.o \
                           $(OBJDIR)/PlaneTool.o \
                           $(OBJDIR)/RawKinectViewer.o
.PHONY: RawKinectViewer
RawKinectViewer: $(EXEDIR)/RawKinectViewer

#
# 3D space alignment program for external calibration of multiple Kinect
# devices, based on two files containing 3D tie points:
#

$(EXEDIR)/AlignPoints: PACKAGES += MYVRUI
$(EXEDIR)/AlignPoints: $(OBJDIR)/AlignPoints.o
.PHONY: AlignPoints
AlignPoints: $(EXEDIR)/AlignPoints

#
# Server for real-time 3D video streaming protocol:
#

# Tell Kinect server to print status info:
$(OBJDIR)/KinectServer.o: CFLAGS += -DVERBOSE

# Tell Kinect server to be extremely verbose:
#$(OBJDIR)/KinectServer.o: CFLAGS += -DVVERBOSE

$(OBJDIR)/KinectServerMain.o: CFLAGS += -DKINECTSERVER_CONFIGURATIONFILENAME='"KinectServer.cfg"'

$(EXEDIR)/KinectServer: PACKAGES += MYKINECT MYCOMM
$(EXEDIR)/KinectServer: $(OBJDIR)/KinectServer.o \
                        $(OBJDIR)/KinectServerMain.o
.PHONY: KinectServer
KinectServer: $(EXEDIR)/KinectServer

#
# Viewer for 3D image streams from one or more Kinect devices, pre-
# recorded files or 3D video streaming servers:
#

$(EXEDIR)/KinectViewer: PACKAGES += MYVRUI MYKINECT
$(EXEDIR)/KinectViewer: $(OBJDIR)/KinectViewer.o
.PHONY: KinectViewer
KinectViewer: $(EXEDIR)/KinectViewer

#
# Several obsolete or testing utilities or applications:
#

$(EXEDIR)/CompressDepthFile: PACKAGES += MYKINECT
$(EXEDIR)/CompressDepthFile: $(OBJDIR)/CompressDepthFile.o
.PHONY: CompressDepthFile
CompressDepthFile: $(EXEDIR)/CompressDepthFile

$(EXEDIR)/DepthCompressionTest: PACKAGES += MYKINECT
$(EXEDIR)/DepthCompressionTest: $(OBJDIR)/DepthCompressionTest.o
.PHONY: DepthCompressionTest
DepthCompressionTest: $(EXEDIR)/DepthCompressionTest

$(EXEDIR)/ColorCompressionTest: PACKAGES += MYKINECT
$(EXEDIR)/ColorCompressionTest: $(OBJDIR)/ColorCompressionTest.o
.PHONY: ColorCompressionTest
ColorCompressionTest: $(EXEDIR)/ColorCompressionTest

$(EXEDIR)/CalibrateDepth: PACKAGES += MYMATH MYIO
$(EXEDIR)/CalibrateDepth: $(OBJDIR)/CalibrateDepth.o
.PHONY: CalibrateDepth
CalibrateDepth: $(EXEDIR)/CalibrateDepth

$(EXEDIR)/CalibrateCameras: PACKAGES += MYMATH MYIO
$(EXEDIR)/CalibrateCameras: $(OBJDIR)/CalibrateCameras.o
.PHONY: CalibrateCameras
CalibrateCameras: $(EXEDIR)/CalibrateCameras

$(EXEDIR)/NewCalibrateCameras: PACKAGES += MYGEOMETRY MYMATH MYIO
$(EXEDIR)/NewCalibrateCameras: $(OBJDIR)/NewCalibrateCameras.o
.PHONY: NewCalibrateCameras
NewCalibrateCameras: $(EXEDIR)/NewCalibrateCameras

$(EXEDIR)/TestAlignment: PACKAGES += MYGEOMETRY MYMATH MYIO
$(EXEDIR)/TestAlignment: $(OBJDIR)/TestAlignment.o
.PHONY: TestAlignment
TestAlignment: $(EXEDIR)/TestAlignment

$(EXEDIR)/SpaceCarver: PACKAGES += MYKINECT MYGEOMETRY MYMATH MYMISC
$(EXEDIR)/SpaceCarver: $(OBJDIR)/SpaceCarver.o
.PHONY: SpaceCarver
SpaceCarver: $(EXEDIR)/SpaceCarver

########################################################################
# Specify build rules for vislet plug-ins
########################################################################

# Implicit rule for creating vislet plug-ins:
$(call VISLETNAME,%): PACKAGES += MYVRUI MYKINECT
$(call VISLETNAME,%): $(OBJDIR)/pic/Vislets/%.o
	@mkdir -p $(VISLETDESTDIR)
ifdef SHOWCOMMAND
	$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $(filter %.o,$^) $(LINKDIRFLAGS) $(LINKLIBFLAGS)
else
	@echo Linking $@...
	@$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $(filter %.o,$^) $(LINKDIRFLAGS) $(LINKLIBFLAGS)
endif

#
# Vislet to render live 3D video in any Vrui application:
#

$(call VISLETNAME,KinectViewer): $(OBJDIR)/pic/Vislets/KinectViewer.o

#
# Vislet to record 3D video from any Vrui application:
#

$(call VISLETNAME,KinectRecorder): $(OBJDIR)/pic/Vislets/KinectRecorder.o

#
# Vislet to play back previously recorded 3D video in any Vrui
# application:
#

$(call VISLETNAME,KinectPlayer): $(OBJDIR)/pic/Vislets/KinectPlayer.o

# Mark all vislet plugin object files as intermediate:
.SECONDARY: $(VISLET_NAMES:%=$(OBJDIR)/pic/Vislets/%.o)

########################################################################
# Specify build rules for collaboration infrastructure plug-ins
########################################################################

# Implicit rule for creating collaboration infrastructure plug-ins:
$(call PLUGINNAME,%): $(OBJDIR)/pic/Plugins/%.o
	@mkdir -p $(PLUGINDESTDIR)
ifdef SHOWCOMMAND
	$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $(filter %.o,$^) $(LINKDIRFLAGS) $(LINKLIBFLAGS)
else
	@echo Linking $@...
	@$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $(filter %.o,$^) $(LINKDIRFLAGS) $(LINKLIBFLAGS)
endif

#
# Server-side plugin:
#

$(call PLUGINNAME,KinectServer): PACKAGES += MYCOLLABORATIONSERVER
$(call PLUGINNAME,KinectServer): $(OBJDIR)/pic/Plugins/KinectProtocol.o \
                                 $(OBJDIR)/pic/Plugins/KinectServer.o

#
# Client-side plugin:
#

$(call PLUGINNAME,KinectClient): PACKAGES += MYKINECT MYCOLLABORATIONCLIENT
$(call PLUGINNAME,KinectClient): $(OBJDIR)/pic/Plugins/KinectProtocol.o \
                                 $(OBJDIR)/pic/Plugins/KinectClient.o

########################################################################
# Specify installation rules for header files, libraries, executables,
# configuration files, and shared files.
########################################################################

# Pseudo-target to dump Kinect configuration settings
$(MAKECONFIGFILE): config
	@echo Creating configuration makefile fragment...
	@echo '# Makefile fragment for Kinect configuration options' > $(MAKECONFIGFILE)
	@echo '# Autogenerated by Kinect installation on $(shell date)' >> $(MAKECONFIGFILE)
	@echo >> $(MAKECONFIGFILE)
	@echo '# Configuration settings:'>> $(MAKECONFIGFILE)
	@echo 'KINECT_CONFIGDIR = $(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)' >> $(MAKECONFIGFILE)
	@echo 'KINECT_HAVE_COLLABORATION = $(HAVE_COLLABORATION)' >> $(MAKECONFIGFILE)
	@echo >> $(MAKECONFIGFILE)
	@echo '# Version information:'>> $(MAKECONFIGFILE)
	@echo 'KINECT_VERSION = $(KINECT_VERSION)' >> $(MAKECONFIGFILE)
	@echo 'KINECT_NAME = $(KINECT_NAME)' >> $(MAKECONFIGFILE)

ifdef INSTALLPREFIX
  HEADERINSTALLDIR := $(INSTALLPREFIX)/$(HEADERINSTALLDIR)
  LIBINSTALLDIR := $(INSTALLPREFIX)/$(LIBINSTALLDIR)
  PLUGININSTALLDIR := $(INSTALLPREFIX)/$(PLUGININSTALLDIR)
  EXECUTABLEINSTALLDIR := $(INSTALLPREFIX)/$(EXECUTABLEINSTALLDIR)
  ETCINSTALLDIR := $(INSTALLPREFIX)/$(ETCINSTALLDIR)
  SHAREINSTALLDIR := $(INSTALLPREFIX)/$(SHAREINSTALLDIR)
endif

install: all
# Install all header files in HEADERINSTALLDIR:
	@echo Installing header files...
	@install -d $(HEADERINSTALLDIR)/Kinect
	@install -m u=rw,go=r $(LIBKINECT_HEADERS) $(HEADERINSTALLDIR)/Kinect
# Install all library files in LIBINSTALLDIR:
	@echo Installing libraries...
	@install -d $(LIBINSTALLDIR)
	@install $(LIBRARIES) $(LIBINSTALLDIR)
	@echo Configuring run-time linker...
	$(foreach LIBNAME,$(LIBRARY_NAMES),$(call CREATE_SYMLINK,$(LIBNAME)))
ifneq ($(HAVE_COLLABORATION),0)
  # Install all collaboration protocol plugins in PLUGININSTALLDIR/COLLABORATIONPLUGINSDIREXT:
	@echo Installing collaboration protocol plugins...
	@install -d $(PLUGININSTALLDIR)/$(COLLABORATIONPLUGINSDIREXT)
	@install $(PLUGINS) $(PLUGININSTALLDIR)/$(COLLABORATIONPLUGINSDIREXT)
endif
# Install all vislet plugins in PLUGININSTALLDIR/VRVISLETSDIREXT:
	@echo Installing vislet plugins...
	@install -d $(PLUGININSTALLDIR)/$(VRVISLETSDIREXT)
	@install $(VISLETS) $(PLUGININSTALLDIR)/$(VRVISLETSDIREXT)
# Install all binaries in EXECUTABLEINSTALLDIR:
	@echo Installing executables...
	@install -d $(EXECUTABLEINSTALLDIR)
	@install $(EXECUTABLES) $(EXECUTABLEINSTALLDIR)
# Install all configuration files in ETCINSTALLDIR:
	@echo Installing configuration files...
	@install -d $(ETCINSTALLDIR)
	@install -d $(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)
	@install -m u=rw,go=r etc/KinectServer.cfg $(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)
# Install the package and configuration files in MAKEINSTALLDIR:
	@echo Installing makefile fragments...
	@install -d $(MAKEINSTALLDIR)
	@install -m u=rw,go=r BuildRoot/Packages.Kinect $(MAKEINSTALLDIR)
	@install -m u=rw,go=r $(MAKECONFIGFILE) $(MAKEINSTALLDIR)

uninstall:
	@echo Removing header files...
	@rm -rf $(HEADERINSTALLDIR)/Kinect
	@echo Removing libraries...
	@rm -f $(LIBRARIES:$(LIBDESTDIR)/%=$(LIBINSTALLDIR)/%)
	$(foreach LIBNAME,$(LIBRARY_NAMES),$(call DESTROY_SYMLINK,$(LIBNAME)))
ifneq ($(HAVE_COLLABORATION),0)
	@echo Removing collaboration protocol plugins...
	@rm -f $(PLUGINS:$(PLUGINDESTDIR)/%=$(PLUGININSTALLDIR)/$(COLLABORATIONPLUGINSDIREXT)/%)
endif
	@echo Removing vislet plugins...
	@rm -f $(VISLETS:$(VISLETDESTDIR)/%=$(PLUGININSTALLDIR)/$(VRVISLETSDIREXT)/%)
	@echo Removing executables...
	@rm -f $(EXECUTABLES:$(EXEDIR)/%=$(EXECUTABLEINSTALLDIR)/%)
	@echo Removing configuration files...
	@rm -rf $(ETCINSTALLDIR)/$(KINECTCONFIGDIREXT)
	@echo Removing makefile fragments...
	@rm -f $(MAKEINSTALLDIR)/Packages.Kinect
	@rm -f $(MAKEINSTALLDIR)/Configuration.Kinect
