#
# OMNeT++/OMNEST Makefile for dvcastlayer
#
# This file was generated with the command:
#  opp_makemake -f --deep -IC:/veins-veins-4.4/src -LC:/veins-veins-4.4/out/$$\(CONFIGNAME\)/src -lveins -KVEINS_PROJ=C:/veins-veins-4.4
#

# Name of target to be created (-o option)
TARGET = dvcastlayer$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = -I$(VEINS_PROJ)/src -I. -Iapplication -Imessages

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L$(VEINS_PROJ)/out/$(CONFIGNAME)/src  -lveins
LIBS += -Wl,-rpath,`abspath $(VEINS_PROJ)/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = $O/application/DVCastLayer.o $O/messages/DVCast_m.o

# Message files
MSGFILES = \
    messages/DVCast.msg

# SM files
SMFILES =

# Other makefile variables (-K)
VEINS_PROJ=C:/veins-veins-4.4

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f dvcastlayer dvcastlayer.exe libdvcastlayer.so libdvcastlayer.a libdvcastlayer.dll libdvcastlayer.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f application/*_m.cc application/*_m.h application/*_sm.cc application/*_sm.h
	$(Q)-rm -f messages/*_m.cc messages/*_m.h messages/*_sm.cc messages/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc application/*.cc messages/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/application/DVCastLayer.o: application/DVCastLayer.cc \
	application/DVCastLayer.h \
	messages/DVCast_m.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/BaseConnectionManager.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/ChannelAccess.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/NicEntry.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseApplLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseBattery.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseMobility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseModule.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseWorldUtility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BatteryAccess.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/FindModule.h \
	$(VEINS_PROJ)/src/veins/base/utils/HostState.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/Move.h \
	$(VEINS_PROJ)/src/veins/base/utils/NetwToMacControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/utils/PassedMessage.h \
	$(VEINS_PROJ)/src/veins/base/utils/SimpleAddress.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_PROJ)/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	$(VEINS_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIBuffer.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIColor.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICommandInterface.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIConnection.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICoord.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIMobility.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIScenarioManager.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/Obstacle.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/ObstacleControl.h \
	$(VEINS_PROJ)/src/veins/modules/utility/Consts80211p.h \
	$(VEINS_PROJ)/src/veins/modules/world/annotations/AnnotationManager.h
$O/messages/DVCast_m.o: messages/DVCast_m.cc \
	messages/DVCast_m.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h

