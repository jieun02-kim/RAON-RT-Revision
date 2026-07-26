##############################################################################
#	Name: Makefile
#	Author: Raimarius Delgado (Asst. Professor)
#	Affiliation: RAIMLAB @ Myongji University
#	Description: Common makefile to build Indy7 Control
#	Copyright: RAIMLAB (2025)
##############################################################################


## COMMON
CURR_DIR=.
PROJ_ROOT_DIR=../..
ROBOT_DIR=$(PROJ_ROOT_DIR)/CRobot
ECAT_DIR=$(PROJ_ROOT_DIR)/EMasterApp
GLOBAL_DIR=$(PROJ_ROOT_DIR)/Global
COMM_DIR=$(GLOBAL_DIR)/Comm

#THIS
RT_POSIX_DIR=/opt/rt_posix
RT_POSIX_INCLUDE=$(RT_POSIX_DIR)/include
RT_POSIX_LIB=$(RT_POSIX_DIR)/lib
IGH_DIR=/opt/etherlab
IGH_INCLUDE=$(IGH_DIR)/include

#CFLAGS AND LDFLAGS
ECAT_INCLUDE=/opt/emaster_app/include
ECAT_LIB=/opt/emaster_app/lib
EIGEN3_INCLUDE=/usr/include/eigen3

# CFLAGS_OPTIONS = -Wall -O3 -Wextra -Wpedantic -std=gnu++17 -DNDEBUG -Dlinux -mtune=native -flto -std=c++17 -shared-libgcc -fexceptions
CFLAGS_OPTIONS = -w -O2 -std=gnu++17 -Dlinux -mtune=native -std=c++17 -shared-libgcc -fexceptions
CFLAGS = $(CFLAGS_OPTIONS) -I$(GLOBAL_DIR) -I$(ROBOT_DIR) -I$(ECAT_INCLUDE) -I$(COMM_DIR) -I$(RT_POSIX_INCLUDE) -I$(IGH_INCLUDE) -I$(EIGEN3_INCLUDE) -I$(ECAT_DIR)/Device $(CFLAGS_ROS)

LDFLAG_ECAT = -L$(ECAT_LIB) -lEMasterApp
LDFLAG_RT_POSIX = -L$(RT_POSIX_LIB) -lrtposix
LDFLAGS = -lm -lrt -lpthread $(LDFLAG_ECAT) $(LDFLAG_RT_POSIX) $(LDFLAGS_ROS)

# RBDL + URDF reader (rbdl-orb). System install (/usr/local) by default;
# override RBDL_DIR to point at a staged prefix for pre-flight builds.
RBDL_DIR ?=
ifneq ($(RBDL_DIR),)
CFLAGS  += -I$(RBDL_DIR)/include
LDFLAGS += -L$(RBDL_DIR)/lib -Wl,-rpath,$(RBDL_DIR)/lib
endif
LDFLAGS += -lrbdl -lrbdl_urdfreader

# [kv260-merge] ViSP / librealsense / OpenCV / PCL removed: the ViSP visual
# servo is out of this app (camera stays with the ROS2 perception pipeline);
# CalibCapture is now ViSP-free (Eigen AngleAxisd).

# [kv260-merge] Gate 2b — ROS2 bridge to the perception pipeline.
# Pattern from App/Indy7_ROS2/Makefile with jazzy→humble and
# indy_iface→my_interfaces (ros2_ws). rpath baked in so the app runs
# without sourcing any ROS environment.
ROS_DIR=/opt/ros/humble
ROS_INCLUDE=$(ROS_DIR)/include
ROS_LIB=$(ROS_DIR)/lib
MY_IFACE_DIR=/home/ubuntu/ros2_ws/install/my_interfaces
MY_IFACE_INCLUDE=$(MY_IFACE_DIR)/include
MY_IFACE_LIB=$(MY_IFACE_DIR)/lib

# Whitelist instead of the upstream `find -maxdepth 1` shotgun: that pulls in
# CycloneDDS's include/dds and include/idl, whose features.h/string.h/endian.h
# SHADOW the glibc headers of the same name and break libstdc++ compilation
# (the upstream `-include cstdio/cstring/cstdlib` flags were a band-aid for
# exactly this). Only packages our TUs actually include headers from:
ROS_PKGS = rclcpp rcl rcutils rmw rcl_yaml_param_parser rcpputils \
           rosidl_runtime_c rosidl_runtime_cpp rosidl_typesupport_interface \
           rosidl_typesupport_cpp rosidl_typesupport_c \
           builtin_interfaces rcl_interfaces std_msgs statistics_msgs \
           libstatistics_collector tracetools libyaml_vendor
CFLAGS_ROS = -I$(MY_IFACE_INCLUDE)/my_interfaces \
             $(addprefix -I$(ROS_INCLUDE)/,$(ROS_PKGS))
LDFLAGS_ROS = -L$(MY_IFACE_LIB) \
              -lmy_interfaces__rosidl_typesupport_cpp \
              -lmy_interfaces__rosidl_typesupport_fastrtps_cpp \
              -lmy_interfaces__rosidl_typesupport_fastrtps_c \
              -lmy_interfaces__rosidl_typesupport_introspection_cpp \
              -lmy_interfaces__rosidl_typesupport_introspection_c \
              -lmy_interfaces__rosidl_typesupport_c \
              -lmy_interfaces__rosidl_generator_c \
              -L$(ROS_LIB) \
              -lrclcpp -lrcl -lrcutils -lrmw \
              -lrosidl_runtime_c -lrosidl_typesupport_c -lrosidl_typesupport_cpp \
              -lrosidl_typesupport_fastrtps_c -lrosidl_typesupport_fastrtps_cpp \
              -lstd_msgs__rosidl_generator_c \
              -lstd_msgs__rosidl_typesupport_c \
              -lstd_msgs__rosidl_typesupport_cpp \
              -lstd_msgs__rosidl_typesupport_fastrtps_c \
              -lstd_msgs__rosidl_typesupport_fastrtps_cpp \
              -lbuiltin_interfaces__rosidl_generator_c \
              -lbuiltin_interfaces__rosidl_typesupport_c \
              -lbuiltin_interfaces__rosidl_typesupport_cpp \
              -lbuiltin_interfaces__rosidl_typesupport_fastrtps_c \
              -lbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp \
              -lrcl_interfaces__rosidl_generator_c \
              -lrcl_interfaces__rosidl_typesupport_c \
              -lrcl_interfaces__rosidl_typesupport_cpp \
              -lrcl_interfaces__rosidl_typesupport_fastrtps_c \
              -lrcl_interfaces__rosidl_typesupport_fastrtps_cpp \
              -ltracetools \
              -lstatistics_msgs__rosidl_typesupport_cpp \
              -llibstatistics_collector \
              -Wl,-rpath,$(ROS_LIB) -Wl,-rpath,$(MY_IFACE_LIB)

#SOURCES
SOURCES	+= $(GLOBAL_DIR)/ConfigParser.cpp
SOURCES	+= $(COMM_DIR)/CRC16.cpp
SOURCES	+= $(COMM_DIR)/Socket.cpp
SOURCES	+= $(ROBOT_DIR)/Axis.cpp
SOURCES	+= $(ROBOT_DIR)/AxisCIA402.cpp
SOURCES	+= $(ROBOT_DIR)/AxisNRMKCore.cpp
SOURCES	+= $(ROBOT_DIR)/Sensor.cpp
SOURCES	+= $(ROBOT_DIR)/SensorFT.cpp
SOURCES	+= $(ROBOT_DIR)/SensorNRMKEndTool.cpp
SOURCES	+= $(ROBOT_DIR)/ConfigRobot.cpp
SOURCES	+= $(ROBOT_DIR)/ExtInterface.cpp
SOURCES	+= $(ROBOT_DIR)/Robot.cpp
SOURCES	+= $(CURR_DIR)/Controller.cpp
SOURCES	+= $(CURR_DIR)/FullDynControllerRT.cpp
SOURCES	+= $(CURR_DIR)/CalibCapture.cpp
SOURCES	+= $(CURR_DIR)/ROS2PickBridge.cpp
SOURCES	+= $(CURR_DIR)/Indy7Ctrl.cpp
SOURCES	+= $(CURR_DIR)/main.cpp

#OUTPUT
CC=g++
OBJECT_DIR=$(CURR_DIR)/obj
OUTPUT_DIR=$(CURR_DIR)/bin
OUTPUT_BIN=Indy7Ctrl

#INSTALL DIRECTORY
INSTALL_DIR := $(PROJ_ROOT_DIR)

#######################################################################################################
OBJECTS = $(addprefix $(OBJECT_DIR)/, $(notdir $(patsubst %.cpp, %.o, $(SOURCES))))
#######################################################################################################
vpath %.cpp  $(ROBOT_DIR) $(GLOBAL_DIR) $(COMM_DIR) $(CURR_DIR) $(HAPTIC_DIR)
#######################################################################################################

all: executable

install: executable
	@echo COPYING BIN TO $(INSTALL_DIR)/
	@ mkdir -p $(INSTALL_DIR)/bin; pwd > /dev/null
	cp -rfp $(OUTPUT_DIR)/* $(INSTALL_DIR)/bin
	
executable: $(OUTPUT_DIR)/$(OUTPUT_BIN).out
$(OUTPUT_DIR)/$(OUTPUT_BIN).out: $(OBJECTS)
	@echo -n "GENERATING EXECUTABLE..."
	@ mkdir -p $(OUTPUT_DIR); pwd > /dev/null
	$(CC) -o $(OUTPUT_DIR)/$(OUTPUT_BIN).out $(OBJECTS) $(LDFLAGS)
	@echo "DONE!"

objects: $(OBJECTS)
$(OBJECT_DIR)/%.o : %.cpp
	@echo "GENERATING SOURCE OBJECTS..."
	@mkdir -p $(OBJECT_DIR); pwd > /dev/null
	$(CC) -MD $(CFLAGS) -c -o $@ $<
	
# [kv260-merge] Gate 2b harness: bridge-only test binary (no EtherCAT/robot).
# Drives the same CROS2PickBridge the app links, from stdin commands.
gate2b_test: $(OBJECT_DIR)/ROS2PickBridge.o
	@ mkdir -p $(OUTPUT_DIR); pwd > /dev/null
	$(CC) $(CFLAGS) -I$(CURR_DIR) -o $(OUTPUT_DIR)/gate2b_bridge_test.out \
		$(PROJ_ROOT_DIR)/tools/gate2b_bridge_test.cpp \
		$(OBJECT_DIR)/ROS2PickBridge.o $(LDFLAGS)
	@echo "gate2b harness: $(OUTPUT_DIR)/gate2b_bridge_test.out"

clean:
	@echo -n "CLEANING..."
	@if [ -d $(OBJECT_DIR) ]; then rm -rf $(OBJECT_DIR); fi
	@if [ -d $(OUTPUT_DIR) ]; then rm -rf $(OUTPUT_DIR); fi
	@echo "DONE!"

.PHONY: all clean gate2b_test
##############################################################################
# Include header file dependencies generated by -MD option:
-include $(OBJECT_DIR)/*.d
