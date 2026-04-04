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

# Mujoco DIR
MUJOCO_DIR=/home/raimlab/dev_env/mujoco-2.3.7
MUJOCO_INCLUDE=-I$(MUJOCO_DIR)/include
MUJOCO_LIB=-L$(MUJOCO_DIR)/lib -lmujoco


# ROS2 INDY IFACE
ROS_DIR=/opt/ros/humble
ROS_INCLUDE=$(ROS_DIR)/include
ROS_LIB=$(ROS_DIR)/lib

INDY_IFACE_DIR=/home/raimlab/ros_ws/install/indy_iface
INDY_IFACE_INCLUDE=$(INDY_IFACE_DIR)/include
INDY_IFACE_LIB=$(INDY_IFACE_DIR)/lib

# CFLAGS_ROS= -I$(INDY_IFACE_INCLUDE)/indy_iface -I$(ROS_INCLUDE) -I$(ROS_INCLUDE)/rclcpp -I$(ROS_INCLUDE)/std_msgs -I$(ROS_INCLUDE)/rcl -I$(ROS_INCLUDE)/rcutils -I$(ROS_INCLUDE)/rmw -I$(ROS_INCLUDE)/rcl_yaml_param_parser -I$(ROS_INCLUDE)/rosidl_runtime_c -I$(ROS_INCLUDE)/rosidl_typesupport_interface -I$(ROS_INCLUDE)/rcpputils -I$(ROS_INCLUDE)/builtin_interfaces -I$(ROS_INCLUDE)/rosidl_runtime_cpp -I$(ROS_INCLUDE)/tracetools -I$(ROS_INCLUDE)/rcl_interfaces -I$(ROS_INCLUDE)/libstatistics_collector -I$(ROS_INCLUDE)/statistics_msgs -I$(ROS_LIB)
CFLAGS_ROS = -I$(INDY_IFACE_INCLUDE)/indy_iface \
             -I$(ROS_INCLUDE) \
             -I$(ROS_INCLUDE)/rclcpp \
             -I$(ROS_INCLUDE)/std_msgs \
             -I$(ROS_INCLUDE)/rcl \
             -I$(ROS_INCLUDE)/rcutils \
             -I$(ROS_INCLUDE)/rmw \
             -I$(ROS_INCLUDE)/rcl_yaml_param_parser \
             -I$(ROS_INCLUDE)/rosidl_runtime_c \
             -I$(ROS_INCLUDE)/rosidl_typesupport_interface \
             -I$(ROS_INCLUDE)/rcpputils \
             -I$(ROS_INCLUDE)/builtin_interfaces \
             -I$(ROS_INCLUDE)/rosidl_runtime_cpp \
             -I$(ROS_INCLUDE)/tracetools \
             -I$(ROS_INCLUDE)/rcl_interfaces \
             -I$(ROS_INCLUDE)/libstatistics_collector \
             -I$(ROS_INCLUDE)/statistics_msgs

# LDFLAGS_ROS= -L$(INDY_IFACE_LIB) -lindy_iface__rosidl_generator_c -lindy_iface__rosidl_typesupport_fastrtps_cpp -lindy_iface__rosidl_generator_py -lindy_iface__rosidl_typesupport_fastrtps_c -lindy_iface__rosidl_typesupport_cpp -lindy_iface__rosidl_typesupport_introspection_cpp -lindy_iface__rosidl_typesupport_c -lindy_iface__rosidl_typesupport_introspection_c  -L$(ROS_LIB) -lstatistics_msgs__rosidl_typesupport_cpp -llibstatistics_collector -lrmw -lrclcpp -lrcutils -lrcl -ltracetools -lstd_msgs__rosidl_typesupport_cpp 
LDFLAGS_ROS = -L$(INDY_IFACE_LIB) \
              -lindy_iface__rosidl_generator_c \
              -lindy_iface__rosidl_typesupport_fastrtps_cpp \
              -lindy_iface__rosidl_typesupport_fastrtps_c \
              -lindy_iface__rosidl_typesupport_cpp \
              -lindy_iface__rosidl_typesupport_introspection_cpp \
              -lindy_iface__rosidl_typesupport_c \
              -lindy_iface__rosidl_typesupport_introspection_c \
              -L$(ROS_LIB) \
              -lrclcpp \
              -lrcl \
              -lrcutils \
              -lrmw \
              -lrosidl_runtime_c \
              -lrosidl_typesupport_c \
              -lrosidl_typesupport_cpp \
              -lrosidl_typesupport_fastrtps_c \
              -lrosidl_typesupport_fastrtps_cpp \
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
              -ltracetools \
              -lstatistics_msgs__rosidl_typesupport_cpp \
              -llibstatistics_collector

# CFLAGS_OPTIONS = -Wall -O3 -Wextra -Wpedantic -std=gnu++17 -DNDEBUG -Dlinux -mtune=native -flto -std=c++17 -shared-libgcc -fexceptions
# CFLAGS_OPTIONS = -w -O3 -std=gnu++17 -DNDEBUG -Dlinux -mtune=native -flto -std=c++17 -shared-libgcc -fexceptions
CFLAGS_OPTIONS = -w -O3 -std=gnu++17 -DNDEBUG -Dlinux -mtune=native -flto -std=c++17 -shared-libgcc -fexceptions -D_GNU_SOURCE -include cstdio -include cstring -include cstdlib
# CFLAGS_OPTIONS = -w -O3 -std=gnu++17 -DNDEBUG -Dlinux -mtune=native -flto -std=c++17 -shared-libgcc -fexceptions -D_GNU_SOURCE
CFLAGS = $(CFLAGS_OPTIONS) -I$(GLOBAL_DIR) -I$(ROBOT_DIR) -I$(ECAT_INCLUDE) -I$(COMM_DIR) -I$(RT_POSIX_INCLUDE) -I$(IGH_INCLUDE) -I$(EIGEN3_INCLUDE) $(CFLAGS_ROS)
CFLAGS += -I/usr/include -I/usr/include/c++/11 $(MUJOCO_INCLUDE) -DMUJOCO_ENABLED

LDFLAG_ECAT = -L$(ECAT_LIB) -lEMasterApp
LDFLAG_RT_POSIX = -L$(RT_POSIX_LIB) -lrtposix
LDFLAGS = -lm -lrt -lpthread $(LDFLAG_ECAT) $(LDFLAG_RT_POSIX) $(LDFLAGS_ROS) $(ROS2_LIBS) $(MUJOCO_LIB)

# add RBDL and URDF Reader
LDFLAGS += -lrbdl -lrbdl_urdfreader -lglfw -lGL -ldl

#SOURCES
SOURCES	+= $(GLOBAL_DIR)/ConfigParser.cpp
SOURCES	+= $(COMM_DIR)/CRC16.cpp
SOURCES	+= $(COMM_DIR)/Socket.cpp
SOURCES	+= $(COMM_DIR)/ROS2Node.cpp
SOURCES	+= $(COMM_DIR)/ROS2Executor.cpp
SOURCES	+= $(COMM_DIR)/ROS2IndyIface.cpp
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
SOURCES	+= $(CURR_DIR)/Indy7Ctrl.cpp
SOURCES += $(ROBOT_DIR)/AxisRT.cpp
SOURCES += $(ROBOT_DIR)/AxisRTMuJoCo.cpp
SOURCES += $(ROBOT_DIR)/MuJoCoVisualization.cpp
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
	
clean:	
	@echo -n "CLEANING..."
	@if [ -d $(OBJECT_DIR) ]; then rm -rf $(OBJECT_DIR); fi
	@if [ -d $(OUTPUT_DIR) ]; then rm -rf $(OUTPUT_DIR); fi
	@echo "DONE!"
	
.PHONY: all clean 
##############################################################################
# Include header file dependencies generated by -MD option:
-include $(OBJECT_DIR)/*.d
