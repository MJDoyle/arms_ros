# Webots Makefile for supervisor_controller
TARGET = supervisor_controller
SOURCES = supervisor_controller.cpp

# ROS2 setup
ROS_DISTRO = jazzy
ROS2_INCLUDE = /opt/ros/$(ROS_DISTRO)/include
ROS2_LIB = /opt/ros/$(ROS_DISTRO)/lib

# ROS2 libraries - complete set
ROS2_LIBS = -lrclcpp \
           -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_cpp \
           -lgeometry_msgs__rosidl_generator_c -lgeometry_msgs__rosidl_typesupport_cpp \
           -lrcl_interfaces__rosidl_generator_c -lrcl_interfaces__rosidl_typesupport_cpp \
           -lbuiltin_interfaces__rosidl_generator_c -lbuiltin_interfaces__rosidl_typesupport_cpp \
           -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c \
           -lrcutils -lrmw -lrcl

# Compiler and linker flags with complete include paths
CXX_SOURCES = $(SOURCES)
CXXFLAGS = -std=c++17 \
           -I$(ROS2_INCLUDE) \
           -I$(ROS2_INCLUDE)/rclcpp \
           -I$(ROS2_INCLUDE)/std_msgs \
           -I$(ROS2_INCLUDE)/geometry_msgs \
           -I$(ROS2_INCLUDE)/rcl_interfaces \
           -I$(ROS2_INCLUDE)/builtin_interfaces \
           -I$(ROS2_INCLUDE)/rosidl_runtime_c \
           -I$(ROS2_INCLUDE)/rosidl_typesupport_cpp

LDFLAGS = -L$(ROS2_LIB) $(ROS2_LIBS)

CXXFLAGS += -I/opt/ros/jazzy/include
LDFLAGS += -L/opt/ros/jazzy/lib -lrclcpp -lrcutils

# Find WEBOTS_HOME if not set
ifndef WEBOTS_HOME
    # First, try to find Makefile.include directly
    MAKEFILE_LOCATIONS := $(shell find /usr/local /opt /snap -name "Makefile.include" -path "*/webots/*" 2>/dev/null | head -1)
    
    ifneq ($(MAKEFILE_LOCATIONS),)
        # Extract the webots home directory (two levels up from resources/Makefile.include)
        WEBOTS_HOME := $(shell dirname $(shell dirname $(MAKEFILE_LOCATIONS)))
        $(info Found Webots at: $(WEBOTS_HOME))
    else
        # Fallback: try standard locations
        STANDARD_LOCATIONS := /usr/local/webots /opt/webots /snap/webots/current
        WEBOTS_HOME := $(foreach dir,$(STANDARD_LOCATIONS),$(shell test -f $(dir)/resources/Makefile.include && echo $(dir)))
        WEBOTS_HOME := $(firstword $(WEBOTS_HOME))
        
        ifeq ($(WEBOTS_HOME),)
            $(error Cannot find Webots installation. Please set WEBOTS_HOME manually: export WEBOTS_HOME=/path/to/webots)
        endif
    endif
endif

# Clean up WEBOTS_HOME (remove any trailing slashes and resolve)
WEBOTS_HOME := $(shell realpath $(WEBOTS_HOME) 2>/dev/null || echo $(WEBOTS_HOME))
WEBOTS_HOME := $(patsubst %/,%,$(WEBOTS_HOME))

# Verify WEBOTS_HOME
MAKEFILE_PATH := $(WEBOTS_HOME)/resources/Makefile.include
ifeq ($(wildcard $(MAKEFILE_PATH)),)
    $(error WEBOTS_HOME=$(WEBOTS_HOME) is invalid. Cannot find $(MAKEFILE_PATH))
endif

# Show what we found
$(info Using WEBOTS_HOME: $(WEBOTS_HOME))
$(info Makefile.include found at: $(MAKEFILE_PATH))

# Include Webots Makefile
include $(MAKEFILE_PATH)