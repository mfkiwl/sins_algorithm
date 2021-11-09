#################################################################
#  Black Vision Makefile
# 
#  if build bvmoverrobot debug mode,use  make ver=DEBUG
#  this mode for GDB debug
#
#  if build bvmoverrobot release mode,use  make ver=RELEASE
#  this mode close all bv_logx for release
#  
#  if build bvmoverrobot test mode,use  make
# 
#################################################################
SHELL = /bin/bash

TAR := $(shell ls slam_robot* 2>/dev/null)

#source file
SRC_DIR = ./code
INC_DIR = .

INC_H = $(shell find $(INC_DIR) -name '*.h')
INC_H += $(shell find $(INC_DIR) -name '*.hpp')

TEMP_INC_H_DIR += $(dir $(INC_H))

TEMP_INC_H_DIR += ./inc

INC_H_DIR = $(sort $(TEMP_INC_H_DIR))
INC_H_DIR_I += $(foreach temp, $(INC_H_DIR), -I$(temp))

SRC = $(shell find $(SRC_DIR) -name '*.c')
SRC += $(shell find $(SRC_DIR) -name '*.cpp')

OBJS := $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SRC)))

###############################################################################
ifeq ($(ver),DEBUG)
	TARGET  := sins_algorithm-debug
else
	TARGET  := sins_algorithm 
endif
###############################################################################

#compile and lib parameter
CC      := arm-linux-gcc
CXX     := arm-linux-g++
LIBS    := -lm -lpthread
SLAM_LIBS:= -L./lib #-lceres -lg2o_core -lg2o_csparse_extension -lg2o_ext_csparse -lg2o_solver_csparse -lg2o_stuff 
OPENCV_LIB :=  #-L./lib #-lopencv_world 
OPENCV_INC :=  #-I./inc/#opencv 
PCL_INC := #-I./inc/#PCL
ENGEN3_INC :=  #-I./inc/eigen3


LIBS += $(SLAM_LIBS) $(OPENCV_LIB) -lrt -ldl
 
LDFLAGS := -Wl,--as-needed


ifeq ($(ver),DEBUG)
	DEFINES := -w -g -O0  -Wl,-Map
else 
	DEFINES := -W -O2
endif
###############################################################################
INCLUDE :=  $(INC_H_DIR_I) $(OPENCV_INC) $(PCL_INC) $(ENGEN3_INC)

CFLAGS  += $(DEFINES) $(INCLUDE)
CXXFLAGS:= -std=c++0x  $(CFLAGS)

.PHONY: all help clean cleanall

all:$(TARGET)
	@echo "============CC:$(CC)   CXX:$(CXX)============"

$(TARGET):$(OBJS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS)   -o $@  $^ $(LIBS)
	@echo -e "\033[31m\033[1m make $@ done. \033[0m";#red
	
%.o:%.c
	$(CXX) $(CXXFLAGS) -c $< -o $@
	@echo -e "\033[32m\033[1m make $@ done. \033[0m";#green
	
%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
	@echo -e "\033[32m\033[1m make $@ done. \033[0m";#green
		
clean:
	@rm -f $(TARGET)
	@rm -f slam_robot*
	@echo -e "\033[32m\033[1m remove $(TAR) done. \033[0m";#green
	
cleanall:
	@rm -f $(TARGET)
	@rm -f slam_robot*
	@rm -f $(OBJS) *.map
	@echo -e "\033[32m\033[1m remove $(TAR) done. \033[0m"; 	#green
	@echo -e "\033[32m\033[1m remove $(OBJS) done. \033[0m"; 	#green

help:
	@echo -e "\033[32m\033[1m \tmake ver=DEBUG:\t\t Debug版本，具备debug log信息\033[0m";   	#green
	@echo -e "\033[32m\033[1m \tmake :\t\t\t 不加任何参数，直接make， Release版本，发布版本，仅具备error log信息，所有文件log均为加密形式\033[0m";   	#green
	@echo -e "";
