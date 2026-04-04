##############################################################################
#	Name: Makefile
#	Author: Raimarius Delgado (Post-Doc)
#	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
#	Description: Common makefile to build the entire project
#	Copyright: ROBOGRAM LAB (2022)
##############################################################################
PROJ_ROOT_DIR=.
ROBOT_DIR=$(PROJ_ROOT_DIR)/CRobot
CONFIG_DIR=$(PROJ_ROOT_DIR)/Config
ECAT_DIR=$(PROJ_ROOT_DIR)/EMasterApp
PI_CONTROLLER_DIR=$(PROJ_ROOT_DIR)/PI_GCS2
EXAMPLES_DIR=$(PROJ_ROOT_DIR)/Examples
USURGERY_ROBOT_DIR=$(EXAMPLES_DIR)/SuperMicroSurgery


DESTDIR ?= /opt
INSTALL_DIR = $(DESTDIR)/emaster_app

MKDIR = /bin/mkdir
ECHO = echo
CHMOD = /bin/chmod
RM = /bin/rm

#all: library_ecat examples
all: library_ecat


install: library_ecat
	@echo Creating installation directory at $(INSTALL_DIR)/
	@$(MKDIR) -p $(INSTALL_DIR); pwd > /dev/null

	@echo Copying headers to $(INSTALL_DIR)/include/
	@$(MKDIR) -p $(INSTALL_DIR)/include/; pwd > /dev/null
	cp -rfp $(PROJ_ROOT_DIR)/include/EMasterApp/* $(INSTALL_DIR)/include/

	@echo Copying library to $(INSTALL_DIR)/lib/EMasterApp
	@$(MKDIR) -p $(INSTALL_DIR)/lib/; pwd > /dev/null
	cp -rfp $(PROJ_ROOT_DIR)/lib/EMasterApp/* $(INSTALL_DIR)/lib

	@echo DONE!

examples: library_ecat
	cd $(USURGERY_ROBOT_DIR)/ && make install
	@if [ -d $(PROJ_ROOT_DIR)/bin ]; then cp $(CONFIG_DIR)/* $(PROJ_ROOT_DIR)/bin; fi
	
library_ecat: 
	cd $(ECAT_DIR)/ && make install
	
clean:
	@if [ -d $(PROJ_ROOT_DIR)/include ]; then rm -rf $(PROJ_ROOT_DIR)/include; fi
	@if [ -d $(PROJ_ROOT_DIR)/lib ]; then rm -rf $(PROJ_ROOT_DIR)/lib; fi
	@if [ -d $(PROJ_ROOT_DIR)/bin ]; then rm -rf $(PROJ_ROOT_DIR)/bin; fi
	
distclean: clean
	cd $(ECAT_DIR)/ && make clean
	cd $(USURGERY_ROBOT_DIR)/ && make clean
	
re_hard:
	make distclean
	make all

re:
	make clean
	make all
	
.PHONY: all clean 
##############################################################################
