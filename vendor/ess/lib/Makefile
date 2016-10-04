TOP=../../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

USR_INCLUDES += $(ADSIS8300)/include

INC += sis8300drvbpm.h  sis8300drvbpm_types.h  sis8300bpm_reg.h

LIBRARY_IOC = sis8300drvbpm

LIB_SYS_LIBS += 
LIB_LIBS += sis8300drv

sis8300drvbpm_SRCS += sis8300bpm_acquisition.c  sis8300bpm_general.c \
sis8300drvbpm.c
        
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

