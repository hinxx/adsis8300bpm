#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += bpmApp
DIRS += vendor
bpmApp_DEPEND_DIRS += vendor

# do not build IOCs
#ifeq ($(BUILD_IOCS), YES)
#DIRS += bpmDemoApp
#bpmDemoApp_DEPEND_DIRS += bpmApp
#iocBoot_DEPEND_DIRS += bpmDemoApp
#DIRS += iocBoot
#endif

include $(TOP)/configure/RULES_TOP
