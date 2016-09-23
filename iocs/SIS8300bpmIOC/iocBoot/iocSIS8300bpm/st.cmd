< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/SIS8300bpmDemoApp.dbd")
SIS8300bpmDemoApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "SIS8300BPM:")
# The port name for the detector
epicsEnvSet("PORT",   "SIS8300BPM")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "10")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2000")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("T1", "AI1")
epicsEnvSet("T2", "AI2")
epicsEnvSet("T3", "AI3")
epicsEnvSet("T4", "AI4")
epicsEnvSet("T5", "AI5")
epicsEnvSet("T6", "AI6")
epicsEnvSet("T7", "AI7")
epicsEnvSet("T8", "AI8")
epicsEnvSet("T9", "AI9")
epicsEnvSet("T10", "AI10")

asynSetMinTimerPeriod(0.001)

# The EPICS environment variable EPICS_CA_MAX_ARRAY_BYTES needs to be set to a value at least as large
# as the largest image that the standard arrays plugin will send.
# That vlaue is $(XSIZE) * $(YSIZE) * sizeof(FTVL data type) for the FTVL used when loading the NDStdArrays.template file.
# The variable can be set in the environment before running the IOC or it can be set here.
# It is often convenient to set it in the environment outside the IOC to the largest array any client 
# or server will need.  For example 10000000 (ten million) bytes may be enough.
# If it is set here then remember to also set it outside the IOC for any CA clients that need to access the waveform record.  
# Do not set EPICS_CA_MAX_ARRAY_BYTES to a value much larger than that required, because EPICS Channel Access actually
# allocates arrays of this size every time it needs a buffer larger than 16K.
# Uncomment the following line to set it in the IOC.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

# Create an ADCimDetector driver
# SIS8300bpmConfig(const char *portName, const char *devicePath, int numTimePoints, int dataType,
#                      int maxBuffers, int maxMemory, int priority, int stackSize)
SIS8300BpmConfig("$(PORT)", "/dev/sis8300-1", $(YSIZE), 7, 0, 0)
dbLoadRecords("$(ADSIS8300)/db/SIS8300.template",  "P=$(PREFIX),R=,  PORT=$(PORT),ADDR=0,TIMEOUT=1,AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(T1),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI2:,PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(T2),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI3:,PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(T3),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI4:,PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(T4),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI5:,PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(T5),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI6:,PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(T6),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI7:,PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(T7),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI8:,PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(T8),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI9:,PORT=$(PORT),ADDR=8,TIMEOUT=1,NAME=$(T9),AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=AI10:,PORT=$(PORT),ADDR=9,TIMEOUT=1,NAME=$(T10),AINELM=$(YSIZE)")

dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpm.template",  "P=$(PREFIX),R=,  PORT=$(PORT),ADDR=0,TIMEOUT=1,AINELM=$(YSIZE)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=BPM1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(T1),AINELM=$(YSIZE)")

# Create a standard arrays plugin, set it to get data from SIS8300bpm driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 100000x10 arrays.
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1000000")

# Time series plugin
NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 0, 10)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS:,   PORT=TS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TimeStep CP MS,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:1:, PORT=TS1,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:2:, PORT=TS1,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:3:, PORT=TS1,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:4:, PORT=TS1,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:5:, PORT=TS1,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:6:, PORT=TS1,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:7:, PORT=TS1,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:8:, PORT=TS1,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:9:, PORT=TS1,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:10:, PORT=TS1,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T10)")

# FFT plugins
NDFFTConfigure("FFT1", $(QSIZE), 0, "TS1", 0)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT1:,PORT=FFT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T1)")
NDFFTConfigure("FFT2", $(QSIZE), 0, "TS1", 1)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT2:,PORT=FFT2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T2)")
NDFFTConfigure("FFT3", $(QSIZE), 0, "TS1", 2
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT3:,PORT=FFT3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T3)")
NDFFTConfigure("FFT4", $(QSIZE), 0, "TS1", 3)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT4:,PORT=FFT4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=3,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T4)")
NDFFTConfigure("FFT5", $(QSIZE), 0, "TS1", 4)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT5:,PORT=FFT5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=4,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T5)")
NDFFTConfigure("FFT6", $(QSIZE), 0, "TS1", 5)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT6:,PORT=FFT6,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=5,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T6)")
NDFFTConfigure("FFT7", $(QSIZE), 0, "TS1", 6)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT7:,PORT=FFT7,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=6,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T7)")
NDFFTConfigure("FFT8", $(QSIZE), 0, "TS1", 7)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT8:,PORT=FFT8,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=7,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T8)")
NDFFTConfigure("FFT9", $(QSIZE), 0, "TS1", 8)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT9:,PORT=FFT9,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=8,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T9)")
NDFFTConfigure("FFT10", $(QSIZE), 0, "TS1", 9)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT10:,PORT=FFT10,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=9,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(T10)")

## Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADSIS8300)/SIS8300App/Db")
set_requestfile_path("$(ADSIS8300BPM)/SIS8300bpmApp/Db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")
