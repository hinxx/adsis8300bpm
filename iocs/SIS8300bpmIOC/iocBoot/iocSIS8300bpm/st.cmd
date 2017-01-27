< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/SIS8300bpmDemoApp.dbd")
SIS8300bpmDemoApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "BPM:")
# The port name for the detector
epicsEnvSet("PORT",   "SIS8300")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
#epicsEnvSet("XSIZE",  "12")
# The maximim image height; used for column profiles in the NDPluginStats plugin
#epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "64")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS", "600000")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("AICH0",      "AI0")
epicsEnvSet("AICH1",      "AI1")
epicsEnvSet("AICH2",      "AI2")
epicsEnvSet("AICH3",      "AI3")
epicsEnvSet("AICH4",      "AI4")
epicsEnvSet("AICH5",      "AI5")
epicsEnvSet("AICH6",      "AI6")
epicsEnvSet("AICH7",      "AI7")
epicsEnvSet("AICH8",      "AI8")
epicsEnvSet("AICH9",      "AI9")

epicsEnvSet("BPM1",       "BPM1")
epicsEnvSet("BPM2",       "BPM2")
epicsEnvSet("BPMCH1",     "XPOS")
epicsEnvSet("BPMCH2",     "YPOS")
epicsEnvSet("BPMCH3",     "MSUM")
epicsEnvSet("BPMCH4",     "PSUM")
epicsEnvSet("BPMCH5",     "AMAG")
epicsEnvSet("BPMCH6",     "BMAG")
epicsEnvSet("BPMCH7",     "CMAG")
epicsEnvSet("BPMCH8",     "DMAG")
epicsEnvSet("BPMCH9",     "APHA")
epicsEnvSet("BPMCH10",    "BPHA")
epicsEnvSet("BPMCH11",    "CPHA")
epicsEnvSet("BPMCH12",    "DPHA")

# This is sum of AI and BPM asyn addresses
# ADDR 0 .. 9 are for AI
# ADDR 10 .. 11 are for BPM1 and BPM2
epicsEnvSet("NUM_CH",        "12")
# Number of samples to acquire
epicsEnvSet("NUM_SAMPLES",   "300000")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS",      "600000")

asynSetMinTimerPeriod(0.001)

# Uncomment the following line to set it in the IOC.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "30000000")

# Create an SIS8300bpm driver
# SIS8300BpmConfig(const char *portName, const char *devicePath,
#            int maxAddr, int numSamples, NDDataType_t dataType,
#            int maxBuffers, size_t maxMemory, int priority, int stackSize)
SIS8300BpmConfig("$(PORT)", "/dev/sis8300-11", $(NUM_CH), $(NUM_SAMPLES), 7, 0, 0)
dbLoadRecords("$(ADSIS8300)/db/SIS8300.template",        "P=$(PREFIX),R=,           PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH0):,  PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(AICH0)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH1):,  PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(AICH1)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH2):,  PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(AICH2)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH3):,  PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(AICH3)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH4):,  PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(AICH4)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH5):,  PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(AICH5)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH6):,  PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(AICH6)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH7):,  PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(AICH7)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH8):,  PORT=$(PORT),ADDR=8,TIMEOUT=1,NAME=$(AICH8)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template",       "P=$(PREFIX),R=$(AICH9):,  PORT=$(PORT),ADDR=9,TIMEOUT=1,NAME=$(AICH9)")

# BPM related records
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpm.template",  "P=$(PREFIX),R=,           PORT=$(PORT),ADDR=0,TIMEOUT=1")

# BPM1 and BPM2 related records
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):,   PORT=$(PORT),ADDR=10,TIMEOUT=1,NAME=$(BPM1)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):,   PORT=$(PORT),ADDR=11,TIMEOUT=1,NAME=$(BPM2)")

# Create a standard arrays plugin, set it to get data from ADSIS8300bpm driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 1000000x10 arrays.
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=10000000")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=3000")

# Time series plugin for converted AI data
NDTimeSeriesConfigure("TS0", $(QSIZE), 0, "$(PORT)", 0, 10)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS0:,   PORT=TS0,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:0:, PORT=TS0,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH0)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:1:, PORT=TS0,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:2:, PORT=TS0,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:3:, PORT=TS0,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:4:, PORT=TS0,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:5:, PORT=TS0,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:6:, PORT=TS0,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:7:, PORT=TS0,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:8:, PORT=TS0,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS0:9:, PORT=TS0,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AICH9)")

# Time series plugin for BPM1
NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 1, 12)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS1:,   PORT=TS1,ADDR=0, TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:0:, PORT=TS1,ADDR=0, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:1:, PORT=TS1,ADDR=1, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:2:, PORT=TS1,ADDR=2, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:3:, PORT=TS1,ADDR=3, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:4:, PORT=TS1,ADDR=4, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:5:, PORT=TS1,ADDR=5, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:6:, PORT=TS1,ADDR=6, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:7:, PORT=TS1,ADDR=7, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:8:, PORT=TS1,ADDR=8, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:9:, PORT=TS1,ADDR=9, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH10)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:10:,PORT=TS1,ADDR=10,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH11)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:11:,PORT=TS1,ADDR=11,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH12)")

# Time series plugin for BPM2
NDTimeSeriesConfigure("TS2", $(QSIZE), 0, "$(PORT)", 2, 12)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS2:,   PORT=TS2,ADDR=0, TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:0:, PORT=TS2,ADDR=0, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:1:, PORT=TS2,ADDR=1, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:2:, PORT=TS2,ADDR=2, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:3:, PORT=TS2,ADDR=3, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:4:, PORT=TS2,ADDR=4, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:5:, PORT=TS2,ADDR=5, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:6:, PORT=TS2,ADDR=6, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:7:, PORT=TS2,ADDR=7, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:8:, PORT=TS2,ADDR=8, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:9:, PORT=TS2,ADDR=9, TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH10)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:10:,PORT=TS2,ADDR=10,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH11)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:11:,PORT=TS2,ADDR=11,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH12)")

# FFT plugins
NDFFTConfigure("FFT0", $(QSIZE), 0, "TS0", 0)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT0:,PORT=FFT0,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH0)")
NDFFTConfigure("FFT1", $(QSIZE), 0, "TS0", 1)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT1:,PORT=FFT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH1)")
NDFFTConfigure("FFT2", $(QSIZE), 0, "TS0", 2
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT2:,PORT=FFT2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH2)")
NDFFTConfigure("FFT3", $(QSIZE), 0, "TS0", 3)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT3:,PORT=FFT3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=3,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH3)")
NDFFTConfigure("FFT4", $(QSIZE), 0, "TS0", 4)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT4:,PORT=FFT4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=4,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH4)")
NDFFTConfigure("FFT5", $(QSIZE), 0, "TS0", 5)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT5:,PORT=FFT5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=5,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH5)")
NDFFTConfigure("FFT6", $(QSIZE), 0, "TS0", 6)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT6:,PORT=FFT6,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=6,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH6)")
NDFFTConfigure("FFT7", $(QSIZE), 0, "TS0", 7)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT7:,PORT=FFT7,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=7,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH7)")
NDFFTConfigure("FFT8", $(QSIZE), 0, "TS0", 8)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT8:,PORT=FFT8,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=8,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH8)")
NDFFTConfigure("FFT9", $(QSIZE), 0, "TS0", 9)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=FFT9:,PORT=FFT9,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS0,NDARRAY_ADDR=9,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(AICH9)")


# FFT plugins
NDFFTConfigure("$(BPM1)FFT0", $(QSIZE), 0, "TS1", 0)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT0:,PORT=$(BPM1)FFT0,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH1)")
NDFFTConfigure("$(BPM1)FFT1", $(QSIZE), 0, "TS1", 1)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT1:,PORT=$(BPM1)FFT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=1,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH2)")
NDFFTConfigure("$(BPM1)FFT2", $(QSIZE), 0, "TS1", 2
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT2:,PORT=$(BPM1)FFT2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH3)")
NDFFTConfigure("$(BPM1)FFT3", $(QSIZE), 0, "TS1", 3)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT3:,PORT=$(BPM1)FFT3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=3,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH4)")
NDFFTConfigure("$(BPM1)FFT4", $(QSIZE), 0, "TS1", 4)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT4:,PORT=$(BPM1)FFT4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=4,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH5)")
NDFFTConfigure("$(BPM1)FFT5", $(QSIZE), 0, "TS1", 5)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT5:,PORT=$(BPM1)FFT5,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=5,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH6)")
NDFFTConfigure("$(BPM1)FFT6", $(QSIZE), 0, "TS1", 6)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT6:,PORT=$(BPM1)FFT6,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=6,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH7)")
NDFFTConfigure("$(BPM1)FFT7", $(QSIZE), 0, "TS1", 7)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT7:,PORT=$(BPM1)FFT7,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=7,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH8)")
NDFFTConfigure("$(BPM1)FFT8", $(QSIZE), 0, "TS1", 8)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT8:,PORT=$(BPM1)FFT8,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=8,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH9)")
NDFFTConfigure("$(BPM1)FFT9", $(QSIZE), 0, "TS1", 9)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT9:,PORT=$(BPM1)FFT9,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=9,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH10)")
NDFFTConfigure("$(BPM1)FFT10", $(QSIZE), 0, "TS1", 10)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT10:,PORT=$(BPM1)FFT10,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=10,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH11)")
NDFFTConfigure("$(BPM1)FFT11", $(QSIZE), 0, "TS1", 11)
dbLoadRecords("$(ADCORE)/db/NDFFT.template","P=$(PREFIX),R=$(BPM1):FFT11:,PORT=$(BPM1)FFT11,ADDR=0,TIMEOUT=1,NDARRAY_PORT=TS1,NDARRAY_ADDR=11,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TS:TSAveragingTime_RBV CP MS,ENABLED=1,NAME=$(BPM1):$(BPMCH12)")



# Timing MTCA EVR 300
# As per EVR MTCA 300 engineering manual ch 5.3.5
epicsEnvSet("SYS"               "EVR")
epicsEnvSet("DEVICE"            "MTCA")
epicsEnvSet("EVR_PCIDOMAIN"     "0x0")
epicsEnvSet("EVR_PCIBUS"        "0x6")
epicsEnvSet("EVR_PCIDEVICE"     "0x0")
epicsEnvSet("EVR_PCIFUNCTION"   "0x0")

#require mrfioc2,2.7.13
mrmEvrSetupPCI($(DEVICE), $(EVR_PCIDOMAIN), $(EVR_PCIBUS), $(EVR_PCIDEVICE), $(EVR_PCIFUNCTION))
dbLoadRecords("$(MRFIOC2)/db/evr-mtca-300.db", "DEVICE=$(DEVICE), SYS=$(SYS), Link-Clk-SP=88.0525")

# NOT USED ON BPM!
# PULSE_COMING_EVENT = 1
#dbLoadRecords("$(MRFIOC2)/db/evr-softEvent.template", "DEVICE=$(DEVICE), SYS=$(SYS), EVT=1, CODE=14")
# MLVDS 0 (RearUniv32)
#dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=0, F=Trig, ID=0, EVT=1")
# MLVDS 4 (RearUniv36)
#dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=4, F=Trig, ID=1, EVT=2")

# PULSE_START_EVENT = 2
dbLoadRecords("$(MRFIOC2)/db/evr-softEvent.template", "DEVICE=$(DEVICE), SYS=$(SYS), EVT=2, CODE=14")
# MLVDS 1 (RearUniv33)
dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=1, F=Trig, ID=0, EVT=2")
# MLVDS 5 (RearUniv37)
#dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=5, F=Trig, ID=1, EVT=2")

# PULSE_STOP_EVENT = 3
dbLoadRecords("$(MRFIOC2)/db/evr-softEvent.template", "DEVICE=$(DEVICE), SYS=$(SYS), EVT=3, CODE=14")
# MLVDS 2 (RearUniv34)
dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=2, F=Trig, ID=0, EVT=3")
# MLVDS 6 (RearUniv38)
#dbLoadRecords("$(MRFIOC2)/db/evr-pulserMap.template", "DEVICE=$(DEVICE), SYS=$(SYS), PID=6, F=Trig, ID=1, EVT=3")

set_requestfile_path("$(ADSIS8300)/SIS8300App/Db")
set_requestfile_path("$(ADSIS8300BPM)/SIS8300bpmApp/Db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()


# Set some defaults for BPM
dbpf $(PREFIX)ClockSource 2
dbpf $(PREFIX)ClockDiv 1
dbpf $(PREFIX)ClockFreq 88052500
dbpf $(PREFIX)TrigSetup 0
dbpf $(PREFIX)RTMType 2
dbpf $(PREFIX)RTMTempGet 1
dbpf $(PREFIX)Enable 1

# No conversion is made for BPM data
dbpf $(PREFIX)$(AICH0):ConvFactor 1
dbpf $(PREFIX)$(AICH1):ConvFactor 1
dbpf $(PREFIX)$(AICH2):ConvFactor 1
dbpf $(PREFIX)$(AICH3):ConvFactor 1
dbpf $(PREFIX)$(AICH4):ConvFactor 1
dbpf $(PREFIX)$(AICH5):ConvFactor 1
dbpf $(PREFIX)$(AICH6):ConvFactor 1
dbpf $(PREFIX)$(AICH7):ConvFactor 1
dbpf $(PREFIX)$(AICH8):ConvFactor 1
dbpf $(PREFIX)$(AICH9):ConvFactor 1

dbpf $(PREFIX)$(AICH0):ConvOffset 0
dbpf $(PREFIX)$(AICH1):ConvOffset 0
dbpf $(PREFIX)$(AICH2):ConvOffset 0
dbpf $(PREFIX)$(AICH3):ConvOffset 0
dbpf $(PREFIX)$(AICH4):ConvOffset 0
dbpf $(PREFIX)$(AICH5):ConvOffset 0
dbpf $(PREFIX)$(AICH6):ConvOffset 0
dbpf $(PREFIX)$(AICH7):ConvOffset 0
dbpf $(PREFIX)$(AICH8):ConvOffset 0
dbpf $(PREFIX)$(AICH9):ConvOffset 0


# Disable Rear Universal Output 32
#dbpf $(SYS)-$(DEVICE):RearUniv32-Ena-SP "Disabled"
# Map Rear Universal Output 32 to pulser 0
#dbpf $(SYS)-$(DEVICE):RearUniv32-Src-SP 0
# Map pulser 0 to event 14
#dbpf $(SYS)-$(DEVICE):Pul0-Evt-Trig0-SP 14
# Set pulser 1 width to 1 us
#dbpf $(SYS)-$(DEVICE):Pul0-Width-SP 100
# event 1 received the SIS8300 will announce pulse
#dbpf $(SYS)-$(DEVICE):RearUniv32-Ena-SP "Enabled"

# Disable Rear Universal Output 33
dbpf $(SYS)-$(DEVICE):RearUniv33-Ena-SP "Disabled"
# Map Rear Universal Output 33 to pulser 1
dbpf $(SYS)-$(DEVICE):RearUniv33-Src-SP 1
# Map pulser 1 to event 14
dbpf $(SYS)-$(DEVICE):Pul1-Evt-Trig0-SP 14
# Set pulser 1 width to 1 us
dbpf $(SYS)-$(DEVICE):Pul1-Width-SP 100
# Set the delay time of the pulser 1 to 0.3 ms
#dbpf $(SYS)-$(DEVICE):Pul1-Delay-SP 300
# event 2 received the SIS8300 will start the data acquisition
dbpf $(SYS)-$(DEVICE):RearUniv33-Ena-SP "Enabled"

# Disable Rear Universal Output 34
dbpf $(SYS)-$(DEVICE):RearUniv34-Ena-SP "Disabled"
# Map Rear Universal Output 34 to pulser 2
dbpf $(SYS)-$(DEVICE):RearUniv34-Src-SP 2
# Map pulser 2 to event 14
dbpf $(SYS)-$(DEVICE):Pul2-Evt-Trig0-SP 14
# Set pulser 2 width to 1 us
dbpf $(SYS)-$(DEVICE):Pul2-Width-SP 100
# Set the delay time of the pulser 2 to 3.16 ms (pulse width of 2.86 ms)
dbpf $(SYS)-$(DEVICE):Pul2-Delay-SP 2860
# event 3 received the SIS8300 will stop the data acquisition
dbpf $(SYS)-$(DEVICE):RearUniv34-Ena-SP "Enabled"
