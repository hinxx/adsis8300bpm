< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/SIS8300bpmDemoApp.dbd")
SIS8300bpmDemoApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "SIS8300:")
# The port name for the detector
epicsEnvSet("PORT",   "SIS8300")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "24")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "64")
# The maximum number of time series points in the NDPluginTimeSeries plugin
epicsEnvSet("TSPOINTS", "600000")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("AI1",  "AI1")
epicsEnvSet("AI2",  "AI2")
epicsEnvSet("AI3",  "AI3")
epicsEnvSet("AI4",  "AI4")
epicsEnvSet("AI5",  "AI5")
epicsEnvSet("AI6",  "AI6")
epicsEnvSet("AI7",  "AI7")
epicsEnvSet("AI8",  "AI8")
epicsEnvSet("AI9",  "AI9")
epicsEnvSet("AI10", "AI10")

epicsEnvSet("BPM1",    "BPM1")
epicsEnvSet("BPM2",    "BPM2")

epicsEnvSet("BPMCH1",  "XPOS")
epicsEnvSet("BPMCH2",  "YPOS")
epicsEnvSet("BPMCH3",  "MSUM")
epicsEnvSet("BPMCH4",  "PSUM")
epicsEnvSet("BPMCH5",  "AMAG")
epicsEnvSet("BPMCH6",  "BMAG")
epicsEnvSet("BPMCH7",  "CMAG")
epicsEnvSet("BPMCH8",  "DMAG")
epicsEnvSet("BPMCH9",  "APHA")
epicsEnvSet("BPMCH10", "BPHA")
epicsEnvSet("BPMCH11", "CPHA")
epicsEnvSet("BPMCH12", "DPHA")

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
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "30000000")

# Create an SIS8300bpm driver
# SIS8300BpmConfig(const char *portName, const char *devicePath,
#            int maxAddr, int numParams, int numTimePoints, NDDataType_t dataType,
#            int maxBuffers, size_t maxMemory, int priority, int stackSize)
SIS8300BpmConfig("$(PORT)", "/dev/sis8300-11", $(XSIZE), $(YSIZE), 7, 0, 0)
dbLoadRecords("$(ADSIS8300)/db/SIS8300.template",  "P=$(PREFIX),R=,        PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI1):, PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(AI1)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI2):, PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(AI2)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI3):, PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(AI3)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI4):, PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(AI4)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI5):, PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(AI5)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI6):, PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(AI6)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI7):, PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(AI7)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI8):, PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(AI8)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI9):, PORT=$(PORT),ADDR=8,TIMEOUT=1,NAME=$(AI9)")
dbLoadRecords("$(ADSIS8300)/db/SIS8300N.template", "P=$(PREFIX),R=$(AI10):,PORT=$(PORT),ADDR=9,TIMEOUT=1,NAME=$(AI10)")

# BPM related records
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpm.template",  "P=$(PREFIX),R=,                   PORT=$(PORT),ADDR=0,TIMEOUT=1,BPM1_ADDR=10,BPM2_ADDR=22")

# BPM1 instance related records
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmI.template", "P=$(PREFIX),R=$(BPM1):,           PORT=$(PORT),ADDR=10,TIMEOUT=1,NAME=$(BPM1)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH1):, PORT=$(PORT),ADDR=10,TIMEOUT=1,NAME=$(BPM1):$(BPMCH1)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH2):, PORT=$(PORT),ADDR=11,TIMEOUT=1,NAME=$(BPM1):$(BPMCH2)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH3):, PORT=$(PORT),ADDR=12,TIMEOUT=1,NAME=$(BPM1):$(BPMCH3)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH4):, PORT=$(PORT),ADDR=13,TIMEOUT=1,NAME=$(BPM1):$(BPMCH4)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH5):, PORT=$(PORT),ADDR=14,TIMEOUT=1,NAME=$(BPM1):$(BPMCH5)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH6):, PORT=$(PORT),ADDR=15,TIMEOUT=1,NAME=$(BPM1):$(BPMCH6)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH7):, PORT=$(PORT),ADDR=16,TIMEOUT=1,NAME=$(BPM1):$(BPMCH7)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH8):, PORT=$(PORT),ADDR=17,TIMEOUT=1,NAME=$(BPM1):$(BPMCH8)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH9):, PORT=$(PORT),ADDR=18,TIMEOUT=1,NAME=$(BPM1):$(BPMCH9)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH10):,PORT=$(PORT),ADDR=19,TIMEOUT=1,NAME=$(BPM1):$(BPMCH10)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH11):,PORT=$(PORT),ADDR=20,TIMEOUT=1,NAME=$(BPM1):$(BPMCH11)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM1):$(BPMCH12):,PORT=$(PORT),ADDR=21,TIMEOUT=1,NAME=$(BPM1):$(BPMCH12)")

# BPM2 instance related records
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmI.template", "P=$(PREFIX),R=$(BPM2):,           PORT=$(PORT),ADDR=22,TIMEOUT=1,NAME=$(BPM2)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH1):, PORT=$(PORT),ADDR=22,TIMEOUT=1,NAME=$(BPM2):$(BPMCH1)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH2):, PORT=$(PORT),ADDR=23,TIMEOUT=1,NAME=$(BPM2):$(BPMCH2)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH3):, PORT=$(PORT),ADDR=24,TIMEOUT=1,NAME=$(BPM2):$(BPMCH3)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH4):, PORT=$(PORT),ADDR=25,TIMEOUT=1,NAME=$(BPM2):$(BPMCH4)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH5):, PORT=$(PORT),ADDR=26,TIMEOUT=1,NAME=$(BPM2):$(BPMCH5)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH6):, PORT=$(PORT),ADDR=27,TIMEOUT=1,NAME=$(BPM2):$(BPMCH6)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH7):, PORT=$(PORT),ADDR=28,TIMEOUT=1,NAME=$(BPM2):$(BPMCH7)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH8):, PORT=$(PORT),ADDR=29,TIMEOUT=1,NAME=$(BPM2):$(BPMCH8)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH9):, PORT=$(PORT),ADDR=30,TIMEOUT=1,NAME=$(BPM2):$(BPMCH9)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH10):,PORT=$(PORT),ADDR=31,TIMEOUT=1,NAME=$(BPM2):$(BPMCH10)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH11):,PORT=$(PORT),ADDR=32,TIMEOUT=1,NAME=$(BPM2):$(BPMCH11)")
dbLoadRecords("$(ADSIS8300BPM)/db/SIS8300bpmN.template", "P=$(PREFIX),R=$(BPM2):$(BPMCH12):,PORT=$(PORT),ADDR=33,TIMEOUT=1,NAME=$(BPM2):$(BPMCH12)")

# Create a standard arrays plugin, set it to get data from ADSIS8300bpm driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 1000000x10 arrays.
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=10000000")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=3000")

# Time series plugin for AI
NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 0, 10)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS1:,   PORT=TS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TimeStep CP MS,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:1:, PORT=TS1,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:2:, PORT=TS1,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:3:, PORT=TS1,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:4:, PORT=TS1,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:5:, PORT=TS1,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:6:, PORT=TS1,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:7:, PORT=TS1,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:8:, PORT=TS1,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:9:, PORT=TS1,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS1:10:,PORT=TS1,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(AI10)")

# Time series plugin for BPM1
NDTimeSeriesConfigure("TS2", $(QSIZE), 0, "$(PORT)", 0, 12)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS2:,   PORT=TS2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TimeStep CP MS,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:1:, PORT=TS2,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:2:, PORT=TS2,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:3:, PORT=TS2,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:4:, PORT=TS2,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:5:, PORT=TS2,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:6:, PORT=TS2,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:7:, PORT=TS2,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:8:, PORT=TS2,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:9:, PORT=TS2,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:10:,PORT=TS2,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH10)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:11:,PORT=TS2,ADDR=10,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH11)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS2:12:,PORT=TS2,ADDR=11,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM1):$(BPMCH12)")

# Time series plugin for BPM2
NDTimeSeriesConfigure("TS3", $(QSIZE), 0, "$(PORT)", 0, 12)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS3:,   PORT=TS3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)TimeStep CP MS,ENABLED=1")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:1:, PORT=TS3,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH1)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:2:, PORT=TS3,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH2)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:3:, PORT=TS3,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH3)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:4:, PORT=TS3,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH4)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:5:, PORT=TS3,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH5)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:6:, PORT=TS3,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH6)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:7:, PORT=TS3,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH7)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:8:, PORT=TS3,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH8)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:9:, PORT=TS3,ADDR=8,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH9)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:10:,PORT=TS3,ADDR=9,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH10)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:11:,PORT=TS3,ADDR=10,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH11)")
dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS3:12:,PORT=TS3,ADDR=11,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(BPM2):$(BPMCH12)")


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
dbpf $(PREFIX)TrigSource 1
dbpf $(PREFIX)TrigSetup 0
dbpf $(PREFIX)RTMType 2
dbpf $(PREFIX)RTMTempGet 1
dbpf $(PREFIX)Enable 1


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

#dbpf $(PREFIX)$(AI1):ConvFactor 1
#dbpf $(PREFIX)$(AI2):ConvFactor 1
#dbpf $(PREFIX)$(AI3):ConvFactor 1
#dbpf $(PREFIX)$(AI4):ConvFactor 1
#dbpf $(PREFIX)$(AI5):ConvFactor 1
#dbpf $(PREFIX)$(AI6):ConvFactor 1
#dbpf $(PREFIX)$(AI7):ConvFactor 1
#dbpf $(PREFIX)$(AI8):ConvFactor 1
#dbpf $(PREFIX)$(AI9):ConvFactor 1
#dbpf $(PREFIX)$(AI10):ConvFactor 1

#dbpf $(PREFIX)$(AI1):ConvOffset -32768
#dbpf $(PREFIX)$(AI2):ConvOffset -32768
#dbpf $(PREFIX)$(AI3):ConvOffset -32768
#dbpf $(PREFIX)$(AI4):ConvOffset -32768
#dbpf $(PREFIX)$(AI5):ConvOffset -32768
#dbpf $(PREFIX)$(AI6):ConvOffset 0
#dbpf $(PREFIX)$(AI7):ConvOffset 0
#dbpf $(PREFIX)$(AI8):ConvOffset 0
#dbpf $(PREFIX)$(AI9):ConvOffset 0
#dbpf $(PREFIX)$(AI10):ConvOffset 0
