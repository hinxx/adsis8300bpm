/* ADSIS8300bpm.h
 *
 * This is a driver for a Struck SIS8300 BPM digitizer.
 *
 * Author: Hinko Kocevar
 *         ESS ERIC, Lund, Sweden
 *
 * Created:  September 22, 2016
 *
 */

#include <stdint.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <asynNDArrayDriver.h>

#include <sis8300drv.h>
#include <sis8300drvbpm.h>

/* System wide parameters */
#define BpmPulseDoneString                          "BPM_PULSE_DONE"
#define BpmPulseCountString                         "BPM_PULSE_COUNT"
#define BpmPulseMissedString                        "BPM_PULSE_MISSED"
#define BpmNearIQMString                            "BPM_NEARIQ_M"
#define BpmNearIQNString                            "BPM_NEARIQ_N"
#define BpmNumSamplesString                         "BPM_NUM_SAMPLES"
#define BpmNumIQSamplesString                       "BPM_NUM_IQ_SAMPLES"
#define BpmMemMuxString                             "BPM_MEM_MUX"
#define BpmMemMux10String                           "BPM_MEM_MUX10"
#define BpmFilterControlString                      "BPM_FILTER_CONTROL"
#define BpmFilterCoeff0String                       "BPM_FILTER_COEFF_0"
#define BpmFilterCoeff1String                       "BPM_FILTER_COEFF_1"
#define BpmFilterCoeff2String                       "BPM_FILTER_COEFF_2"
#define BpmFilterCoeff3String                       "BPM_FILTER_COEFF_3"
#define BpmFilterCoeff4String                       "BPM_FILTER_COEFF_4"
#define BpmFilterCoeff5String                       "BPM_FILTER_COEFF_5"
#define BpmFilterGainString                         "BPM_FILTER_GAIN"
#define BpmFilterApplyString                        "BPM_FILTER_APPLY"
#define BpmFirmwareVersionString                    "BPM_FW_VERSION"
/* BPM instance wide parameters (BPM1 or BPM2)*/
#define BpmIEnableString                            "BPMI_ENABLE"
#define BpmIThrXPosLowString                        "BPMI_THR_XPOS_LOW"
#define BpmIThrXPosHighString                       "BPMI_THR_XPOS_HIGH"
#define BpmIThrYPosLowString                        "BPMI_THR_YPOS_LOW"
#define BpmIThrYPosHighString                       "BPMI_THR_YPOS_HIGH"
#define BpmIThrMagnitudeString                      "BPMI_THR_MAGNITUDE"
#define BpmIThrSelectString                         "BPMI_THR_SELECT"
#define BpmIIlkControlString                        "BPMI_ILK_CONTROL"
#define BpmIIlkClearString                          "BPMI_ILK_CLEAR"
#define BpmIIlkStatusString                         "BPMI_ILK_STATUS"
/* BPM channel wide parameters */
#define BpmNConvFactorString                        "BPMN_CONV_FACTOR"

#define SIS8300BPM_IRQ_WAIT_TIME      0

/** Struck SIS8300 BPM driver; does 1-D waveforms on 12 channels.
  * Inherits from ADSIS8300 */
class epicsShareClass ADSIS8300bpm : public ADSIS8300 {
public:
	ADSIS8300bpm(const char *portName, const char *devicePath,
			int maxAddr, int numParams, int numTimePoints, NDDataType_t dataType,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);
	~ADSIS8300bpm();

    /* These are the methods that we override from asynNDArrayDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);

protected:
    /* System wide parameters */
    int P_PulseDone;
    #define FIRST_SIS8300BPM_PARAM P_PulseDone
    int P_PulseCount;
    int P_PulseMissed;
    int P_NearIQM;
    int P_NearIQN;
    int P_NumSamples;
    int P_NumIQSamples;
    int P_MemMux;
    int P_MemMux10;
    int P_FilterControl;
    int P_FilterCoeff0;
    int P_FilterCoeff1;
    int P_FilterCoeff2;
    int P_FilterCoeff3;
    int P_FilterCoeff4;
    int P_FilterCoeff5;
    int P_FilterGain;
    int P_FilterApply;
    int P_BPMFirmwareVersion;
    /* BPM instance wide parameters (BPM1 or BPM2)*/
    int P_IEnable;
    int P_IThrXPosLow;
    int P_IThrXPosHigh;
    int P_IThrYPosLow;
    int P_IThrYPosHigh;
    int P_IThrMagnitude;
    int P_IThrSelect;
    int P_IIlkControl;
    int P_IIlkClear;
    int P_IIlkStatus;
    /* BPM channel wide parameters */
    int P_NConvFactor;

    int P_Dummy2;
    #define LAST_SIS8300BPM_PARAM P_Dummy2

    /* These are the methods that are new to this class */
    template <typename epicsType> int acquireArraysT();
    virtual int acquireArrays();
    virtual int initDevice();
    virtual int destroyDevice();
//    virtual int enableChannel(unsigned int channel);
//    virtual int disableChannel(unsigned int channel);
    virtual int initDeviceDone();
    virtual int armDevice();
    virtual int disarmDevice();
    virtual int waitForDevice();
    virtual int deviceDone();

private:

    /* Our data */
    uint32_t mBPMChannelMask;
    bool mDoRegisterUpdate;
};


#define NUM_SIS8300BPM_PARAMS ((int)(&LAST_SIS8300BPM_PARAM - &FIRST_SIS8300BPM_PARAM + 1))
