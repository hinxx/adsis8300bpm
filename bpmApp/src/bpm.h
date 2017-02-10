/* Bpm.h
 *
 * This is a driver for a BPM based on Struck SIS8300 digitizer.
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

typedef enum _BpmChannelIndex {
	BPMChannelXPos = 0,
	BPMChannelYPos,
	BPMChannelMSum,
	BPMChannelPSum,
	BPMChannelAMag,
	BPMChannelBMag,
	BPMChannelCMag,
	BPMChannelDMag,
	BPMChannelAPha,
	BPMChannelBPha,
	BPMChannelCPha,
	BPMChannelDPha,
} BPMChannelIndex;
#define BPM_NUM_CHANNELS      12
#define BPM_IRQ_WAIT_TIME     2000

/* System wide parameters */
#define BpmFwVersionString                          "BPM_FW_VERSION"
#define BpmPulseDoneString                          "BPM_PULSE_DONE"
#define BpmPulseCountString                         "BPM_PULSE_COUNT"
#define BpmPulseMissedString                        "BPM_PULSE_MISSED"
#define BpmNearIQMString                            "BPM_NEARIQ_M"
#define BpmNearIQNString                            "BPM_NEARIQ_N"
#define BpmNumSamplesString                         "BPM_NUM_SAMPLES"
#define BpmNumIQSamplesString                       "BPM_NUM_IQ_SAMPLES"
#define BpmNumBPMSamplesString                      "BPM_NUM_BPM_SAMPLES"
#define BpmMemMuxString                             "BPM_MEM_MUX"
#define BpmMemMux10String                           "BPM_MEM_MUX10"
#define BpmTrigSetupString                          "BPM_TRIG_SETUP"
#define BpmRegReadErrString                         "BPM_REG_READ_ERR"
#define BpmRegWriteErrString                        "BPM_REG_WRITE_ERR"
#define BpmFilterControlString                      "BPM_FILTER_CONTROL"
#define BpmFilterCoeff0String                       "BPM_FILTER_COEFF_0"
#define BpmFilterCoeff1String                       "BPM_FILTER_COEFF_1"
#define BpmFilterCoeff2String                       "BPM_FILTER_COEFF_2"
#define BpmFilterCoeff3String                       "BPM_FILTER_COEFF_3"
#define BpmFilterCoeff4String                       "BPM_FILTER_COEFF_4"
#define BpmFilterCoeff5String                       "BPM_FILTER_COEFF_5"
#define BpmFilterGainString                         "BPM_FILTER_GAIN"
#define BpmFilterApplyString                        "BPM_FILTER_APPLY"
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
#define BpmIIlkIRQString                            "BPMI_ILK_IRQ"
#define BpmIDivXPosErrString                        "BPMI_DIV_XPOS_ERR"
#define BpmIDivYPosErrString                        "BPMI_DIV_YPOS_ERR"

class epicsShareClass Bpm : public SIS8300 {
public:
	Bpm(const char *portName, const char *devicePath,
			int maxAddr, int numTimePoints, NDDataType_t dataType,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);
	virtual ~Bpm();

    /* These are the methods that we override from asynNDArrayDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);

protected:
    /* System wide parameters */
    int mBpmFwVersion;
    #define BPM_FIRST_PARAM mBpmFwVersion
    int mBpmPulseDone;
    int mBpmPulseCount;
    int mBpmPulseMissed;
    int mBpmNearIQM;
    int mBpmNearIQN;
    int mBpmNumSamples;
    int mBpmNumIQSamples;
    int mBpmNumBPMSamples;
    int mBpmMemMux;
    int mBpmMemMux10;
    int mBpmTrigSetup;
    int mBpmRegReadErr;
    int mBpmRegWriteErr;
    int mBpmFilterControl;
    int mBpmFilterCoeff0;
    int mBpmFilterCoeff1;
    int mBpmFilterCoeff2;
    int mBpmFilterCoeff3;
    int mBpmFilterCoeff4;
    int mBpmFilterCoeff5;
    int mBpmFilterGain;
    int mBpmFilterApply;
    /* BPM instance wide parameters (BPM1 or BPM2)*/
    int mBpmIEnable;
    int mBpmIThrXPosLow;
    int mBpmIThrXPosHigh;
    int mBpmIThrYPosLow;
    int mBpmIThrYPosHigh;
    int mBpmIThrMagnitude;
    int mBpmIThrSelect;
    int mBpmIIlkControl;
    int mBpmIIlkClear;
    int mBpmIIlkStatus;
    int mBpmIIlkIRQ;
    int mBpmIDivXPosErr;
    int mBpmIDivYPosErr;
    #define BPM_LAST_PARAM mBpmIDivYPosErr

    /* These are the methods that are new to this class */
    template <typename epicsType> int convertArraysT();
    template <typename epicsType> int convertAIArraysT(int aich);
    template <typename epicsType> int convertBPMArraysT(int aich);
    virtual int acquireArrays();
    virtual int initDevice();
    virtual int destroyDevice();
    virtual int initDeviceDone();
    virtual int armDevice();
    virtual int disarmDevice();
    virtual int waitForDevice();
    virtual int deviceDone();
    virtual int updateParameters();

private:
    int updateBoardSetup();
    int updateNearIQ();
    int updateFilter();
    int updateThreshold(int addr);

    /* Our data */
    uint32_t mBPMChannelMask;
    bool mDoNearIQUpdate;
    bool mDoBoardSetupUpdate;
    bool mDoFilterControlUpdate;
    bool mDoFilterCoeffUpdate;
    bool mDoBpm1ThresholdUpdate;
    bool mDoBpm2ThresholdUpdate;
};

#define BPM_NUM_PARAMS ((int)(&BPM_LAST_PARAM - &BPM_FIRST_PARAM + 1))
