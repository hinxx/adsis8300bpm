/* SIS8300bpm.h
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

#define SisDummy1String               "SIS_DUMMY1"
#define SisDummy2String               "SIS_DUMMY2"

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
    int P_Dummy1;
    #define FIRST_SIS8300BPM_PARAM P_Dummy1

    int P_Dummy2;
    #define LAST_SIS8300BPM_PARAM P_Dummy2

    /* These are the methods that are new to this class */
    template <typename epicsType> int acquireArraysT();
    int acquireArrays();
    int initDevice();
    int destroyDevice();
    int enableChannel(unsigned int channel);
    int disableChannel(unsigned int channel);

private:

    /* Our data */
    uint32_t mBPMChannelMask;
};


#define NUM_SIS8300BPM_PARAMS ((int)(&LAST_SIS8300BPM_PARAM - &FIRST_SIS8300BPM_PARAM + 1))
