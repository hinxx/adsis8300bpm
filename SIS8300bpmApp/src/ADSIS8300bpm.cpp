/* ADSIS8300bpm.cpp
 *
 * This is a driver for a Struck SIS8300 BPM digitizer.
 *
 * Author: Hinko Kocevar
 *         ESS ERIC, Lund, Sweden
 *
 * Created:  September 22, 2016
 *
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <string>
#include <stdarg.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <iocsh.h>

#include <asynNDArrayDriver.h>
#include <epicsExport.h>

#include <ADSIS8300.h>
#include <ADSIS8300bpm.h>


static const char *driverName = "ADSIS8300bpm";

#define SIS8300DRV_CALL(s, x) ({\
	char message[256]; \
	int ret = x; \
	if (ret) {\
        sprintf(message, "%s::%s: %s() error: %s (%d)", \
        		driverName,__func__, s, sis8300drv_strerror(ret), ret); \
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
      	    	  "%s:%s: %s\n", driverName, __func__, message); \
      	setStringParam(P_Message, message); \
	} \
	ret; \
})

/**
 * Exit handler, delete the ADSIS8300 BPM object.
 */
//static void exitHandler(void *drvPvt) {
//	ADSIS8300bpm *pPvt = (ADSIS8300bpm *) drvPvt;
//	delete pPvt;
//}

/** Constructor for SIS8300bpm; most parameters are simply passed to ADSIS8300::ADSIS8300.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class and ADSIS8300.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] devicePath The path to the /dev entry.
  * \param[in] maxAddr The maximum  number of asyn addr addresses this driver supports. 1 is minimum.
  * \param[in] numParams The number of parameters in the derived class.
  * \param[in] numTimePoints The initial number of time points.
  * \param[in] dataType The initial data type (NDDataType_t) of the arrays that this driver will create.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
ADSIS8300bpm::ADSIS8300bpm(const char *portName, const char *devicePath,
		int maxAddr, int numParams, int numTimePoints, NDDataType_t dataType,
		int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADSIS8300(portName, devicePath,
    		SIS8300DRV_NUM_AI_CHANNELS+maxAddr,
    		NUM_SIS8300BPM_PARAMS+numParams,
			numTimePoints,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
    int status = asynSuccess;

    printf("%s::%s: %d channels, %d parameters\n", driverName, __func__,
    		SIS8300DRV_NUM_AI_CHANNELS+maxAddr,NUM_SIS8300BPM_PARAMS+numParams);

    /* System wide parameters */
    createParam(BpmPulseDoneString,               asynParamInt32, &P_PulseDone);
    createParam(BpmPulseCountString,              asynParamInt32, &P_PulseCount);
    createParam(BpmPulseMissedString,             asynParamInt32, &P_PulseMissed);
    createParam(BpmNearIQMString,                 asynParamInt32, &P_NearIQM);
    createParam(BpmNearIQNString,                 asynParamInt32, &P_NearIQN);
    createParam(BpmNumSamplesString,              asynParamInt32, &P_NumSamples);
    createParam(BpmNumIQSamplesString,            asynParamInt32, &P_NumIQSamples);
    createParam(BpmMemMuxString,                  asynParamInt32, &P_MemMux);
    createParam(BpmMemMux10String,                asynParamInt32, &P_MemMux10);
    createParam(BpmFilterControlString,           asynParamInt32, &P_FilterControl);
    createParam(BpmFilterCoeff0String,          asynParamFloat64, &P_FilterCoeff0);
    createParam(BpmFilterCoeff1String,          asynParamFloat64, &P_FilterCoeff1);
    createParam(BpmFilterCoeff2String,          asynParamFloat64, &P_FilterCoeff2);
    createParam(BpmFilterCoeff3String,          asynParamFloat64, &P_FilterCoeff3);
    createParam(BpmFilterCoeff4String,          asynParamFloat64, &P_FilterCoeff4);
    createParam(BpmFilterCoeff5String,          asynParamFloat64, &P_FilterCoeff5);
    createParam(BpmFilterGainString,            asynParamFloat64, &P_FilterGain);
    createParam(BpmFilterApplyString,             asynParamInt32, &P_FilterApply);
    createParam(BpmFirmwareVersionString,         asynParamInt32, &P_BPMFirmwareVersion);
    /* BPM instance wide parameters (BPM1 or BPM2)*/
    createParam(BpmIEnableString,                 asynParamInt32, &P_IEnable);
    createParam(BpmIThrXPosLowString,           asynParamFloat64, &P_IThrXPosLow);
    createParam(BpmIThrXPosHighString,          asynParamFloat64, &P_IThrXPosHigh);
    createParam(BpmIThrYPosLowString,           asynParamFloat64, &P_IThrYPosLow);
    createParam(BpmIThrYPosHighString,          asynParamFloat64, &P_IThrYPosHigh);
    createParam(BpmIThrMagnitudeString,         asynParamFloat64, &P_IThrMagnitude);
    createParam(BpmIThrSelectString,              asynParamInt32, &P_IThrSelect);
    createParam(BpmIIlkControlString,             asynParamInt32, &P_IIlkControl);
    createParam(BpmIIlkClearString,               asynParamInt32, &P_IIlkClear);
    createParam(BpmIIlkStatusString,              asynParamInt32, &P_IIlkStatus);
    /* BPM channel wide parameters */
    createParam(BpmNConvFactorString,           asynParamFloat64, &P_NConvFactor);

    if (status) {
        printf("%s::%s: unable to set parameters\n", driverName, __func__);
        return;
    }

    this->lock();
    initDevice();
    this->unlock();

	printf("%s::%s: Init done...\n", driverName, __func__);
}

ADSIS8300bpm::~ADSIS8300bpm() {
	printf("%s::%s: Shutdown and freeing up memory...\n", driverName, __func__);

	this->lock();
	printf("%s::%s: Data thread is already down!\n", driverName, __func__);
	destroyDevice();

	this->unlock();
	printf("%s::%s: Shutdown complete!\n", driverName, __func__);
}

/** Template function to compute the simulated detector data for any data type */
template <typename epicsType> int ADSIS8300bpm::acquireArraysT()
{
    size_t dims[2], rawDims[1];
    int numTimePoints;
    NDDataType_t dataType;
    epicsType *pData;
    epicsUInt16 *pRawData;
    double acquireTime;
    double timeStep;
    int ch;
    int i;
    double convFactor, convOffset;
    int ret;
    
    getIntegerParam(NDDataType, (int *)&dataType);
    getIntegerParam(P_NumTimePoints, &numTimePoints);
    getDoubleParam(P_AcquireTime, &acquireTime);
    getDoubleParam(P_TimeStep, &timeStep);

    dims[0] = SIS8300DRV_NUM_AI_CHANNELS;
    dims[1] = numTimePoints;

    if (this->pArrays[0]) this->pArrays[0]->release();
    this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[0]->pData;
    memset(pData, 0, SIS8300DRV_NUM_AI_CHANNELS * numTimePoints * sizeof(epicsType));

    /* raw data is always 16-bit, allocate for one channel */
    if (mRawDataArray && (mRawDataArray->dims[0].size != (size_t)numTimePoints)) {
    	mRawDataArray->release();
    	mRawDataArray = NULL;
    }
    if (! mRawDataArray) {
    	rawDims[0] = numTimePoints;
        mRawDataArray = pNDArrayPool->alloc(1, rawDims, NDUInt16, 0, 0);
    }
	pRawData = (epicsUInt16 *)mRawDataArray->pData;

    for (ch = 0; ch < SIS8300DRV_NUM_AI_CHANNELS; ch++) {
        if (!(mChannelMask & (1 << ch))) {
            continue;
        }

		ret = SIS8300DRV_CALL("sis8300drv_read_ai", sis8300drv_read_ai(mSisDevice, ch, pRawData));
		if (ret) {
			return ret;
		}
		getDoubleParam(ch, P_ConvFactor, &convFactor);
		getDoubleParam(ch, P_ConvOffset, &convOffset);
		this->unlock();
		printf("%s::%s: CH %d [%d] CF %f, CO %f:\n", driverName, __func__,
				ch, numTimePoints, convFactor, convOffset);
		for (i = 0; i < numTimePoints; i++) {
//			printf("%d ", *(pRawData + i));
			pData[SIS8300DRV_NUM_AI_CHANNELS*i + ch] = (epicsType)((double)*(pRawData + i) * convFactor + convOffset);
		}
		printf("\n");
		this->lock();
    }

    return 0;
}

/** Computes the new image data */
int ADSIS8300bpm::acquireArrays()
{
    int dataType;
    getIntegerParam(NDDataType, &dataType); 

    switch (dataType) {
        case NDInt8:
            return acquireArraysT<epicsInt8>();
            break;
        case NDUInt8:
        	return acquireArraysT<epicsUInt8>();
            break;
        case NDInt16:
        	return acquireArraysT<epicsInt16>();
            break;
        case NDUInt16:
        	return acquireArraysT<epicsUInt16>();
            break;
        case NDInt32:
        	return acquireArraysT<epicsInt32>();
            break;
        case NDUInt32:
        	return acquireArraysT<epicsUInt32>();
            break;
        case NDFloat32:
        	return acquireArraysT<epicsFloat32>();
            break;
        case NDFloat64:
        	return acquireArraysT<epicsFloat64>();
            break;
        default:
        	return -1;
        	break;
    }
}

int ADSIS8300bpm::initDeviceDone()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_init_done", sis8300llrfdrv_init_done(mSisDevice));

	return ret;
}

int ADSIS8300bpm::armDevice()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_clear_latched_statuses", sis8300llrfdrv_clear_latched_statuses(mSisDevice, SIS8300LLRFDRV_STATUS_CLR_GENERAL));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300llrfdrv_clear_pulse_done_count", sis8300llrfdrv_clear_pulse_done_count(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300llrfdrv_arm_device", sis8300llrfdrv_arm_device(mSisDevice));

	return ret;
}

int ADSIS8300bpm::disarmDevice()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_sw_reset", sis8300llrfdrv_sw_reset(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = ADSIS8300::disarmDevice();

	return ret;
}

int ADSIS8300bpm::waitForDevice()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

//	ret = SIS8300DRV_CALL("sis8300llrfdrv_wait_pulse_done_pms", sis8300llrfdrv_wait_pulse_done_pms(mSisDevice, SIS8300BPM_IRQ_WAIT_TIME));
// XXX: Debug!
	ret = SIS8300DRV_CALL("sis8300llrfdrv_wait_pulse_done_pms", sis8300llrfdrv_wait_pulse_done_pms(mSisDevice, 1000));

	return ret;
}

int ADSIS8300bpm::deviceDone()
{
	int ret;
	unsigned int pulseCount;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_get_pulse_done_count", sis8300llrfdrv_get_pulse_done_count(mSisDevice, &pulseCount));
	printf("%s::%s: Pulse count = %d\n", driverName, __func__, pulseCount);

	// XXX: Check for missed pulse
	// XXX: Increment pulse count and do param callbacks

	return ret;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADSIS8300bpm::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int addr;
//    int ret;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    printf("%s::%s: ENTER %d (%d) = %d\n", driverName, __func__, function, addr, value);
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

    if (function == P_NearIQM) {
    } else if (function == P_NearIQN) {
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_SIS8300BPM_PARAM) {
        	status = ADSIS8300::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADSIS8300bpm::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int addr;
//    int ret;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    printf("%s::%s: ENTER %d (%d) = %f\n", driverName, __func__, function, addr, value);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(addr, function, value);

    if (function == P_FilterCoeff0) {
    } else if (function == P_FilterCoeff1) {
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_SIS8300BPM_PARAM) {
        	status = ADSIS8300::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeFloat64: function=%d, value=%f\n",
              driverName, function, value);
    return status;
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void ADSIS8300bpm::report(FILE *fp, int details)
{
    fprintf(fp, "Struck BPM       : %s\n", this->portName);
    if (details > 0) {
    }

    /* Invoke the base class method */
    ADSIS8300::report(fp, details);
}

int ADSIS8300bpm::initDevice()
{
	int ret;
	unsigned int ver_device;
	unsigned int ver_major;
	unsigned int ver_minor;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_get_fw_version", sis8300llrfdrv_get_fw_version(mSisDevice, &ver_device, &ver_major, &ver_minor));
	if (ret) {
		return ret;
	}
	setIntegerParam(P_BPMFirmwareVersion, ver_major << 8 | ver_minor);

	ret = SIS8300DRV_CALL("sis8300llrfdrv_sw_reset", sis8300llrfdrv_sw_reset(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300llrfdrv_setup_dac", sis8300llrfdrv_setup_dac(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300llrfdrv_setup_adc_tap_delay", sis8300llrfdrv_setup_adc_tap_delay(mSisDevice));
	if (ret) {
		return ret;
	}

	return ret;
}

int ADSIS8300bpm::destroyDevice()
{

	printf("%s::%s: Enter\n", driverName, __func__);

	// XXX Add BPM specific destroy here

	return 0;
}

//int ADSIS8300bpm::enableChannel(unsigned int channel)
//{
//   	mBPMChannelMask |= (1 << channel);
//    printf("%s::%s: channel mask %X\n", driverName, __func__, mBPMChannelMask);
//	return 0;
//}
//
//int ADSIS8300bpm::disableChannel(unsigned int channel)
//{
//   	mBPMChannelMask &= ~(1 << channel);
//    printf("%s::%s: channel mask %X\n", driverName, __func__, mBPMChannelMask);
//	return 0;
//}

/** Configuration command, called directly or from iocsh */
extern "C" int SIS8300BpmConfig(const char *portName, const char *devicePath,
		int maxAddr, int numTimePoints, int dataType, int maxBuffers, int maxMemory,
		int priority, int stackSize)
{
    new ADSIS8300bpm(portName, devicePath,
    		maxAddr,
			0,
    		numTimePoints,
			(NDDataType_t)dataType,
			(maxBuffers < 0) ? 0 : maxBuffers,
			(maxMemory < 0) ? 0 : maxMemory,
			priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SIS8300BpmConfigArg0 = {"Port name",     iocshArgString};
static const iocshArg SIS8300BpmConfigArg1 = {"Device path",   iocshArgString};
static const iocshArg SIS8300BpmConfigArg2 = {"# channels",    iocshArgInt};
static const iocshArg SIS8300BpmConfigArg3 = {"# time points", iocshArgInt};
static const iocshArg SIS8300BpmConfigArg4 = {"Data type",     iocshArgInt};
static const iocshArg SIS8300BpmConfigArg5 = {"maxBuffers",    iocshArgInt};
static const iocshArg SIS8300BpmConfigArg6 = {"maxMemory",     iocshArgInt};
static const iocshArg SIS8300BpmConfigArg7 = {"priority",      iocshArgInt};
static const iocshArg SIS8300BpmConfigArg8 = {"stackSize",     iocshArgInt};
static const iocshArg * const SIS8300BpmConfigArgs[] = {&SIS8300BpmConfigArg0,
                                                     &SIS8300BpmConfigArg1,
													 &SIS8300BpmConfigArg2,
													 &SIS8300BpmConfigArg3,
													 &SIS8300BpmConfigArg4,
													 &SIS8300BpmConfigArg5,
													 &SIS8300BpmConfigArg6,
													 &SIS8300BpmConfigArg7,
													 &SIS8300BpmConfigArg8};
static const iocshFuncDef configSIS8300Bpm = {"SIS8300BpmConfig", 9, SIS8300BpmConfigArgs};
static void configSIS8300BpmCallFunc(const iocshArgBuf *args)
{
    SIS8300BpmConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
    		args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}


static void SIS8300BpmRegister(void)
{
    iocshRegister(&configSIS8300Bpm, configSIS8300BpmCallFunc);
}

extern "C" {
epicsExportRegistrar(SIS8300BpmRegister);
}
