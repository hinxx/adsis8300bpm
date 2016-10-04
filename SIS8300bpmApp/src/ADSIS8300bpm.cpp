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

#include <sis8300bpm_reg.h>
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

/* asyn addresses:
 * 0 - 9         analog channels
 * 10 - 21       BPM1 channels
 * 22 - 33       BPM2 channels
 */
#define SIS8300BPM_BPM1_ADDR		10
#define SIS8300BPM_BPM2_ADDR		22

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
    		maxAddr,
    		NUM_SIS8300BPM_PARAMS/*+numParams*/,
			numTimePoints,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
    int status = asynSuccess;

    printf("%s::%s: %d channels, %d parameters\n", driverName, __func__,
    		maxAddr,NUM_SIS8300BPM_PARAMS+numParams);

    /* System wide parameters */
    createParam(BpmFirmwareVersionString,         asynParamInt32, &P_BPMFirmwareVersion);
    createParam(BpmPulseDoneString,               asynParamInt32, &P_PulseDone);
    createParam(BpmPulseCountString,              asynParamInt32, &P_PulseCount);
    createParam(BpmPulseMissedString,             asynParamInt32, &P_PulseMissed);
    createParam(BpmNearIQMString,                 asynParamInt32, &P_NearIQM);
    createParam(BpmNearIQNString,                 asynParamInt32, &P_NearIQN);
    createParam(BpmNumSamplesString,              asynParamInt32, &P_NumSamples);
    createParam(BpmNumIQSamplesString,            asynParamInt32, &P_NumIQSamples);
    createParam(BpmMemMuxString,                  asynParamInt32, &P_MemMux);
    createParam(BpmMemMux10String,                asynParamInt32, &P_MemMux10);
    createParam(BpmRegReadErrString,              asynParamInt32, &P_RegReadErr);
    createParam(BpmRegWriteErrString,             asynParamInt32, &P_RegWriteErr);
    createParam(BpmTrigSetupString,               asynParamInt32, &P_TrigSetup);
    createParam(BpmFilterControlString,           asynParamInt32, &P_FilterControl);
    createParam(BpmFilterCoeff0String,          asynParamFloat64, &P_FilterCoeff0);
    createParam(BpmFilterCoeff1String,          asynParamFloat64, &P_FilterCoeff1);
    createParam(BpmFilterCoeff2String,          asynParamFloat64, &P_FilterCoeff2);
    createParam(BpmFilterCoeff3String,          asynParamFloat64, &P_FilterCoeff3);
    createParam(BpmFilterCoeff4String,          asynParamFloat64, &P_FilterCoeff4);
    createParam(BpmFilterCoeff5String,          asynParamFloat64, &P_FilterCoeff5);
    createParam(BpmFilterGainString,            asynParamFloat64, &P_FilterGain);
    createParam(BpmFilterApplyString,             asynParamInt32, &P_FilterApply);
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
    createParam(BpmIIlkIRQString,                 asynParamInt32, &P_IIlkIRQ);
    createParam(BpmIDivXPosErrString,             asynParamInt32, &P_IDivXPosErr);
    createParam(BpmIDivYPosErrString,             asynParamInt32, &P_IDivYPosErr);
    /* BPM channel wide parameters */
    createParam(BpmNConvFactorString,           asynParamFloat64, &P_NConvFactor);

    mDoBoardSetupUpdate = false;
    mDoNearIQUpdate = false;
    mDoFilterCoeffUpdate = false;

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

	ret = SIS8300DRV_CALL("sis8300drvbpm_init_done", sis8300drvbpm_init_done(mSisDevice));
	if (ret) {
		return ret;
	}

	return ret;
}

int ADSIS8300bpm::armDevice()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300drvbpm_clear_gop", sis8300drvbpm_clear_gop(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300drvbpm_clear_pulse_done_count", sis8300drvbpm_clear_pulse_done_count(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300drvbpm_arm_device", sis8300drvbpm_arm_device(mSisDevice));

	setIntegerParam(P_PulseDone, 0);
//	setIntegerParam(P_PulseMissed, 0);
//	setIntegerParam(10, P_IDivXPosErr, 0);
//	setIntegerParam(10, P_IDivYPosErr, 0);
//	setIntegerParam(22, P_IDivXPosErr, 0);
//	setIntegerParam(22, P_IDivYPosErr, 0);
//	setIntegerParam(P_RegReadErr, 0);
//	setIntegerParam(P_RegWriteErr, 0);
//	callParamCallbacks(0);

	return ret;
}

int ADSIS8300bpm::disarmDevice()
{
	int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = SIS8300DRV_CALL("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));
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

//	ret = SIS8300DRV_CALL("sis8300drvbpm_wait_pulse_done_pposition", sis8300drvbpm_wait_pulse_done_pposition(mSisDevice, SIS8300BPM_IRQ_WAIT_TIME));
// XXX: Debug!
	ret = SIS8300DRV_CALL("sis8300drvbpm_wait_pulse_done_pposition", sis8300drvbpm_wait_pulse_done_pposition(mSisDevice, 1000));

	return ret;
}

int ADSIS8300bpm::deviceDone()
{
	int ret;
	int oldCount;
	unsigned int pulseCount;
	unsigned int sampleCount;
	unsigned int gop;

	printf("%s::%s: Enter\n", driverName, __func__);

	pulseCount = 0;
	ret = SIS8300DRV_CALL("sis8300drvbpm_get_pulse_done_count", sis8300drvbpm_get_pulse_done_count(mSisDevice, &pulseCount));
	if (pulseCount != 1) {
		setIntegerParam(P_PulseMissed, pulseCount - 1);
	} else {
		setIntegerParam(P_PulseMissed, 0);
	}
	getIntegerParam(P_PulseCount, &oldCount);
	oldCount += pulseCount;
	setIntegerParam(P_PulseCount, oldCount);
	setIntegerParam(P_PulseDone, 1);

	ret = SIS8300DRV_CALL("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_SAMPLE_CNT_R_REG, &sampleCount));
	setIntegerParam(P_NumSamples, sampleCount);
	ret = SIS8300DRV_CALL("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_IQ_SAMPLE_CNT_REG, &sampleCount));
	setIntegerParam(P_NumIQSamples, sampleCount);

	ret = SIS8300DRV_CALL("sis8300drvbpm_get_gop", sis8300drvbpm_get_gop(mSisDevice, gop_all, &gop));
	if (gop & (1 << gop_X1_pos_div_error)) {
		setIntegerParam(SIS8300BPM_BPM1_ADDR, P_IDivXPosErr, 1);
	} else {
		setIntegerParam(SIS8300BPM_BPM1_ADDR, P_IDivXPosErr, 0);
	}
	if (gop & (1 << gop_Y1_pos_div_error)) {
		setIntegerParam(SIS8300BPM_BPM1_ADDR, P_IDivYPosErr, 1);
	} else {
		setIntegerParam(SIS8300BPM_BPM1_ADDR, P_IDivYPosErr, 0);
	}
	if (gop & (1 << gop_X2_pos_div_error)) {
		setIntegerParam(SIS8300BPM_BPM2_ADDR, P_IDivXPosErr, 1);
	} else {
		setIntegerParam(SIS8300BPM_BPM2_ADDR, P_IDivXPosErr, 0);
	}
	if (gop & (1 << gop_Y2_pos_div_error)) {
		setIntegerParam(SIS8300BPM_BPM2_ADDR, P_IDivYPosErr, 1);
	} else {
		setIntegerParam(SIS8300BPM_BPM2_ADDR, P_IDivYPosErr, 0);
	}
	if (gop & (1 << gop_read_error)) {
		setIntegerParam(P_RegReadErr, 1);
	} else {
		setIntegerParam(P_RegReadErr, 0);
	}
	if (gop & (1 << gop_write_error)) {
		setIntegerParam(P_RegWriteErr, 1);
	} else {
		setIntegerParam(P_RegWriteErr, 0);
	}
	if (gop & (1 << gop_position1_error)) {
		setIntegerParam(SIS8300BPM_BPM1_ADDR, P_IIlkStatus, 1);
	}
	if (gop & (1 << gop_position2_error)) {
		setIntegerParam(SIS8300BPM_BPM2_ADDR, P_IIlkStatus, 1);
	}

	return ret;
}

int ADSIS8300bpm::updateParameters()
{
	int ret = 0;
    bool doShadowUpdate = false;

	printf("%s::%s: Enter\n", driverName, __func__);

	if (mDoNearIQUpdate) {
		ret = updateNearIQ();
		doShadowUpdate = true;
	}
	if (mDoBoardSetupUpdate) {
		ret = updateBoardSetup();
	}
	if (mDoFilterControlUpdate || mDoFilterCoeffUpdate) {
		ret = updateFilter();
	}
	if (mDoBpm1ThresholdUpdate) {
		ret = updateThreshold(SIS8300BPM_BPM1_ADDR);
		doShadowUpdate = true;
	}
	if (mDoBpm2ThresholdUpdate) {
		ret = updateThreshold(SIS8300BPM_BPM2_ADDR);
		doShadowUpdate = true;
	}
	if (doShadowUpdate) {
		ret = SIS8300DRV_CALL("sis8300drvbpm_update_parameters", sis8300drvbpm_update_parameters(mSisDevice));
	}

	return ret;
}

int ADSIS8300bpm::updateBoardSetup()
{
	int ret;
	int ilk1IRQ, ilk2IRQ;
	int ilk1Ctrl, ilk2Ctrl;
	int memMux, memMux10;
	int trigSetup;
	unsigned int boardSetup;

	printf("%s::%s: Enter\n", driverName, __func__);

	getIntegerParam(P_MemMux, &memMux);
	getIntegerParam(P_MemMux10, &memMux10);
	getIntegerParam(P_TrigSetup, &trigSetup);
	getIntegerParam(SIS8300BPM_BPM1_ADDR, P_IIlkControl, &ilk1Ctrl);
	getIntegerParam(SIS8300BPM_BPM2_ADDR, P_IIlkControl, &ilk2Ctrl);
	getIntegerParam(SIS8300BPM_BPM1_ADDR, P_IIlkIRQ, &ilk1IRQ);
	getIntegerParam(SIS8300BPM_BPM2_ADDR, P_IIlkIRQ, &ilk2IRQ);

	/* XXX: Handle the rest of the bits in board setup reg!
	 *      Do not clobber the other bits in board setup reg! */
	boardSetup = (ilk1IRQ & 0x1) << 21 | (ilk2IRQ & 0x1) << 20 |
			(ilk1Ctrl & 0x1) << 19 | (ilk2Ctrl & 0x1) << 18 |
			(memMux & 0x3) << 8 | (memMux10 & 0x3) << 6 |
			(trigSetup & 0x3) << 0;
	printf("%s::%s: New board setup 0x%08X\n", driverName, __func__, boardSetup);

	ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, SIS8300BPM_BOARD_SETUP_REG, boardSetup));
	if (ret) {
		return ret;
	}

	mDoBoardSetupUpdate = false;

	return ret;
}

int ADSIS8300bpm::updateNearIQ()
{
	int ret;
	int n, m;

	printf("%s::%s: Enter\n", driverName, __func__);

	getIntegerParam(P_NearIQM, &m);
	getIntegerParam(P_NearIQN, &n);

	printf("%s::%s: New near IQ M = %d, N = %d\n", driverName, __func__, m, n);

	ret = SIS8300DRV_CALL("sis8300drvbpm_set_near_iq", sis8300drvbpm_set_near_iq(mSisDevice, m, n));
	if (ret) {
		return ret;
	}

	mDoNearIQUpdate = false;

	return ret;
}

int ADSIS8300bpm::updateFilter()
{
	int ret;
	epicsFloat64 coeff[SIS8300DRVBPM_FIR_FILTER_PARAM_NUM];
	epicsFloat64 gain;
	int filterControl;

	printf("%s::%s: Enter\n", driverName, __func__);

	ret = 0;
	if (mDoFilterCoeffUpdate) {
		getDoubleParam(P_FilterCoeff0, &coeff[0]);
		getDoubleParam(P_FilterCoeff1, &coeff[1]);
		getDoubleParam(P_FilterCoeff2, &coeff[2]);
		getDoubleParam(P_FilterCoeff3, &coeff[3]);
		getDoubleParam(P_FilterCoeff4, &coeff[4]);
		getDoubleParam(P_FilterCoeff5, &coeff[5]);
		ret = SIS8300DRV_CALL("sis8300drvbpm_set_fir_filter_param", sis8300drvbpm_set_fir_filter_param(mSisDevice, coeff, SIS8300DRVBPM_FIR_FILTER_PARAM_NUM));
		if (ret) {
			return ret;
		}

		gain = 2 * coeff[0] + \
				2 * coeff[2] + \
				2 * coeff[3] + \
				2 * coeff[4] + \
				2 * coeff[5] + \
				coeff[1];

		setDoubleParam(P_FilterGain, gain);

		mDoFilterCoeffUpdate = false;
	}

	if (mDoFilterControlUpdate) {
		getIntegerParam(P_FilterControl, &filterControl);

		printf("%s::%s: New filter control %d\n", driverName, __func__, filterControl);

		ret = SIS8300DRV_CALL("sis8300drvbpm_set_fir_filter_enable", sis8300drvbpm_set_fir_filter_enable(mSisDevice, filterControl));
		if (ret) {
			return ret;
		}
		mDoFilterControlUpdate = false;
	}

	return ret;
}

int ADSIS8300bpm::updateThreshold(int addr)
{
	int ret;
	double xPosLow, xPosHigh;
	double yPosLow, yPosHigh;
	double magnitude;
	int thrMagCtrlReg;
	int thrXValReg;
	int thrYValReg;
	int thrControl;
	int thrVal;
	uint32_t conv;
	double err;

	printf("%s::%s: Enter\n", driverName, __func__);

	if (addr == SIS8300BPM_BPM1_ADDR) {
		thrMagCtrlReg = SIS8300BPM_POS_MAG_CTRL1_REG;
		thrXValReg = SIS8300BPM_POS_PARAM_X1_REG;
		thrYValReg = SIS8300BPM_POS_PARAM_Y1_REG;
	} else if (addr == SIS8300BPM_BPM2_ADDR) {
		thrMagCtrlReg = SIS8300BPM_POS_MAG_CTRL2_REG;
		thrXValReg = SIS8300BPM_POS_PARAM_X2_REG;
		thrYValReg = SIS8300BPM_POS_PARAM_Y2_REG;
	} else {
		return -1;
	}
	getDoubleParam(addr, P_IThrXPosLow, &xPosLow);
	getDoubleParam(addr, P_IThrXPosHigh, &xPosHigh);
	getDoubleParam(addr, P_IThrYPosLow, &yPosLow);
	getDoubleParam(addr, P_IThrYPosHigh, &yPosHigh);
	getDoubleParam(addr, P_IThrMagnitude, &magnitude);
	getIntegerParam(addr, P_IThrSelect, &thrControl);

	/* X position threshold */
	ret = sis8300drvbpm_double_2_Qmn(xPosHigh, sis8300drvbpm_Qmn_pos_thr, &conv, &err);
	if (ret) {
		return ret;
	}
	thrVal = (int)(conv & 0xFFFF) << 16;
	ret = sis8300drvbpm_double_2_Qmn(xPosLow, sis8300drvbpm_Qmn_pos_thr, &conv, &err);
	if (ret) {
		return ret;
	}
	thrVal |= (int)(conv  & 0xFFFF);
	ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrXValReg, thrVal));
	if (ret) {
		return ret;
	}
	/* Y position threshold */
	ret = sis8300drvbpm_double_2_Qmn(yPosHigh, sis8300drvbpm_Qmn_pos_thr, &conv, &err);
	if (ret) {
		return ret;
	}
	thrVal = (int)(conv & 0xFFFF) << 16;
	ret = sis8300drvbpm_double_2_Qmn(yPosLow, sis8300drvbpm_Qmn_pos_thr, &conv, &err);
	if (ret) {
		return ret;
	}
	thrVal |= (int)(conv  & 0xFFFF);
	ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrYValReg, thrVal));
	if (ret) {
		return ret;
	}
	/* magnitude threshold and control */
	thrVal = (int)(thrControl & 0x1) << 16;
	ret = sis8300drvbpm_double_2_Qmn(xPosLow, sis8300drvbpm_Qmn_mag_thr, &conv, &err);
	if (ret) {
		return ret;
	}
	thrVal |= (int)(conv  & 0xFFFF);
	ret = SIS8300DRV_CALL("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrMagCtrlReg, thrVal));
	if (ret) {
		return ret;
	}

	if (addr == SIS8300BPM_BPM1_ADDR) {
		mDoBpm1ThresholdUpdate = false;
	} else if (addr == SIS8300BPM_BPM2_ADDR) {
		mDoBpm2ThresholdUpdate = false;
	}

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
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    printf("%s::%s: ENTER %d (%d) = %d\n", driverName, __func__, function, addr, value);
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

    if (function == P_NearIQM) {
		mDoNearIQUpdate = true;
    } else if (function == P_NearIQN) {
		mDoNearIQUpdate = true;
    } else if (function == P_MemMux) {
    	mDoBoardSetupUpdate = true;
    } else if (function == P_MemMux10) {
    	mDoBoardSetupUpdate = true;
    } else if (function == P_FilterControl) {
    	mDoFilterControlUpdate = true;
    } else if (function == P_FilterApply) {
    	mDoFilterCoeffUpdate = true;
    } else if (function == P_IThrSelect) {
    	if (addr == SIS8300BPM_BPM1_ADDR) {
    		mDoBpm1ThresholdUpdate = true;
    	} else if (addr == SIS8300BPM_BPM2_ADDR) {
    		mDoBpm2ThresholdUpdate = true;
    	}
    } else if (function == P_IIlkControl ||
    		function == P_IIlkIRQ ||
			function == P_TrigSetup) {
    	mDoBoardSetupUpdate = true;
    } else if (function == P_IIlkClear) {
    	setIntegerParam(addr, P_IIlkStatus, 0);
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
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    printf("%s::%s: ENTER %d (%d) = %f\n", driverName, __func__, function, addr, value);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(addr, function, value);

    if (function == P_IThrXPosLow  ||
    	function == P_IThrXPosHigh ||
		function == P_IThrYPosLow  ||
		function == P_IThrYPosLow  ||
		function == P_IThrMagnitude) {
    	if (addr == SIS8300BPM_BPM1_ADDR) {
    		mDoBpm1ThresholdUpdate = true;
    	} else if (addr == SIS8300BPM_BPM2_ADDR) {
    		mDoBpm2ThresholdUpdate = true;
    	}
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
    fprintf(fp, "Struck SIS8300 based BPM\n");
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

	ret = SIS8300DRV_CALL("sis8300drvbpm_get_fw_version", sis8300drvbpm_get_fw_version(mSisDevice, &ver_device, &ver_major, &ver_minor));
	if (ret) {
		return ret;
	}
	setIntegerParam(P_BPMFirmwareVersion, ver_major << 8 | ver_minor);

	ret = SIS8300DRV_CALL("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300drvbpm_setup_dac", sis8300drvbpm_setup_dac(mSisDevice));
	if (ret) {
		return ret;
	}

	ret = SIS8300DRV_CALL("sis8300drvbpm_setup_adc_tap_delay", sis8300drvbpm_setup_adc_tap_delay(mSisDevice));
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
