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
    createParam(BpmNumBPMSamplesString,           asynParamInt32, &P_NumBPMSamples);
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
    mDoFilterControlUpdate = false;
    mDoBpm1ThresholdUpdate = false;
    mDoBpm2ThresholdUpdate = false;

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
template <typename epicsType> int ADSIS8300bpm::convertAIArraysT(int aich)
{
    int numTimePoints;
    epicsType *pData, *pVal;
    epicsUInt16 *pRaw, *pChRaw;
    int i;
    bool negative;

	printf("%s::%s: Enter\n", driverName, __func__);

    getIntegerParam(P_NumTimePoints, &numTimePoints);

    /* 0th NDArray is for raw AI data samples */
    if (! this->pArrays[0]) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)this->pArrays[0]->pData;

    if (! this->pArrays[1]) {
    	return -1;
    }
    pData = (epicsType *)this->pArrays[1]->pData;
	pChRaw = pRaw + (aich * numTimePoints);
	pVal = pData + aich;

//	char fname[32];
//	sprintf(fname, "/tmp/%d.txt", aich);
//	FILE *fp = fopen(fname, "w");
	printf("%s::%s: CH %d [%d] ", driverName, __func__, aich, numTimePoints);
	for (i = 0; i < numTimePoints; i++) {
		negative = (*(pChRaw + i) & (1 << 15)) != 0;
		if (negative) {
			*pVal = (epicsType)((double)(*(pChRaw + i) | ~((1 << 16) - 1))/* * convFactor + convOffset*/);
		} else {
			*pVal = (epicsType)((double)(*(pChRaw + i))/* * convFactor + convOffset*/);
		}

//		printf("%f ", (double)*pVal);
//		fprintf(fp, "%f\n", (double)*pVal);
		pVal += SIS8300DRV_NUM_AI_CHANNELS;
	}
	printf("\n");
//	fclose(fp);

    return 0;
}

/** Template function to compute the simulated detector data for any data type */
template <typename epicsType> int ADSIS8300bpm::convertBPMArraysT(int aich)
{
    int numTimePoints;
    int numBPMSamples;
    epicsType *pData, *pVal;
    epicsUInt16 *pRaw, *pChRaw;
    int i, j;
    double converted;
    int nearIQN, memMux, memMux10;
    
	printf("%s::%s: Enter\n", driverName, __func__);

    getIntegerParam(P_NumTimePoints, &numTimePoints);
    getIntegerParam(P_NumBPMSamples, &numBPMSamples);
    getIntegerParam(P_NearIQN, &nearIQN);
    getIntegerParam(P_MemMux, &memMux);
    getIntegerParam(P_MemMux10, &memMux10);

    /* 0th NDArray is for raw AI data samples */
    if (! this->pArrays[0]) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)this->pArrays[0]->pData;

    if (! this->pArrays[1]) {
    	return -1;
    }
    pData = (epicsType *)this->pArrays[2]->pData;

	i = 0;
	j = 0;
	pChRaw = pRaw + (aich * numTimePoints);
	pVal = pData;
	/* find the first non 0xDEAD sample in this channel */
	while (i < numTimePoints) {
		if (*pChRaw != 0xDEAD) {
			printf("%s::%s: First non 0xDEAD sample index %d\n", driverName, __func__, i);
			break;
		}
		i++;
		pChRaw++;
	}

	printf("%s::%s: CH %d [%d] BPM samples %d\n", driverName, __func__,
			aich, numTimePoints, numBPMSamples);

//	char fname[32];
//	sprintf(fname, "/tmp/bpm_X1_%d.txt", aich);
//	FILE *fp = fopen(fname, "w");
	while (i < numTimePoints) {
		/* since will always take less IQ samples from raw data than available
		 * we need to bail out when desired amount was collected */
		if (j == numBPMSamples) {
			break;
		}

		assert(i < numTimePoints);

		if ((aich == 5) || ((aich == 9) && (memMux == 2) && (memMux10 == 1))) {
			/* BPM magnitude and phase sum data is here */
			/* magnitude sum BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelMSum1) = converted;
			/* phase sum BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelPSum1) = converted * 180.0 / M_PI;
			/* magnitude sum BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelMSum2) = converted;
			/* phase sum BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelPSum2) = converted * 180.0 / M_PI;
		} else if ((aich == 6) || ((aich == 9) && (memMux == 2) && (memMux10 == 2))) {
			/* BPM antenna magnitude data is here */
			/* antenna magnitude A BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelAMag1) = converted;
			/* antenna magnitude A BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelAMag2) = converted;
			/* antenna magnitude B BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelBMag1) = converted;
			/* antenna magnitude B BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelBMag2) = converted;
			/* antenna magnitude C BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 4), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelCMag1) = converted;
			/* antenna magnitude C BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 5), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelCMag2) = converted;
			/* antenna magnitude D BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 6), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelDMag1) = converted;
			/* antenna magnitude D BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 7), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal + BPMChannelDMag2) = converted;
		} else if ((aich == 7) || ((aich == 9) && (memMux == 2) && (memMux10 == 3))) {
			/* BPM antenna phase data is here */
			/* antenna phase A BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelAPha1) = converted;
			/* antenna phase A BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelAPha2) = converted;
			/* antenna phase B BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelBPha1) = converted;
			/* antenna phase B BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelBPha2) = converted;
			/* antenna phase C BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 4), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelCPha1) = converted;
			/* antenna phase C BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 5), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelCPha2) = converted;
			/* antenna phase D BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 6), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelDPha1) = converted;
			/* antenna phase D BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 7), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal + BPMChannelDPha2) = converted;
		} else if ((aich == 8) || ((aich == 9) && (memMux == 2) && (memMux10 == 0))) {
			/* BPM X & Y position data is here */
			/* X position BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_position, &converted));
			*(pVal + BPMChannelXPos1) = converted;
//			fprintf(fp, "%f\n", converted);
			/* Y position BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_position, &converted));
			*(pVal + BPMChannelYPos1) = converted;
			/* X position BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_position, &converted));
			*(pVal + BPMChannelXPos2) = converted;
			/* Y position BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_position, &converted));
			*(pVal + BPMChannelYPos2) = converted;
		} else {
			printf("%s::%s: Should not be here!!!\n", driverName, __func__);
			assert(1 == 0);
		}

		/* adjust raw AI offset */
		i += nearIQN;
		/* adjust raw AI data pointer */
		pChRaw += nearIQN;
		/* adjust BPM offset for all channels */
		j++;
		/* adjust BPM data pointer */
		pVal += 2 * SIS8300DRV_NUM_BPM_CHANNELS;
		}
//		fclose(fp);

    return 0;
}

/** Template function to compute the simulated detector data for any data type */
template <typename epicsType> int ADSIS8300bpm::convertArraysT()
{
    size_t dims[2];
    int numTimePoints;
    int numBPMSamples;
    int numIQSamples;
    NDDataType_t dataType;
    epicsType *pData;
    int aich;
    int nearIQN, memMux, memMux10;
    int ret;

	printf("%s::%s: Enter\n", driverName, __func__);

    getIntegerParam(NDDataType, (int *)&dataType);
    getIntegerParam(P_NumTimePoints, &numTimePoints);
    getIntegerParam(P_NearIQN, &nearIQN);
    getIntegerParam(P_MemMux, &memMux);
    getIntegerParam(P_MemMux10, &memMux10);
    setIntegerParam(P_NumBPMSamples, 0);
    getIntegerParam(P_NumIQSamples, &numIQSamples);

    numBPMSamples = (int)(numTimePoints / nearIQN);
    /* number of available IQ samples might be less than we expect from above
     * calculation which is based on requested number of raw samples */
    if (numBPMSamples > numIQSamples) {
    	numBPMSamples = numIQSamples;
    }
    /* we want to ignore the first couple of samples, arbitrarily value is chosen */
    numBPMSamples -= 10;
    if (numBPMSamples < 1) {
    	printf("%s::%s: not enough raw samples requested %d for used near IQ N %d!! Need at least %d raw samples\n", driverName, __func__,
    			numTimePoints, nearIQN, 11 * nearIQN);
    	return -1;
    }
    setIntegerParam(P_NumBPMSamples, numBPMSamples);
	printf("%s::%s: nearIQ N %d, num samples %d, num BPM samples %d, memMux %d, memMux10 %d\n", driverName, __func__,
			nearIQN, numTimePoints, numBPMSamples, memMux, memMux10);

    /* 0th NDArray is for raw AI data samples */
    if (! (epicsUInt16 *)this->pArrays[0]->pData) {
    	return -1;
    }

    /* converted AI data samples of all channel are interleaved */
    dims[0] = SIS8300DRV_NUM_AI_CHANNELS;
    dims[1] = numTimePoints;

    /* 1th NDArray is for converted AI data samples */
    if (this->pArrays[1]) {
    	this->pArrays[1]->release();
    }
    this->pArrays[1] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[1]->pData;
    memset(pData, 0, SIS8300DRV_NUM_AI_CHANNELS * numTimePoints * sizeof(epicsType));

    /* converted BPM data samples of all channel are interleaved */
    dims[0] = 2 * SIS8300DRV_NUM_BPM_CHANNELS;
    dims[1] = numBPMSamples;

    /* 2nd NDArray is for converted BPM data samples */
    if (this->pArrays[2]) {
    	this->pArrays[2]->release();
    }
    this->pArrays[2] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[2]->pData;
    memset(pData, 0, 2 * SIS8300DRV_NUM_BPM_CHANNELS * numBPMSamples * sizeof(epicsType));

    for (aich = 0; aich < SIS8300DRV_NUM_AI_CHANNELS; aich++) {
        if (!(mChannelMask & (1 << aich))) {
            continue;
        }

		if ((aich < 5) || ((aich < 9) && (memMux == 2))) {
			/* AI data is here */
			ret = convertAIArraysT<epicsType>(aich);
		} else if ((aich == 9) && (memMux != 2)) {
			printf("%s::%s: Not interested in aich 9 data (memMux != 2)..\n", driverName, __func__);
		} else {
			/* BPM data is here */
			ret = convertBPMArraysT<epicsType>(aich);
		}
		if (ret) {
			return ret;
		}
    }

    return 0;
}

/** Computes the new image data */
int ADSIS8300bpm::acquireArrays()
{
    int dataType;
    int ret;

    ret = acquireRawArrays();
    if (ret) {
    	return ret;
    }

    getIntegerParam(NDDataType, &dataType);
    switch (dataType) {
        case NDInt8:
            return convertArraysT<epicsInt8>();
            break;
        case NDUInt8:
        	return convertArraysT<epicsUInt8>();
            break;
        case NDInt16:
        	return convertArraysT<epicsInt16>();
            break;
        case NDUInt16:
        	return convertArraysT<epicsUInt16>();
            break;
        case NDInt32:
        	return convertArraysT<epicsInt32>();
            break;
        case NDUInt32:
        	return convertArraysT<epicsUInt32>();
            break;
        case NDFloat32:
        	return convertArraysT<epicsFloat32>();
            break;
        case NDFloat64:
        	return convertArraysT<epicsFloat64>();
            break;
        default:
        	return -1;
        	break;
    }
}

int ADSIS8300bpm::initDeviceDone()
{
	printf("%s::%s: Enter\n", driverName, __func__);

	SIS8300DRV_CALL_RET("sis8300drvbpm_init_done", sis8300drvbpm_init_done(mSisDevice));

	return 0;
}

int ADSIS8300bpm::armDevice()
{
	printf("%s::%s: Enter\n", driverName, __func__);

	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_gop", sis8300drvbpm_clear_gop(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_pulse_done_count", sis8300drvbpm_clear_pulse_done_count(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_arm_device", sis8300drvbpm_arm_device(mSisDevice));

	setIntegerParam(P_PulseDone, 0);

	return 0;
}

int ADSIS8300bpm::disarmDevice()
{
	printf("%s::%s: Enter\n", driverName, __func__);

	SIS8300DRV_CALL_VOID("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));

	return ADSIS8300::disarmDevice();
}

int ADSIS8300bpm::waitForDevice()
{
	printf("%s::%s: Enter\n", driverName, __func__);

//	ret = SIS8300DRV_CALL("sis8300drvbpm_wait_pulse_done_position", sis8300drvbpm_wait_pulse_done_pposition(mSisDevice, SIS8300BPM_IRQ_WAIT_TIME));
// XXX: Debug!
	SIS8300DRV_CALL_RET("sis8300drvbpm_wait_pulse_done_position", sis8300drvbpm_wait_pulse_done_position(mSisDevice, 1000));

	return 0;
}

int ADSIS8300bpm::deviceDone()
{
	int oldCount;
	unsigned int pulseCount;
	unsigned int sampleCount;
	unsigned int gop;

	printf("%s::%s: Enter\n", driverName, __func__);

	pulseCount = 0;
	SIS8300DRV_CALL_RET("sis8300drvbpm_get_pulse_done_count", sis8300drvbpm_get_pulse_done_count(mSisDevice, &pulseCount));
	if (pulseCount != 1) {
		setIntegerParam(P_PulseMissed, pulseCount - 1);
	} else {
		setIntegerParam(P_PulseMissed, 0);
	}
	getIntegerParam(P_PulseCount, &oldCount);
	oldCount += pulseCount;
	setIntegerParam(P_PulseCount, oldCount);
	setIntegerParam(P_PulseDone, 1);

	SIS8300DRV_CALL_RET("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_SAMPLE_CNT_R_REG, &sampleCount));
	setIntegerParam(P_NumSamples, sampleCount);
	SIS8300DRV_CALL_RET("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_IQ_SAMPLE_CNT_REG, &sampleCount));
	setIntegerParam(P_NumIQSamples, sampleCount);

	SIS8300DRV_CALL_RET("sis8300drvbpm_get_gop", sis8300drvbpm_get_gop(mSisDevice, gop_all, &gop));
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

	return 0;
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
		SIS8300DRV_CALL_RET("sis8300drvbpm_update_parameters", sis8300drvbpm_update_parameters(mSisDevice));
	}

	return ret;
}

int ADSIS8300bpm::updateBoardSetup()
{
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

	SIS8300DRV_CALL_RET("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, SIS8300BPM_BOARD_SETUP_REG, boardSetup));
	mDoBoardSetupUpdate = false;

	return 0;
}

int ADSIS8300bpm::updateNearIQ()
{
	int n, m;

	printf("%s::%s: Enter\n", driverName, __func__);

	getIntegerParam(P_NearIQM, &m);
	getIntegerParam(P_NearIQN, &n);

	printf("%s::%s: New near IQ M = %d, N = %d\n", driverName, __func__, m, n);

	SIS8300DRV_CALL_RET("sis8300drvbpm_set_near_iq", sis8300drvbpm_set_near_iq(mSisDevice, m, n));

	mDoNearIQUpdate = false;

	return 0;
}

int ADSIS8300bpm::updateFilter()
{
	epicsFloat64 coeff[SIS8300BPM_FIR_FILTER_PARAM_NUM];
	epicsFloat64 gain;
	int filterControl;

	printf("%s::%s: Enter\n", driverName, __func__);

	if (mDoFilterCoeffUpdate) {
		getDoubleParam(P_FilterCoeff0, &coeff[0]);
		getDoubleParam(P_FilterCoeff1, &coeff[1]);
		getDoubleParam(P_FilterCoeff2, &coeff[2]);
		getDoubleParam(P_FilterCoeff3, &coeff[3]);
		getDoubleParam(P_FilterCoeff4, &coeff[4]);
		getDoubleParam(P_FilterCoeff5, &coeff[5]);
		SIS8300DRV_CALL_RET("sis8300drvbpm_set_fir_filter_param", sis8300drvbpm_set_fir_filter_param(mSisDevice, coeff, SIS8300BPM_FIR_FILTER_PARAM_NUM));

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

		SIS8300DRV_CALL_RET("sis8300drvbpm_set_fir_filter_enable", sis8300drvbpm_set_fir_filter_enable(mSisDevice, filterControl));
		mDoFilterControlUpdate = false;
	}

	return 0;
}

int ADSIS8300bpm::updateThreshold(int addr)
{
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
	SIS8300DRV_CALL_RET("sis8300drvbpm_double_2_Qmn", sis8300drvbpm_double_2_Qmn(xPosHigh, sis8300drvbpm_Qmn_position, &conv, &err));
	thrVal = (int)(conv & 0xFFFF) << 16;
	SIS8300DRV_CALL_RET("sis8300drvbpm_double_2_Qmn", sis8300drvbpm_double_2_Qmn(xPosLow, sis8300drvbpm_Qmn_position, &conv, &err));
	thrVal |= (int)(conv  & 0xFFFF);
	SIS8300DRV_CALL_RET("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrXValReg, thrVal));
	/* Y position threshold */
	SIS8300DRV_CALL_RET("sis8300drvbpm_double_2_Qmn", sis8300drvbpm_double_2_Qmn(yPosHigh, sis8300drvbpm_Qmn_position, &conv, &err));
	thrVal = (int)(conv & 0xFFFF) << 16;
	SIS8300DRV_CALL_RET("sis8300drvbpm_double_2_Qmn",sis8300drvbpm_double_2_Qmn(yPosLow, sis8300drvbpm_Qmn_position, &conv, &err));
	thrVal |= (int)(conv  & 0xFFFF);
	SIS8300DRV_CALL_RET("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrYValReg, thrVal));
	/* magnitude threshold and control */
	thrVal = (int)(thrControl & 0x1) << 16;
	SIS8300DRV_CALL_RET("sis8300drvbpm_double_2_Qmn", sis8300drvbpm_double_2_Qmn(xPosLow, sis8300drvbpm_Qmn_magnitude, &conv, &err));
	thrVal |= (int)(conv  & 0xFFFF);
	SIS8300DRV_CALL_RET("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, thrMagCtrlReg, thrVal));

	if (addr == SIS8300BPM_BPM1_ADDR) {
		mDoBpm1ThresholdUpdate = false;
	} else if (addr == SIS8300BPM_BPM2_ADDR) {
		mDoBpm2ThresholdUpdate = false;
	}

	return 0;
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
	unsigned int ver_device;
	unsigned int ver_major;
	unsigned int ver_minor;
	char message[128];

	printf("%s::%s: Enter\n", driverName, __func__);

	SIS8300DRV_CALL_RET("sis8300drvbpm_get_fw_version", sis8300drvbpm_get_fw_version(mSisDevice, &ver_device, &ver_major, &ver_minor));
	setIntegerParam(P_BPMFirmwareVersion, ver_major << 8 | ver_minor);

    if (ver_major != SIS8300BPM_VERSION_MAJOR ||
        ver_minor < SIS8300BPM_VERSION_MINOR_FIRST ||
        ver_minor > SIS8300BPM_VERSION_MINOR_LAST) {
        snprintf(message, 128, "firmware %dv%02d incompatible with software %dv%02d - %dv%02d",
        		ver_major, ver_minor, SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_FIRST,
				SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_LAST);
    	ADSIS8300_ERR(message);
        destroyDevice();
        return -1;
    }

	SIS8300DRV_CALL_RET("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_setup_dac", sis8300drvbpm_setup_dac(mSisDevice));

    snprintf(message, 128, "firmware %dv%02d compatible with software %dv%02d - %dv%02d",
    		ver_major, ver_minor, SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_FIRST,
			SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_LAST);
	ADSIS8300_INF(message);

	return 0;
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
