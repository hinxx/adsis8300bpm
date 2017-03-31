/* Bpm.cpp
 *
 * This is a driver for a BPM based on Struck SIS8300 digitizer.
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
#include <SIS8300.h>
#include <bpm.h>


static const char *driverName = "Bpm";

/* asyn addresses:
 * 0 .. 9    AI channels
 * 10        BPM1 channels
 * 11        BPM2 channels
 */
#define BPM_BPM1_ADDR		10
#define BPM_BPM2_ADDR		11

/** Constructor for Bpm; most parameters are simply passed to SIS8300::SIS8300.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class and SIS8300.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] devicePath The path to the /dev entry.
  * \param[in] maxAddr The maximum  number of asyn addr addresses this driver supports. 1 is minimum.
  * \param[in] numParams The number of parameters in the derived class.
  * \param[in] numSamples The initial number of samples.
  * \param[in] dataType The initial data type (NDDataType_t) of the arrays that this driver will create.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
Bpm::Bpm(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, NDDataType_t dataType,
		int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : SIS8300(portName, devicePath,
    		maxAddr,
    		BPM_NUM_PARAMS,
			numSamples,
			dataType,
			maxBuffers, maxMemory,
			priority,
			stackSize)

{
	int i;
    int status = asynSuccess;

    D(printf("%d addresses, %d parameters\n", maxAddr, BPM_NUM_PARAMS));

    /* adjust number of NDArrays we need to handle, 0 - AI, 1 - BPM1 and 2 - BPM2 */
    mNumArrays = 3;

    /* System wide parameters */
    createParam(BpmFwVersionString,               asynParamInt32, &mBpmFwVersion);
    createParam(BpmPulseDoneString,               asynParamInt32, &mBpmPulseDone);
    createParam(BpmPulseCountString,              asynParamInt32, &mBpmPulseCount);
    createParam(BpmPulseMissedString,             asynParamInt32, &mBpmPulseMissed);
    createParam(BpmNearIQMString,                 asynParamInt32, &mBpmNearIQM);
    createParam(BpmNearIQNString,                 asynParamInt32, &mBpmNearIQN);
    createParam(BpmNumSamplesString,              asynParamInt32, &mBpmNumSamples);
    createParam(BpmNumIQSamplesString,            asynParamInt32, &mBpmNumIQSamples);
    createParam(BpmNumBPMSamplesString,           asynParamInt32, &mBpmNumBPMSamples);
    createParam(BpmMemMuxString,                  asynParamInt32, &mBpmMemMux);
    createParam(BpmMemMux10String,                asynParamInt32, &mBpmMemMux10);
    createParam(BpmRegReadErrString,              asynParamInt32, &mBpmRegReadErr);
    createParam(BpmRegWriteErrString,             asynParamInt32, &mBpmRegWriteErr);
    createParam(BpmTrigSetupString,               asynParamInt32, &mBpmTrigSetup);
    createParam(BpmFilterControlString,           asynParamInt32, &mBpmFilterControl);
    createParam(BpmFilterCoeff0String,          asynParamFloat64, &mBpmFilterCoeff0);
    createParam(BpmFilterCoeff1String,          asynParamFloat64, &mBpmFilterCoeff1);
    createParam(BpmFilterCoeff2String,          asynParamFloat64, &mBpmFilterCoeff2);
    createParam(BpmFilterCoeff3String,          asynParamFloat64, &mBpmFilterCoeff3);
    createParam(BpmFilterCoeff4String,          asynParamFloat64, &mBpmFilterCoeff4);
    createParam(BpmFilterCoeff5String,          asynParamFloat64, &mBpmFilterCoeff5);
    createParam(BpmFilterGainString,            asynParamFloat64, &mBpmFilterGain);
    createParam(BpmFilterApplyString,             asynParamInt32, &mBpmFilterApply);
    /* BPM instance wide parameters (BPM1 or BPM2)*/
    for (i = BPM_BPM1_ADDR; i <= BPM_BPM2_ADDR; i++) {
		createParam(i, BpmIEnableString,                 asynParamInt32, &mBpmIEnable);
		createParam(i, BpmIThrXPosLowString,           asynParamFloat64, &mBpmIThrXPosLow);
		createParam(i, BpmIThrXPosHighString,          asynParamFloat64, &mBpmIThrXPosHigh);
		createParam(i, BpmIThrYPosLowString,           asynParamFloat64, &mBpmIThrYPosLow);
		createParam(i, BpmIThrYPosHighString,          asynParamFloat64, &mBpmIThrYPosHigh);
		createParam(i, BpmIThrMagnitudeString,         asynParamFloat64, &mBpmIThrMagnitude);
		createParam(i, BpmIThrSelectString,              asynParamInt32, &mBpmIThrSelect);
		createParam(i, BpmIIlkControlString,             asynParamInt32, &mBpmIIlkControl);
		createParam(i, BpmIIlkClearString,               asynParamInt32, &mBpmIIlkClear);
		createParam(i, BpmIIlkStatusString,              asynParamInt32, &mBpmIIlkStatus);
		createParam(i, BpmIIlkIRQString,                 asynParamInt32, &mBpmIIlkIRQ);
		createParam(i, BpmIDivXPosErrString,             asynParamInt32, &mBpmIDivXPosErr);
		createParam(i, BpmIDivYPosErrString,             asynParamInt32, &mBpmIDivYPosErr);
    }

    mDoBoardSetupUpdate = false;
    mDoNearIQUpdate = false;
    mDoFilterCoeffUpdate = false;
    mDoFilterControlUpdate = false;
    mDoBpm1ThresholdUpdate = false;
    mDoBpm2ThresholdUpdate = false;

    if (status) {
        E(printf("unable to set parameters\n"));
        return;
    }

    this->lock();
    initDevice();
    this->unlock();

	I(printf("Init done...\n"));
}

Bpm::~Bpm() {
	I(printf("Shutdown complete!\n"));
}

/** Template function to compute the simulated detector data for any data type */
template <typename epicsType> int Bpm::convertAIArraysT(int aich)
{
    int numAiSamples;
    epicsType *pData, *pVal;
    epicsUInt16 *pRaw, *pChRaw;
    int i;
    bool negative;

	D(printf("Enter\n"));

    getIntegerParam(mSISNumAiSamples, &numAiSamples);

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)mRawDataArray->pData;

    /* 0th NDArray is for converted AI data samples */
    if (! this->pArrays[0]) {
    	return -1;
    }
    pData = (epicsType *)this->pArrays[0]->pData;
	pChRaw = pRaw + (aich * numAiSamples);
	pVal = pData + aich;

//	char fname[32];
//	sprintf(fname, "/tmp/%d.txt", aich);
//	FILE *fp = fopen(fname, "w");
	D(printf("CH %d [%d] ", aich, numAiSamples));
	for (i = 0; i < numAiSamples; i++) {
		negative = (*(pChRaw + i) & (1 << 15)) != 0;
		if (negative) {
			*pVal = (epicsType)((double)(*(pChRaw + i) | ~((1 << 16) - 1))/* * convFactor + convOffset*/);
		} else {
			*pVal = (epicsType)((double)(*(pChRaw + i))/* * convFactor + convOffset*/);
		}

//		printf("%f ", (double)*pVal);
//		fprintf(fp, "%f\n", (double)*pVal);
		pVal += SIS8300_NUM_CHANNELS;
	}
	D0(printf("\n"));
//	fclose(fp);

    return 0;
}

template <typename epicsType> int Bpm::convertBPMArraysT(int aich)
{
    int numAiSamples;
    int numBPMSamples;
    epicsType *pData1, *pVal1;
    epicsType *pData2, *pVal2;
    epicsUInt16 *pRaw, *pChRaw;
    int i, j;
    double converted;
    int nearIQN, memMux, memMux10;
    
	D(printf("Enter\n"));

    getIntegerParam(mSISNumAiSamples, &numAiSamples);
    getIntegerParam(mBpmNumBPMSamples, &numBPMSamples);
    getIntegerParam(mBpmNearIQN, &nearIQN);
    getIntegerParam(mBpmMemMux, &memMux);
    getIntegerParam(mBpmMemMux10, &memMux10);

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }
    pRaw = (epicsUInt16 *)mRawDataArray->pData;
    /* 1st NDArray is for BPM 1 data samples */
    if (! this->pArrays[1]) {
    	return -1;
    }
    pData1 = (epicsType *)this->pArrays[1]->pData;
    /* 2nd NDArray is for BPM 2 data samples */
    if (! this->pArrays[2]) {
    	return -1;
    }
    pData2 = (epicsType *)this->pArrays[2]->pData;

	i = 0;
	j = 0;
	pChRaw = pRaw + (aich * numAiSamples);
	pVal1 = pData1;
	pVal2 = pData2;

	/* find the first non 0xDEAD sample in this channel */
	while (i < numAiSamples) {
		if (*pChRaw != 0xDEAD) {
			D(printf("First non 0xDEAD sample index %d\n", i));
			break;
		}
		i++;
		pChRaw++;
	}

	D(printf("CH %d [%d] BPM samples %d\n", aich, numAiSamples, numBPMSamples));

//	char fname[32];
//	sprintf(fname, "/tmp/bpm_X1_%d.txt", aich);
//	FILE *fp = fopen(fname, "w");
	while (i < numAiSamples) {
		/* since will always take less IQ samples from raw data than available
		 * we need to bail out when desired amount was collected */
		if (j == numBPMSamples) {
			break;
		}

		assert(i < numAiSamples);

		if ((aich == 5) || ((aich == 9) && (memMux == 2) && (memMux10 == 1))) {
			/* BPM magnitude and phase sum data is here */
			/* magnitude sum BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelMSum) = converted;
			/* phase sum BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelPSum) = converted * 180.0 / M_PI;
			/* magnitude sum BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal2 + eBPMChannelMSum) = converted;
			/* phase sum BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal2 + eBPMChannelPSum) = converted * 180.0 / M_PI;
		} else if ((aich == 6) || ((aich == 9) && (memMux == 2) && (memMux10 == 2))) {
			/* BPM antenna magnitude data is here */
			/* antenna magnitude A BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelAMag) = converted;
			/* antenna magnitude A BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal2 + eBPMChannelAMag) = converted;
			/* antenna magnitude B BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelBMag) = converted;
			/* antenna magnitude B BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal2 + eBPMChannelBMag) = converted;
			/* antenna magnitude C BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 4), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelCMag) = converted;
			/* antenna magnitude C BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 5), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal2 + eBPMChannelCMag) = converted;
			/* antenna magnitude D BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 6), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelDMag) = converted;
			/* antenna magnitude D BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 7), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal2 + eBPMChannelDMag) = converted;
			/* reference magnitude BPM 1 and BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 8), sis8300drvbpm_Qmn_magnitude, &converted));
			*(pVal1 + eBPMChannelRefMag) = converted;
			*(pVal2 + eBPMChannelRefMag) = converted;
		} else if ((aich == 7) || ((aich == 9) && (memMux == 2) && (memMux10 == 3))) {
			/* BPM antenna phase data is here */
			/* antenna phase A BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelAPha) = converted * 180.0 / M_PI;
			/* antenna phase A BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal2 + eBPMChannelAPha) = converted * 180.0 / M_PI;
			/* antenna phase B BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelBPha) = converted * 180.0 / M_PI;
			/* antenna phase B BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal2 + eBPMChannelBPha) = converted * 180.0 / M_PI;
			/* antenna phase C BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 4), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelCPha) = converted * 180.0 / M_PI;
			/* antenna phase C BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 5), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal2 + eBPMChannelCPha) = converted * 180.0 / M_PI;
			/* antenna phase D BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 6), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelDPha) = converted * 180.0 / M_PI;
			/* antenna phase D BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 7), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal2 + eBPMChannelDPha) = converted * 180.0 / M_PI;
			/* reference phase BPM 1 and BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 8), sis8300drvbpm_Qmn_phase, &converted));
			*(pVal1 + eBPMChannelRefPha) = converted * 180.0 / M_PI;
			*(pVal2 + eBPMChannelRefPha) = converted * 180.0 / M_PI;
		} else if ((aich == 8) || ((aich == 9) && (memMux == 2) && (memMux10 == 0))) {
			/* BPM X & Y position data is here */
			/* X position BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw), sis8300drvbpm_Qmn_position, &converted));
			*(pVal1 + eBPMChannelXPos) = converted;
//			fprintf(fp, "%f\n", converted);
			/* Y position BPM 1 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 1), sis8300drvbpm_Qmn_position, &converted));
			*(pVal1 + eBPMChannelYPos) = converted;
			/* X position BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 2), sis8300drvbpm_Qmn_position, &converted));
			*(pVal2 + eBPMChannelXPos) = converted;
			/* Y position BPM 2 */
			SIS8300DRV_CALL_RET("sis8300drvbpm_Qmn_2_double", sis8300drvbpm_Qmn_2_double((epicsUInt32)*(pChRaw + 3), sis8300drvbpm_Qmn_position, &converted));
			*(pVal2 + eBPMChannelYPos) = converted;
		} else {
			E(printf("Should not be here!!!\n"));
			assert(1 == 0);
		}

		/* adjust raw AI offset */
		i += nearIQN;
		/* adjust raw AI data pointer */
		pChRaw += nearIQN;
		/* adjust BPM offset for all channels */
		j++;
		/* adjust BPM data pointer */
		pVal1 += BPM_NUM_CHANNELS;
		pVal2 += BPM_NUM_CHANNELS;
		}
//		fclose(fp);

    return 0;
}

template <typename epicsType> int Bpm::convertArraysT()
{
    size_t dims[2];
    int numAiSamples;
    int numBPMSamples;
    NDDataType_t dataType;
    epicsType *pData;
    int aich;
    int nearIQN, memMux;
    int ret;

	D(printf("Enter\n"));

    getIntegerParam(NDDataType, (int *)&dataType);
    getIntegerParam(mSISNumAiSamples, &numAiSamples);
    getIntegerParam(mBpmNearIQN, &nearIQN);
    getIntegerParam(mBpmMemMux, &memMux);
    getIntegerParam(mBpmNumBPMSamples, &numBPMSamples);

    /* local NDArray is for raw AI data samples */
    if (! mRawDataArray) {
    	return -1;
    }

    /* converted AI data samples of all channel are interleaved */
    dims[0] = SIS8300_NUM_CHANNELS;
    dims[1] = numAiSamples;

    /* 0th NDArray is for converted AI data samples */
    if (this->pArrays[0]) {
    	this->pArrays[0]->release();
    }
    this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[0]->pData;
    memset(pData, 0, SIS8300_NUM_CHANNELS * numAiSamples * sizeof(epicsType));

    /* converted BPM data samples of all channels are interleaved */
    dims[0] = BPM_NUM_CHANNELS;
    dims[1] = numBPMSamples;

    /* 1st NDArray is for converted BPM1 data samples */
    if (this->pArrays[1]) {
    	this->pArrays[1]->release();
    }
    this->pArrays[1] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[1]->pData;
    memset(pData, 0, BPM_NUM_CHANNELS * numBPMSamples * sizeof(epicsType));

    /* 2nd NDArray is for converted BPM2 data samples */
    if (this->pArrays[2]) {
    	this->pArrays[2]->release();
    }
    this->pArrays[2] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsType *)this->pArrays[2]->pData;
    memset(pData, 0, BPM_NUM_CHANNELS * numBPMSamples * sizeof(epicsType));

    for (aich = 0; aich < SIS8300_NUM_CHANNELS; aich++) {
        if (!(mChannelMask & (1 << aich))) {
            continue;
        }

		if ((aich < 5) || ((aich < 9) && (memMux == 2))) {
			/* AI data is here */
			ret = convertAIArraysT<epicsType>(aich);
		} else if ((aich == 9) && (memMux != 2)) {
			D(printf("Not interested in aich 9 data (memMux != 2)..\n"));
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

int Bpm::acquireArrays()
{
    int dataType;
    int ret;

	D(printf("Enter\n"));

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

int Bpm::initDeviceDone()
{
	D(printf("Enter\n"));

	SIS8300DRV_CALL_RET("sis8300drvbpm_init_done", sis8300drvbpm_init_done(mSisDevice));

	return 0;
}

int Bpm::armDevice()
{
	D(printf("Enter\n"));

	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_gop", sis8300drvbpm_clear_gop(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_pulse_done_count", sis8300drvbpm_clear_pulse_done_count(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_arm_device", sis8300drvbpm_arm_device(mSisDevice));

	setIntegerParam(mBpmPulseDone, 0);

	return 0;
}

int Bpm::disarmDevice()
{
	D(printf("Enter\n"));

	/* XXX: Calling this here causes major problems in next pulse acquisition!
	 *      It seems it is not needed.. ask firmware guy for verification.
	 */
	//SIS8300DRV_CALL_VOID("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));

	return SIS8300::disarmDevice();
}

int Bpm::waitForDevice()
{
	D(printf("Enter\n"));

	SIS8300DRV_CALL_RET("sis8300drvbpm_wait_pulse_done_position", sis8300drvbpm_wait_pulse_done_position(mSisDevice, BPM_IRQ_WAIT_TIME));

	return 0;
}

int Bpm::deviceDone()
{
	int oldCount;
	unsigned int pulseCount;
	unsigned int sampleCount;
	unsigned int gop;
	int numAiSamples;
    int numBPMSamples;
    int numIQSamples;
    int nearIQN;

	D(printf("Enter\n"));

	pulseCount = 0;
	SIS8300DRV_CALL_RET("sis8300drvbpm_get_pulse_done_count", sis8300drvbpm_get_pulse_done_count(mSisDevice, &pulseCount));
	if (pulseCount != 1) {
		setIntegerParam(mBpmPulseMissed, pulseCount - 1);
	} else {
		setIntegerParam(mBpmPulseMissed, 0);
	}
	getIntegerParam(mBpmPulseCount, &oldCount);
	oldCount += pulseCount;
	setIntegerParam(mBpmPulseCount, oldCount);
	setIntegerParam(mBpmPulseDone, 1);

	SIS8300DRV_CALL_RET("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_IQ_SAMPLE_CNT_REG, &sampleCount));
	numIQSamples = sampleCount;
	SIS8300DRV_CALL_RET("sis8300drv_reg_read", sis8300drv_reg_read(mSisDevice, SIS8300BPM_SAMPLE_CNT_R_REG, &sampleCount));
	D(printf("SAMPLES %10d IQ %10d\n", sampleCount, numIQSamples));
	setIntegerParam(mBpmNumSamples, sampleCount);
	setIntegerParam(mBpmNumIQSamples, numIQSamples);

    getIntegerParam(mSISNumAiSamples, &numAiSamples);
    getIntegerParam(mBpmNearIQN, &nearIQN);
    numBPMSamples = (int)(numAiSamples / nearIQN);
    /* number of available IQ samples might be less than we expect from above
     * calculation which is based on requested number of raw samples */
    if (numBPMSamples > numIQSamples) {
    	numBPMSamples = numIQSamples;
    }
    /* we want to ignore the first couple of samples, arbitrarily value is chosen */
    numBPMSamples -= 10;
    if (numBPMSamples < 1) {
    	E(printf("not enough raw samples requested %d for used near IQ N %d!! Need at least %d raw samples\n",
    			numAiSamples, nearIQN, 11 * nearIQN));
    	return -1;
    }
    setIntegerParam(mBpmNumBPMSamples, numBPMSamples);
	D(printf("nearIQ N %d, num samples %d, num BPM samples %d\n",
			nearIQN, numAiSamples, numBPMSamples));

	SIS8300DRV_CALL_RET("sis8300drvbpm_get_gop", sis8300drvbpm_get_gop(mSisDevice, gop_all, &gop));
	if (gop & (1 << gop_X1_pos_div_error)) {
		setIntegerParam(BPM_BPM1_ADDR, mBpmIDivXPosErr, 1);
	} else {
		setIntegerParam(BPM_BPM1_ADDR, mBpmIDivXPosErr, 0);
	}
	if (gop & (1 << gop_Y1_pos_div_error)) {
		setIntegerParam(BPM_BPM1_ADDR, mBpmIDivYPosErr, 1);
	} else {
		setIntegerParam(BPM_BPM1_ADDR, mBpmIDivYPosErr, 0);
	}
	if (gop & (1 << gop_X2_pos_div_error)) {
		setIntegerParam(BPM_BPM2_ADDR, mBpmIDivXPosErr, 1);
	} else {
		setIntegerParam(BPM_BPM2_ADDR, mBpmIDivXPosErr, 0);
	}
	if (gop & (1 << gop_Y2_pos_div_error)) {
		setIntegerParam(BPM_BPM2_ADDR, mBpmIDivYPosErr, 1);
	} else {
		setIntegerParam(BPM_BPM2_ADDR, mBpmIDivYPosErr, 0);
	}
	if (gop & (1 << gop_read_error)) {
		setIntegerParam(mBpmRegReadErr, 1);
	} else {
		setIntegerParam(mBpmRegReadErr, 0);
	}
	if (gop & (1 << gop_write_error)) {
		setIntegerParam(mBpmRegWriteErr, 1);
	} else {
		setIntegerParam(mBpmRegWriteErr, 0);
	}
	if (gop & (1 << gop_position1_error)) {
		setIntegerParam(BPM_BPM1_ADDR, mBpmIIlkStatus, 1);
	}
	if (gop & (1 << gop_position2_error)) {
		setIntegerParam(BPM_BPM2_ADDR, mBpmIIlkStatus, 1);
	}

	return 0;
}

int Bpm::updateParameters()
{
	int ret = 0;
    bool doShadowUpdate = false;

	D(printf("Enter\n"));

	ret = SIS8300::updateParameters();
	if (ret) {
		return ret;
	}

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
		ret = updateThreshold(BPM_BPM1_ADDR);
		doShadowUpdate = true;
	}
	if (mDoBpm2ThresholdUpdate) {
		ret = updateThreshold(BPM_BPM2_ADDR);
		doShadowUpdate = true;
	}
	if (doShadowUpdate) {
		SIS8300DRV_CALL_RET("sis8300drvbpm_update_parameters", sis8300drvbpm_update_parameters(mSisDevice));
	}

	return ret;
}

int Bpm::updateBoardSetup()
{
	int ilk1IRQ, ilk2IRQ;
	int ilk1Ctrl, ilk2Ctrl;
	int memMux, memMux10;
	int trigSetup;
	unsigned int boardSetup;

	D(printf("Enter\n"));

	getIntegerParam(mBpmMemMux, &memMux);
	getIntegerParam(mBpmMemMux10, &memMux10);
	getIntegerParam(mBpmTrigSetup, &trigSetup);
	getIntegerParam(BPM_BPM1_ADDR, mBpmIIlkControl, &ilk1Ctrl);
	getIntegerParam(BPM_BPM2_ADDR, mBpmIIlkControl, &ilk2Ctrl);
	getIntegerParam(BPM_BPM1_ADDR, mBpmIIlkIRQ, &ilk1IRQ);
	getIntegerParam(BPM_BPM2_ADDR, mBpmIIlkIRQ, &ilk2IRQ);

	/* XXX: Handle the rest of the bits in board setup reg!
	 *      Do not clobber the other bits in board setup reg! */
	boardSetup = (ilk1IRQ & 0x1) << 21 | (ilk2IRQ & 0x1) << 20 |
			(ilk1Ctrl & 0x1) << 19 | (ilk2Ctrl & 0x1) << 18 |
			(memMux & 0x3) << 8 | (memMux10 & 0x3) << 6 |
			(trigSetup & 0x3) << 0;
	D(printf("New board setup 0x%08X\n", boardSetup));

	SIS8300DRV_CALL_RET("sis8300drv_reg_write", sis8300drv_reg_write(mSisDevice, SIS8300BPM_BOARD_SETUP_REG, boardSetup));
	mDoBoardSetupUpdate = false;

	return 0;
}

int Bpm::updateNearIQ()
{
	int n, m;

	D(printf("Enter\n"));

	getIntegerParam(mBpmNearIQM, &m);
	getIntegerParam(mBpmNearIQN, &n);

	D(printf("New near IQ M = %d, N = %d\n", m, n));

	SIS8300DRV_CALL_RET("sis8300drvbpm_set_near_iq", sis8300drvbpm_set_near_iq(mSisDevice, m, n));

	mDoNearIQUpdate = false;

	return 0;
}

int Bpm::updateFilter()
{
	epicsFloat64 coeff[SIS8300BPM_FIR_FILTER_PARAM_NUM];
	epicsFloat64 gain;
	int filterControl;

	D(printf("Enter\n"));

	if (mDoFilterCoeffUpdate) {
		getDoubleParam(mBpmFilterCoeff0, &coeff[0]);
		getDoubleParam(mBpmFilterCoeff1, &coeff[1]);
		getDoubleParam(mBpmFilterCoeff2, &coeff[2]);
		getDoubleParam(mBpmFilterCoeff3, &coeff[3]);
		getDoubleParam(mBpmFilterCoeff4, &coeff[4]);
		getDoubleParam(mBpmFilterCoeff5, &coeff[5]);
		SIS8300DRV_CALL_RET("sis8300drvbpm_set_fir_filter_param", sis8300drvbpm_set_fir_filter_param(mSisDevice, coeff, SIS8300BPM_FIR_FILTER_PARAM_NUM));

		gain = 2 * coeff[0] + \
				2 * coeff[2] + \
				2 * coeff[3] + \
				2 * coeff[4] + \
				2 * coeff[5] + \
				coeff[1];
		setDoubleParam(mBpmFilterGain, gain);
		mDoFilterCoeffUpdate = false;
	}

	if (mDoFilterControlUpdate) {
		getIntegerParam(mBpmFilterControl, &filterControl);

		D(printf("New filter control %d\n", filterControl));

		SIS8300DRV_CALL_RET("sis8300drvbpm_set_fir_filter_enable", sis8300drvbpm_set_fir_filter_enable(mSisDevice, filterControl));
		mDoFilterControlUpdate = false;
	}

	return 0;
}

int Bpm::updateThreshold(int addr)
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

	D(printf("Enter\n"));

	if (addr == BPM_BPM1_ADDR) {
		thrMagCtrlReg = SIS8300BPM_POS_MAG_CTRL1_REG;
		thrXValReg = SIS8300BPM_POS_PARAM_X1_REG;
		thrYValReg = SIS8300BPM_POS_PARAM_Y1_REG;
	} else if (addr == BPM_BPM2_ADDR) {
		thrMagCtrlReg = SIS8300BPM_POS_MAG_CTRL2_REG;
		thrXValReg = SIS8300BPM_POS_PARAM_X2_REG;
		thrYValReg = SIS8300BPM_POS_PARAM_Y2_REG;
	} else {
		return -1;
	}
	getDoubleParam(addr, mBpmIThrXPosLow, &xPosLow);
	getDoubleParam(addr, mBpmIThrXPosHigh, &xPosHigh);
	getDoubleParam(addr, mBpmIThrYPosLow, &yPosLow);
	getDoubleParam(addr, mBpmIThrYPosHigh, &yPosHigh);
	getDoubleParam(addr, mBpmIThrMagnitude, &magnitude);
	getIntegerParam(addr, mBpmIThrSelect, &thrControl);

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

	if (addr == BPM_BPM1_ADDR) {
		mDoBpm1ThresholdUpdate = false;
	} else if (addr == BPM_BPM2_ADDR) {
		mDoBpm2ThresholdUpdate = false;
	}

	return 0;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus Bpm::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int addr;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    D(printf("Enter %d (%d) = %d\n", function, addr, value));
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(addr, function, value);

    if (function == mBpmNearIQM) {
		mDoNearIQUpdate = true;
    } else if (function == mBpmNearIQN) {
		mDoNearIQUpdate = true;
    } else if (function == mBpmMemMux) {
    	mDoBoardSetupUpdate = true;
    } else if (function == mBpmMemMux10) {
    	mDoBoardSetupUpdate = true;
    } else if (function == mBpmFilterControl) {
    	mDoFilterControlUpdate = true;
    } else if (function == mBpmFilterApply) {
    	mDoFilterCoeffUpdate = true;
    } else if (function == mBpmIThrSelect) {
    	if (addr == BPM_BPM1_ADDR) {
    		mDoBpm1ThresholdUpdate = true;
    	} else if (addr == BPM_BPM2_ADDR) {
    		mDoBpm2ThresholdUpdate = true;
    	}
    } else if (function == mBpmIIlkControl ||
    		function == mBpmIIlkIRQ ||
			function == mBpmTrigSetup) {
    	mDoBoardSetupUpdate = true;
    } else if (function == mBpmIIlkClear) {
    	setIntegerParam(addr, mBpmIIlkStatus, 0);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < BPM_FIRST_PARAM) {
        	status = SIS8300::writeInt32(pasynUser, value);
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
asynStatus Bpm::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int addr;
    asynStatus status = asynSuccess;

    getAddress(pasynUser, &addr);
    D(printf("Enter %d (%d) = %f\n", function, addr, value));

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(addr, function, value);

    if (function == mBpmIThrXPosLow  ||
    	function == mBpmIThrXPosHigh ||
		function == mBpmIThrYPosLow  ||
		function == mBpmIThrYPosLow  ||
		function == mBpmIThrMagnitude) {
    	if (addr == BPM_BPM1_ADDR) {
    		mDoBpm1ThresholdUpdate = true;
    	} else if (addr == BPM_BPM2_ADDR) {
    		mDoBpm2ThresholdUpdate = true;
    	}
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < BPM_FIRST_PARAM) {
        	status = SIS8300::writeFloat64(pasynUser, value);
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
void Bpm::report(FILE *fp, int details)
{
    fprintf(fp, "Struck SIS8300 based BPM\n");
    if (details > 0) {
    }

    /* Invoke the base class method */
    SIS8300::report(fp, details);
}

int Bpm::initDevice()
{
	unsigned int ver_device;
	unsigned int ver_major;
	unsigned int ver_minor;
	char message[128];

	D(printf("Enter\n"));

	SIS8300DRV_CALL_RET("sis8300drvbpm_get_fw_version", sis8300drvbpm_get_fw_version(mSisDevice, &ver_device, &ver_major, &ver_minor));
	setIntegerParam(mBpmFwVersion, ver_major << 8 | ver_minor);
/*
	
	XXX: Ignore this until dust settles..
	
    if (ver_major != SIS8300BPM_VERSION_MAJOR ||
        ver_minor < SIS8300BPM_VERSION_MINOR_FIRST ||
        ver_minor > SIS8300BPM_VERSION_MINOR_LAST) {
        snprintf(message, 128, "firmware %dv%02d incompatible with software %dv%02d - %dv%02d",
        		ver_major, ver_minor, SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_FIRST,
				SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_LAST);
    	SIS8300_ERR(message);
        destroyDevice();
        return -1;
    }
*/
	SIS8300DRV_CALL_RET("sis8300drvbpm_sw_reset", sis8300drvbpm_sw_reset(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_setup_dac", sis8300drvbpm_setup_dac(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_gop", sis8300drvbpm_clear_gop(mSisDevice));
	SIS8300DRV_CALL_RET("sis8300drvbpm_clear_pulse_done_count", sis8300drvbpm_clear_pulse_done_count(mSisDevice));

    snprintf(message, 128, "firmware %dv%02d compatible with software %dv%02d - %dv%02d",
    		ver_major, ver_minor, SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_FIRST,
			SIS8300BPM_VERSION_MAJOR, SIS8300BPM_VERSION_MINOR_LAST);
	SIS8300_INF(message);

	return 0;
}

/** Configuration command, called directly or from iocsh */
extern "C" int BpmConfig(const char *portName, const char *devicePath,
		int maxAddr, int numSamples, int dataType, int maxBuffers, int maxMemory,
		int priority, int stackSize)
{
    new Bpm(portName, devicePath,
    		maxAddr,
    		numSamples,
			(NDDataType_t)dataType,
			(maxBuffers < 0) ? 0 : maxBuffers,
			(maxMemory < 0) ? 0 : maxMemory,
			priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg configArg0 = {"Port name",     iocshArgString};
static const iocshArg configArg1 = {"Device path",   iocshArgString};
static const iocshArg configArg2 = {"# channels",    iocshArgInt};
static const iocshArg configArg3 = {"# samples",     iocshArgInt};
static const iocshArg configArg4 = {"Data type",     iocshArgInt};
static const iocshArg configArg5 = {"maxBuffers",    iocshArgInt};
static const iocshArg configArg6 = {"maxMemory",     iocshArgInt};
static const iocshArg configArg7 = {"priority",      iocshArgInt};
static const iocshArg configArg8 = {"stackSize",     iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                                     &configArg1,
													 &configArg2,
													 &configArg3,
													 &configArg4,
													 &configArg5,
													 &configArg6,
													 &configArg7,
													 &configArg8};
static const iocshFuncDef configSIS8300Bpm = {"BpmConfig", 9, configArgs};
static void configBpmCallFunc(const iocshArgBuf *args)
{
    BpmConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
    		args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}


static void BpmRegister(void)
{
    iocshRegister(&configSIS8300Bpm, configBpmCallFunc);
}

extern "C" {
epicsExportRegistrar(BpmRegister);
}
