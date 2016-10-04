/**
 * Struck 8300 BPM Linux userspace library.
 * Copyright (C) 2016 ESS ERIC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @file sis8300drvbpm.c
 * @brief Implementation of sis8300 BPM userspace API.
 * @author Hinko Kocevar
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <sis8300drv.h>
#include <sis8300drv_utils.h>
#include <sis8300_reg.h>

#include <sis8300drvbpm.h>
#include <sis8300bpm_reg.h>

/* extern */
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_near_iq =
        { .int_bits_m = 2,  .frac_bits_n = 30, .is_signed = 1 };
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_pos_threshold =
        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 1 };
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_mag_threshold =
        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 0 };

/**
 * @brief Setup the DAC
 *
 * @param [in]    sisuser    User context struct
 *
 * @return status_success       Set successful.
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * This will setup the DAC control register. The function should be
 * called once at device initialization.
 *
 * 0x33 = 00110011
 *
 * bit 16&17: mux DAC clk select
 * 00 = FPGA clock
 *
 * bit0 : test mode bit 0
 * bit1 : test mode bit 1
 * 00 = Data from DAC data register
 * 01 = Ramp test mode
 * 02 = ADC1/ADC2 -> DAC1/DAC2
 * 03 = Reserved
 *
 * bit 4: 1 two's complement, 0 binary
 * bit 5: 0 power down, 1 power up
 */

/* XXX: Should probably be part of generic SIS8300 driver library and better
 *      parameterized. */
int sis8300drvbpm_setup_dac(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;
    uint32_t ui32_reg_val = 0x33;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300_DAC_CONTROL_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    printf("%s: DAC setup done, reg = 0x%x, val = 0x%x\n", __func__,
            SIS8300_DAC_CONTROL_REG, ui32_reg_val);

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Set ADC input tap delay for ADC 1 and 0
 *
 * @param [in]    sisuser    User context struct
 *
 * @return status_success       Set successful.
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * This will setup the Data Strobe Timing. The function should be called
 * once at the device initialization
 *
 * bit 8: ADC 1/2 Select Bit
 * bit 7-0: tap delay val (x78 ps)
 */

/* XXX: Should probably be part of generic SIS8300 driver library and better
 *      parameterized. */
int sis8300drvbpm_setup_adc_tap_delay(sis8300drv_usr *sisuser) {
    int status;
    sis8300drv_dev *sisdevice;
    uint32_t ui32_reg_val;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* XXX: This LLRF specific!!!! */
    ui32_reg_val = 0x103;

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300_ADC_INPUT_TAP_DELAY_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    printf("%s: ADC tap delay setup done, reg = 0x%x, val = 0x%x\n", __func__,
			SIS8300_ADC_INPUT_TAP_DELAY_REG, ui32_reg_val);

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}


/**
 * @brief Convert double to signed or unsigned Qmn, where
 *        m = int_bits, n = frac_bits (sign bit inclusive)
 *
 * @param [in]  val          Value to convert
 * @param [in]  int_bits_m   Number of integer bits
 * @param [in]  frac_bits_n  Number of fractional bits
 * @param [out] converted    Wil hold the converted value on success
 * @param [out] err          Holds the error obtained during conversion
 *                           because of limited Qmn format resolution
 *
 * @return status_success           Conversion successful
 * @return status_argument_range    Given double is out of bounds
 * @return argument_invalid         int_bits + frac bits > 32
 *
 * Limits on val parameter are:
 *     Resolution: 2^-n
 *        Range:
 *          Signed:     [-2^(m-1), 2^(m-1) - 2^-n]
 *             Unsigned:   [0, 2^m - 2^-n]
 * Anything out of this range returns an error.
 *
 * Since custom logic defines no types with > 32 bits,
 * int_bits + frac bits <= 32 has to hold or an error is returned.
 *
 * For conversion the function from document is used.
 */
int sis8300drvbpm_double_2_Qmn(
        double val, sis8300drvbpm_Qmn Qmn,
        uint32_t *converted, double *err) {

    int64_t val_int64;

    double pow_2_frac_bits =
                (double) (0x1UL << Qmn.frac_bits_n);
    double pow_2_frac_bits_int_bits =
                (double) (0x1UL << (Qmn.int_bits_m + Qmn.frac_bits_n));


    /* There are no types defined for custom logic that
     * require > 32 bits */
    if (Qmn.int_bits_m + Qmn.frac_bits_n > 32U) {
        return status_argument_invalid;
    }

    /* Convert to fixed number representation */
    val_int64 = (int64_t) (val * pow_2_frac_bits);

    /* Get error */
    *err = val - (double) val_int64 / pow_2_frac_bits;

    /* Check if signed */
    if (Qmn.is_signed) {
        if (val_int64 < 0) {
            val_int64 += (int64_t) pow_2_frac_bits_int_bits;
        }
        /* check upper limit of signed int */
        else if (val_int64 > (pow_2_frac_bits_int_bits / 2.0 - 1.0)) {
            return status_argument_range;
        }
    }

    /* Check limits
     * check if val_int64 has more than int_bits + frac_bits set
     * => will take care of negative unsigned, too large unsigned,
     * to large signed negative */
    if (((uint64_t) val_int64) >> (Qmn.int_bits_m + Qmn.frac_bits_n)) {
        return status_argument_range;
    }

    /* Write the converted value */
    *converted = (uint32_t) val_int64;

    return 0;
}

/**
 * @brief Convert signed or unsigned Qmn to double, where
 *        m = int_bits, n = frac_bits (including sign bit)
 *
 * @param [in]     val          Value to convert
 * @param [in]  int_bits_m   Number of integer bits
 * @param [in]  frac_bits_n  Number of fractional bits
 * @param [in]  is_signed    Is the format to convert from signed or unsigned
 * @param [out] converted      Holds converted value on success
 *
 * @return status_success            Conversion successfull
 * @return argument_invalid         int_bits + frac bits > 32
 *
 * this is the inverse function of @see sis8300llrfdrv_double_2_Qmn
 */
void sis8300drvbpm_Qmn_2_double(
        uint32_t val, sis8300drvbpm_Qmn Qmn, double *converted) {

    double pow_2_frac_bits =
                (double) (0x1UL << Qmn.frac_bits_n);
    double pow_2_frac_bits_int_bits =
                (double) (0x1UL << (Qmn.int_bits_m + Qmn.frac_bits_n));

    *converted = (double) val;

    /* check if val is signed and if it is negative */
    if (Qmn.is_signed &&
        (*converted > (pow_2_frac_bits_int_bits / 2.0 - 1.0))) {

        *converted -= pow_2_frac_bits_int_bits;
    }

    /* convert to double */
    *converted /= pow_2_frac_bits;

    return;
}

/** @brief Setup near IQ parameters and tables, where:
 *         table[n] = sin(2 PI M / N) and cos(2 PI M / N)
 *
 *  @param [in] sisuser Device user context struct
 *  @param [in] M       M value
 *  @param [in] N       N value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 * @return status_argument_range    Argument value is out of range
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300llrfdrv_set_near_iq(
        sis8300drv_usr *sisuser, unsigned M, unsigned N) {

    uint32_t u32_reg_val = 0;
    int32_t i32_reg_val = 0;
    double double_val, conv_fact;
    int status, i;

    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    /* This is not a shadow register - the one that stores the sin
     * and cos values so we need to make sure that device is not active
     * while we set it
     */
    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    /* get the factor */
    conv_fact = (double) (1 << sis8300drvbpm_Qmn_near_iq.frac_bits_n);

    /* ignore numbers bigger than 8 bits */
    M &= 0xff;
    N &= 0xff;

    u32_reg_val |= N << 16;
    u32_reg_val |= M;

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300LLRF_NEAR_IQ_1_PARAM_REG, u32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    i32_reg_val = (int32_t) (conv_fact * 2.0 / N);


    status = sis8300_reg_write(sisdevice->handle,
                SIS8300LLRF_NEAR_IQ_2_PARAM_REG, i32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    /* Calc Sin/cos values and write them */
    status = sis8300_reg_write(sisdevice->handle,
                SIS8300LLRF_NEAR_IQ_ADDR_REG, 0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    double_val = 2.0 * M_PI * M / N;

    for(i = 0; i < (int) N; i++) {

        i32_reg_val = (int32_t) ( sin(double_val * i) * conv_fact );
        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300LLRF_NEAR_IQ_DATA_REG, i32_reg_val);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }

        i32_reg_val = (int32_t) ( cos(double_val * i) * conv_fact );
        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300LLRF_NEAR_IQ_DATA_REG, i32_reg_val);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Get near iq paramters N and M
 *
 * @param [in]  sisuser Device user context struct
 * @param [out] M       Will hold N param val on success
 * @param [out] N       Will hold N param val on success
 *
 * @return status_success       Data retrieved successfully
 * @return status_no_device     Device not opened
 * @return status_device_access Error while accessing device registers
 *
 */
int sis8300llrfdrv_get_near_iq(
        sis8300drv_usr *sisuser,
        unsigned *M, unsigned *N) {

    uint32_t u32_reg_val;
    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300LLRF_NEAR_IQ_1_PARAM_REG, &u32_reg_val);
    if (status) {
        return status_device_access;
    }

    *M = (unsigned) (u32_reg_val & 0xff);
    *N = (unsigned) ( (u32_reg_val >> 16) & 0xff );

    return status_success;
}

/* ==================================================== */
/* =============== BPM filter setup ============= */
/* ==================================================== */
/* Parameters are:
 *  BPM filter constant 0
 *  BPM filter constant 1
 *  BPM filter constant 2
 *  BPM filter constant 3
 *  BPM filter constant 4
 *  BPM filter constant 5
 *  Enable BPM filter
 */
/* Mapping between parameter index and filter constant index */
static const uint32_t bpm_filter_param_map[] = {
   4, 5, 2, 3, 0, 1
};

/**
 * @brief Set BPM Filter parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param_vals  Values of parameters to set
 * @param [int] param_count Number of values (must be equal to number of
 *                          constants of the BPM filter.
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid notch filter or param choice
 * @return status_argument_range    Argument value is out of range
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
 */
int sis8300llrfdrv_set_bpm_filter_param(
        sis8300drv_usr *sisuser,
        double *param_vals, int param_count) {

	int i, val;
    sis8300drv_dev *sisdevice;
    int status = status_success;

    if ((param_count < bpm_fil_en) || (param_count > bpm_fil_en)) {
        return status_argument_range;
    }

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300LLRF_BPM_FILTER_CTRL_REG, 0x2);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    for (i = 0; i < param_count; i++) {
    	/* it is a signed 15-bit value */
    	val = ((int)(param_vals[i] * 32768)) & 0x7FFF;
		printf("%s: Filter param index %d -> %d, %f -> %d\n", __func__,
				i, bpm_filter_param_map[i], param_vals[i], val);
		/* address will internally autoincrement */
	    status = sis8300_reg_write(sisdevice->handle,
	                SIS8300LLRF_BPM_FILTER_DATA_REG, val);
	    if (status) {
	        pthread_mutex_unlock(&sisdevice->lock);
	        return status_device_access;
	    }
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status;
}

int sis8300llrfdrv_set_bpm_filter_enable(
        sis8300drv_usr *sisuser, int param_val) {

    sis8300drv_dev *sisdevice;
    int status;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300LLRF_BPM_FILTER_CTRL_REG, param_val & 0x1);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}

int sis8300llrfdrv_get_bpm_filter_enable(
            sis8300drv_usr *sisuser, int *param_val) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    /* get the value */
    status = sis8300_reg_read(sisdevice->handle,
    		SIS8300LLRF_BPM_FILTER_CTRL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *param_val = ui32_reg_val & 0x1;

    return status_success;
}
