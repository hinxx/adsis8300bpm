/**
 * Struck 8300 BPM Linux userspace library.
 * Copyright (C) 2016 ESS ERIC
 * Copyright (C) 2014-2015  Cosylab
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
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_position =
        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 1 };
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_magnitude =
        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 0 };
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_filter_coeff =
        { .int_bits_m = 16, .frac_bits_n = 16,  .is_signed = 1 };
const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_phase  =
        { .int_bits_m = 3,  .frac_bits_n = 13, .is_signed = 1 };

/* ==================================================== */
/* ================ Initialization ==================== */
/* ==================================================== */
/**
 * @brief Setup the DAC
 *
 * @param [in]    sisuser    User context struct
 *
 * @return status_success       Set successful
 * @return status_device_access Can't access device registers
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device
 * @return status_no_device     Device not opened
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
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
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
 * @return status_success       Set successful
 * @return status_device_access Can't access device registers
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device
 * @return status_no_device     Device not opened
 *
 * This will setup the Data Strobe Timing. The function should be called
 * once at the device initialization
 *
 * bit 8: ADC 1/2 Select Bit
 * bit 7-0: tap delay val (x78 ps)
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
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

    /* XXX: This is probably not correct!! */
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

/* ==================================================== */
/* ================ Basic information ================= */
/* ==================================================== */
/**
 * @brief Get custom firmware version of this SIS8300 device
 *
 * @param [in]  sisuser      User context struct
 * @param [out] ver_device   Will hold device id on success, should be
 *                           #SIS8300BPM_HW_ID for a BPM controller
 * @param [out] ver_major    Will hold major fw version on success
 * @param [out] ver_minor    Will hold minor fw version on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300drvbpm_get_fw_version(
    sis8300drv_usr *sisuser,
    unsigned *ver_device, unsigned *ver_major, unsigned *ver_minor) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300BPM_ID_R_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *ver_device = (unsigned)((ui32_reg_val & 0xffff0000) >> 16);
    *ver_major  = (unsigned)((ui32_reg_val & 0x0000ff00) >> 8);
    *ver_minor  = (unsigned)( ui32_reg_val & 0x000000ff);

    return status_success;
}

/**
 * @brief Get software id of a BPM controller
 *
 * @param [in]  sisuser User context struct
 * @param [out] sw_id   Will hold SW id on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300drvbpm_get_sw_id(sis8300drv_usr *sisuser, unsigned *sw_id) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300BPM_INST_ID_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *sw_id = (unsigned) ui32_reg_val;

    return status_success;
}

/**
 * @brief Set software id of a BPM controller
 *
 * @param [in] sisuser     User context struct
 * @param [in] sw_id       SW id to write
 *
 * @return status_success       Set successfull
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 * The function sets Software id of this instance of BPM controller.
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
 */
int sis8300drvbpm_set_sw_id(sis8300drv_usr *sisuser, unsigned sw_id) {
    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_INST_ID_REG, (uint32_t) sw_id);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/* ==================================================== */
/* ================= Controller status rw ============= */
/* ==================================================== */
/**
 * @brief Get GOP = General Output register status
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  gop_bit     #sis8300drvbpm_gop_bit
 * @param [out] gop_status  Holds 0 if status not active and 1 if active.
 *                          If parameter is gop_all than val holds a 8
 *                          bit mask, with bits 3 - 7 representing the
 *                          status.
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 * @return status_argument_invalid Wrong choice for gop_bit
 *
 * Reads the value of GOP (General Output register).
 */
int sis8300drvbpm_get_gop(sis8300drv_usr *sisuser, unsigned gop_bit,
		unsigned *gop_status) {

    int status;
    uint32_t ui32_reg_val, flag;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
    		SIS8300BPM_GOP_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    ui32_reg_val &= SIS8300BPM_GOP_MASK;

    switch (gop_bit) {
        case gop_all:
            *gop_status = (unsigned) ui32_reg_val;
            break;
        case gop_position2_error:
        case gop_position1_error:
        case gop_write_error:
        case gop_read_error:
        case gop_Y2_pos_div_error:
        case gop_X2_pos_div_error:
        case gop_Y1_pos_div_error:
        case gop_X1_pos_div_error:
        case gop_daq_done:
            flag = (uint32_t) (0x1 << gop_bit);
            *gop_status = (unsigned) ((ui32_reg_val & flag) ? 1 : 0);
            break;
        default:
            return status_argument_invalid;
    }

    return status_success;
}

/**
 * @brief Clear all latched status bits in GOP register
 *
 * @param [in] sisuser    User context struct
 *
 * @return status_success       Set successful
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * NOTE: Does not clear the pulse done counter.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_clear_gop(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;

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
				SIS8300BPM_GOP_REG, 0x1);
	if (status) {
		pthread_mutex_lock(&sisdevice->lock);
		return status_device_access;
	}

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Get pulse done count from GOP register
 *
 * @param [in]  sisuser     User context struct
 * @param [out] count  Holds pulse count number on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300drvbpm_get_pulse_done_count(
        sis8300drv_usr *sisuser, unsigned *pulse_count) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300BPM_GOP_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *pulse_count = (unsigned) (
        (ui32_reg_val & SIS8300BPM_GOP_PULSE_DONE_CNT_MASK) >>
                SIS8300BPM_GOP_PULSE_DONE_CNT_SHIFT);

    return status_success;
}

/**
 * @brief Clears pulse done counter in GOP register
 *
 * @param [in] sisuser    User context struct
 *
 * @return status_success       Data retrieved successfully
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_clear_pulse_done_count(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;

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
    			SIS8300BPM_GIP_REG, SIS8300BPM_GIP_CLEAR_PULSE_DONE_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Update parameters based on reason
 *
 * @param [in] sisuser         User context struct
 *
 * @return status_success             Set successfull
 * @return status_device_access     Can't access device registers
 * @return status_no_device         Device not opened
 *
 * All calls to this function will result in some kind of update of
 * controller operational parameters.
 *
 * This function typically gets called after writing to a shadow register,
 * to make the controller load the new values.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_update_parameters(sis8300drv_usr *sisuser) {

	int status;
    sis8300drv_dev *sisdevice;

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
    			SIS8300BPM_GIP_REG,
				SIS8300BPM_GIP_NEW_PARAMS_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Move BPM controller from INIT to IDLE state
 *
 * @param [in] sisuser User context struct
 *
 * @return status_success       On successful set
 * @return status_device_access Can't access device registers.
 * @return status_no_device     Device not opened.
 *
 * Call when all initialization is done. The function does not perform
 * any checks, just sets the init_done bit. User is responsible for
 * performing necessary initialization procedures before calling this
 * function.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_init_done(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;

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
                SIS8300BPM_GIP_REG,
				SIS8300BPM_GIP_INIT_DONE_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief SW reset of the custom logic to enter INIT state
 *
 * @param [in] sisuser User context struct
 *
 * @return status_success       On successful set
 * @return status_device_access Can't access device registers.
 * @return status_no_device     Device not opened.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_sw_reset(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);;

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_GIP_REG,
				SIS8300BPM_GIP_SW_RESET_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/* ==================================================== */
/* ================== Acquisition ===================== */
/* ==================================================== */
/**
 * @brief Arm the device
 *
 * @param [in]  sisuser     User context struct
 *
 * @return status_success       Arm successful
 * @return status_no_device     Device not opened
 * @return status_device_access Cannot access device registers
 *
 * Overrides generic sis8300drv_arm_device function, to make sure that
 * trigger type ARM is always used. The BPM controller ignores the
 * SIS8300_SAMPLE_CONTROL_REG completely.
 */
int sis8300drvbpm_arm_device(sis8300drv_usr *sisuser) {
    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    /* Should not arm if there are pending operations on device. */
    pthread_mutex_lock(&sisdevice->lock);

    if (!__sync_lock_test_and_set(&sisdevice->armed, 1)) {
        /* Reset sampling logic. */
        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300_ACQUISITION_CONTROL_STATUS_REG,
                    SIS8300DRV_RESET_ACQ);
        if (status) {
            __sync_lock_release(&sisdevice->armed);
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }

        /* Wait until internal sampling logic is not busy anymore. */
        do {
            status = sis8300_reg_read(sisdevice->handle,
                        SIS8300_ACQUISITION_CONTROL_STATUS_REG,
                        &ui32_reg_val);
            if (status) {
                    __sync_lock_release(&sisdevice->armed);
                pthread_mutex_unlock(&sisdevice->lock);
                return status_device_access;
            }
        } while (ui32_reg_val & 0x30);

        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300_ACQUISITION_CONTROL_STATUS_REG,
                    SIS8300DRV_TRG_ARM);
        if (status) {
            __sync_lock_release(&sisdevice->armed);
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief wait for pulse done or position software interrupt
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  timeout     Wait irq timeout
 *
 * @return status_no_device     Device not opened
 * @return @see #sis8300drv_wait_irq
 *
 * The function will wait for the board to fire a user interrupt,
 * indicating pulse done or position out of bounds. This interrupt
 * replaces DAQ done interrupt on the generic version - it should
 * not be counted on for BPM controller.
 */
int sis8300drvbpm_wait_pulse_done_position(sis8300drv_usr *sisuser,
		unsigned timeout) {

    int status;
    sis8300drv_dev  *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300drv_wait_irq(sisuser, irq_type_usr, timeout);

    __sync_lock_release(&sisdevice->armed);

    return status;
}

/**
 * @brief Set trigger setup for the controller
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  trg_setup   @see #sis8300drvbpm_trg_setup
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 *
 * Selects the trigger setup that should be used for this instance of
 * the BPM controller. The BPM specific implementation ignores generic
 * sis8300 trigger settings. This is the function that replaces them.
 *
 * Calls to this function are serialized with respect to other calls
 * that alter the functionality of the device. This means that this
 * function may block.
 */
int sis8300drvbpm_set_trigger_setup(
        sis8300drv_usr *sisuser, sis8300drvbpm_trg_setup trg_setup) {

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

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300BPM_BOARD_SETUP_REG, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    /* clear bits 0 and 1 */
    ui32_reg_val &= ~0x3;
    /* set the new value */
    ui32_reg_val |= (uint32_t) trg_setup & 0x3;

    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_BOARD_SETUP_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/* ==================================================== */
/* ===================== Triggers ===================== */
/* ==================================================== */
/**
 * @brief Get trigger setup for the BPM controller
 *
 * @param [in]  sisuser     User context struct
 * @param [out] trg_setup   @see #sis8300drvbpm_trg_setup
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 *
 * Returns the trigger setup that is currently used by the device.
 *
 */
int sis8300drvbpm_get_trigger_setup(sis8300drv_usr *sisuser,
		sis8300drvbpm_trg_setup *trg_setup) {

    int status;
    unsigned u_reg_val;

    status = sis8300drv_reg_read(sisuser,
                SIS8300BPM_BOARD_SETUP_REG, &u_reg_val);
    if (status) {
        return status_device_access;
    }

    u_reg_val &= 0x3;

    if (u_reg_val != (unsigned) mlvds_456) {
        /* values 0, 2 and 3 are mlvds_012 */
        *trg_setup = mlvds_012;
    }
    else {
        *trg_setup = mlvds_456;
    }

    return status_success;
}

/* ===================================================== */
/* ================= Near IQ control =================== */
/* ===================================================== */
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
int sis8300drvbpm_set_near_iq(sis8300drv_usr *sisuser, unsigned M, unsigned N) {

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
                SIS8300BPM_NEAR_IQ_1_PARAM_REG, u32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    i32_reg_val = (int32_t) (conv_fact * 2.0 / N);


    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_NEAR_IQ_2_PARAM_REG, i32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    /* Calc Sin/cos values and write them */
    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_NEAR_IQ_ADDR_REG, 0);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    double_val = 2.0 * M_PI * M / N;

    for(i = 0; i < (int) N; i++) {

        i32_reg_val = (int32_t) ( sin(double_val * i) * conv_fact );
        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300BPM_NEAR_IQ_DATA_REG, i32_reg_val);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }

        i32_reg_val = (int32_t) ( cos(double_val * i) * conv_fact );
        status = sis8300_reg_write(sisdevice->handle,
                    SIS8300BPM_NEAR_IQ_DATA_REG, i32_reg_val);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_access;
        }
    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Get near iq parameters N and M
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
int sis8300drvbpm_get_near_iq(sis8300drv_usr *sisuser, unsigned *M, unsigned *N) {

    uint32_t u32_reg_val;
    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle,
                SIS8300BPM_NEAR_IQ_1_PARAM_REG, &u32_reg_val);
    if (status) {
        return status_device_access;
    }

    *M = (unsigned) (u32_reg_val & 0xff);
    *N = (unsigned) ( (u32_reg_val >> 16) & 0xff );

    return status_success;
}

/* ==================================================== */
/* ================ FIR filter setup ================== */
/* ==================================================== */
/* Mapping between parameter index and filter constant index */
static const uint32_t bpm_filter_param_map[] = {
   4, 5, 2, 3, 0, 1
};

/**
 * @brief Set FIR filter parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param_vals  Values of parameters to set
 * @param [int] param_count Number of values (must be equal to number of
 *                          constants of the FIR filter.
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid notch filter or param choice
 * @return status_argument_range    Argument value is out of range
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300drvbpm_update is needed.
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
 */
int sis8300drvbpm_set_fir_filter_param(sis8300drv_usr *sisuser,
        double *param_vals, int param_count) {

	int i;
	uint32_t val;
	int16_t reg_val;
	double err;
    sis8300drv_dev *sisdevice;
    int status = status_success;

    if (param_count != SIS8300BPM_FIR_FILTER_PARAM_NUM) {
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

    /* Enable loading FIR coefficients into firmware */
    status = sis8300_reg_write(sisdevice->handle,
                SIS8300BPM_FILTER_CTRL_REG, 0x2);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    for (i = 0; i < param_count; i++) {
    	status = sis8300drvbpm_double_2_Qmn(param_vals[i],
    			sis8300drvbpm_Qmn_filter_coeff, &val, &err);
	    if (status) {
	        pthread_mutex_unlock(&sisdevice->lock);
	        return status;
	    }
	    reg_val = val & 0xFFFF;
		printf("%s: Filter param index %d -> %d, %f -> %d, err %f, reg val %d\n", __func__,
				i, bpm_filter_param_map[i], param_vals[i], val, err, reg_val);
		/* address will internally auto increment */
	    status = sis8300_reg_write(sisdevice->handle,
	                SIS8300BPM_FILTER_DATA_REG, reg_val);
	    if (status) {
	        pthread_mutex_unlock(&sisdevice->lock);
	        return status;
	    }
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status;
}

/**
 * @brief Set FIR filter control bit
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param_val   Enable or disable FIR filter
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function
 * may block.
 */
int sis8300drvbpm_set_fir_filter_enable(sis8300drv_usr *sisuser, int param_val) {

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
                SIS8300BPM_FILTER_CTRL_REG, param_val & 0x1);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    pthread_mutex_unlock(&sisdevice->lock);

    return status_success;
}

/**
 * @brief Get FIR filter control bit
 *
 * @param [in]  sisuser     User context struct
 * @param [out] param_val   Status of FIR filter (enabled or disabled)
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 */
int sis8300drvbpm_get_fir_filter_enable(
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
    		SIS8300BPM_FILTER_CTRL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *param_val = ui32_reg_val & 0x1;

    return status_success;
}

/* ==================================================== */
/* ================== Type Conversion ================= */
/* ==================================================== */
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

    return status_success;
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
 * This is the inverse function of @see sis8300drvbpm_double_2_Qmn
 */
int sis8300drvbpm_Qmn_2_double(
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

    return status_success;
}
