/*
 * m-kmod-sis8300llrf
 * Copyright (C) 2014-2015  Cosylab

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
 * @file sis8300llrfdrv.c
 * @brief Implementation of sis8300 digitizer LLRF FW upgraded userspace api.
 */
#if 0
#include <stdint.h>

#include <sis8300drv.h>
#include <sis8300drv_utils.h>

/* Debugging */
#define SIS8300LLRFDRV_DEBUG


#include "sis8300drvbpm.h"
#include "sis8300drvbpm_types.h"
#include "sis8300bpm_reg.h"

/* ==================================================== */
/* ================ Basic information ================= */
/* ==================================================== */
/**
 * @brief Get custom firmware version of this SIS8300 device
 *
 * @param [in]  sisuser      User context struct
 * @param [out] ver_device   Will hold device id on success, should be 
 *                           #SIS8300LLRFDRV_HW_ID
 *                           if this is a llrf controller
 * @param [out] ver_major    Will hold major fw version on success
 * @param [out] ver_minor    Will hold minor fw version on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300llrfdrv_get_fw_version(
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
                SIS8300LLRF_ID_R_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *ver_device = (unsigned)((ui32_reg_val & 0xffff0000) >> 16);
    *ver_major  = (unsigned)((ui32_reg_val & 0x0000ff00) >> 8);
    *ver_minor  = (unsigned)( ui32_reg_val & 0x000000ff);

    return status_success;
}

/**
 * @brief Get software id of LLRF ctrl digitizer
 *
 * @param [in]  sisuser User context struct
 * @param [out] sw_id   Will hold sw id on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300llrfdrv_get_sw_id(sis8300drv_usr *sisuser, unsigned *sw_id) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_INST_ID_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *sw_id = (unsigned) ui32_reg_val;

    return status_success;
}

/**
 * @brief Set SW id
 *
 * @param [in] sisuser     User context struct
 * @param [in] sw_id     SW id to write
 *
 * @return status_success       Set successfull
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 * The function sets Software id of this instance of LLRF ctrl
 * digitizer.
 *
 * Calls to this function are serialized with respect to other calls that 
 * alter the functionality of the device. This means that this function 
 * may block.
 */
int sis8300llrfdrv_set_sw_id(sis8300drv_usr *sisuser, unsigned sw_id) {
    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    status = sis8300_reg_write(sisdevice->handle, 
                SIS8300LLRF_INST_ID_REG, (uint32_t ) sw_id);
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
 * @brief get GOP = General Output register status
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  gop_bit     #sis8300llrf_drv_GOP_bit
 * @param [out] gop_status  Holds 0 if status not active and 1 if active.
 *                          If parameter is GOP_all than val holds a 8 
 *                          bit mask, with bits 3 - 7 representing the 
 *                          status.
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 * @return status_argument_invalid Wrong choice for gop_bit
 *
 * Reads the value of GOP (General Output register).
 *
 * If gop_bit is gen_status_pi_overflow_mag, than specific status bits 
 * can be obtained either by masks defined in #sis8300llrfdrv.h or the 
 * enumerator #sis8300llrf_drv_gop_bit.
 */
int sis8300llrfdrv_get_general_status(sis8300drv_usr *sisuser, 
        unsigned gen_status_bit, unsigned *gen_status) {

    int status;
    uint32_t ui32_reg_val, flag;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_GEN_STATUS_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    ui32_reg_val &= SIS8300LLRF_GEN_STATUS_MASK;

    switch (gen_status_bit) {
        case gen_status_all:
            *gen_status = (unsigned) ui32_reg_val;
            break;
//HK LLRF specific
//        case gen_status_PMS_active:
//        case gen_status_pi_overflow_I:
//        case gen_status_pi_overflow_Q:
//        case gen_status_read_error:
//        case gen_status_write_error:
//        case gen_status_vm_mag_limiter_active:
        case gen_status_position2_error:
        case gen_status_position1_error:
        case gen_status_write_error:
        case gen_status_read_error:
        case gen_status_Y2_pos_div_error:
        case gen_status_X2_pos_div_error:
        case gen_status_Y1_pos_div_error:
        case gen_status_X1_pos_div_error:
        case gen_status_daq_done:
//HK BPM to fool LLRF
//        case gen_status_dummy:
            flag = (uint32_t) (0x1 << gen_status_bit);
            *gen_status = (unsigned) ((ui32_reg_val & flag) ? 1 : 0);
            break;
        default:
            return status_argument_invalid;
    }

    return status_success;
}

/**
 * @brief Get signal monitoring status
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set, #sis8300llrfdrv_sigmon_stat
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 *
 */
int sis8300llrfdrv_get_sigmon_status(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_sigmon_stat status_select, unsigned *status_val) {
    
    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;
    
    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_MON_STATUS_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    switch (status_select) {
        case sigmon_stat_alarm:
            break;
        case sigmon_stat_pms:
            ui32_reg_val >>= 8;
            break;
        case sigmon_stat_ilock:
            ui32_reg_val >>= 16;
            break;
        default:
            return status_argument_invalid;
    }

    ui32_reg_val &= 0xff;

    *status_val = (unsigned) ui32_reg_val;

    return status_success;
}

/**
 * @brief Clear all latched statuses from GOP register
 *
 * @param [in] sisuser    User context struct
 *
 * @return status_success       Set successfull
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an 
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 *
 * TODO: fix doxygen
 */
int sis8300llrfdrv_clear_latched_statuses(
        sis8300drv_usr *sisuser, unsigned what) {
    
    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

/*  TODO: recheck why this is disabled
    if (sisdevice->armed) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_armed;
    }
*/
    if (what & SIS8300LLRFDRV_STATUS_CLR_GENERAL) {
        status = sis8300_reg_write(sisdevice->handle, 
                    SIS8300LLRF_GOP_REG, 0x1);
        if (status) {
			pthread_mutex_lock(&sisdevice->lock);
            return status_device_access;
        }
    }
//    if (what & SIS8300LLRFDRV_STATUS_CLR_SIGMON) {
//        status = sis8300_reg_write(sisdevice->handle,
//                    SIS8300LLRF_MON_STATUS_REG, 0x1);
//        if (status) {
//			pthread_mutex_lock(&sisdevice->lock);
//            status = status_device_access;
//        }
//    }

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Get pulse done count
 *
 * @param [in]  sisuser     User context struct
 * @param [out] count  Holds pulse count number on success
 *
 * @return status_success       Information retrieved successfully
 * @return status_device_access Can't access device registers
 * @return status_no_device     Device not opened
 *
 */
int sis8300llrfdrv_get_pulse_done_count(
        sis8300drv_usr *sisuser, unsigned *pulse_count) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_GOP_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *pulse_count = (unsigned) (
        (ui32_reg_val & SIS8300LLRF_GOP_PULSE_DONE_CNT_MASK) >> 
                SIS8300LLRF_GOP_PULSE_DONE_CNT_SHIFT);

    return status_success;
}

/**
 * @brief Clears PULSE_DONE counter
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
int sis8300llrfdrv_clear_pulse_done_count(sis8300drv_usr *sisuser) {

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
//HK LRRF specific
//                SIS8300LLRF_GIP_S_REG, 0x1 << 12);
    			SIS8300LLRF_GIP_REG, SIS8300LLRF_GIP_CLEAR_PULSE_DONE_BIT);
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
 * @param [in] update_type     What setup to update, 
 *                             #sis8300llrfdrv_update_reason
 *
 * @return status_success             Set successfull
 * @return status_device_access     Can't access device registers
 * @return status_no_device         Device not opened
 * @return status_argument_invalid    Invalid choice for update_reason
 *
 * All calls to this function will result in some kind of update of 
 * controller operational parameters. The parameters updated depend on 
 * update_reason. Possible reasons are @see #sis8300llrfdrv_update_reason
 *
 * This function typically gets called after writing to a shadow register,
 * to make the controller load the new values.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_update(
        sis8300drv_usr *sisuser, unsigned update_reason) {

    uint32_t ui32_reg_val;
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

    /* this register also holds sw reset and debug options.
     * Make sure they don't get overwritten */
    ui32_reg_val = update_reason & SIS8300LLRF_GIP_UPDATE_REASON_MASK;

    status = sis8300_reg_write(sisdevice->handle, 
//HK LRRF specific
//                SIS8300LLRF_GIP_S_REG, ui32_reg_val);
    			SIS8300LLRF_GIP_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Move LLRF controller from init state
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
int sis8300llrfdrv_init_done(sis8300drv_usr *sisuser) {

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
//HK LRRF specific
//            SIS8300LLRF_GIP_S_REG,
                SIS8300LLRF_GIP_REG,
                SIS8300LLRFDRV_UPDATE_REASON_INIT_DONE);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief SW reset of the custom logic. Call this function to get out
 *        of PMS state and move to INIT
 *
 * @see #sis8300llrfdrv_ilock (IMPORTAINT)
 *
 * @param [in] sisuser User context struct
 *
 * @return status_success       On successfull set
 * @return status_device_access Can't access device registers.
 * @return status_no_device     Device not opened.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_sw_reset(sis8300drv_usr *sisuser) {

    int status;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);;

    status = sis8300_reg_write(sisdevice->handle,
//HK LRRF specific
//            SIS8300LLRF_GIP_S_REG, SIS8300LLRF_GIP_SW_RESET_BIT);
                SIS8300LLRF_GIP_REG, SIS8300LLRF_GIP_SW_RESET_BIT);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}


#endif


