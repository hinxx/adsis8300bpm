#include <stdint.h>

#include <sis8300drv.h>
#include <sis8300drv_utils.h>
#include <sis8300_reg.h>

#include "sis8300drvbpm.h"
#include "sis8300bpm_reg.h"
#include "sis8300drvbpm_types.h"

//static inline int _sis8300llrfdrv_get_sigmon_mag(
//                    sis8300drv_usr *sisuser,
//                    int chan, double *mag_val, int shift);

/**
 * @brief Arm the device
 *
 * @param [in]  sisuser     User context struct
 *
 * @return status_success Arm successful
 * @return status_no_device Device not opened
 * @return status_device_access Cannot access device registers
 *
 * Overrides generic function, to make sure that trigg type ARM is always 
 * used. The llrf functionality ignores the SIS8300_SAMPLE_CONTROL_REG
 * completely. We want to make sure that it does not accidentally happen
 *
 * TODO: think on this
 */
int sis8300llrfdrv_arm_device(sis8300drv_usr *sisuser) {
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
 * @brief wait for PULSE_DONE or PMS
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  timeout     Wait irq timeout
 *
 * @return status_no_device     Device not opened
 * @return @see #sis8300drv_wait_irq
 *
 * The function will wait for the board to fire a user interrupt,
 * indicating PMS event or PULSE_DONE. This interrupt replaces DAQ DONE
 * interrupt on the generic version - it should not be counted on
 * here
 */
int sis8300llrfdrv_wait_pulse_done_pms(
        sis8300drv_usr *sisuser, unsigned timeout) {

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
 * @param [in]  trg_setup   @see #sis8300llrfdrv_trg_setup
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 *
 * Selects the trigger setup that should be used for this instance of 
 * the LLRF controller. The LLRF specific implementation ignores generic 
 * sis8300 trigger settings. This is the function that replaces them.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_set_trigger_setup(
        sis8300drv_usr *sisuser, sis8300llrfdrv_trg_setup trg_setup) {

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
                SIS8300LLRF_BOARD_SETUP_REG, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    /* clear bits 0 and 1 */
    ui32_reg_val &= ~0x3;
    /* set the new value */
    ui32_reg_val |= (uint32_t) trg_setup & 0x3;

    status = sis8300_reg_write(sisdevice->handle, 
                SIS8300LLRF_BOARD_SETUP_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Get trigger setup for the controller
 *
 * @param [in]  sisuser     User context struct
 * @param [out] trg_setup   @see #sis8300llrfdrv_trg_setup
 *
 * @return status_success          Information retrieved successfully
 * @return status_device_access    Can't access device registers
 * @return status_no_device        Device not opened
 *
 * Returns the trigger setup that is currently used by the device.
 *
 */
int sis8300llrfdrv_get_trigger_setup(
        sis8300drv_usr *sisuser, sis8300llrfdrv_trg_setup *trg_setup) {

    int status;
    unsigned u_reg_val;

    status = sis8300drv_reg_read(sisuser, 
                SIS8300LLRF_BOARD_SETUP_REG, &u_reg_val);
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
#if 0
/**
 * @brief get number of acquired samples during the passed pulse
 *
 * @param [in]  sisuser             User context struct
 * @param [in]  samples_cnt_source  see #sis8300llrfdrv_samples_cnt
 * @param [out] samples_Cnt_Val     Will heold the number of samples on 
 *                                  success
 *
 * @retval status_success Device successfully initialized.
 * @retval status_device_access Can't access device registers.
 * @retval status_no_device Device not opened.
 */
int sis8300llrfdrv_get_acquired_nsamples(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_samples_cnt samples_cnt_source, 
        unsigned *samples_cnt_val) {
            
    uint32_t reg_addr;

    switch (samples_cnt_source) {
        case samples_cnt_pi_total:
            reg_addr = SIS8300LLRF_CNT_PI_ERR_TOTAL_R_REG;
            break;
        case samples_cnt_pi_ramp_up_phase:
            reg_addr = SIS8300LLRF_CNT_PI_ERR_PULSE_START_R_REG;
            break;
        case samples_cnt_pi_active_phase:
            reg_addr = SIS8300LLRF_CNT_PI_ERR_PULSE_ACTIVE_R_REG;
            break;
        case samples_cnt_cavity_total:
            reg_addr = SIS8300LLRF_CNT_CAVITY_SAMPLES_R_REG;
            break;
        default:
            return status_argument_range;
    }
    
    return sis8300drv_reg_read(sisuser, reg_addr, samples_cnt_val);
}

/**
 * @brief Get max number of IQ samples for PI error that will fit into 
 *        device memory
 *
 * @param [in]  sisuser             User context struct
 * @param [out] samples_Cnt_Val     Will hold the number of samples on 
 *                                  success
 *
 * @return status_no_device     Device not opened
 * @return status_device_access Cannot access device registers
 * @return status_success       Value retrieved successfully
 *
 * One IQ sample has 32 bits, 16 bits for amplitude and 16 for phase.
 */
int sis8300llrfdrv_get_pi_err_max_nsamples(
        sis8300drv_usr *sisuser, unsigned *max_nsamples) {

    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_MEM_SIZE_PI_ERR_R_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    /* convert from size in 256 bit chunks (#SIS8300DRV_BLOCK_BYTES) to 
     * size in IQ samples of 32 bits (#SIS8300LLRF_IQ_SAMPLE_BYTES) */
    *max_nsamples = (unsigned) (
        ui32_reg_val * 
        (SIS8300LLRF_MEM_CTRL_BLOCK_BYTES / SIS8300LLRF_IQ_SAMPLE_BYTES));

    return status_success;
}

/**
 * @brief Reads nsamples of raw PI error from device memory, where 
 *        nsamples is the value set by 
 *        #sis8300llrfdrv_set_pi_err_nsamples
 *
 * @param [in]  sisuser     User context struct
 * @param [out] data        Will hold PI error raw values
 * @param [out] data_angle  Will hold PI angle errors on success
 *
 * @return status_success           Data transfered successfully
 * @return status_device_access     Can't access device memory location 
 *                                  or device registers.
 * @return status_device_read       Can't transfer data from device 
 *                                  memory.
 * @return status_no_device         Device not opened.
 *
 * The function reads raw data, as-is, from the register. Data is in 
 * form of 32 bit samples, where 16 bits represent magnitude and 16 bits
 * represent angle.
 *
 * The nsamples value can be obtained from 
 * #sis8300llrfdrv_get_pi_err_nsamples
 *
 * The best way to interpret the data is to read it as int16 type and 
 * treat the I and Q table as interleaved, where first I sample is at 
 * offset 0 and first Q sample at offset 1.
 *
 * Calls to this function are serialized with respect to other calls that 
 * alter the functionality of the device. This means that this function 
 * may block.
 */
int sis8300llrfdrv_read_pi_error_raw(
        sis8300drv_usr *sisuser, void *raw_data, unsigned nsamples) {

    int status;
    uint32_t base_offset;
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

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_MEM_CTRL_PI_ERR_BASE_REG, &base_offset);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    status = sis8300drv_read_ram_unlocked(sisdevice, 
                base_offset, nsamples * SIS8300LLRF_IQ_SAMPLE_BYTES, 
                raw_data);

    pthread_mutex_unlock(&sisdevice->lock);
    return status;
}

/**
 * @brief get current signal angle and magnitude.
 *
 * @param [in]  sisuser     Device user context struct
 * @param [in]  signal      which signal to get data for, can be
 *                          #SIS8300LLRFDRV_AI_CHAN_CAV or 
 *                          #SIS8300LLRFDRV_AI_CHAN_REF
 * @param [out] angle       Will hold angle on success
 * @param [out] magnitude   Will hold magnitude on success
 *
 * @return status_success           Data retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid signal choice
 */
int sis8300llrfdrv_get_signal_ma(
        sis8300drv_usr *sisuser, 
        int signal, double *angle, double *magnitude) {
    /* sis8300llrfdrv_Qmn Qmn = {.int_bits_m = 5, .frac_bits_n = 27, .is_signed = 1}; */

    int status;

    uint32_t ui32_reg_val;
    double conv_fact_ang, conv_fact_mag;
    int16_t i16_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    conv_fact_ang = (double)(0x1 << sis8300llrfdrv_Qmn_angle.frac_bits_n);
    conv_fact_mag = (double)(0x1 << 15);

    switch (signal) {
//        case SIS8300LLRFDRV_AI_CHAN_CAV:
//            status = sis8300_reg_read(sisdevice->handle,
//                        SIS8300LLRF_MA_CAV_R_REG, &ui32_reg_val);
//            break;
        case SIS8300LLRFDRV_AI_CHAN_REF:
            status = sis8300_reg_read(sisdevice->handle, 
                        SIS8300LLRF_MA_REF_R_REG, &ui32_reg_val);
            break;
        default:
            return status_argument_invalid;
    }

    if (status) {
        return status_device_access;
    }

    i16_val = (ui32_reg_val & 0xffff0000) >> 16;
    *magnitude = (double) i16_val / conv_fact_mag;
    i16_val  =  ui32_reg_val & 0x0000ffff;
    *angle     = (double) i16_val  / conv_fact_ang;

    return status_success;
}

/**
 * @brief Get the current magnitude for a specific signal monitor capable 
 *        AI channel (channels AI2 - AI9)
 * 
 * @param [in]  sisuser         User context struct
 * @param [in]  chan            AI channel to get the magnitude for
 * @param [out] mag_curr_val    Will contain the current magnitude value 
 *                              on success 
 * 
 * @return see #_sis8300llrfdrv_get_sigmon_mag
 */ 
int sis8300llrfdrv_get_sigmon_mag_curr(
        sis8300drv_usr *sisuser, int chan, double *mag_curr_val) {
	
    return 
        _sis8300llrfdrv_get_sigmon_mag(sisuser, chan, mag_curr_val, 0);
}

/**
 * @brief Get the minimum or maximum magnitude during active monitor 
 *        period for a specific signal monitor capable AI channel 
 *        (channels AI2 - AI9). 
 * 
 * @param [in]  sisuser         User context struct
 * @param [in]  chan            AI channel to get the magnitude for
 * @param [out] mag_curr_val    Will contain the current magnitude value 
 *                              on success 
 * 
 * @return see #_sis8300llrfdrv_get_sigmon_mag
 * 
 * If alarm is set to trigger below treshold than the returned magnitude
 * is the minimum, if the trigger is set to above treshold, than the 
 * returned magnitude is the maximum apmplitude during the active 
 * sampling period. Alarm trigger setting are avaialble from 
 * #sis8300llrfdrv_set_sigmon_param and can be read out with 
 * #sis8300llrfdrv_get_sigmon_param
 */ 
int sis8300llrfdrv_get_sigmon_mag_minmax(
        sis8300drv_usr *sisuser, int chan, double *mag_minmax_val) {	
	
    return 
        _sis8300llrfdrv_get_sigmon_mag(sisuser, chan, mag_minmax_val, 8);
}

/* ====================== INTERNAL LIBRARY FUNCTIONS ================ */
/**
 * @brief Internal library function, that can either return 
 *        current or min/max signal magnitude
 * 
 * @param [in]  sisuser    User context struct
 * @param [in]  chan       AI channel to get the magnitude for
 * @param [out] mag_val    Will contain the magnitude value on success 
 * @param [in]  shift      Should be 0 for getting the current magnitude 
 *                         and 8 for min/max
 * 
 * @return status_success           Data retrieved successfully
 * @return status_no_device         Device not found
 * @return status_argument_range    Channel is not AI2 - AI9
 * @return status_device_access     Could not access device registers
 */
inline int _sis8300llrfdrv_get_sigmon_mag(
                sis8300drv_usr *sisuser, 
                int chan, double *mag_val, int shift) {
	
    int status;
	
	uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;
    sis8300llrfdrv_Qmn Qmn = 
            {.int_bits_m = 0, .frac_bits_n = 8, .is_signed = 0};
    
    uint32_t addr[] = {
        0, 0,
        SIS8300LLRF_MON_STATUS_MAG_1_REG, SIS8300LLRF_MON_STATUS_MAG_1_REG,
        SIS8300LLRF_MON_STATUS_MAG_2_REG, SIS8300LLRF_MON_STATUS_MAG_2_REG,
        SIS8300LLRF_MON_STATUS_MAG_3_REG, SIS8300LLRF_MON_STATUS_MAG_3_REG,
        SIS8300LLRF_MON_STATUS_MAG_4_REG, SIS8300LLRF_MON_STATUS_MAG_4_REG
    };
    int chan_shift[] = {0, 0, 
                        0, 16, 0, 16, 0, 16, 0, 16};

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    if (chan < SIS8300LLRFDRV_SIGMON_CHAN_FIRST ||
        chan > SIS8300DRV_NUM_AI_CHANNELS) {
            return status_argument_range;
    }

    status = sis8300_reg_read(sisdevice->handle, addr[chan], &ui32_reg_val);
    if (status) {
        return status_device_access;
    }
    
    shift += chan_shift[chan];
	ui32_reg_val &= (0xff << shift);
	ui32_reg_val >>= shift;
	
	sis8300llrfdrv_Qmn_2_double(ui32_reg_val, Qmn, mag_val);
	
	return status_success;
}
#endif

