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
 * @file sis8300drvbpm.h
 * @brief Header of sis8300 BPM userspace API.
 * @author Hinko Kocevar
 */

#ifndef SIS8300LLRFDRV_H_
#define SIS8300LLRFDRV_H_

#define SIS8300LLRFDRV_DEBUG

#ifdef __cplusplus
extern "C" {
#endif

#include <limits.h>
#include <stdint.h>
#include <math.h>

#if UINT_MAX == 0xffffffff
#else
#error UNSIGNED DATA TYPE IS NOT 4 BYTES UINT_MAX
#endif


#undef SIS8300LLRF_PDEBUG
#ifdef SIS8300LLRF_DEBUG
#include <stdio.h>
#define SIS8300LLRF_PDEBUG(fmt, args...) printf("sis8300llrfdrv DEBUG:" fmt, ## args)
#else
#define SIS8300LLRF_PDEBUG(fmt, args...)
#endif

#define SIS8300LLRF_INFO(fmt, args...) printf("sis8300llrfdrv INFO:" fmt, ## args)

/* Block bytes check for write ram */
//#define SIS8300LLRF_IQ_SAMPLE_BYTES      4
//#define SIS8300LLRF_MEM_CTRL_BLOCK_BYTES 32

/*
#if SIS8300LLRF_MEM_CTRL_BLOCK_BYTES == SIS8300DRV_BLOCK_BYTES
#else
#error: SIS8300LLRFDRV_MEM_CTRL_BLOCK_BYTES not the same as SIS8300DRV_BLOCK_BYTES
#endif
*/


//#define SIS8300LLRFDRV_HW_ID                    0xB00B  /** < Unique LLRF controller ID */
//#define SIS8300LLRFDRV_VERSION_MAJOR            0x02    /** < Library supports this Major FW revision */
//#define SIS8300LLRFDRV_VERSION_MINOR_FIRST      0x0f    /** < Minor revision - first minor revision in the supported range */
//#define SIS8300LLRFDRV_VERSION_MINOR_LAST       0x10    /** < Minor revision - last minor revison in the supported range */
//HK for BPM
#define SIS8300DRVBPM_HW_ID                     0xCA5E  /** < Unique BPM controller ID */
#define SIS8300LLRFDRV_VERSION_MAJOR            0x00    /** < Library supports this Major FW revision */
#define SIS8300LLRFDRV_VERSION_MINOR_FIRST      0x06    /** < Minor revision - first minor revision in the supported range */
#define SIS8300LLRFDRV_VERSION_MINOR_LAST       0x06    /** < Minor revision - last minor revison in the supported range */

//#define SIS8300LLRFDRV_FW_VERSION_MAJOR_MA      0x01    /** < Magnitude Angle based fw version of the controller, fw major revision */
//#define SIS8300LLRFDRV_FW_VERSION_MAJOR_IQ      0x02    /** < IQ based fw version of the controller, fw major revision */
//HK for BPM
//#define SIS8300LLRFDRV_FW_VERSION_MAJOR_IQ      0x00    /** < The only BPM fw version of the controller, fw major revision */

/* meaning of analogue input channels */
//#define SIS8300LLRFDRV_AI_CHAN_CAV              0   /** < AI channel 0 always corresponds to cavity probe input */
//#define SIS8300LLRFDRV_AI_CHAN_REF              1   /** < AI channel 1 always corresponds to reference input */
//#define SIS8300LLRFDRV_SIGMON_CHAN_FIRST        2   /** < first channel that has signal monitoring available - AI2 */

/* GOP = General Output register status bits */
//#define SIS8300LLRFDRV_GEN_STATUS_VM_MAG_LIMITER_ACTIVE 0x100   /**< VM output limiter is active and constrining ouput to
//                                                                   SIS8300LLRFDRV_VM_MAG_LIMIT */
//#define SIS8300LLRFDRV_GEN_STATUS_PI_OVERFLOW_I         0x80    /**< Overflow in PI Ctrl for I controller */
//#define SIS8300LLRFDRV_GEN_STATUS_PI_OVERFLOW_Q         0x40    /**< Overflow in PI Ctrl for Q controller */
//#define SIS8300LLRFDRV_GEN_STATUS_READ_ERR              0x20    /**< Read error while accessing register */
//#define SIS8300LLRFDRV_GEN_STATUS_WRITE_ERR             0x10    /**< Write error while accessing register */
//#define SIS8300LLRFDRV_GEN_STATUS_PMS_ACT               0x08    /**< PMS active */

/* Update reasons, to be used with #sis8300llrfdrv_update */
//#define SIS8300LLRFDRV_UPDATE_REASON_SP                 0x010   /**< Set this to get the controller to reload the sp table from memory, when calling #sis8300llrfdrv_update */
//#define SIS8300LLRFDRV_UPDATE_REASON_FF                 0x008   /**< Set this to get the controller to reload the ff table from memory, when calling #sis8300llrfdrv_update */
//#define SIS8300LLRFDRV_UPDATE_REASON_NEW_PT             0x004   /**< Set this to get the controller to load new FF and SP tables (corresponding to new pt) from memory, when calling #sis8300llrfdrv_update */
#define SIS8300LLRFDRV_UPDATE_REASON_NEW_PARAMS         0x002   /**< Set this to get the controller to use new parameters, when calling #sis8300llrfdrv_update */
#define SIS8300LLRFDRV_UPDATE_REASON_INIT_DONE          0x001   /**< Set this to get the controller to reload everything, when calling #sis8300llrfdrv_update */


//#define SIS8300LLRFDRV_STATUS_CLR_GENERAL               1       /** < Set this to clear general latched statuses, when calling #sis8300llrfdrv_clear_latched_statuses */
//#define SIS8300LLRFDRV_STATUS_CLR_SIGMON                2       /** < Set this to clear signal monitor latched statuses, when calling #sis8300llrfdrv_clear_latched_statuses */

/* ==================================================== */
/* ================ Basic information ================= */
/* ==================================================== */
int sis8300drvbpm_get_fw_version(sis8300drv_usr *sisuser, unsigned *ver_device, unsigned *ver_major, unsigned *ver_minor);
int sis8300drvbpm_get_sw_id(sis8300drv_usr *sisuser, unsigned *sw_id);
int sis8300drvbpm_set_sw_id(sis8300drv_usr *sisuser, unsigned sw_id);

/* ==================================================== */
/* ================ Initialization ==================== */
/* ==================================================== */
int sis8300drvbpm_setup_dac(sis8300drv_usr *sisuser);
int sis8300drvbpm_setup_adc_tap_delay(sis8300drv_usr *sisuser);

/* ==================================================== */
/* ============ Triggers and Interlocks =============== */
/* ==================================================== */
/**
 * @brief Trigger setup
 *
 * Enumerator of trigger setups for PULSE_COMMING, PULSE_START, PULSE_END,
 * PMS trigger is always on mlvds3,7 and harlink4 (high on any oif these outputs will
 * cause the trigger)
 */
typedef enum {
    mlvds_012 = 0,  /**< PULSE_COMMING on mlvds0, PULSE_START on mlvds1, PULSE_END on mlvds2 */
    mlvds_456 = 1   /**< PULSE_COMMING on mlvds4, PULSE_START on mlvds5, PULSE_END on mlvds6 */
} sis8300llrfdrv_trg_setup;
int sis8300llrfdrv_set_trigger_setup(sis8300drv_usr *sisuser, sis8300llrfdrv_trg_setup trg_setup);
int sis8300llrfdrv_get_trigger_setup(sis8300drv_usr *sisuser, sis8300llrfdrv_trg_setup *trg_setup);

/**
 * @brief Interlock condition
 *
 * Enumerator for all the available ILOCK conditions, an active ILOCK will trigger
 * PMS in custom logic. This will happen only once after activation of ILOCK
 *
 * IMPORTAINT: a call to #sis8300llrfdrv_sw_reset will disable level sensitive ILOCK,
 * which will allow the controller to run again after a PMS, even though the HW ILOCK
 * might still be active.
 */
//typedef enum {
//    ilock_disabled     = 0,
//    ilock_rising_edge  = 1,
//    ilock_falling_edge = 2,
//    ilock_high_level   = 3,
//    ilock_low_level    = 4
//} sis8300llrfdrv_ilock_condition;
//int sis8300llrfdrv_set_ilock_condition(sis8300drv_usr *sisuser, unsigned harl_inp, sis8300llrfdrv_ilock_condition condition);
//int sis8300llrfdrv_get_ilock_condition(sis8300drv_usr *sisuser, unsigned harl_inp, sis8300llrfdrv_ilock_condition *condition);

/* ==================================================== */
/* ================= Controller status rw ============= */
/* ==================================================== */
/**
 * @brief GOP - General Output register bits
 *
 * Value of each element corresponds to bit that represents the
 * element in the register (except for gop_all).
 */
typedef enum {
    gop_all = 0,                  /** < all status bits */
	gop_position2_error = 3,      /** < position 2 out of bounds */
    gop_position1_error = 4,      /** < position 1 out of bounds */
    gop_write_error = 5,          /** < write error status */
    gop_read_error = 6,           /** < read error status */
    gop_Y2_pos_div_error = 7,     /** < Y2 position divider was 0 */
    gop_X2_pos_div_error = 8,     /** < X2 position divider was 0 */
    gop_Y1_pos_div_error = 9,     /** < Y1 position divider was 0 */
    gop_X1_pos_div_error = 10,    /** < X1 position divider was 0 */
    gop_daq_done = 11,            /** < DAQ done signal (from Struck FW) */
} sis8300drvbpm_gop_bit;
int sis8300drvbpm_get_gop(sis8300drv_usr *sisuser, sis8300drvbpm_gop_bit gop_bit, unsigned *gop_status);
int sis8300llrfdrv_clear_gop(sis8300drv_usr *sisuser);

int sis8300drvbpm_get_pulse_done_count(sis8300drv_usr *sisuser, unsigned *pulse_count);
int sis8300drvbpm_clear_pulse_done_count(sis8300drv_usr *sisuser);

int sis8300drvbpm_update_parameters(sis8300drv_usr *sisuser);
int sis8300drvbpm_init_done(sis8300drv_usr *sisuser);
int sis8300drvbpm_sw_reset(sis8300drv_usr *sisuser);

/* ==================================================== */
/* ================== ACQUISITION ===================== */
/* ==================================================== */
int sis8300llrfdrv_arm_device(sis8300drv_usr *sisuser);
int sis8300llrfdrv_wait_pulse_done_pms(sis8300drv_usr *sisuser, unsigned timeout);

/* ===================================================== */
/* ================= Near IQ control =================== */
/* ===================================================== */
int sis8300drvbpm_set_near_iq(sis8300drv_usr *sisuser, unsigned M, unsigned N);
int sis8300drvbpm_get_near_iq(sis8300drv_usr *sisuser, unsigned *M, unsigned *N);

/* ==================================================== */
/* ================ FIR filter setup ================== */
/* ==================================================== */
#define SIS8300DRVBPM_FIR_FILTER_PARAM_NUM         6
int sis8300drvbpm_set_bpm_filter_param(sis8300drv_usr *sisuser, double *param_vals, int param_count);
int sis8300drvbpm_set_fir_filter_enable(sis8300drv_usr *sisuser, int param_val);
int sis8300drvbpm_get_fir_filter_enable(sis8300drv_usr *sisuser, int *param_val);

/* ==================================================== */
/* ================== Type Conversion ================= */
/* ==================================================== */
/* Controller uses fixed point representation of fractional numbers.
 * They can be either Signed(m,n) or Unsigned(m,n), where m = nr. of bits
 * to represent the integer and n = nr. of bits to represent the fraction.
 * The types are marked as Qmn in function, with sign bit inclusive.
 *
 * None of the types defined in BPM custom logic have more than 32 bits.
 *
 * All the values that are read/written to/from register are in uint32_t
 * format, since this is the format used for register r/w in API.
 * The conversion functions provided here thus convert between double and Qmn.
 *
 * None of these functions require the device context to work.
 */

/**
 * @brief Structure for Qmn type
 *
 * This structure represents the fixed point format
 * radix that the device uses
 */
typedef struct t_sis8300drvbpm_Qmn {
    unsigned int_bits_m;    /** < Number of integer bits (sign bit inclusive) */
    unsigned frac_bits_n;   /** < Number of fraction bits */
    unsigned is_signed;     /** < Number is signed or unsigned */
} sis8300drvbpm_Qmn;

/* Type definitions */
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_near_iq;
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_pos_thr;
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_mag_thr;
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_filter_coeff;

int sis8300drvbpm_double_2_Qmn(double val, sis8300drvbpm_Qmn Qmn, uint32_t *converted, double *err);
void sis8300drvbpm_Qmn_2_double(uint32_t val, sis8300drvbpm_Qmn Qmn, double *converted);

#ifdef __cplusplus
}
#endif

#endif /* SIS8300LLRFDRV_H_ */
