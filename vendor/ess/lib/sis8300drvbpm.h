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
 * @file sis8300llrfdrv.h
 * @brief Header file for the sis8300 LLRF firmware user space library. 
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
#define SIS8300LLRFDRV_HW_ID                    0xCA5E  /** < Unique BPM controller ID */
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


#define SIS8300LLRFDRV_STATUS_CLR_GENERAL               1       /** < Set this to clear general latched statuses, when calling #sis8300llrfdrv_clear_latched_statuses */
//#define SIS8300LLRFDRV_STATUS_CLR_SIGMON                2       /** < Set this to clear signal monitor latched statuses, when calling #sis8300llrfdrv_clear_latched_statuses */

/* ==================================================== */
/* ====== Type Conversion and limit checks ============ */
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
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_pos_threshold;
extern const sis8300drvbpm_Qmn sis8300drvbpm_Qmn_mag_threshold;

int sis8300drvbpm_double_2_Qmn(double val, sis8300drvbpm_Qmn Qmn, uint32_t *converted, double *err);
void sis8300drvbpm_Qmn_2_double(uint32_t val, sis8300drvbpm_Qmn Qmn, double *converted);

/* ==================================================== */
/* ================ Basic information ================= */
/* ==================================================== */
int sis8300llrfdrv_get_fw_version(sis8300drv_usr *sisuser, unsigned *ver_device, unsigned *ver_major, unsigned *ver_minor);
int sis8300llrfdrv_get_sw_id(sis8300drv_usr *sisuser, unsigned *sw_id);
int sis8300llrfdrv_set_sw_id(sis8300drv_usr *sisuser, unsigned sw_id);

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
 * element in the register (except for status_all).
 */
//HK LLRF specific
//typedef enum {
//    gen_status_all = 2,                  /** < GOP all status bits */
//    gen_status_PMS_active = 3,           /** < PMS active status */
//    gen_status_write_error = 4,          /** < write error status */
//    gen_status_read_error = 5,           /** < read error status */
//    gen_status_pi_overflow_I = 6,        /** < PI overflow in Q part */
//    gen_status_pi_overflow_Q = 7,        /** < PI overflow in I part */
//    gen_status_vm_mag_limiter_active = 8 /** VM output limiter is active and constraining output to SIS8300LLRFDRV_VM_MAG_LIMIT */
//} sis8300llrfdrv_gen_status_bit;
typedef enum {
    gen_status_all = 2,                  /** < GOP all status bits */
	gen_status_position2_error = 3,      /** < position 2 out of bounds */
    gen_status_position1_error = 4,      /** < position 1 out of bounds */
    gen_status_write_error = 5,          /** < write error status */
    gen_status_read_error = 6,           /** < read error status */
    gen_status_Y2_pos_div_error = 7,     /** < Y2 position divider was 0 */
    gen_status_X2_pos_div_error = 8,     /** < X2 position divider was 0 */
    gen_status_Y1_pos_div_error = 9,     /** < Y1 position divider was 0 */
    gen_status_X1_pos_div_error = 10,    /** < X1 position divider was 0 */
    gen_status_daq_done = 11,            /** < DAQ done signal (from Struck FW) */
} sis8300llrfdrv_gen_status_bit;
int sis8300llrfdrv_get_general_status(sis8300drv_usr *sisuser, sis8300llrfdrv_gen_status_bit gen_status_bit, unsigned *gen_status);
/**
 * @brief Enumerator of signal monitor statuses
 */
//typedef enum {
//    sigmon_stat_ilock = 0,
//    sigmon_stat_pms   = 1,
//    sigmon_stat_alarm = 2
//} sis8300llrfdrv_sigmon_stat;
//int sis8300llrfdrv_get_sigmon_status(sis8300drv_usr *sisuser, sis8300llrfdrv_sigmon_stat status_select, unsigned *status_val);

int sis8300llrfdrv_clear_latched_statuses(sis8300drv_usr *sisuser, unsigned what); /* TODO: rename */

int sis8300llrfdrv_get_pulse_done_count(sis8300drv_usr *sisuser, unsigned *pulse_count);
int sis8300llrfdrv_clear_pulse_done_count(sis8300drv_usr *sisuser);

int sis8300llrfdrv_update(sis8300drv_usr *sisuser, unsigned update_reason);
int sis8300llrfdrv_init_done(sis8300drv_usr *sisuser);
int sis8300llrfdrv_sw_reset(sis8300drv_usr *sisuser);

/* ==================================================== */
/* =============== Control tables ===================== */
/* ==================================================== */
/**
 * @brief Control table types
 */
//typedef enum {
//    ctrl_table_ff      = 0, /**< Set Point control table */
//    ctrl_table_sp      = 1, /**< Feed forward control table */
//    ctrl_table_invlaid = 2
//} sis8300llrfdrv_ctrl_table;
//
//int sis8300llrfdrv_set_pulse_type(sis8300drv_usr *sisuser, unsigned pulse_type);
//int sis8300llrfdrv_get_pulse_type(sis8300drv_usr *sisuser, unsigned *pulse_type);
//
//int sis8300llrfdrv_get_ctrl_table_max_nelm(sis8300drv_usr *sisuser, sis8300llrfdrv_ctrl_table ctrl_table, unsigned *max_nelm);
//
//int sis8300llrfdrv_set_ctrl_table_nelm(sis8300drv_usr *sisuser, sis8300llrfdrv_ctrl_table ctrl_table, unsigned nelm);
//int sis8300llrfdrv_get_ctrl_table_nelm(sis8300drv_usr *sisuser, sis8300llrfdrv_ctrl_table ctrl_table, unsigned *nelm);
//
//int sis8300llrfdrv_set_ctrl_table_ff_speed(sis8300drv_usr *sisuser, unsigned speed);
//int sis8300llrfdrv_get_ctrl_table_ff_speed(sis8300drv_usr *sisuser, unsigned *speed);
//
//int sis8300llrfdrv_set_ctrl_table_raw(sis8300drv_usr *sisuser, sis8300llrfdrv_ctrl_table ctrl_table, unsigned pt, unsigned nelm, void *table_raw);
//int sis8300llrfdrv_get_ctrl_table_raw(sis8300drv_usr *sisuser, sis8300llrfdrv_ctrl_table ctrl_table, unsigned pt, unsigned nelm, void *table_raw);

/* ==================================================== */
/* ================== ACQUISITION ===================== */
/* ==================================================== */
int sis8300llrfdrv_arm_device(sis8300drv_usr *sisuser);
int sis8300llrfdrv_wait_pulse_done_pms(sis8300drv_usr *sisuser, unsigned timeout);

/**
 * @brief Sample count readbacks
 */
//typedef enum {
//    samples_cnt_pi_total         = 0,
//    samples_cnt_pi_ramp_up_phase = 1,
//    samples_cnt_pi_active_phase  = 2,
//    samples_cnt_cavity_total     = 3
//} sis8300llrfdrv_samples_cnt;
//#define SIS8300LLRFDRV_SAMPLES_CNT_NUM 4
//int sis8300llrfdrv_get_acquired_nsamples(sis8300drv_usr *sisuser, sis8300llrfdrv_samples_cnt samples_cnt_source, unsigned *samples_cnt_val);
//int sis8300llrfdrv_get_pi_err_max_nsamples(sis8300drv_usr *sisuser, unsigned *max_nsamples);
//
//int sis8300llrfdrv_read_pi_error_raw(sis8300drv_usr *sisuser, void *raw_data, unsigned nsamples);
//int sis8300llrfdrv_get_signal_ma(sis8300drv_usr *sisuser, int signal, double *angle, double *magnitude);

/* ==================================================== */
/* =============== PARAMTER SETUPS ==================== */
/* ==================================================== */
/* Below are paramer setters and getters for:
 *              * PI Cotnroller
 *              * Modulator Ripple Filter
 *              * Non IQ Sampling
 *              * Vector Modulator
 *              * Signal Monitoring
 * They all work in the same principle of having an
 * enumerator of possible paramters and than a getter
 * and a setter which take/return a double. When setting
 * a param they provide also the rounding error because
 * the paramter needs to be converted to fixed point.
 * If it is an integer param, the error is always zero  */
/* ==================================================== */
/* ==================================================== */


/* =============== PI CONTROLLER SETUP ================ */

/**
 * @brief output mode
 */
//typedef enum {
//    output_drive_pi = 0,
//    output_drive_ff = 1
//} sis8300llrfdrv_output_drive_src;
//int sis8300llrfdrv_set_output_drive_src(sis8300drv_usr *sisuser, sis8300llrfdrv_output_drive_src src);
//int sis8300llrfdrv_get_output_drive_src(sis8300drv_usr *sisuser, sis8300llrfdrv_output_drive_src* src);

/**
 * @brief PI instance enumerator
 */
//typedef enum {
//    pi_I   = 0, /** < Q PI instance */
//    pi_Q   = 1  /** < I PI instance */
//} sis8300llrfdrv_pi_type;

/**
 * @brief Enumerator of parameters/settings required to setup the PI controller.
 */
//typedef enum {
//    pi_param_gain_K        = 0,  /**< PI controller K gain */
//    pi_param_gain_TSdivTI  = 1,  /**< PI controller ts/ti gain */
//    pi_param_sat_max       = 2,  /**< Saturation max value */
//    pi_param_sat_min       = 3,  /**< Saturation min value */
//    pi_param_fixed_sp_val  = 4,  /**< PI Controller Fixed SP val */
//    pi_param_fixed_ff_val  = 5,  /**< PI Controller Fixed FF val */
//    /* enable or disable flags */
//    pi_param_fixed_sp_en   = 6,  /**< Enable fixed set point */
//    pi_param_fixed_ff_en   = 7   /**< Enable fixed Feed forward */
//} sis8300llrfdrv_pi_param;
//#define SIS8300LLRFDRV_PI_PARAM_INT_FIRST pi_param_fixed_sp_en
//#define SIS8300LLRFDRV_PI_PARAM_NUM       8
//
//int sis8300llrfdrv_set_pi_param(sis8300drv_usr *sisuser, sis8300llrfdrv_pi_type pi, sis8300llrfdrv_pi_param pi_param, double param_val, double *param_err);
//int sis8300llrfdrv_get_pi_param(sis8300drv_usr *sisuser, sis8300llrfdrv_pi_type pi, sis8300llrfdrv_pi_param pi_param, double *param_val);


/* =============== Modulator ripple setup ============= */
/**
 * @brief Enumerator of modulator ripple filter paramters/settings
 */
//typedef enum {
//    mod_ripple_fil_const_s      = 0,    /**< Modulator Ripple Filter constant s */
//    mod_ripple_fil_const_c      = 1,    /**< Modulator Ripple Filter Constant c */
//    mod_ripple_fil_const_a      = 2,    /**< Modulator Ripple Filter Constant A */
//    /* integer params */
//    /* enable or disable flags */
//    mod_ripple_fil_Q_en         = 3,    /**< Enable Modulator Ripple Filter for Q */
//    mod_ripple_fil_I_en         = 4     /**< Enable Modulator Ripple Filter for I */
//} sis8300llrfdrv_mod_ripple_param;
//#define SIS8300LLRFDRV_MOD_RIPPLE_PARAM_INT_FIRST   mod_ripple_fil_Q_en
//#define SIS8300LLRFDRV_MOD_RIPPLE_PARAM_NUM         5
//
//int sis8300llrfdrv_set_mod_ripple_param(sis8300drv_usr *sisuser, sis8300llrfdrv_mod_ripple_param param, double param_val, double *param_err);
//int sis8300llrfdrv_get_mod_ripple_param(sis8300drv_usr *sisuser, sis8300llrfdrv_mod_ripple_param param, double *param_val);


/* =============== Notch Filter setup ============= */
/**
 * @brief Enumerator of notch filter paramters/settings
 */
//typedef enum {
//    notch_fil_const_a_real      = 0,    /**< Notch Filter constant a real part */
//    notch_fil_const_a_imag      = 1,    /**< Notch Filter constant a imaginary part */
//    notch_fil_const_b_real      = 2,    /**< Notch Filter constant b real part */
//    notch_fil_const_b_imag      = 3,    /**< Notch Filter constant b imaginary part */
//    /* integer params */
//    /* enable or disable flags */
//    notch_fil_en                = 4     /**< Enable Notch Filter */
//} sis8300llrfdrv_notch_filter_param;
//#define SIS8300LLRFDRV_NOTCH_FILTER_PARAM_INT_FIRST   notch_fil_en
//#define SIS8300LLRFDRV_NOTCH_FILTER_PARAM_NUM         5
//
//int sis8300llrfdrv_set_notch_filter_param(sis8300drv_usr *sisuser, sis8300llrfdrv_notch_filter_param param, double param_val, double *param_err);
//int sis8300llrfdrv_get_notch_filter_param(sis8300drv_usr *sisuser, sis8300llrfdrv_notch_filter_param param, double *param_val);


/* =============== BPM Filter setup ============= */
/**
 * @brief Enumerator of BPM filter parameters/settings
 */
typedef enum {
    bpm_fil_const_0      = 0,    /**< BPM Filter constant 0 */
    bpm_fil_const_1      = 1,    /**< BPM Filter constant 1 */
    bpm_fil_const_2      = 2,    /**< BPM Filter constant 2 */
    bpm_fil_const_3      = 3,    /**< BPM Filter constant 3 */
    bpm_fil_const_4      = 4,    /**< BPM Filter constant 4 */
    bpm_fil_const_5      = 5,    /**< BPM Filter constant 5 */
    /* integer params */
    /* enable or disable flags */
    bpm_fil_en           = 6     /**< Enable BPM Filter */
} sis8300llrfdrv_bpm_filter_param;
#define SIS8300LLRFDRV_BPM_FILTER_PARAM_INT_FIRST   bpm_fil_en
#define SIS8300LLRFDRV_BPM_FILTER_PARAM_NUM         7

int sis8300llrfdrv_set_bpm_filter_param(sis8300drv_usr *sisuser, double *param_vals, int param_count);
int sis8300llrfdrv_set_bpm_filter_enable(sis8300drv_usr *sisuser, int param_val);
int sis8300llrfdrv_get_bpm_filter_enable(sis8300drv_usr *sisuser, int *param_val);


/* =============== Vector Modulator =============== */
/**
 * @brief Enumerator of Vecotrm Modulator parameters/settings
 */
//typedef enum {
//    vm_param_mag_lim_val        = 0,
//    vm_param_predist_rc00       = 1,
//    vm_param_predist_rc01       = 2,
//    vm_param_predist_rc10       = 3,
//    vm_param_predist_rc11       = 4,
//    vm_param_predist_dcoffset_i = 5,
//    vm_param_predist_dcoffset_q = 6,
//    /* enable or disable flags */
//    vm_param_inverse_q_en       = 7,
//    vm_param_inverse_i_en       = 8,
//    vm_param_mag_lim_en         = 9,
//    vm_param_swap_iq            = 10,
//    vm_param_predistort_en      = 11
//} sis8300llrfdrv_vm_param;
//#define SIS8300LLRFDRV_VM_PARAM_INT_FIRST   vm_param_inverse_q_en
//#define SIS8300LLRFDRV_VM_PARAM_NUM         12
//
//int sis8300llrfdrv_set_vm_param(sis8300drv_usr *sisuser, sis8300llrfdrv_vm_param param, double param_val, double *param_err);
//int sis8300llrfdrv_get_vm_param(sis8300drv_usr *sisuser, sis8300llrfdrv_vm_param param, double *param_val);

/* =============== Non IQ sampling =============== */
/**
 * @brief enumerator of non-IQ sampling parameters
 */
//typedef enum {
//    iq_param_angle_offset_val    = 0,
//    /* integer params */
//    iq_param_cav_inp_delay_val   = 1,
//	/* enable or disable flags */
//    iq_param_angle_offset_en     = 2,
//    iq_param_cav_inp_delay_en	 = 3
//}sis8300llrfdrv_iq_param;
//#define SIS8300LLRFDRV_IQ_PARAM_INT_FIRST   iq_param_cav_inp_delay_val
//#define SIS8300LLRFDRV_IQ_PARAM_NUM         4
//
//int sis8300llrfdrv_set_iq_param(sis8300drv_usr *sisuser, sis8300llrfdrv_iq_param param, double param_val, double *param_err);
//int sis8300llrfdrv_get_iq_param(sis8300drv_usr *sisuser, sis8300llrfdrv_iq_param param, double *param_val);

int sis8300llrfdrv_set_near_iq(sis8300drv_usr *sisuser, unsigned M, unsigned N);
int sis8300llrfdrv_get_near_iq(sis8300drv_usr *sisuser, unsigned *M, unsigned *N);

/* =================== Signal Monitoring ============== */
/**
 * @brief Eumerator of signal monitoring paramters
 */
//typedef enum {
//    sigmon_treshold       = 0,
//    /* integer params */
//    sigmon_start_evnt     = 1,
//    sigmon_end_evnt       = 2,
//    /* enable or disable flags */
//    sigmon_alarm_cnd       = 3,
//    sigmon_pms_en     	  = 4,
//    sigmon_ilock_en   	  = 5,
//    sigmon_dc         	  = 6
//} sis8300llrfdrv_sigmon_param;
//#define SIS8300LLRFDRV_SIGMON_PARAM_INT_FIRST sigmon_start_evnt
//#define SIS8300LLRFDRV_SIGMON_PARAM_NUM       7
//
//int sis8300llrfdrv_set_sigmon_param(sis8300drv_usr *sisuser, sis8300llrfdrv_sigmon_param param, int chan, double param_val, double *param_err);
//int sis8300llrfdrv_get_sigmon_param(sis8300drv_usr *sisuser, sis8300llrfdrv_sigmon_param param, int chan, double *param_val);
//
//int sis8300llrfdrv_get_sigmon_mag_curr(sis8300drv_usr *sisuser, int chan, double *mag_curr_val);
//int sis8300llrfdrv_get_sigmon_mag_minmax(sis8300drv_usr *sisuser, int chan, double *mag_minmax_val);

#ifdef __cplusplus
}
#endif

#endif /* SIS8300LLRFDRV_H_ */
