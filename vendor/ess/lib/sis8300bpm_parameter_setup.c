#include <stdio.h>
#include <stdint.h>
#include <stdio.h>

#include <sis8300drv.h>
#include <sis8300drv_utils.h>


#include "sis8300drvbpm.h"
#include "sis8300bpm_reg.h"
#include "sis8300drvbpm_types.h"

#if 0
static inline int _sis8300llrfdrv_set_param(
                    sis8300drv_usr *sisuser, 
                    int is_int_param, 
                    uint32_t reg_flags, uint32_t shift, uint32_t addr, 
                    sis8300llrfdrv_Qmn param_Qmn, 
                    double param_val, double *param_err);
static inline int _sis8300llrfdrv_get_param(
                    sis8300drv_usr *sisuser, 
                    int is_int_param, 
                    uint32_t reg_flags, uint32_t shift, uint32_t addr, 
                    sis8300llrfdrv_Qmn param_Qmn, 
                    double *param_val);


/* ===================================================== */
/* =============== PI CONTROLLER SETUP ================= */
/* === gain and saturation, Fixed FF and SP control ==== */
/* ===================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_pi_param */
/**
 * @brief PI parameters: Internal list of corresponding registers
 */
static const uint32_t pi_param_addr[][SIS8300LLRFDRV_PI_PARAM_NUM] = {
    { /* PI1 - I */
     SIS8300LLRF_PI_1_K_REG,         SIS8300LLRF_PI_1_TS_DIV_TI_REG,
     SIS8300LLRF_PI_1_SAT_MAX_REG,   SIS8300LLRF_PI_1_SAT_MIN_REG,
     SIS8300LLRF_PI_1_FIXED_SP_REG,  SIS8300LLRF_PI_1_FIXED_FF_REG,
     SIS8300LLRF_PI_1_CTRL_REG,      SIS8300LLRF_PI_1_CTRL_REG
    },
    { /* PI2 - Q */
     SIS8300LLRF_PI_2_K_REG,         SIS8300LLRF_PI_2_TS_DIV_TI_REG,
     SIS8300LLRF_PI_2_SAT_MAX_REG,   SIS8300LLRF_PI_2_SAT_MIN_REG,
     SIS8300LLRF_PI_2_FIXED_SP_REG,  SIS8300LLRF_PI_2_FIXED_FF_REG,
     SIS8300LLRF_PI_2_CTRL_REG,      SIS8300LLRF_PI_2_CTRL_REG
     }
};
/** 
 * @brief PI parameters: Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn pi_param_Qmn[] = {
    { .int_bits_m = 8,  .frac_bits_n = 24, .is_signed = 1 },    /* K gain */
    { .int_bits_m = 8,  .frac_bits_n = 24, .is_signed = 1 },    /* Ts/Ti gain */
    { .int_bits_m = 16, .frac_bits_n = 16, .is_signed = 1 },    /* maximum saturation value */
    { .int_bits_m = 16, .frac_bits_n = 16, .is_signed = 1 },    /* minimum saturation value */
    { .int_bits_m =  1, .frac_bits_n = 15, .is_signed = 1 },    /* fixed SP value */
    { .int_bits_m =  1, .frac_bits_n = 15, .is_signed = 1 },    /* fixed FF value */
    { .int_bits_m =  1, .frac_bits_n = 0,  .is_signed = 0 },    /* enable fixed SP */
    { .int_bits_m =  1, .frac_bits_n = 0,  .is_signed = 0 }     /* enable fixed FF */
};
/**
 * @brief PI parameters: Internal list of corresponding masks
 */
static const uint32_t pi_param_mask[] = {
    0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff,
    0x0000ffff, 0x0000ffff,
    0x01,
    0x01
};
/**
 * @brief PI parameters: Internal list of corresponding bit-shifts
 */
static const uint32_t pi_param_shift[] = {
    0, 0, 0, 0, 0, 0, 0, 1
};

/**
 * @brief Set PI parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  pi          The pi controller, #sis8300llrfdrv_pi_type
 * @param [in]  param       Parameter to set, #sis8300llrfdrv_pi_params
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid pi or param choice
 * @return status_argument_range    Argument value is out of range
 *
 *
 * Sets the PI controller parameters. The available setting are:
 *
 * @see #sis8300llrfdrv_pi_param
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls that 
 * alter the functionality of the device. This means that this function 
 * may block.
 */
int sis8300llrfdrv_set_pi_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_pi_type pi, sis8300llrfdrv_pi_param param,
        double param_val, double *param_err) {

    if (param >= SIS8300LLRFDRV_PI_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_PI_PARAM_INT_FIRST,
            pi_param_mask[param] << pi_param_shift[param],
            pi_param_shift[param],
            pi_param_addr[pi][param],
            pi_param_Qmn[param],
            param_val,
            param_err);
}

/**
 * @brief Get PI parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  pi          The pi controller, #sis8300llrfdrv_pi_type
 * @param [in]  param       Parameter to set, #sis8300llrfdrv_pi_params
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid pi or param choice
 *
 */
int sis8300llrfdrv_get_pi_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_pi_type pi, sis8300llrfdrv_pi_param param,
        double *param_val) {

    if (param >= SIS8300LLRFDRV_PI_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_get_param(sisuser,
            param >= SIS8300LLRFDRV_PI_PARAM_INT_FIRST,
            pi_param_mask[param] << pi_param_shift[param],
            pi_param_shift[param],
            pi_param_addr[pi][param],
            pi_param_Qmn[param],
            param_val);
}

/**
 * @brief Select output drive for the controller
 *
 * @param [in] sisuser    User context Struct
 * @param [in] src        Desired output drive 
 *                        #sis8300llrfdrv_output_drive_src
 *
 * @return status_success       Success
 * @return status_device_armed  Operatinq is not allowed when the device 
 *                              is armed
 * @return status_no_device     Device not opened
 * @return status_device_access Cannot access device registers
 *
 * @warning It is impooirtaint to be aware, that htis setting is for 
 * NORMAL operation mode. Hence, make sure that mode_normal is selected. 
 * For this, also @see #sis8300llrfdrv_set_operating_mode
 *
 * This will select either FF or PI driven output. In case of FF driven
 * output, the SP table is used as fill table and the fixed SP value is
 * used during beam on time.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_set_output_drive_src(
        sis8300drv_usr *sisuser, sis8300llrfdrv_output_drive_src src) {
    
    int status;
    sis8300drv_dev *sisdevice;
    uint32_t ui32_reg_val;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_PI_1_CTRL_REG, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    switch (src) {
        case output_drive_pi:
            /* Closed loop cavity filling */
            ui32_reg_val &= ~(0x1 << 10);
            break;
        case output_drive_ff:
            /* Open loop cavity filling */
            ui32_reg_val |= 0x1 << 10;
            break;
        default:
            pthread_mutex_unlock(&sisdevice->lock);
            return status_argument_invalid;
    }

    status = sis8300_reg_write(sisdevice->handle, 
                SIS8300LLRF_PI_1_CTRL_REG, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

int sis8300llrfdrv_get_output_drive_src(
        sis8300drv_usr *sisuser, sis8300llrfdrv_output_drive_src* src) {
    
    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    status = sis8300_reg_read(sisdevice->handle, 
                SIS8300LLRF_PI_1_CTRL_REG, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    *src = (ui32_reg_val & (0x1 << 10)) ? 
                output_drive_ff : output_drive_pi;

    return status_success;
}

/* ==================================================== */
/* =============== Modulator ripple setup ============= */
/* ==================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_mod_ripple_param 
 * Parameters are:
 *  modulator ripple filter constant S
 *  modulator ripple filter constant C
 *  modulator ripple filter constant A
 *  modulator filter start event
 *  enable modulator ripple filter for Q
 *  enable modulator ripple filter for I
 */
/**
 * @brief Modulator Ripple Filter parameters: 
 *        Internal list of corresponding registers
 */
static const uint32_t mod_ripple_param_addr[] = {
    SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTS_REG,
    SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTC_REG,
    SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTA_REG,
    SIS8300LLRF_MOD_RIPPLE_FILTER_CTRL_REG,
    SIS8300LLRF_MOD_RIPPLE_FILTER_CTRL_REG
};
/**
 * @brief Modulator Ripple Filter parameters: 
 *        Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn mod_ripple_param_Qmn[] = {
     { .int_bits_m = 1, .frac_bits_n = 31, .is_signed = 1 },
     { .int_bits_m = 1, .frac_bits_n = 31, .is_signed = 1 },
     { .int_bits_m = 0, .frac_bits_n = 16, .is_signed = 0 },
     { .int_bits_m = 1, .frac_bits_n = 0,  .is_signed = 0 },
     { .int_bits_m = 1, .frac_bits_n = 0,  .is_signed = 0 }
};
/**
 * @brief Modulator Ripple Filter parameters: Internal list of 
 *        corresponding masks
 */
static const uint32_t mod_ripple_param_mask[] = {
   0xffffffff, 0xffffffff, 0x0000ffff, 0x01, 0x01
};
/**
 * @brief Modulator Ripple Filter parameters: Internal list of 
 *        corresponding bit-sifts
 */
static const uint32_t mod_ripple_param_shift[] = {
   0, 0, 16, 1, 0
};

/**
 * @brief Set Modulator Ripple Filter parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_mod_ripple_param
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid pi or param choice
 * @return status_argument_range    Argument value is out of range
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls that
 * alter the functionality of the device. This means that this function 
 * may block.
 */
int sis8300llrfdrv_set_mod_ripple_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_mod_ripple_param param, 
        double param_val, double *param_err) {

    if (param >= SIS8300LLRFDRV_MOD_RIPPLE_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_MOD_RIPPLE_PARAM_INT_FIRST,
            mod_ripple_param_mask[param] << mod_ripple_param_shift[param],
            mod_ripple_param_shift[param],
            mod_ripple_param_addr[param],
            mod_ripple_param_Qmn[param],
            param_val,
            param_err);
}

/**
 * @brief Get Modulator Ripple Filter Parameter Value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_mod_ripple_param
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid pi param choice
 *
 */
int sis8300llrfdrv_get_mod_ripple_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_mod_ripple_param param, double *param_val) {
    
    int status;

    if (param >= SIS8300LLRFDRV_MOD_RIPPLE_PARAM_NUM) {
        return status_argument_invalid;
    }

    status = _sis8300llrfdrv_get_param(sisuser,
            param >= SIS8300LLRFDRV_MOD_RIPPLE_PARAM_INT_FIRST,
            mod_ripple_param_mask[param] << mod_ripple_param_shift[param],
            mod_ripple_param_shift[param],
            mod_ripple_param_addr[param],
            mod_ripple_param_Qmn[param],
            param_val);

    return status;
}

/* ==================================================== */
/* =============== Notch filter setup ============= */
/* ==================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_mod_ripple_param 
 * Parameters are:
 *  Notch filter constant A Real-part
 *  Notch filter constant A Imaginary-part
 *  Notch filter constant B Real-part
 *  Notch filter constant B Imaginary-part
 *  Enable Notch filter
 */
/**
 * @brief Notch Filter parameters: 
 *        Internal list of corresponding registers
 */
static const uint32_t notch_filter_param_addr[] = {
    SIS8300LLRF_NOTCH_FILTER_CONSTA_REG,
    SIS8300LLRF_NOTCH_FILTER_CONSTA_REG,
    SIS8300LLRF_NOTCH_FILTER_CONSTB_REG,
    SIS8300LLRF_NOTCH_FILTER_CONSTB_REG,
    SIS8300LLRF_NOTCH_FILTER_CTRL_REG
};
/**
 * @brief Notch Filter parameters: 
 *        Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn notch_filter_param_Qmn[] = {
     { .int_bits_m = 1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m = 1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m = 1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m = 1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m = 1, .frac_bits_n = 0,  .is_signed = 0 }
};
/**
 * @brief Notch Filter parameters: Internal list of 
 *        corresponding masks
 */
static const uint32_t notch_filter_param_mask[] = {
   0x0000ffff, 0x0000ffff, 0x0000ffff, 0x0000ffff, 0x01
};
/**
 * @brief Notch Filter parameters: Internal list of 
 *        corresponding bit-sifts
 */
static const uint32_t notch_filter_param_shift[] = {
   16, 0, 16, 0, 0
};

/**
 * @brief Set Notch Filter parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_notch_filter_param
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
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
int sis8300llrfdrv_set_notch_filter_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_notch_filter_param param, 
        double param_val, double *param_err) {

    if (param >= SIS8300LLRFDRV_NOTCH_FILTER_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_NOTCH_FILTER_PARAM_INT_FIRST,
            notch_filter_param_mask[param] << notch_filter_param_shift[param],
            notch_filter_param_shift[param],
            notch_filter_param_addr[param],
            notch_filter_param_Qmn[param],
            param_val,
            param_err);
}
/**
 * @brief Get Notch Filter Parameter Value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_mod_ripple_param
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid notch filter param choice
 *
 */

int sis8300llrfdrv_get_notch_filter_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_notch_filter_param param, double *param_val) {
    
    int status;

    if (param >= SIS8300LLRFDRV_NOTCH_FILTER_PARAM_NUM) {
        return status_argument_invalid;
    }

    status = _sis8300llrfdrv_get_param(sisuser,
            param >= SIS8300LLRFDRV_NOTCH_FILTER_PARAM_INT_FIRST,
            notch_filter_param_mask[param] << notch_filter_param_shift[param],
            notch_filter_param_shift[param],
            notch_filter_param_addr[param],
            notch_filter_param_Qmn[param],
            param_val);

    return status;
}

/* ===================================================== */
/* ==================== VM control ===================== */
/* ===================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_vm_param 
 * Parameters are:
 *  magnitude limit value
 *  predistortion matrix RC 00 
 *  predistortion matrix RC 01
 *  predistortion matrix RC 10
 *  predistortion matrix RC 11
 *  DC offset for I
 *  DC offset for Q
 *  enable Q inversion
 *  enable I inversion
 *  enable magnitude limiter
 *  enable swap IQ
 *  enable predistortion
 */
/**
 * @brief Vector Modulator parameters: Internal list of corresponding registers
 */
static const uint32_t vm_param_addr[] = {
    SIS8300LLRF_VM_MAG_LIMIT_REG,
    SIS8300LLRF_VM_PREDIST_R0_REG,
    SIS8300LLRF_VM_PREDIST_R0_REG,
    SIS8300LLRF_VM_PREDIST_R1_REG,
    SIS8300LLRF_VM_PREDIST_R1_REG,
    SIS8300LLRF_VM_PREDIST_DC_REG,
    SIS8300LLRF_VM_PREDIST_DC_REG,
    SIS8300LLRF_VM_CTRL_REG,
    SIS8300LLRF_VM_CTRL_REG,
    SIS8300LLRF_VM_CTRL_REG,
    SIS8300LLRF_VM_CTRL_REG,
    SIS8300LLRF_VM_CTRL_REG
};
/**
 * @brief Vector Modulator parameters: Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn vm_param_Qmn[] = {
     { .int_bits_m = 16, .frac_bits_n = 16, .is_signed = 0 },
     { .int_bits_m =  2, .frac_bits_n = 14, .is_signed = 1 },
     { .int_bits_m =  2, .frac_bits_n = 14, .is_signed = 1 },
     { .int_bits_m =  2, .frac_bits_n = 14, .is_signed = 1 },
     { .int_bits_m =  2, .frac_bits_n = 14, .is_signed = 1 },
     { .int_bits_m =  1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m =  1, .frac_bits_n = 15, .is_signed = 1 },
     { .int_bits_m =  1, .frac_bits_n =  0, .is_signed = 0 },
     { .int_bits_m =  1, .frac_bits_n =  0, .is_signed = 0 },
     { .int_bits_m =  1, .frac_bits_n =  0, .is_signed = 0 },
     { .int_bits_m =  1, .frac_bits_n =  0, .is_signed = 0 },
     { .int_bits_m =  1, .frac_bits_n =  0, .is_signed = 0 }
};
/**
 * @brief Vector Modulator parameters: 
 *        Internal list of corresponding masks
 */
static const uint32_t vm_param_mask[] = {
   0xffffffff, 
   0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
   0x01, 0x01, 0x01, 0x01, 0x1
};
/**
 * @brief Vector Modulator parameters: 
 *        Internal list of corresponding bit-shifts
 */
static const uint32_t vm_param_shift[] = {
   0,
   16, 0, 16, 0, 16, 0,
   0, 1, 2, 3, 6
};

/**
 * @brief Set Vector Modulator parameter/setting
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          sis8300llrfdrv_vm_param
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 * @return status_argument_range    Argument value is out of range
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_set_vm_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_vm_param param, 
        double param_val, double *param_err) {

    if (param >= SIS8300LLRFDRV_VM_PARAM_NUM) {
        return status_argument_invalid;
    }

	return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_VM_PARAM_INT_FIRST,
            vm_param_mask[param] << vm_param_shift[param],
            vm_param_shift[param],
            vm_param_addr[param],
            vm_param_Qmn[param],
            param_val,
            param_err);
}

/**
 * @brief Get Vector Modulator paramter/setting value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to get, #sis8300llrfdrv_vm_param
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 *
 */
int sis8300llrfdrv_get_vm_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_vm_param param, double *param_val) {

    if (param >= SIS8300LLRFDRV_VM_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_get_param(sisuser,
            param >= SIS8300LLRFDRV_VM_PARAM_INT_FIRST,
            vm_param_mask[param] << vm_param_shift[param],
            vm_param_shift[param],
            vm_param_addr[param],
            vm_param_Qmn[param],
            param_val);
}
/* ===================================================== */
/* ====================== IQ control =================== */
/* ===================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_iq_param 
 * Parameters are:
 *  LLRF IQ angle
 *  LLRF cavity input delay value
 *  LLRF IQ angle rotate enable
 *  LLRF cavity input delay enable
 */
/**
 * @brief Non IQ sampling parameters: 
 *        Internal list of corresponding registers
 */
static const uint32_t iq_param_addr[] = {
    SIS8300LLRF_IQ_ANGLE_REG,
    SIS8300LLRF_IQ_CTRL_REG,
    SIS8300LLRF_IQ_CTRL_REG,
    SIS8300LLRF_IQ_CTRL_REG
};
/**
 * @brief Non IQ sampling parameters: 
 *        Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn iq_param_Qmn[] = {
     { .int_bits_m = 16, .frac_bits_n = 16, .is_signed = 1 },
     { .int_bits_m = 7,  .frac_bits_n = 0,  .is_signed = 0 },
     { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 },
     { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 }
};
/**
 * @brief Non IQ sampling parameters: 
 *        Internal list of corresponding masks
 */
static const uint32_t iq_param_mask[] = {
     0xffffffff, 0x3f, 0x1, 0x1               
};
/**
 * @brief Non IQ sampling parameters: 
 *        Internal list of corresponding bit-shifts
 */
static const uint32_t iq_param_shift[] = {
     0, 7, 0, 1
};

/**
 * @brief Set IQ parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_io_ctrl_param
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 * @return status_argument_range    Argument value is out of range
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_set_iq_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_iq_param param, 
        double param_val, double *param_err) {

    if (param >= SIS8300LLRFDRV_IQ_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_IQ_PARAM_INT_FIRST,
            iq_param_mask[param] << iq_param_shift[param],
            iq_param_shift[param],
            iq_param_addr[param],
            iq_param_Qmn[param],
            param_val,
            param_err);
}

/**
 * @brief Get IQ parameter value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_io_ctrl_param
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 *
 */
int sis8300llrfdrv_get_iq_param(
        sis8300drv_usr *sisuser, 
        sis8300llrfdrv_iq_param param, double *param_val) {

    if (param >= SIS8300LLRFDRV_IQ_PARAM_NUM) {
        return status_argument_invalid;
    }

    return _sis8300llrfdrv_get_param(sisuser,
            param >= SIS8300LLRFDRV_IQ_PARAM_INT_FIRST,
            iq_param_mask[param] << iq_param_shift[param],
            iq_param_shift[param],
            iq_param_addr[param],
            iq_param_Qmn[param],
            param_val);
}
#endif

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
    conv_fact = (double) (1 << sis8300llrfdrv_Qmn_near_iq.frac_bits_n);

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

#if 0
/* ==================================================== */
/* =================== Signal Monitoring ============== */
/* ==================================================== */
/* Bits, masks and registers @see sis8300llrfdrv_sigmon_param 
 * Singal monitoring setting are only available for AI2 - AI9,
 * Cavity (AI0) and reference (AI1) input do not have them 
 * Parameters are: 
 *      magnitude treshold
 *      monitor start event
 *      monitor end event
 *      alarm enable
 *      PMS enable
 *      ILOCK enable
 *      is signal DC?
 * 
 * */
/**
 * @brief Singal Monitoring parameters: 
 *        Internal list of corresponding registers
 * 
 * Magnitude treshold values have a separate set of registers. All the 
 * other parameters are in the same signal monitor parameter register
 * #SIS8300LLRF_MON_PARAM_1_REG or #SIS8300LLRF_MON_PARAM_2_REG
 */
static const uint32_t sigmon_param_addr[][SIS8300DRV_NUM_AI_CHANNELS] = {
    {0, 0,
     SIS8300LLRF_MON_LIMIT_1_REG, SIS8300LLRF_MON_LIMIT_1_REG,
     SIS8300LLRF_MON_LIMIT_2_REG, SIS8300LLRF_MON_LIMIT_2_REG,
     SIS8300LLRF_MON_LIMIT_3_REG, SIS8300LLRF_MON_LIMIT_3_REG,
     SIS8300LLRF_MON_LIMIT_4_REG, SIS8300LLRF_MON_LIMIT_4_REG},
    {0, 0,
     SIS8300LLRF_MON_PARAM_1_REG, SIS8300LLRF_MON_PARAM_1_REG, 
     SIS8300LLRF_MON_PARAM_1_REG, SIS8300LLRF_MON_PARAM_1_REG,
     SIS8300LLRF_MON_PARAM_2_REG, SIS8300LLRF_MON_PARAM_2_REG, 
     SIS8300LLRF_MON_PARAM_2_REG, SIS8300LLRF_MON_PARAM_2_REG}
};
/**
 * @brief Singal Monitoring parameters: 
 *        Internal list of corresponding radixes
 */
static const sis8300llrfdrv_Qmn sigmon_param_Qmn[] = {
    { .int_bits_m = 0,  .frac_bits_n = 16, .is_signed = 0 },
    { .int_bits_m = 2,  .frac_bits_n = 0,  .is_signed = 0 },
    { .int_bits_m = 2,  .frac_bits_n = 0,  .is_signed = 0 },
    { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 },
    { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 },
    { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 },
    { .int_bits_m = 1,  .frac_bits_n = 0,  .is_signed = 0 }
};
/**
 * @brief Singal Monitoring parameters:
 *        Internal list of corresponding masks
 */
static const uint32_t sigmon_param_mask[] = {
    0x0000ffff, 0x03, 0x03, 0x1, 0x1, 0x1, 0x1
};
/**
 * @brief Singal Monitoring parameters: 
 *        Internal list of corresponding bit-shifts
 */
static const uint32_t sigmon_param_shift[][SIS8300DRV_NUM_AI_CHANNELS] = {
    {0, 0, 0,   16,     0,    16,   0,   16,     0,    16},
    {0, 0, 0,    8,    16,    24,   0,    8,    16,    24},
    {0, 0, 2,  2+8,  2+16,  2+24,   2,  2+8,  2+16,  2+24},
    {0, 0, 4,  4+8,  4+16,  4+24,   4,  4+8,  4+16,  4+24},
    {0, 0, 5,  5+8,  5+16,  5+24,   5,  5+8,  5+16,  5+24},
    {0, 0, 6,  6+8,  6+16,  6+24,   6,  6+8,  6+16,  6+24},
    {0, 0, 7,  7+8,  7+16,  7+24,   7,  7+8,  7+16,  7+24}
};
/**
 * @brief Set Signal Monitor parameter/setting
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set,
 *                          #sis8300llrfdrv_sigmon_param
 * @param [in]  chan        Channel to which the setting applies
 * @param [in]  param_val   Value of parameter to set
 * @param [out] param_err   Contains difference between set and desired 
 *                          value
 *
 * @return status_success           On successful set
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice  
 * @return status_argument_range    Argument value is out of range or 
 *                                  chan is not between 
 *                                  #SIS8300LLRFDRV_SIGMON_CHAN_FIRST
 *                                  and #SIS8300DRV_NUM_AI_CHANNELS
 *
 * This is a shadow register. To make the controller see new parameters,
 * a call to #sis8300llrfdrv_update is needed.
 *
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
int sis8300llrfdrv_set_sigmon_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_sigmon_param param, 
        int chan, double param_val, double *param_err) {

	if (param >= SIS8300LLRFDRV_SIGMON_PARAM_NUM) {
        return status_argument_invalid;
    }
    
    if (chan == SIS8300LLRFDRV_AI_CHAN_CAV ||
		chan == SIS8300LLRFDRV_AI_CHAN_REF ||
	    chan >= SIS8300DRV_NUM_AI_CHANNELS) { 
		return status_argument_range;
	}

    if (param == sigmon_end_evnt) {
        param_val -= 1.0;
    }

    return _sis8300llrfdrv_set_param(sisuser,
            param >= SIS8300LLRFDRV_SIGMON_PARAM_INT_FIRST,
            sigmon_param_mask[param] << sigmon_param_shift[param][chan],
            sigmon_param_shift[param][chan],
            sigmon_param_addr[param != sigmon_treshold][chan],
            sigmon_param_Qmn[param],
            param_val, param_err);
}

/**
 * @brief Get signal monitoring paramter/setting value
 *
 * @param [in]  sisuser     User context struct
 * @param [in]  param       Parameter to set, 
 *                          #sis8300llrfdrv_sigmon_param
 * @param [in[  chan        Signal monitor channel to set
 * @param [out] param_val   Will contain parameter value on success
 *
 * @return status_success           Information retrieved successfully
 * @return status_device_access     Can't access device registers.
 * @return status_no_device         Device not opened.
 * @return status_argument_invalid  Invalid param choice
 *
 */
int sis8300llrfdrv_get_sigmon_param(
        sis8300drv_usr *sisuser, sis8300llrfdrv_sigmon_param param, 
        int chan, double *param_val) {
    
    int status;
    
	if (param >= SIS8300LLRFDRV_SIGMON_PARAM_NUM) {
        return status_argument_invalid;
    }
    
    if (chan == SIS8300LLRFDRV_AI_CHAN_CAV ||
		chan == SIS8300LLRFDRV_AI_CHAN_REF ||
	    chan >= SIS8300DRV_NUM_AI_CHANNELS) { 
		return status_argument_range;
	}

    status = _sis8300llrfdrv_get_param(sisuser, 
			 param >= SIS8300LLRFDRV_SIGMON_PARAM_INT_FIRST,
             sigmon_param_mask[param] << sigmon_param_shift[param][chan],
             sigmon_param_shift[param][chan],
             sigmon_param_addr[param != sigmon_treshold][chan],
             sigmon_param_Qmn[param],
             param_val);

    if (param == sigmon_end_evnt) {
        *param_val += 1.0;
    }

    return status;
}



/* ======================= INTERNAL LIBRARY FUNCTIONS =============== */
/**
 * @brief internal library function to set a controller parameter
 * 
 * @param [in]  sisuser         Device user context
 * @param [in]  is_int_param    Is this an intiger type parameter
 * @param [in]  reg_flags       Register flags to mask out the 
 *                              non-relavant bits
 * @param [in]  shift           Register bit-shift to get the relevant 
 *                              value
 * @param [in]  addr            Register address where the parameter is 
 *                              located
 * @param [in]  param_Qmn       Paramter radix to convert from doulbe to 
 *                              device fixed fpoint format
 * @param [in]  param_val       Value of the parameter to write to the 
 *                              device
 * @param [out] param_err       Will hold the conversion error from 
 *                              double to fixed point on success
 * 
 * @return status_success           Data written successfully
 * @return status_no_device         Device not found
 * @return status_argument_invalid  Faulty register flags assume that 
 *                                  this is not a valid setting
 * @return  status_device_access    Could not acces sdevice registers
 * 
 * The function is used by #sis8300llrfdrv_set_pi_param, 
 * #sis8300llrfdrv_set_mod_ripple_param, #sis8300llrfdrv_set_vm_param, 
 * #sis8300llrfdrv_set_iq_param and #sis8300llrfdrv_set_sigmon_param
 * 
 * Calls to this function are serialized with respect to other calls 
 * that alter the functionality of the device. This means that this 
 * function may block.
 */
static inline int _sis8300llrfdrv_set_param(
            sis8300drv_usr *sisuser, int is_int_param, 
            uint32_t reg_flags, uint32_t shift, uint32_t addr, 
            sis8300llrfdrv_Qmn param_Qmn, 
            double param_val, double *param_err) {
    
    int status;
    uint32_t ui32_reg_val, ui32_tmp_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&sisdevice->lock);

    /* check if argument is valid */
    if (reg_flags == 0) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_argument_invalid;
    }

    status = sis8300_reg_read(sisdevice->handle, addr, &ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }

    /* convert the double value to fixed point accepted by device */
    if (is_int_param) {
        ui32_tmp_val = (uint32_t) param_val;
        *param_err = 0.0;
    }
    else {
        status = sis8300llrfdrv_double_2_Qmn(param_val, param_Qmn, 
                        &ui32_tmp_val, param_err);
        if (status) {
            pthread_mutex_unlock(&sisdevice->lock);
            return status;
        }
    }

    /* mask out the old val and set the new one */
    ui32_reg_val &= ~reg_flags;
    ui32_reg_val |= ((ui32_tmp_val << shift) & reg_flags);

    status = sis8300_reg_write(sisdevice->handle, addr, ui32_reg_val);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status_device_access;
    }
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief internal library function to read a controller parameter
 * 
 * @param [in]  sisuser         Device user context
 * @param [in]  is_int_param    Is this an intiger type parameter
 * @param [in]  reg_flags       Register flags to mask out the 
 *                              non-relavant bits
 * @param [in]  shift           Register bit-shift to get the relevant 
 *                              value
 * @param [in]  addr            Register address where the parameter is 
 *                              located
 * @param [in]  param_Qmn       Paramter radix to convert from device 
 *                              fixed point format to doulbe
 * @param [out] param_val       Will hold the parameter value on success
 * 
 * @return status_success           Data written successfully
 * @return status_no_device         Device not found
 * @return status_argument_invalid  Faulty register flags assume that 
 *                                  this is not a valid setting
 * @return status_device_access     Could not acces sdevice registers
 * 
 * The function is used by #sis8300llrfdrv_get_pi_param, 
 * #sis8300llrfdrv_get_mod_ripple_param, #sis8300llrfdrv_get_vm_param, 
 * #sis8300llrfdrv_get_iq_param and #sis8300llrfdrv_get_sigmon_param
 */
static inline int _sis8300llrfdrv_get_param(
            sis8300drv_usr *sisuser, int is_int_param, 
            uint32_t reg_flags, uint32_t shift, uint32_t addr, 
            sis8300llrfdrv_Qmn param_Qmn, double *param_val) {
                
    int status;
    uint32_t ui32_reg_val;
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    /* check if argument is valid */
    if (reg_flags == 0) {
        return status_argument_invalid;
    }

    /* get the value */
    status = sis8300_reg_read(sisdevice->handle, addr, &ui32_reg_val);
    if (status) {
        return status_device_access;
    }

    ui32_reg_val &= reg_flags;
    ui32_reg_val >>= shift;

    /* convert the value */
    if (is_int_param) {
        *param_val = (double) ui32_reg_val;
    }
    else {
        sis8300llrfdrv_Qmn_2_double(ui32_reg_val, param_Qmn, param_val);
    }

    return status_success;
}

#endif
