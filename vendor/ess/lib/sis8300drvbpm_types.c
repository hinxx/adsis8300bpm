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
 * @file sis8300llrfdrv_types.c
 * @brief implementation of sis8300 LLRF type conversion functions
 */

#include "sis8300drvbpm_types.h"
#include "sis8300drv.h"

/** EXTERN TYPEDEFS **/
const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_IQ_sample =
        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 1 };
const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_near_iq =      
        { .int_bits_m = 2,  .frac_bits_n = 30, .is_signed = 1 };
//const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_sp_mag =
//        { .int_bits_m = 0,  .frac_bits_n = 16, .is_signed = 0 };
//const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_ff_mag =
//        { .int_bits_m = 1,  .frac_bits_n = 15, .is_signed = 1 };
const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_angle  =
        { .int_bits_m = 3,  .frac_bits_n = 13, .is_signed = 1 };

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
int sis8300llrfdrv_double_2_Qmn(
        double val, sis8300llrfdrv_Qmn Qmn, 
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
void sis8300llrfdrv_Qmn_2_double(
        uint32_t val, sis8300llrfdrv_Qmn Qmn, double *converted) {
            
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

#if 0
/**
 * @brief Convert raw sample to I and Q part
 * 
 * @param [in]  raw_sample  Raw sample to convert
 * @param [out] I           I value
 * @param [out] Q           Q Value
 * 
 * @return status_success   Always
 */
void sis8300llrfdrv_raw_sample_2_IQ(
        unsigned raw_sample, double *I, double *Q) {
    
    uint32_t i_ui32, q_ui32;

    i_ui32 = raw_sample & 0xffff0000;
    i_ui32 >>= 16;
    sis8300llrfdrv_Qmn_2_double(i_ui32, sis8300llrfdrv_Qmn_IQ_sample, I);

    q_ui32 = raw_sample & 0x0000ffff;
    sis8300llrfdrv_Qmn_2_double(q_ui32, sis8300llrfdrv_Qmn_IQ_sample, Q);

    return;
}

/**
 * @brief Convert I and Q to a raw IQ sample of 32 bits accepted by the 
 *        device
 * 
 * @param [in]  I           I value
 * @param [in]  Q           Q Value
 * @param [out] raw_sample  Will hold converted value on success
 * @param [out] I_err       Will hold conversion error for I on success
 * @param [out] Q_err       Will hold conversion error for Q on success
 * 
 * @return status_success   Conversion sccessfull
 * @return @see #sis8300llrfdrv_double_2_Qmn
 */
int sis8300llrfdrv_IQ_2_raw_sample(
        double I, double Q, unsigned *raw_sample, 
        double *I_err, double *Q_err) {
    
    int status;
    uint32_t i_ui32, q_ui32;

    status = sis8300llrfdrv_double_2_Qmn(
                Q, sis8300llrfdrv_Qmn_IQ_sample, &q_ui32, Q_err);
    if (status) {
        return status;
    }

    status = sis8300llrfdrv_double_2_Qmn(
                I, sis8300llrfdrv_Qmn_IQ_sample, &i_ui32, I_err);
    if (status) {
        return status;
    }

    *raw_sample = i_ui32;
    *raw_sample <<= 16;
    *raw_sample |= q_ui32;

    return status_success;
}
#endif
