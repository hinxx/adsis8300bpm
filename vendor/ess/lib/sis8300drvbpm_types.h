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
#if 0
/**
 * @file sis8300llrfdrv_utils.h
 * @brief Header file for the sis8300 LLRF type conversion functions
 * 
 * Defines helper functions to convert from/to fixed point types
 * Signed(int bits, frac bits) and Unsigned(int bits, frac bits) 
 * accepted by custom logic.
 * Defines helper functions to convert a 32 bit raw IQ sample on
 * the device to I and Q and back.
 */

#ifndef SIS8300LLRFDRV_TYPES_H_
#define SIS8300LLRFDRV_TYPES_H_

#include <stdint.h>
/*
#ifndef __USE_BSD
	#define __USE_BSD
#endif
*/
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====== Type Conversion and limit checks ======
 * Controller uses fixed point representation of fractional numbers. They can be either
 * Signed(m,n) or Unsigned(m,n), where m = nr. of bits to represent the integer and n = nr. 
 * of bits to represent the fraction. The types are marked as Qmn in function, with sign bit inclusive.
 * 
 * None of the types defined in LLRF custom logic have more than 32 bits.
 * 
 * All the values that are read/written to/from register are in uint32_t format, since this
 * is the format used for register r/w troughout this api. The conversion functions provided here thus 
 * convert between double and Qmn.
 *
 * None of these functions require the device context to work. They just do conversion
 */
 

//#define SIS8300LLRFDRV_RAW_SAMPLE_I_OFFSET    1 /**< Offset of the I value in the raw 32 bit IQ sample on the device */
//#define SIS8300LLRFDRV_RAW_SAMPLE_Q_OFFSET    0 /**< Offset of the Q value in the raw 32 bit IQ sample on the device */

/**
 * @brief Structure for Qmn type
 * 
 * This structure represents the fixed point format 
 * radix that the device uses
 */
typedef struct t_sis8300llrfdrv_Qmn {
    unsigned int_bits_m;    /** < Number of int bits (sign bit inclusive) */
    unsigned frac_bits_n;   /** < Number of frac bits */
    unsigned is_signed;     /** < Number is signed or unsigned */
} sis8300llrfdrv_Qmn;

/* Type definitions */
extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_near_iq;
extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_IQ_sample;

//extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_sp_mag;
//extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_ff_mag;
extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Qmn_angle;
extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Position_threshold;
extern const sis8300llrfdrv_Qmn sis8300llrfdrv_Magnitude_threshold;

/* uint limits */
#define SIS8300LLRFDRV_CHECK_UINT_LIMITS(uint_val,int_bits)\
        (uint_val >> int_bits) ? status_argument_range : status_success /**< Check limits of unsigned int with int_bits representation */

#define SIS8300LLRFDRV_CHECK_ANGLE(angle) \
        ( angle < -M_PI || angle > M_PI ) ? status_argument_range : status_success /** <Check if angle val is in [-pi,pi] */


int sis8300llrfdrv_double_2_Qmn(double val, sis8300llrfdrv_Qmn Qmn, uint32_t *converted, double *err);
void sis8300llrfdrv_Qmn_2_double(uint32_t val, sis8300llrfdrv_Qmn Qmn, double *converted);

void sis8300llrfdrv_raw_sample_2_IQ(unsigned raw_sample, double *I, double *Q);
int sis8300llrfdrv_IQ_2_raw_sample(double I, double Q, unsigned *raw_sample, double *I_err, double *Q_err);

#ifdef __cplusplus
}
#endif

#endif /* SIS8300LLRFDRV_TYPES_H_ */
#endif
