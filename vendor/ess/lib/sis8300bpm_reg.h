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

#ifndef SIS8300LLRFDRVREG_H_
#define SIS8300LLRFDRVREG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* the following tags apply
 * _REG     Read + write reg
 * _R_REG   Read only reg
 * _W_REG   Write only reg
 * _C_REG   Clear reg
 * _S_REG   Set REg
 */

/* BASE
 * #define SIS8300LLRF_BASE 0x400
 */
#if 0
/* ID */
#define SIS8300LLRF_ID_R_REG                         0x400
#define SIS8300LLRF_INST_ID_REG                      0x401

/* GOP */
#define SIS8300LLRF_GOP_REG                          0x402
#define SIS8300LLRF_GEN_STATUS_REG                       SIS8300LLRF_GOP_REG
#define SIS8300LLRF_GOP_MASK                             0x1f8 //last two bits represent FSM state. Only debug, IGNORE!!!
#define SIS8300LLRF_GEN_STATUS_MASK                      SIS8300LLRF_GOP_MASK
#define SIS8300LLRF_GOP_PULSE_DONE_CNT_MASK              0xffff0000
#define SIS8300LLRF_GOP_PULSE_DONE_CNT_SHIFT             16

/* GIP */
#define SIS8300LLRF_GIP_REG                          0x403
//#define SIS8300LLRF_GIP_S_REG                        0x404
//#define SIS8300LLRF_GIP_C_REG                        0x405

#define SIS8300LLRF_GIP_PT_MASK                          0xffff0000
#define SIS8300LLRF_GIP_PT_SHIFT                         16
//#define SIS8300LLRF_GIP_UPDATE_REASON_MASK               0x1e
#define SIS8300LLRF_GIP_UPDATE_REASON_MASK               0x2

#define SIS8300LLRF_GIP_SW_RESET_BIT                     0x400
/* #define SIS8300LLRF_GIP_INIT_DONE_BIT                    0x001 
 */

/* PI control */
#define SIS8300LLRF_PI_1_K_REG                       0x406
#define SIS8300LLRF_PI_1_TS_DIV_TI_REG               0x407
#define SIS8300LLRF_PI_1_SAT_MAX_REG                 0x408
#define SIS8300LLRF_PI_1_SAT_MIN_REG                 0x409
#define SIS8300LLRF_PI_1_CTRL_REG                    0x40a
#define SIS8300LLRF_PI_1_FIXED_SP_REG                0x40b
#define SIS8300LLRF_PI_1_FIXED_FF_REG                0x40c

#define SIS8300LLRF_FF_TABLE_SPEED_REG                   SIS8300LLRF_PI_1_CTRL_REG
#define SIS8300LLRF_FF_TABLE_SPEED_MASK                  0x3c0
#define SIS8300LLRF_FF_TABLE_SPEED_SHIFT                 6

#define SIS8300LLRF_PI_2_K_REG                       0x40d
#define SIS8300LLRF_PI_2_TS_DIV_TI_REG               0x40e
#define SIS8300LLRF_PI_2_SAT_MAX_REG                 0x40f
#define SIS8300LLRF_PI_2_SAT_MIN_REG                 0x410
#define SIS8300LLRF_PI_2_CTRL_REG                    0x411
#define SIS8300LLRF_PI_2_FIXED_SP_REG                0x412
#define SIS8300LLRF_PI_2_FIXED_FF_REG                0x413

/* IQ control */
#define SIS8300LLRF_IQ_CTRL_REG                      0x414
#define SIS8300LLRF_IQ_ANGLE_REG                     0x415
#define SIS8300LLRF_IQ_DC_OFFSET_REG                 0x416

/* VM control */
#define SIS8300LLRF_VM_CTRL_REG                      0x417
#define SIS8300LLRF_VM_MAG_LIMIT_REG                 0x418

/* SAMPLE SIZE REGISTERS */
#define SIS8300LLRF_CNT_CAVITY_SAMPLES_R_REG         0x419
#define SIS8300LLRF_CNT_PI_ERR_PULSE_START_R_REG     0x41a
#define SIS8300LLRF_CNT_PI_ERR_PULSE_ACTIVE_R_REG    0x41b

/* LOOKUP TABLES */
#define SIS8300LLRF_LUT_CTRL_1_PARAM_REG             0x41c
#define SIS8300LLRF_LUT_CTRL_FF_NSAMPLES_REG             SIS8300LLRF_LUT_CTRL_1_PARAM_REG

#define SIS8300LLRF_LUT_CTRL_2_PARAM_REG             0x41d
#define SIS8300LLRF_LUT_CTRL_SP_NSAMPLES_REG             SIS8300LLRF_LUT_CTRL_2_PARAM_REG

#define SIS8300LLRF_LUT_CTRL_NSAMPLES_MASK              0x0007ffff

/* MEM CTRL PARAMS */
#define SIS8300LLRF_MEM_CTRL_1_PARAM_REG             0x41e
#define SIS8300LLRF_MEM_CTRL_FF_BASE_REG                 SIS8300LLRF_MEM_CTRL_1_PARAM_REG

#define SIS8300LLRF_MEM_CTRL_2_PARAM_REG             0x41f
#define SIS8300LLRF_MEM_CTRL_SP_BASE_REG                 SIS8300LLRF_MEM_CTRL_2_PARAM_REG

#define SIS8300LLRF_MEM_CTRL_3_PARAM_REG             0x420
#define SIS8300LLRF_MEM_CTRL_CTRL_TABLE_SIZE_REG         SIS8300LLRF_MEM_CTRL_3_PARAM_REG

#define SIS8300LLRF_MEM_CTRL_4_PARAM_REG             0x421
#define SIS8300LLRF_MEM_CTRL_PI_ERR_BASE_REG             SIS8300LLRF_MEM_CTRL_4_PARAM_REG

/* PI ERR STATUS */
#define SIS8300LLRF_MEM_SIZE_PI_ERR_R_REG            0x422
#define SIS8300LLRF_CNT_PI_ERR_TOTAL_R_REG           0x423

/* TRIGGERS */
//#define SIS8300LLRF_BOARD_SETUP_REG                  0x424

/* NEAR IQ */
//#define SIS8300LLRF_NEAR_IQ_1_PARAM_REG              0x425
//#define SIS8300LLRF_NEAR_IQ_2_PARAM_REG              0x426
//#define SIS8300LLRF_NEAR_IQ_DATA_REG                 0x427
//#define SIS8300LLRF_NEAR_IQ_ADDR_REG                 0x428

/* MISC */
#define SIS8300LLRF_16_FRAC_MASK                         0xffff0000
#define SIS8300LLRF_16_INT_MASK                          0x0000ffff

/* MODULATOR RIPPLE FILTER */
#define SIS8300LLRF_FILTER_S_REG                     0x429
#define SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTS_REG         SIS8300LLRF_FILTER_S_REG
#define SIS8300LLRF_FILTER_C_REG                     0x42a
#define SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTC_REG         SIS8300LLRF_FILTER_C_REG
#define SIS8300LLRF_FILTER_A_CTRL_REG                0x42b
#define SIS8300LLRF_MOD_RIPPLE_FILTER_CONSTA_REG         SIS8300LLRF_FILTER_A_CTRL_REG
#define SIS8300LLRF_MOD_RIPPLE_FILTER_CTRL_REG           SIS8300LLRF_FILTER_A_CTRL_REG

/* NOTCH FILTER */
#define SIS8300LLRF_NOTCH_FILTER_CONSTA_REG          0x43c
#define SIS8300LLRF_NOTCH_FILTER_CONSTB_REG          0x43d
#define SIS8300LLRF_NOTCH_FILTER_CTRL_REG            0x43e

#define SIS8300LLRF_MA_CAV_R_REG                     0x42c
//#define SIS8300LLRF_MA_REF_R_REG                     0x42d

#define SIS8300LLRF_MON_PARAM_1_REG                  0x42e
#define SIS8300LLRF_MON_PARAM_2_REG                  0x42f
#define SIS8300LLRF_MON_LIMIT_1_REG                  0x430
#define SIS8300LLRF_MON_LIMIT_2_REG                  0x431
#define SIS8300LLRF_MON_LIMIT_3_REG                  0x432
#define SIS8300LLRF_MON_LIMIT_4_REG                  0x433
#define SIS8300LLRF_MON_STATUS_REG                   0x434
#define SIS8300LLRF_MON_STATUS_MAG_1_REG			 0x435
#define SIS8300LLRF_MON_STATUS_MAG_2_REG			 0x436
#define SIS8300LLRF_MON_STATUS_MAG_3_REG			 0x437
#define SIS8300LLRF_MON_STATUS_MAG_4_REG			 0x438

#define SIS8300LLRF_VM_PREDIST_R0_REG                0x439
#define SIS8300LLRF_VM_PREDIST_R1_REG                0x43a
#define SIS8300LLRF_VM_PREDIST_DC_REG                0x43b
#endif

//HK for BPM
#define SIS8300LLRF_ID_R_REG                         0x400
#define SIS8300LLRF_INST_ID_REG                      0x401
#define SIS8300LLRF_GOP_REG                          0x402
#define SIS8300LLRF_GEN_STATUS_REG                   SIS8300LLRF_GOP_REG
#define SIS8300LLRF_GOP_MASK                         0xff8 //last three bits represent FSM state. Only debug, IGNORE!!!
#define SIS8300LLRF_GEN_STATUS_MASK                  SIS8300LLRF_GOP_MASK
#define SIS8300LLRF_GOP_PULSE_DONE_CNT_MASK          0xffff0000
#define SIS8300LLRF_GOP_PULSE_DONE_CNT_SHIFT         16
#define SIS8300LLRF_GIP_REG                          0x403
//#define SIS8300LLRF_GIP_PT_MASK                      0xffff0000
//#define SIS8300LLRF_GIP_PT_SHIFT                     16
//#define SIS8300LLRF_GIP_UPDATE_REASON_MASK               0x1e
#define SIS8300LLRF_GIP_UPDATE_REASON_MASK           0x003
//#define SIS8300LLRF_GIP_INIT_DONE_BIT                0x000
//#define SIS8300LLRF_GIP_UPDATE_PARAMS_BIT            0x001
#define SIS8300LLRF_GIP_INIT_DONE_BIT                0x001
#define SIS8300LLRF_GIP_NEW_PARAMS_BIT               0x002
#define SIS8300LLRF_GIP_SW_RESET_BIT                 0x040
#define SIS8300LLRF_GIP_CLEAR_PULSE_DONE_BIT         0x080

#define SIS8300LLRF_SAMPLE_CNT_R_REG                 0x404
/* XXX: Is this used? */
#define SIS8300LLRF_IQ_SAMPLE_CNT_REG                0x405
#define SIS8300LLRF_BOARD_SETUP_REG                  0x406
#define SIS8300LLRF_NEAR_IQ_1_PARAM_REG              0x407
#define SIS8300LLRF_NEAR_IQ_2_PARAM_REG              0x408
#define SIS8300LLRF_NEAR_IQ_DATA_REG                 0x409
#define SIS8300LLRF_NEAR_IQ_ADDR_REG                 0x40A
#define SIS8300LLRF_MA_REF_R_REG                     0x40B
#define SIS8300LLRF_MA_SUM1_R_REG                    0x40C
#define SIS8300LLRF_MA_SUM2_R_REG                    0x40D
#define SIS8300LLRF_POS_XY1_R_REG                    0x40E
#define SIS8300LLRF_POS_XY2_R_REG                    0x40F
#define SIS8300LLRF_POS_PARAM_X1_REG                 0x410
#define SIS8300LLRF_POS_PARAM_Y1_REG                 0x411
#define SIS8300LLRF_POS_MAG_CTRL1_REG                0x412
#define SIS8300LLRF_POS_PARAM_X2_REG                 0x413
#define SIS8300LLRF_POS_PARAM_Y2_REG                 0x414
#define SIS8300LLRF_POS_MAG_CTRL2_REG                0x415
/* Not used */
#define SIS8300LLRF_DSP_PARAM_REG                    0x416
#define SIS8300LLRF_BPM_FILTER_DATA_REG              0x417
#define SIS8300LLRF_BPM_FILTER_CTRL_REG              0x418

#ifdef __cplusplus
}
#endif

#endif /* SIS8300LLRFDRVREG_H_ */
