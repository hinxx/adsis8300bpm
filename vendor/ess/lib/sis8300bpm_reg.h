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
 * @file sis8300bpm_reg.h
 * @brief Header of sis8300 BPM registers.
 * @author Hinko Kocevar
 */

#ifndef SIS8300DRVBPMREG_H_
#define SIS8300DRVBPMREG_H_

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

#define SIS8300BPM_ID_R_REG                         0x400
#define SIS8300BPM_INST_ID_REG                      0x401
#define SIS8300BPM_GOP_REG                          0x402
/* Last three bits represent FSM state. Only debug, IGNORE!!! */
# define SIS8300BPM_GOP_MASK                        0xFF8
# define SIS8300BPM_GOP_PULSE_DONE_CNT_MASK         0xFFFF0000
# define SIS8300BPM_GOP_PULSE_DONE_CNT_SHIFT        16
#define SIS8300BPM_GIP_REG                          0x403
# define SIS8300BPM_GIP_UPDATE_REASON_MASK          0x003
# define SIS8300BPM_GIP_INIT_DONE_BIT               0x001
# define SIS8300BPM_GIP_NEW_PARAMS_BIT              0x002
# define SIS8300BPM_GIP_SW_RESET_BIT                0x040
# define SIS8300BPM_GIP_CLEAR_PULSE_DONE_BIT        0x080
#define SIS8300BPM_SAMPLE_CNT_R_REG                 0x404
#define SIS8300BPM_IQ_SAMPLE_CNT_REG                0x405
#define SIS8300BPM_BOARD_SETUP_REG                  0x406
#define SIS8300BPM_NEAR_IQ_1_PARAM_REG              0x407
#define SIS8300BPM_NEAR_IQ_2_PARAM_REG              0x408
#define SIS8300BPM_NEAR_IQ_DATA_REG                 0x409
#define SIS8300BPM_NEAR_IQ_ADDR_REG                 0x40A
#define SIS8300BPM_MA_REF_R_REG                     0x40B
#define SIS8300BPM_MA_SUM1_R_REG                    0x40C
#define SIS8300BPM_MA_SUM2_R_REG                    0x40D
#define SIS8300BPM_POS_XY1_R_REG                    0x40E
#define SIS8300BPM_POS_XY2_R_REG                    0x40F
#define SIS8300BPM_POS_PARAM_X1_REG                 0x410
#define SIS8300BPM_POS_PARAM_Y1_REG                 0x411
#define SIS8300BPM_POS_MAG_CTRL1_REG                0x412
#define SIS8300BPM_POS_PARAM_X2_REG                 0x413
#define SIS8300BPM_POS_PARAM_Y2_REG                 0x414
#define SIS8300BPM_POS_MAG_CTRL2_REG                0x415
/* DSP param register is unused */
#define SIS8300BPM_DSP_PARAM_REG                    0x416
#define SIS8300BPM_FILTER_DATA_REG                  0x417
#define SIS8300BPM_FILTER_CTRL_REG                  0x418

#ifdef __cplusplus
}
#endif

#endif /* SIS8300DRVBPMREG_H_ */
