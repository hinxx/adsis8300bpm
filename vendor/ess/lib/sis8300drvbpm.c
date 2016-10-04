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


/**
 * @brief Setup the DAC
 *
 * @param [in]    sisuser    User context struct
 *
 * @return status_success       Set successfull
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
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
 * @return status_success       Set successfull
 * @return status_device_access Can't access device registers.
 * @return status_device_armed  This operation is not allowed on an
 *                              armed device.
 * @return status_no_device     Device not opened.
 *
 * This will setup the Data Strobe Timing. The function should be called
 * once at the device initialization
 *
 * XXX: This LLRF specific!!!!
 * 0x106 = 100000110 for MA version
 * 0x103 = 100000011 for IQ version
 *
 * bit 8: ADC 1/2 Select Bit
 *
 * bit 7-0: tap delay val (x78 ps)
 */

/* XXX: Should probably be part of generic SIS8300 driver library and better
 *      parameterized. */
int sis8300drvbpm_setup_adc_tap_delay(sis8300drv_usr *sisuser) {
    int status;
    sis8300drv_dev *sisdevice;
    unsigned major, minor, device;
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

    status = sis8300llrfdrv_get_fw_version(sisuser,
                &device, &major, &minor);
    if (status) {
        pthread_mutex_unlock(&sisdevice->lock);
        return status;
    }

    switch (major) {
        case SIS8300LLRFDRV_FW_VERSION_MAJOR_MA:
            ui32_reg_val = 0x103;
            break;
        case SIS8300LLRFDRV_FW_VERSION_MAJOR_IQ:
            ui32_reg_val = 0x11F;
            break;
        default:
            pthread_mutex_unlock(&sisdevice->lock);
            return status_device_state;
    }

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

