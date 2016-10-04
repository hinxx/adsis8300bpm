#if 0
#include <stdio.h>
#include <stdint.h>

#include <sis8300drv.h>
#include <sis8300drv_utils.h>
#include <sis8300_reg.h>

#include "sis8300drvbpm.h"
#include "sis8300bpm_reg.h"
#include "sis8300drvbpm_types.h"

#if 0
/* For setting up the memory map */
typedef enum {
    mmap_sp    = 0,
    mmap_ff    = 1,
    mmap_pierr = 2
} sis8300llrf_custom_mmap;

static const char* custom_mmap_name[] = 
                        {"SP table", "FF table", "PI error"};

int _sis8300llrfdrv_get_max_size_bytes(sis8300drv_usr *sisuser, 
    sis8300llrf_custom_mmap mmap, uint32_t *byte_size);
int _sis8300llrfdrv_set_base_offset(sis8300drv_usr *sisuser, 
    sis8300llrf_custom_mmap mmap, uint32_t offset);
#endif

#define SIS8300LLRF_PINIT(fmt, args...) \
            printf("sis8300llrfdrv board init: " fmt, ## args)

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
int sis8300llrfdrv_setup_dac(sis8300drv_usr *sisuser) {

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
    
    SIS8300LLRF_PINIT("DAC setup done, reg = 0x%x, val = 0x%x\n", 
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
 *
 * 0x106 = 100000110 for MA version
 * 0x103 = 100000011 for IQ version
 *
 * bit 8: ADC 1/2 Select Bit
 *
 * bit 7-0: tap delay val (x78 ps)
 */
int sis8300llrfdrv_setup_adc_tap_delay(sis8300drv_usr *sisuser) {
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
    
    SIS8300LLRF_PINIT("ADC tap delay setup done, reg = 0x%x, "
            "val = 0x%x\n", SIS8300_ADC_INPUT_TAP_DELAY_REG, ui32_reg_val);
    
    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}

/**
 * @brief Setup memory map for custom logic
 *
 * @param [in] sisuser    User context Struct
 * @param [in] num_pulse_types Number of pulse types
 *
 * @return status_success       Memory map successful
 * @return status_no_device     Device not opened
 * @return status_device_access Cannot access device registers
 *
 * The function sets up memory map for custom logic and set the 
 * available ADC mem in the device struct.
 *
 * IMPORTAINT: The function needs to be called before any generic data 
 * acquisition (for AI) is done.
 *
 * Memory map can in principle be set by calling a set of other 
 * functions, but since control table reserved space and pi error 
 * reserved space is not something that will be changed during normal 
 * operation but only once at init, this is a user-friendly way to set 
 * it up.
 */
int sis8300llrfdrv_mem_ctrl_set_custom_mem_map(sis8300drv_usr *sisuser, 
        unsigned num_pulse_types) {

//HK LLRF specific
//    int status;
//    uint32_t sp_size, ff_size, pi_err_size, available_mem,
//             total_custom_size; /* all these are in bytes */
    uint32_t available_mem; /* in bytes */
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
    printf("============================================\n");
    SIS8300LLRF_PINIT("Starting Custom memory map\n");

    available_mem = (uint32_t) sisdevice->mem_size;
    SIS8300LLRF_PINIT("Total memory available on device is "
            "%u bytes\n", available_mem);

//HK LLRF specific
//    /* Get maximum sizes in bytes */
//    status = _sis8300llrfdrv_get_max_size_bytes(sisuser,
//                mmap_pierr, &pi_err_size);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }
//
//    status = _sis8300llrfdrv_get_max_size_bytes(sisuser,
//                mmap_ff, &ff_size);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }
//
//    status = _sis8300llrfdrv_get_max_size_bytes(sisuser,
//                mmap_sp, &sp_size);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }
//
//    /* Callculate how much mem is required and check if it fits in
//     * memory */
//    total_custom_size =
//        (sp_size + ff_size) * num_pulse_types + pi_err_size;
//    SIS8300LLRF_PINIT("Total size required by custom mem map is "
//            "%u bytes\n", total_custom_size);
//
//    if (total_custom_size > available_mem) {
//        SIS8300LLRF_PINIT("Total custom size is bigger "
//                "than available memory\n");
//
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status_argument_range;
//    }
//
//    /* Now set offsets, backwards
//     * no need to check if it's aligned to SIS8300DRV_BLOCK_BYTES
//     * because, max_nsamples will always be aligned to this (control
//     * table size in the register is in units of #SIS8300DRV_BLOCK_BYTES.
//     */
//    available_mem -= pi_err_size;
//    status = _sis8300llrfdrv_set_base_offset(sisuser,
//                mmap_pierr, available_mem);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }
//
//    available_mem -= ff_size * (uint32_t) num_pulse_types;
//    status = _sis8300llrfdrv_set_base_offset(sisuser,
//                mmap_ff, available_mem);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }
//
//    available_mem -= sp_size * (uint32_t) num_pulse_types;
//    status = _sis8300llrfdrv_set_base_offset(sisuser,
//                mmap_sp, available_mem);
//    if (status) {
//        pthread_mutex_unlock(&sisdevice->lock);
//        return status;
//    }

    SIS8300LLRF_PINIT("Not using memory for LLRF..\n");

    /* set available ADC mem size on the device */
    sisdevice->adc_mem_size = (unsigned) available_mem;

    SIS8300LLRF_PINIT("Memory left for ADC data is %#010x\n", 
            available_mem);
    SIS8300LLRF_PINIT("Custom mem map DONE!.\n");
    printf("============================================\n");

    pthread_mutex_unlock(&sisdevice->lock);
    return status_success;
}



/* ==================== INTERNAL LIBRARY FUNCTIONS ================== */
/**
 * @brief
 *
 * this function expects that the calling function holds the device lock
 */
int _sis8300llrfdrv_get_max_size_bytes(
        sis8300drv_usr *sisuser, 
        sis8300llrf_custom_mmap mmap, uint32_t *byte_size) {
    
    unsigned nsamples;
    int status; 
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }
    
    switch (mmap) {
        case mmap_sp:
            status = sis8300llrfdrv_get_ctrl_table_max_nelm(sisuser, 
                ctrl_table_sp, &nsamples);
            break;
        case mmap_ff:
            status = sis8300llrfdrv_get_ctrl_table_max_nelm(sisuser, 
                ctrl_table_ff, &nsamples);
            break;
        case mmap_pierr:
            status = sis8300llrfdrv_get_pi_err_max_nsamples(sisuser, 
                &nsamples);
            break;
        default:
            return status_argument_invalid;
    }
    if (status) {
        return status;
    }
    
    *byte_size = (uint32_t) nsamples * SIS8300LLRF_IQ_SAMPLE_BYTES;
    SIS8300LLRF_PINIT("Max. allowed samples in %s is %#010x, "
            "taking up %u bytes\n", 
            custom_mmap_name[mmap], nsamples, *byte_size);
    
    return status_success;
}

/**
 * @brief
 *
 * this function expects that the calling function holds the device lock
 */
int _sis8300llrfdrv_set_base_offset(
        sis8300drv_usr *sisuser, 
        sis8300llrf_custom_mmap mmap, uint32_t offset) {
            
    int      status;
    uint32_t readback;
    uint32_t addr[] = {SIS8300LLRF_MEM_CTRL_SP_BASE_REG,
                       SIS8300LLRF_MEM_CTRL_FF_BASE_REG,
                       SIS8300LLRF_MEM_CTRL_PI_ERR_BASE_REG};
    sis8300drv_dev *sisdevice;

    sisdevice = sisuser->device;
    if (!sisdevice) {
        return status_no_device;
    }

    SIS8300LLRF_PINIT("Setting offset for %s to: %#010x\n", 
            custom_mmap_name[mmap], offset);
            
    /* write the new value */
    status = sis8300_reg_write(sisdevice->handle, addr[mmap], offset);
    if (status) {
        return status_device_access;
    }
    
    /* readback */
    status = sis8300_reg_read(sisdevice->handle, addr[mmap], &readback);
    if (status) {
        return status_device_access;
    }
    
    /* inform the user on alignment and readback check */
    SIS8300LLRF_PINIT("%s offset: readback %s, 32 byte "
            "alignment check %s\n", 
            custom_mmap_name[mmap], 
            (readback != offset) ? "FAIL" : "OK", 
            (offset % SIS8300DRV_BLOCK_BYTES) ? "FAIL" : "OK");
            
    return status_success;
}
#endif

