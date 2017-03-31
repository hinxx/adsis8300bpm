/**
 * BPM based on Struck 8300 Linux userspace filter coefficient update tool.
 * Copyright (C) 2017 EuropeanSpallation Source
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
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <sis8300drv.h>
#include <sis8300bpm_reg.h>
#include <sis8300drvbpm.h>

int main(int argc, char **argv) {
    char c;
    int status, verbose, enable;
    sis8300drv_usr *sisuser;
    double coeff[SIS8300BPM_FIR_FILTER_PARAM_NUM];
    double gain;

    verbose = 0;
    enable = 0;
    coeff[0] = 0.007209976640;
    coeff[1] = -0.001953094770;
    coeff[2] = -0.053060037290;
    coeff[3] = 0.005647590070;
    coeff[4] = 0.295731148150;
    coeff[5] = 0.492273184390;
    
    while ((c = getopt(argc, argv, "hv0:1:2:3:4:5:e")) != -1) {
        switch (c) {
			case '0':
				coeff[0] = strtod(optarg, NULL);
				break;
            case '1':
                coeff[1] = strtod(optarg, NULL);
                break;
            case '2':
                coeff[2] = strtod(optarg, NULL);
                break;
            case '3':
                coeff[3] = strtod(optarg, NULL);
                break;
            case '4':
                coeff[4] = strtod(optarg, NULL);
                break;
            case '5':
                coeff[5] = strtod(optarg, NULL);
                break;
            case 'v':
                verbose = 1;
                break;
            case 'e':
                enable = 1;
                break;
            case ':':
                printf("Option -%c requires an operand.\n\n", optopt);
                break;
            case '?':
            case 'h':
            default:
                printf("Usage: %s device [-h] [-v] [-e] [-0 coeff] [-1 coeff] [-2 coeff] [-3 coeff] [-4 coeff] [-5 coeff]\n", argv[0]);
                printf("   \n");
                printf("       -e                   Enable filter\n");
                printf("       -0 coefficient       0th coefficient \n");
                printf("       -1 coefficient       1st coefficient \n");
                printf("       -2 coefficient       2nd coefficient \n");
                printf("       -3 coefficient       3rd coefficient \n");
                printf("       -4 coefficient       4th coefficient \n");
                printf("       -5 coefficient       5th coefficient \n");
                printf("   \n");                
                printf("       -v                   Verbose output \n");
                printf("       -h                   Print this message \n");
                printf("   \n");
                return -1;
        }
    }
    
    if (optind != argc - 1) {
        printf ("Device argument required.\n");
        return -1;
    }
    
	gain = 2 * coeff[0] + \
			2 * coeff[2] + \
			2 * coeff[3] + \
			2 * coeff[4] + \
			2 * coeff[5] + \
			coeff[1];

    if (verbose) {
        printf("\nSetting filter coefficients on %s with:\n\n", argv[optind]);
        printf("Coefficient 0:       %.12f\n", coeff[0]);
        printf("Coefficient 1:       %.12f\n", coeff[1]);
        printf("Coefficient 2:       %.12f\n", coeff[2]);
        printf("Coefficient 3:       %.12f\n", coeff[3]);
        printf("Coefficient 4:       %.12f\n", coeff[4]);
        printf("Coefficient 5:       %.12f\n", coeff[5]);
        printf("Filter gain:         %.12f\n", gain);
        printf("Filter enable:       %d\n", enable);
    }
    
    sisuser = malloc(sizeof(sis8300drv_usr));
    sisuser->file = argv[optind];

    status = sis8300drv_open_device(sisuser);
    if (status) {
        printf("sis8300drv_open_device error: %s (%d)\n", 
                sis8300drv_strerror(status), status);
        return -1;
    }
    
    if (verbose) {
        printf("\n");
    }

    status = sis8300drvbpm_set_fir_filter_param(sisuser, coeff, SIS8300BPM_FIR_FILTER_PARAM_NUM);
    if (status) {
        printf("sis8300drvbpm_set_fir_filter_param error: %s (%d)\n",
                sis8300drv_strerror(status), status);
        return -1;
    }
    
   	status = sis8300drvbpm_set_fir_filter_enable(sisuser, enable);
    if (status) {
        printf("sis8300drvbpm_set_fir_filter_enable error: %s (%d)\n",
                sis8300drv_strerror(status), status);
        return -1;
    }

	printf("filter coefficients set\n");

	sis8300drv_close_device(sisuser);

    return 0;
}
