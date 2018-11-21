
#include <platform.h>
#include <quadflashlib.h>
#include <quadflash.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <common.h>

// List of QuadSPI devices that are supported by default.
fl_QuadDeviceSpec deviceSpecs[] =
 {
   FL_QUADDEVICE_SPANSION_S25FL116K,
   FL_QUADDEVICE_SPANSION_S25FL132K,
   FL_QUADDEVICE_SPANSION_S25FL164K,
   FL_QUADDEVICE_ISSI_IS25LQ080B,
   FL_QUADDEVICE_ISSI_IS25LQ016B,
   FL_QUADDEVICE_ISSI_IS25LQ032B,
};

// Ports for QuadSPI access on explorerKIT.
fl_QSPIPorts ports = {
   PORT_SQI_CS,
   PORT_SQI_SCLK,
   PORT_SQI_SIO,
   on tile[0]: XS1_CLKBLK_1
};

extern void handle_flash_write();

void flash_server(chanend flash_c) {
    char error = 0;
    // Connect to the QuadSPI device using the quadflash library function fl_connectToDevice. 
    if(fl_connectToDevice(ports, deviceSpecs, sizeof(deviceSpecs)/sizeof(fl_QuadDeviceSpec)) != 0) {
      printf("fl_connectToDevice Error\n");
      error = 1;
    }

    while(1) {
      char flash_command;
    	select {
    		case flash_c :> flash_command:
    		   // Read Motor positions from flash
               char byte_buffer[FLASH_DATA_BYTES];
               // Connect to the QuadSPI device using the quadflash library function fl_connectToDevice. 
               if(fl_readData(0, FLASH_DATA_BYTES, byte_buffer) != 0) {
                 printf("fl_readData Error\n");
                 error = 2;
               }
               flash_c <: error;

               if(!error) {
                 printf("fl_readData Read %d bytes from flash:\n", FLASH_DATA_BYTES);
                 for(unsigned i=0; i<FLASH_DATA_BYTES; ++i) {
                   printf("0x%x %d\n", byte_buffer[i], byte_buffer[i]);
                   flash_c <: byte_buffer[i];
                 }
               }
               error = 0; // reset error
               break;
            default:
               handle_flash_write();
               break;
           
    	}
    }
 }