

#include <quadflashlib.h>
#include <quadflash.h>
#include <common.h>
#include <stdlib.h> // for malloc
#include <stdio.h> 

// shared memory
extern motor_state_s state_m0, state_m1;

void write_motor_position_to_flash(motor_state_s* ms) {   

    unsigned char *page_buffer;
    page_buffer = malloc(fl_getWriteScratchSize(0, 2));
    unsigned char data[2] = {FLASH_DATA_VALID_BYTE, ms->position};

    printf("Writing new motor position %d to flash for Motor %d\n", ms->position, ms->motor_idx);
    fl_writeData(ms->motor_idx*2,
              2,
              data,
              page_buffer);

    free(page_buffer);

    // set flag
    ms->flash_updated = 1;

}


void handle_flash_write() {
	if(!state_m0.flash_updated) {
       write_motor_position_to_flash(&state_m0);
    }	
    if(!state_m1.flash_updated) {
       write_motor_position_to_flash(&state_m1);
    }
}
