

#include <quadflashlib.h>
#include <quadflash.h>
#include <common.h>
#include <stdlib.h> // for malloc
#include <stdio.h> 
#include <string.h> 
// shared memory
extern motor_state_s state_m0, state_m1;

void write_motor_position_to_flash(motor_state_s* ms) {   

    unsigned char *page_buffer;
    page_buffer = malloc(fl_getWriteScratchSize(ms->motor_idx*MOTOR_FLASH_AREA_SIZE, MOTOR_FLASH_AREA_SIZE));
    unsigned char data[MOTOR_FLASH_AREA_SIZE];
    data[0] = FLASH_DATA_VALID_BYTE;
    memcpy(&data[1], &(ms->position), sizeof(int));

    printf("!!!!!! Writing new motor position %d um to flash for Motor %d\n", ms->position, ms->motor_idx);
    fl_writeData(ms->motor_idx*MOTOR_FLASH_AREA_SIZE,
              MOTOR_FLASH_AREA_SIZE,
              data,
              page_buffer);

    free(page_buffer);

    // set flag
    ms->update_flash = 0;

}


void handle_flash_write() {
	if(state_m0.update_flash) {
       write_motor_position_to_flash(&state_m0);
    }	
    if(state_m1.update_flash) {
       write_motor_position_to_flash(&state_m1);
    }
}
