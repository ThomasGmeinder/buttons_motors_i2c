

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

    // reset flag
    ms->update_flash = 0;

}

#define FLASH_TEST_SIZE 2*MOTOR_FLASH_AREA_SIZE

void test_flash() {
    unsigned char *page_buffer;
    unsigned offset = 0;

    printf("!!!!!! Testing flash\n");

    // read
    int error = 0;
    unsigned char byte_buffer[FLASH_TEST_SIZE];
    if(fl_readData(offset, FLASH_TEST_SIZE, byte_buffer) != 0) {
      printf("fl_readData Error\n");
      error = 2;
    }

    if(!error) {
      printf("fl_readData Read %d bytes from flash at offset %d:\n", FLASH_TEST_SIZE, offset);
      for(unsigned i=0; i<FLASH_TEST_SIZE; ++i) {
        printf("0x%x %d\n", byte_buffer[i], byte_buffer[i]);
      }
    }

    // write
    page_buffer = malloc(fl_getWriteScratchSize(offset, FLASH_TEST_SIZE));
    unsigned char data[FLASH_TEST_SIZE];
    for(unsigned i=0; i<FLASH_TEST_SIZE; ++i) {
      data[i] = i;
      printf("byte %d, 0x%x\n", i, data[i]);
    }
    fl_writeData(offset,
              FLASH_TEST_SIZE,
              data,
              page_buffer);

    free(page_buffer);

    // read
    error = 0;
    if(fl_readData(offset, FLASH_TEST_SIZE, byte_buffer) != 0) {
      printf("fl_readData Error\n");
      error = 2;
    }

    if(!error) {
      printf("fl_readData Read %d bytes from flash at offset %d:\n", FLASH_TEST_SIZE, offset);
      for(unsigned i=0; i<FLASH_TEST_SIZE; ++i) {
        printf("0x%x %d\n", byte_buffer[i], byte_buffer[i]);
      }
    }
}


void handle_flash_write() {
	if(state_m0.update_flash) {
     write_motor_position_to_flash(&state_m0);
  }	
  if(state_m1.update_flash) {
     write_motor_position_to_flash(&state_m1);
  }
}
