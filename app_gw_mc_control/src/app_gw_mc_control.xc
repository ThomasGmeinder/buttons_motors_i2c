/*
 * app_gw_mc_control.xc
 *
 *  Created on: 12 Mar 2017
 *      Author: thomas
 */

// Control Specification
// Inputs and actions
// - User presses open button whilst motors stopped. A: start_motor OPENING with target position OPEN_POS_ES
// - User presses open button again whilst motors run. A: stop_motor
// - User presses close button once: A: start_motor CLOSING with target position CLOSED_POS_ES
// - User presses close button again whilst motors run. A: stop_motor
// - Web server sends close command with a new target position. A: start_motor CLOSING with target position X
// - Web server sends open command with a new target position. A: start_motor OPENING with target position X
// Note: When a I2C command comes in from the server whilst the motor is running, then ignore it

// User Guide:
// Flash with 8k Datapartition: xflash bin/app_gw_mc_control.xe --boot-partition-size 516096

// Todo:
// Change relais control signals to active low. Relais inputs are pulled high on he releais so default (high) should mean off
// Review duplicated state in motor_state_s and I2C registers.
// Write a spec for updates of the states based on the actions above
// Currently the control logic on sever and client is based on target_mp, server_target_mp, current_mp. Is this sufficient state?
// php logic:
// read current_mp
// read server_target_mp
// if(current_mp != server_target_mp) {
//   // Button change takes priority
//   target_mp = server_target_mp;
// } 
// else if(current_mp != target_mp) {
//   // Client changed state. 
//   I2C_write(target_position_reg, target_mp);
// }
// return target_mp and current_mp to client

#include <platform.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <i2c.h>
#include "i2c_app.h"
#include <quadflashlib.h>
#include <quadflash.h>
#include "debug_print.h"
#include "otp_board_info.h"

#define ENDSWITCHES_CONNECTED 0

#define MOTOR_SPEED 100 // in mm/s
#define OPEN_POS_MIN 0 // min position in cm
#define CLOSED_POS_MAX 120  // max 

#define OPEN_TOLERANCE 2
#define OPEN_POS_ES OPEN_POS_MIN+OPEN_TOLERANCE  // Open position at Endswitch

#define CLOSED_TOLERANCE 2 
#define CLOSED_POS_ES CLOSED_POS_MAX-CLOSED_TOLERANCE

#define DEBOUNCE_TIME XS1_TIMER_HZ/10  // 100ms
#define POS_UPDATE_PERIOD XS1_TIMER_HZ/10 // 100ms
#define DEBOUNCE 1

#define ES_TRIGGERED 1 // End Switch triggered. Connects between brown and blue from Pin to VCC
#define BUTTON_PRESSED 1 // Button pressed. 

// Position index of buttons on the 4-bit port
#define CLOSE_BUTTON_IDX 0
#define OPEN_BUTTON_IDX 1

// Endswith bit positions
#define ENDSWITCH_OPEN 0b01
#define ENDSWITCH_CLOSED 0b10
#define NO_ENDSWITCH 0

#define MOTOR_OPEN_BUTTON_GREEN_LED_IDX 0
#define MOTOR_OPEN_BUTTON_RED_LED_IDX 1
#define MOTOR_CLOSE_BUTTON_BLUE_LED_IDX 2
#define MOTOR_CLOSE_BUTTON_RED_LED_IDX 3

#define MOTOR_CLOSING_DIR 1
#define MOTOR_OPENING_DIR (!MOTOR_CLOSING_DIR)

#define MOTOR_OFF 1  // Relais inputs are pulled high so default (high) should mean off
#define MOTOR_ON 0

#define PUSHBUTTON_LED_ON 1

#define MC_TILE tile[0]

#define FLASH_DATA_VALID_BYTE 0x3C // arbitrary value
#define FLASH_DATA_BYTES 4
// Login raspiviv
// raspiviv
// nX9NypBk


/** Inputs **/
// Endswitches for both Motors
on MC_TILE : in port p_endswitches = XS1_PORT_4C;   // X0D14 (pin 0), X0D15, X0D20, X0D21 (pin 3)
//on MC_TILE : in port p_endswitches = XS1_PORT_4E;  

// Todo: Move this to a 4-bit port. There are no more 1-bit ports 
// Button to open and close the ventilations
on MC_TILE : in port p_control_buttons = XS1_PORT_4D;  
//on MC_TILE : in port p_control_buttons = XS1_PORT_4E;  

/** Outputs **/
// Motor 1 on/off
on MC_TILE : out port p_m0_on = XS1_PORT_1F;  // X0D13
// Motor 1 direction
on MC_TILE : out port p_m0_dir = XS1_PORT_1E;  // X0D12

// Motor 2 on/off
on MC_TILE : out port p_m1_on = XS1_PORT_1P;  // X0D39
// Motor 2 direction
on MC_TILE : out port p_m1_dir = XS1_PORT_1O;  // X0D38

on MC_TILE : port p_slave_sda = XS1_PORT_1M; // X0D36 // connect to GPIO 3 on rPI
on MC_TILE : port p_slave_scl = XS1_PORT_1N; // X0D37 // connect to GPIO 2 on rPI

on MC_TILE : port p_led = XS1_PORT_4F;

on MC_TILE : port p_m0_pushbutton_leds = XS1_PORT_4A; // X0D02, X0D03, X0D08, X0D09
on MC_TILE : port p_m1_pushbutton_leds = XS1_PORT_4E;

on MC_TILE: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;

// Ports for QuadSPI access on explorerKIT.
fl_QSPIPorts ports = {
   PORT_SQI_CS,
   PORT_SQI_SCLK,
   PORT_SQI_SIO,
   on tile[0]: XS1_CLKBLK_1
};

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

typedef enum {
    OPENING,
    CLOSING,
    STOPPED,
    STATE_UNKNOWN,
    ERROR
} motor_state_t;

typedef enum {
    I2C = 0,
    BUTTON = 1,
} actuator_t;

// struct for motor state
typedef struct {
    int position; // in mm from 0 (open) to MAX_POS (closed)
    int target_position; // the target position
    unsigned motor_idx;
    actuator_t actuator;

    motor_state_t state;
    motor_error_t error;
    motor_error_t prev_error;
} motor_state_s;

void delay_us(unsigned time_us) {
  int t;
  timer tmr;
  tmr :> t;
  tmr when timerafter(t+time_us*100) :> t;
}

// protos
void stop_motor(motor_state_s* ms, client register_if reg);
void check_motor_state_after_endswitch_triggered(motor_state_s* ms, motor_state_t state, int actual_position);
int start_motor(motor_state_s* ms, client register_if reg, actuator_t actuator);

// Functions
unsigned bit_set(unsigned bit_index, unsigned portval) {
  return ((portval >> bit_index) & 1) == BUTTON_PRESSED;
}

// Todo: replace with endswitches_triggered
unsigned endswitch_triggered(unsigned endswitch_index, unsigned portval) {
  return ((portval >> endswitch_index) & 1) == ES_TRIGGERED;
}

int motor_endswitches_triggered(unsigned endswitches_val, unsigned mask) {
  if(mask > 0b11) {
    printf("Error: Invalid 2-bit mask\n");
    return 0;
  }
  // check that every bit in the 2-bit mask is asserted
  for(unsigned shift=0; shift<2; ++shift) {
    if((mask >> shift) & 1) {
      if(((endswitches_val >> shift) & 1) != ES_TRIGGERED) {
        return 0;
      }
    }
  }
  return 1;
}

void update_error_state(motor_state_s* ms, client register_if reg) {

   unsigned error_reg_val = ms->error;
   if(ms->prev_error != NO_ERROR && ms->error == NO_ERROR) {
     // clear the error!
     error_reg_val = ms->prev_error | 0x40; // set cleared bit
     printf("Clearing Error %d for Motor %d\n", ms->prev_error, ms->motor_idx);
   } else if(ms->prev_error == NO_ERROR && ms->error != NO_ERROR) {
     printf("Setting new Error %d for Motor %d\n", ms->error, ms->motor_idx);
   } else if(ms->prev_error != ms->error) {
     printf("A new error %d occured before the previous error %d was cleared\n", ms->error, ms->prev_error);
   }

   ms->prev_error = ms->error; // update prev error

   // Todo review if it is enough to set error reg here
   int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
   reg.set_register(base_reg+MOTOR_ERROR_REG_OFFSET, error_reg_val); 
}

void check_endswitches_and_update_states(unsigned motor_endswitches, motor_state_s* ms, client register_if reg, unsigned init) {
#if ENDSWITCHES_CONNECTED
   printf("Checking endswitches portval 0x%x for Motor %d\n", motor_endswitches, ms->motor_idx);  

   unsigned update_error = 0; 
   if(init) {
     update_error = 1; // update when this function is called during initislisation
   }

   if(motor_endswitches_triggered(motor_endswitches, ENDSWITCH_OPEN | ENDSWITCH_CLOSED)) {
     ms->state = STATE_UNKNOWN;
     ms->error = BOTH_ENDSWITCHES_ON;
     printf("Fatal Error: Both Endswitches on Motor %d triggered at the same time\n", ms->motor_idx);
     update_error = 1;
     // todo: store and report Error
   } else if(motor_endswitches_triggered(motor_endswitches, ENDSWITCH_OPEN)) {
     printf("Motor %d open endswitch triggered\n", ms->motor_idx);
     ms->error = NO_ERROR;
     check_motor_state_after_endswitch_triggered(ms, OPENING, OPEN_POS_ES);

     ms->state = STOPPED;
     ms->actuator = BUTTON; 
     ms->position = OPEN_POS_ES;
     ms->target_position = ms->position;
     update_error = 1;
     stop_motor(ms, reg);
   } else if(motor_endswitches_triggered(motor_endswitches, ENDSWITCH_CLOSED)) {
     printf("Motor %d closed endswitch triggered\n", ms->motor_idx);
     ms->error = NO_ERROR; // can be overridden!
     check_motor_state_after_endswitch_triggered(ms, CLOSING, CLOSED_POS_ES);

     ms->state = STOPPED;
     ms->actuator = BUTTON; 
     ms->position = CLOSED_POS_ES;
     ms->target_position = ms->position;
     update_error = 1;
     stop_motor(ms, reg);
   };

   if(update_error) {
     update_error_state(ms, reg);
   }
#endif
}

void check_control_buttons_and_update_states(unsigned motor_control_buttons, motor_state_s* ms, client register_if reg) {
  // Ignore button in case of severe errors
  if(ms->error == BOTH_ENDSWITCHES_ON ) {
    // Todo: Only do this for errors but not warnings?
    printf("Motor %d is in ERROR state BOTH_ENDSWITCHES_ON, ignoring control button change\n", ms->motor_idx);
    return;
  }
  if(ms->error == POSITION_UNKNOWN ) {
    // Todo: Only do this for errors but not warnings?
    printf("Motor %d is in ERROR state POSITION_UNKNOWN, ignoring control button change\n", ms->motor_idx);
    return;
  }
  if(ms->state == STATE_UNKNOWN) {
    printf("Motor %d is in STATE_UNKNOWN state, ignoring control button change\n", ms->motor_idx);
    return;
  }

  // Process and button change
  if(bit_set(CLOSE_BUTTON_IDX, motor_control_buttons)) {
    if(ms->state == CLOSING) {
      // close button pressed again whilst closing -> switch off
      printf("p_close_button was pressed whilst Motor %d was already closing -> Stop Motor\n", ms->motor_idx);
      ms->target_position = ms->position;
      ms->actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
      stop_motor(ms, reg);
    } else if(ms->state == STOPPED && ms->position >= CLOSED_POS_ES) {
      printf("Motor %d is already in closed position\n", ms->motor_idx);  
      // do nothing
    } else {
      printf("p_close_button was pressed first time -> Switch Motor %d on in closing direction from position %d mm\n", ms->motor_idx, ms->position);
      ms->state = CLOSING;
      ms->target_position = CLOSED_POS_ES;
      start_motor(ms, reg, BUTTON);
    }
  // use else if to give close button the priority
  } else if(bit_set(OPEN_BUTTON_IDX, motor_control_buttons)) {
    if(ms->state == OPENING) {
      // open button pressed again whilst closing -> switch off
      printf("p_open_button was pressed whilst Motor %d was already opening -> Stop Motor\n", ms->motor_idx);
      ms->target_position = ms->position;
      ms->actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
      stop_motor(ms, reg);
    } else if(ms->state == STOPPED && ms->position <= OPEN_POS_ES) {
      printf("Motor %d is already in open position\n", ms->motor_idx);  
      // do nothing
    } else {
      printf("p_open_button was pressed first time -> Switch Motor %d on in opening direction from position %d mm\n", ms->motor_idx, ms->position);
      ms->state = OPENING;
      ms->target_position = OPEN_POS_ES;
      start_motor(ms, reg, BUTTON);
    }
  }
}

void init_motor_state(motor_state_s* ms, client register_if reg, int motor_pos, unsigned endswitches_val, unsigned motor_idx) {
    ms->motor_idx = motor_idx;
    ms->actuator = BUTTON;
    ms->prev_error = NO_ERROR;
    if(motor_pos == -1) {
      ms->state = STATE_UNKNOWN;
      ms->error = POSITION_UNKNOWN;      
    } else {
      ms->state = STOPPED;
      ms->error = NO_ERROR;
    }
    ms->position = motor_pos;
    ms->target_position = ms->position;
    // Check endswitches and compare with position read from flash
    check_endswitches_and_update_states(endswitches_val, ms, reg, 1);

}

void check_and_handle_new_pos(motor_state_s* ms, client register_if reg) {
   // Todo: Tune the system so that endswitches can trigger before OPEN_POS_LIMIT and CLOSED_POS_LIMIT is reached.
   // if position is outside of OPEN_POS_ES and CLOSED_POS_ES then it can be deducted that the Endswitch is broken
   // Todo: Check if position is outside of OPEN_POS_ES-OPEN_TOLERANCE and CLOSED_POS_ES+CLOSED_TOLERANCE

   if(ms->state == CLOSING) {
     // Todo: Fix this. 
     // It has to work like this: Tune speed estimation slightly slower than motor so that Endswitch can trigger before motor is stopped based on postion calulation
     #if ENDSWITCHES_CONNECTED
       if(ms->target_position >= CLOSED_POS_ES) { // Target position is at Closed Endswitch
         // Endswitch should stop the motor. Double heck here based on  
         if(ms->position >= ms->target_position + CLOSED_TOLERANCE) {
           printf("Closing ventilation %d exceeded target position %d by %d cm. Motor slower than expected or closed Endswitch is not working. Emergency motor stop!\n", ms->motor_idx, ms->target_position, CLOSED_TOLERANCE);
           stop_motor(ms, reg);
           ms->error = SPEED_TOO_SLOW;
           update_error_state(ms, reg);
         }
       } else { 
         if(ms->position >= ms->target_position) {
           printf("Closing ventilation %d is >= target position %d cm\n", ms->motor_idx, ms->target_position);
           stop_motor(ms, reg);
         }   
       }
    #else
       if(ms->position >= ms->target_position) {
         printf("Closing ventilation %d is >= target position %d cm\n", ms->motor_idx, ms->target_position);
         stop_motor(ms, reg);
       }         
    #endif 

   } 
   if(ms->state == OPENING) {
     #if ENDSWITCHES_CONNECTED
       if(ms->target_position <= OPEN_POS_ES) { // Target position is at Open Endswitch, 
         if(ms->position <= ms->target_position - OPEN_TOLERANCE) {
           printf("Opening ventilation %d exceeded target position %d by %d cm. Motor slower than expected or closed Endswitch is not working. Emergency motor stop!\n", ms->motor_idx, ms->target_position, OPEN_TOLERANCE);
           stop_motor(ms, reg);
           ms->error = SPEED_TOO_SLOW;
           update_error_state(ms, reg);
         } 
       } else {
         if(ms->position <= ms->target_position) {
           printf("Opening ventilation %d is <= target position %d cm\n", ms->motor_idx, ms->target_position);
           stop_motor(ms, reg);
         }  
       }
    #else
       if(ms->position <= ms->target_position) {
         printf("Opening ventilation %d is <= target position %d cm\n", ms->motor_idx, ms->target_position);
         stop_motor(ms, reg);
       }     
    #endif
   }
}

// Update the Read only registers
void update_position_regs(motor_state_s* ms, client register_if reg) {
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  // Causes server control to take over after I2C control: 
  reg.set_register(base_reg+MOTOR_TARGET_POS_REG_OFFSET,  ms->target_position);
  reg.set_register(base_reg+MOTOR_CURRENT_POS_REG_OFFSET, ms->position);
}

void init_regs(motor_state_s* ms, client register_if reg) {
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  // Causes server control to take over after I2C control: 
  reg.set_register(base_reg, ms->state); 
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

  if(ms->error == NO_ERROR) { // Only init if there was no error
    reg.set_register(base_reg+MOTOR_TARGET_POS_REG_OFFSET,  ms->target_position);
    reg.set_register(base_reg+MOTOR_CURRENT_POS_REG_OFFSET, ms->position);
  }
}

void check_motor_state_after_endswitch_triggered(motor_state_s* ms, motor_state_t state, int actual_position) {
  // Todo: Check OPEN_TOLERANCE, CLOSED_TOLERANCE, motor state
  if(ms->state == OPENING) {
    if(ms->position - OPEN_POS_ES > OPEN_TOLERANCE) {
       printf("OPENING speed slower than estimate. Position is at %d when it should be at OPEN_POS_ES %d\n", ms->position, OPEN_POS_ES);
       ms->error = SPEED_TOO_SLOW; // Motor slower than estimation
    } else if(OPEN_POS_ES - ms->position > OPEN_TOLERANCE) {
       printf("OPENING speed faster than estimate. Position is at %d when it should be at OPEN_POS_ES %d\n", ms->position, OPEN_POS_ES);
       ms->error = SPEED_TOO_FAST; // Motor faster than estimation
    }
  }
  if(ms->state == CLOSING) {
    if(ms->position - CLOSED_POS_ES > OPEN_TOLERANCE) {
       printf("CLOSING speed faster than estimate. Position is at %d when it should be at CLOSED_POS_ES %d\n", ms->position, CLOSED_POS_ES);
       ms->error = SPEED_TOO_FAST; // Motor slower than estimation
    } else if(CLOSED_POS_ES - ms->position > OPEN_TOLERANCE) {
       printf("CLOSING speed slower than estimate. Position is at %d when it should be at CLOSED_POS_ES %d\n", ms->position, CLOSED_POS_ES);
       ms->error = SPEED_TOO_SLOW; // Motor slower than estimation
    }   
  } 
}


void upate_pushbutton_leds(motor_state_s* ms) {
   int led_on_mask; // set bit one where LED is on
   if(ms->state == OPENING) {
     led_on_mask = (1 << MOTOR_OPEN_BUTTON_GREEN_LED_IDX);
   } else if(ms->state == CLOSING) {
     led_on_mask = (1 << MOTOR_CLOSE_BUTTON_BLUE_LED_IDX);
   } else if(ms->state == STOPPED) {
     led_on_mask = 0; // off
   } else { // notify the user that there is some error state
     led_on_mask = (1 << MOTOR_OPEN_BUTTON_RED_LED_IDX) | (1 << MOTOR_CLOSE_BUTTON_RED_LED_IDX);
   }
 
   if(PUSHBUTTON_LED_ON == 0) {
     // invert the mask for low-active LEDs
     led_on_mask = ~led_on_mask;
   }

   if(ms->motor_idx == 0) {
     p_m0_pushbutton_leds <: led_on_mask;
   } else {
     p_m1_pushbutton_leds <: led_on_mask;
   }
}

void stop_motor(motor_state_s* ms, client register_if reg) {
    if(ms->motor_idx == 0) {
      p_m0_on <: MOTOR_OFF;
    } else {
      p_m1_on <: MOTOR_OFF;
    }
    ms->state = STOPPED;

    upate_pushbutton_leds(ms);
    update_position_regs(ms, reg);
    // register stop event
    int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
    reg.set_register(base_reg, ms->state); 
    reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, STOP);
    reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

    printf("Stopped Motor %u at position %d mm\n", ms->motor_idx, ms->position);

    unsigned char *page_buffer;
    page_buffer = malloc(fl_getWriteScratchSize(0, 2));
    unsigned char data[2] = {FLASH_DATA_VALID_BYTE, ms->position};

    printf("Writing new motor position %d to flash for Motor %d\n", ms->position, ms->motor_idx);
    fl_writeData(ms->motor_idx*2,
                 2,
                 data,
                 page_buffer);

    free(page_buffer);
}

int start_motor(motor_state_s* ms, client register_if reg, actuator_t actuator) {

  ms->actuator = actuator;

  // Update regs and other state straight away to avoid races
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  reg.set_register(base_reg, ms->state); 
  reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, START);
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position
  upate_pushbutton_leds(ms);
  update_position_regs(ms, reg);

  // Do the switching
  unsigned dir_val;
  if(ms->state == OPENING) {
    dir_val = MOTOR_OPENING_DIR;
    printf("Starting Motor %u at position %d mm in opening direction\n", ms->motor_idx, ms->position);
  } else if(ms->state == CLOSING) {
    dir_val = MOTOR_CLOSING_DIR;
    printf("Starting Motor %u at position %d mm in closing direction\n", ms->motor_idx, ms->position);
  } else {
    // invalid
    return 1;
  }

  if(ms->motor_idx == 0) {
    p_m0_dir <: dir_val;
    delay_us(10000); // delay 10ms to make sure the big capacitor is connected when motor is switched on
    p_m0_on <: MOTOR_ON;
  } else {
    p_m1_dir <: dir_val;
    delay_us(10000); // delay 10ms to make sure the big capacitor is connected when motor is switched on
    p_m1_on <: MOTOR_ON;  
  }

  return 0;
}

int motor_moved_by_button(motor_state_s* ms, client register_if reg) {
  if(ms->state == OPENING || ms->state == CLOSING) {
    unsigned status_reg = ms->motor_idx*4 + 3;
    actuator_t actuator = (actuator_t) reg.get_register(status_reg);
    if(actuator == BUTTON) {
      return 1;
    }
  }
  return 0; 
}


void mc_control(client register_if reg) {

    timer tmr_pos;
    timer tmr_dbc; // debounce tiemr for p_control_buttons
    timer tmr_dbc_es;  

    int t_dbc, t_dbc_es, t_pos;  // time variables
    motor_state_s state_m0, state_m1;
    unsigned buttons_changed = 0;
    unsigned endswitches_changed = 0;

    unsigned prev_control_buttons_val, prev_endswitches_val;
    unsigned led_val = 0;

    char mac_address[6];
    // Read MAC address:
    otp_board_info_get_mac(otp_ports, 0, mac_address);
    printf("Read MAC Address");
    for(unsigned i=0; i<6; ++i) printf("0x%x ",mac_address[i]);
    printf("\n");  
    // Store lowest byte of Mac address as unique ID controller 
    char id = mac_address[5];
    reg.set_register(SYSTEM_ID_REG_OFFSET, id);

    printf("Starting Greenhouse Motor Control Application on Controller with ID 0x%x\n\n", id);
    printf("Motor speed is set to %u mm/s\n", MOTOR_SPEED);


    #if ES_TRIGGERED==1
      // enable internal pulldowns for high active signals
      set_port_pull_down(p_endswitches);
    #else 
      // enable internal pulldowns for low active 
      set_port_pull_up(p_endswitches);
    #endif
    #if BUTTON_PRESSED==1
      // enable internal pulldowns for high active signals
      set_port_pull_down(p_control_buttons);
    #else 
      // enable internal pulldowns for low active 
      set_port_pull_up(p_control_buttons);
    #endif



    // Connect to the QuadSPI device using the quadflash library function fl_connectToDevice. 
    if(fl_connectToDevice(ports, deviceSpecs, sizeof(deviceSpecs)/sizeof(fl_QuadDeviceSpec)) != 0) {
      printf("fl_connectToDevice Error\n");
      return; 
    }
    // Read Motor positions from flash
    char byte_buffer[FLASH_DATA_BYTES];
    // Connect to the QuadSPI device using the quadflash library function fl_connectToDevice. 
    if(fl_readData(0, FLASH_DATA_BYTES, byte_buffer) != 0) {
      printf("fl_readData Error\n");
      return;
    }
    printf("fl_readData Read %d bytes from flash:\n", FLASH_DATA_BYTES);
    for(unsigned i=0; i<FLASH_DATA_BYTES; ++i) {
      printf("0x%x %d\n", byte_buffer[i], byte_buffer[i]);
    }

    // Init Motor 0 position
    int m0_pos = -1; // ivalid
    if(byte_buffer[0] == FLASH_DATA_VALID_BYTE) {
      m0_pos = byte_buffer[1];
      printf("Found valid position for Motor 0: %d\n", m0_pos);
    } 
    #if !ENDSWITCHES_CONNECTED
    else {
      m0_pos = CLOSED_POS_ES;
      printf("Endswitches not connected. Initialising Motor 0 to arbitrary position %d\n", m0_pos);
    }
    #endif
    
    // Init Motor 1 position
    int m1_pos = -1; // invalid
    if(byte_buffer[2] == FLASH_DATA_VALID_BYTE) {
      m1_pos = byte_buffer[3];
      printf("Found valid position for Motor 1: %d\n", m1_pos);
    } 
    #if !ENDSWITCHES_CONNECTED
    else {
      m1_pos = CLOSED_POS_ES;
      printf("Endswitches not connected. Initialising Motor 1 to arbitrary position %d\n", m1_pos);
    }
    #endif


    // Is this needed??
    p_control_buttons :> prev_control_buttons_val;
    p_endswitches :> prev_endswitches_val;

    unsigned endswitches_m0 = prev_endswitches_val & 0b11;
    unsigned endswitches_m1 = (prev_endswitches_val >> 2) & 0b11;

    init_motor_state(&state_m0, reg, m0_pos, endswitches_m0, 0);
    init_regs(&state_m0, reg);
    init_motor_state(&state_m1, reg, m1_pos, endswitches_m1, 1);
    init_regs(&state_m1, reg);

    // Init!!
    tmr_pos :> t_pos;  // init position update time


    while(1) {
        // input from all input ports and store in prev values
        // prev values are instrumental in the case guards to detect the desired edge

        // event handlers
        select {
            // I2C register changed logic.
            // Note: This does not cause ET_ILLEGAL_RESOURCE!
            case reg.register_changed():
              unsigned regnum = reg.get_changed_regnum();
              unsigned value = reg.get_register(regnum);

              switch(regnum) {
                case MOTOR_STATE_REG_OFFSET: {
                  // To avoid race condition, ignore I2C command from remote client if motor_moved_by_button
                  if(!motor_moved_by_button(&state_m0, reg)) { 
                    state_m0.state = (value & 0x7);
                    start_motor(&state_m0, reg, I2C);
                  } else {
                    printf("Ignoring command from remote client whilst motor 0 is operated by button\n");
                  }
                  break;
                }
                case MOTOR_TARGET_POS_REG_OFFSET: {
                  state_m0.target_position = value;
                  printf("I2C command setting new target_position %d for Motor 0\n", value);
                  break;
                }
                case NUM_REGS_PER_MOTOR+MOTOR_STATE_REG_OFFSET: {
                  // To avoid race condition, ignore I2C command from remote client if motor_moved_by_button
                  if(!motor_moved_by_button(&state_m1, reg)) { // Ignore I2C command if motor_moved_by_button
                    state_m1.state = (value & 0x7);
                    start_motor(&state_m1, reg, I2C);
                  } else {
                    printf("Ignoring command from remote client whilst motor 1 is operated by button\n");
                  }
                  break;
                }
                case NUM_REGS_PER_MOTOR+MOTOR_TARGET_POS_REG_OFFSET: {  
                  state_m1.target_position = value;
                  printf("I2C command setting new target_position %d for Motor 1\n", value);
                  break;
                }
              }
              break;
            
            // Monitor control buttons
            case (!buttons_changed) => p_control_buttons when pinsneq(prev_control_buttons_val) :> prev_control_buttons_val:
              // Port value changed which means some button was pressed or released
              buttons_changed = 1;
              tmr_dbc :> t_dbc; // update timer
#if DEBOUNCE
              break;

            // debounce p_control_buttons
            case (buttons_changed) => tmr_dbc when timerafter(t_dbc+DEBOUNCE_TIME) :> t_dbc:
#endif
              buttons_changed = 0; // reset to re-activate case (buttons_changed) =>
              unsigned control_buttons_val;
              p_control_buttons :> control_buttons_val;

              if(control_buttons_val == prev_control_buttons_val) { // The button change persistet -> No glitch
                unsigned control_buttons_m0 = control_buttons_val & 0b11;
                unsigned control_buttons_m1 = (control_buttons_val >> 2) & 0b11;
                printf("Motor Control Buttons changed to value 0x%x\n", control_buttons_val);
                check_control_buttons_and_update_states(control_buttons_m0, &state_m0, reg); 
                check_control_buttons_and_update_states(control_buttons_m1, &state_m1, reg); 

              }
              break;

            // Monitor Endswitches
            case (!endswitches_changed) => p_endswitches when pinsneq(prev_endswitches_val) :> prev_endswitches_val:
              // Port value changed which means some switch was pressed or released
              endswitches_changed = 1;
              tmr_dbc_es :> t_dbc_es; // update timer
#if DEBOUNCE
              break;

            // debounce p_endswitches
            case (endswitches_changed) =>  tmr_dbc_es when timerafter(t_dbc_es+DEBOUNCE_TIME) :> t_dbc_es:
#endif
              endswitches_changed = 0; // reset to re-activate case (endswitches_changed) =>
              unsigned endswitches_val;
              p_endswitches :> endswitches_val;
              if(endswitches_val == prev_endswitches_val) { // The button change persistet -> No glitchheck
                endswitches_m0 = endswitches_val & 0b11;
                endswitches_m1 = (endswitches_val >> 2) & 0b11;
                printf("Endswitches changed to value 0x%x\n", endswitches_val);
                check_endswitches_and_update_states(endswitches_m0, &state_m0, reg, 0); // Check that Open and Closed are not triggered at the same time
                check_endswitches_and_update_states(endswitches_m1, &state_m1, reg, 0); // Check that Open and Closed are not triggered at the same time
              }
              break;

            // Position estimation
            case tmr_pos when timerafter(t_pos+POS_UPDATE_PERIOD) :> t_pos:
              // divide by 10 because motor speed is in mm/s
              unsigned pos_change =  (unsigned long long) MOTOR_SPEED * POS_UPDATE_PERIOD / (XS1_TIMER_HZ * 10); 

              if(state_m0.state == OPENING || state_m0.state == CLOSING) {
                  if(state_m0.state == OPENING) {
                    state_m0.position -= pos_change;
                    printf("Motor 1 is opening. ");
                  } else {
                    state_m0.position += pos_change;
                    printf("Motor 1 is closing. ");
                  }
                  printf("Updated position estimate to %d mm\n", state_m0.position);
                  update_position_regs(&state_m0, reg);

                  check_and_handle_new_pos(&state_m0, reg);
              }; 
              if(state_m1.state == OPENING || state_m1.state == CLOSING) {
                  if(state_m1.state == OPENING) {
                    state_m1.position -= pos_change;
                    printf("Motor 1 is opening. ");
                  } else {
                    state_m1.position += pos_change;
                    printf("Motor 2 is closing. ");
                  }
                 printf("Updated position estimate to %d mm\n", state_m1.position);
                  update_position_regs(&state_m1, reg);

                  check_and_handle_new_pos(&state_m1, reg);
              }; 
              //Todo: if(pos_out_of_range(state_m1.position)) {

              // Use this timer event to update LEDs
              // toggle LED for activity detection
              led_val = 1-led_val;
              p_led <: (led_val << 3); // Green LED is on P4F3
              // Update pushbutton LEDs. Todo: move to a place where 
              upate_pushbutton_leds(&state_m0);
              upate_pushbutton_leds(&state_m1);
              break;

        }
    }
}


uint8_t device_addr = 0x3c;

void i2c_control(server register_if i_reg) {
    i2c_slave_callback_if i_i2c;
    //[[combine]]
    par {
        i2c_slave_register_file(i_i2c, i_reg);
        i2c_slave(i_i2c, p_slave_scl, p_slave_sda, device_addr);
    }
}

int main() {

  register_if i_reg;
 
  //debug_printf("Starting I2C enabled Motoro Controller\n");

    par {
      on MC_TILE : i2c_control(i_reg);
      on MC_TILE : mc_control(i_reg);
    }
    return 0;
}
