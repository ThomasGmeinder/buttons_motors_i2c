/*
 * app_gw_mc_control.xc
 *
 *  Created on: 12 Mar 2017
 *      Author: thomas
 */

/********** Control Specification ****************/
// Inputs and actions
// - User presses open button whilst motors stopped. A: start_motor OPENING with target position OPEN_POS_ES
// - User presses open button again whilst motors run. A: stop_motor
// - User presses close button once: A: start_motor CLOSING with target position CLOSED_POS_ES
// - User presses close button again whilst motors run. A: stop_motor
// - Web server sends close command with a new target position. A: start_motor CLOSING with target position X
// - Web server sends open command with a new target position. A: start_motor OPENING with target position X
// Note: When a I2C command comes in from the server whilst the motor is running, then ignore it

/************* Flash firmware ********************/
// Flash with 8k Datapartition: xflash bin/app_gw_mc_control.xe --boot-partition-size 516096

/************* Test Protocol *********************/
// Switch both Endswitches on: 
//   BOTH_ENDSWITCHES_ON is correctly flagged via I2C and displayed in Browser
//   Both Buttons light up red
//   The error is correctly cleared when one of the Endswitches is switched off.
// When flash is empty (no known position) and no Endswitch is triggered:
//   POSITION_UNKNOWN error is correctly flagged via I2C and displayed in the Browser
//   Both Buttons light up red
//   The error is correctly cleared when one of the Endswitches is switched on. Works with multiple errors!
// Open/close ventilation with buttons and no Endswitches: MOTOR_TOO_SLOW error is correctly reported.
// Trigger Open Endswitch whilst Motor is Opening: MOTOR_TOO_FAST error is correctly reported
// Trigger Open Endswitch whilst Motor is Closing: POSITION_UNKNOWN error is correctly reported. Can be cleeared by triggering Endswitch again.
// Trigger Closed Endswitch whilst Motor is Opening: POSITION_UNKNOWN error is correctly reported. Can be cleeared by triggering Endswitch again.
// Trigger Closed Endswitch whilst Motor is Closing: MOTOR_TOO_FAST error is correctly reported

// Todo:
// Try to make sure that target_mp is always reached. Then the tolerances on the server can be set to 0.
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
#include "common.h"
#include <quadflashlib.h>
#include <quadflash.h>
#include "debug_print.h"
#include "otp_board_info.h"
#include "string.h"
#include "spi_app.h"
#include "dsp.h"

#define ENDSWITCHES_CONNECTED 1

// Note: Positions are calculated in micrometers so that integer speed can be tuned better.
#define OPEN_POS_MIN MM_to_UM(0) // min position 
#define CLOSED_POS_MAX MM_to_UM(1200)  // max position in um

// Measured times:
// Opening: 76 seconds
// opening speed: 15789 um/s 

// Closing: 77 seconds
// closing speed: 15584 um/s

#define OPEN_POS_ES OPEN_POS_MIN  // Open position at Endswitch
#define CLOSED_POS_ES CLOSED_POS_MAX

#define DISTANCE_BETWEEN_ES = CLOSED_POS_ES-OPEN_POS_ES

#define OPEN_TOLERANCE MM_to_UM(10)   // 10 mm tolerance
#define CLOSED_TOLERANCE MM_to_UM(10) 

#define DEBOUNCE 1
// 250ms are needed because wwitching motor on/off creates current spikes which induce glitches on the Motorcontroller I/Os
#define DEBOUNCE_TIME XS1_TIMER_HZ/4  
#define FLASH_UPDATE_PERIOD_CYCLES POS_UPDATE_PERIOD_CYCLES*20 // Update flash every 2 seconds

#define LED_CYCLES (XS1_TIMER_HZ/10) // 100ms

#define ES_TRIGGERED 0 // GPIO value when Endswitch is triggered. Note: GPIO has pulldown
#define BUTTON_PRESSED 1 // Button pressed. 

#define DEBUG_BUTTON_LOGIC 1

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

// Login raspiviv
// raspiviv
// nX9NypBk


/** Inputs **/
// Endswitches for both Motors
on MC_TILE : in port p_endswitches = XS1_PORT_4C;   // X0D14 (pin 0), X0D15, X0D20, X0D21 (pin 3)
// Motor 0: bits 1:0
// Motor 1: bits 3:2
//on MC_TILE : in port p_endswitches = XS1_PORT_4E;  

// Button to open and close the ventilations
on MC_TILE : in port p_control_buttons = XS1_PORT_4D;  // X0D16, X0D17, X0D18, X0D19
 
/** Outputs **/
// Motor 0 on/off
on MC_TILE : out port p_m0_on = XS1_PORT_1F;  // X0D13 ()
// Motor 0 direction
on MC_TILE : out port p_m0_dir = XS1_PORT_1E;  // X0D12 ()

// Motor 1 on/off
on MC_TILE : out port p_m1_on = XS1_PORT_1G;  // X0D22 ()
// Motor 1 direction
on MC_TILE : out port p_m1_dir = XS1_PORT_1H;  // X0D23 ()

on MC_TILE : port p_slave_sda = XS1_PORT_1M; // X0D36 // connect to GPIO 3 on rPI
on MC_TILE : port p_slave_scl = XS1_PORT_1N; // X0D37 // connect to GPIO 2 on rPI

on MC_TILE : port p_led = XS1_PORT_4F;

on MC_TILE : port p_m0_pushbutton_leds = XS1_PORT_4A; // X0D02, X0D03, X0D08, X0D09
on MC_TILE : port p_m1_pushbutton_leds = XS1_PORT_4E; // X0D26, X0D27, X0D32, X0D33

on MC_TILE: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;

#if ACCESS_ADC_VIA_SPI
/* These ports are used for the SPI master */
out buffered port:32   p_sclk  = on MC_TILE: XS1_PORT_1I; // X0D24 (23)
out port               p_ss[1] = on MC_TILE: {XS1_PORT_1L}; // X0D35 (17)
in buffered port:32    p_miso  = on MC_TILE: XS1_PORT_1J; // X0D25 (21)
out buffered port:32   p_mosi  = on MC_TILE: XS1_PORT_1K; // X0D34 (19)

clock clk0 = on tile[0]: XS1_CLKBLK_2;
clock clk1 = on tile[0]: XS1_CLKBLK_3;
#endif

extern void flash_server(chanend flash_c);


void delay_us(unsigned time_us) {
  int t;
  timer tmr;
  tmr :> t;
  tmr when timerafter(t+time_us*100) :> t;
}

// protos
void stop_motor(motor_state_s* ms, client register_if reg);
void check_motor_state_after_endswitch_triggered(motor_state_s* ms, int actual_position);
int start_motor(motor_state_s* ms, client register_if reg, actuator_t actuator);

// Functions
unsigned bit_set(unsigned bit_index, unsigned portval) {
  return ((portval >> bit_index) & 1) == BUTTON_PRESSED;
}

unsigned position_in_range(int position) {
   const int min = OPEN_POS_ES-OPEN_TOLERANCE;
   const int max = CLOSED_POS_ES+CLOSED_TOLERANCE;
   if(position < min|| position > max) {
     printf("Position %d is out of range %d..%d\n", position, min, max);
     return 0;
   }
   return 1;
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
     // Todo: store and report Error
   } else if(motor_endswitches_triggered(motor_endswitches, ENDSWITCH_OPEN)) {
     printf("Motor %d open endswitch triggered\n", ms->motor_idx);
     ms->error = NO_ERROR;
     check_motor_state_after_endswitch_triggered(ms, OPEN_POS_ES);

     ms->state = STOPPED;
     ms->actuator = BUTTON; 
     ms->position = OPEN_POS_ES;
     ms->target_position = ms->position;
     update_error = 1;
     stop_motor(ms, reg);
   } else if(motor_endswitches_triggered(motor_endswitches, ENDSWITCH_CLOSED)) {
     printf("Motor %d closed endswitch triggered\n", ms->motor_idx);
     ms->error = NO_ERROR; // can be overridden!
     check_motor_state_after_endswitch_triggered(ms, CLOSED_POS_ES);

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
      // Blink the close color LED
      ms->close_button_blink_counter = 20;
    } else {
      printf("p_close_button was pressed first time -> Switch Motor %d on in closing direction from position %d mm\n", ms->motor_idx, UM_to_MM(ms->position));
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
      // Blink the open color LED
      ms->open_button_blink_counter = 20;
    } else {
      printf("p_open_button was pressed first time -> Switch Motor %d on in opening direction from position %d mm\n", ms->motor_idx, UM_to_MM(ms->position));
      ms->state = OPENING;
      ms->target_position = OPEN_POS_ES;
      start_motor(ms, reg, BUTTON);
    }
  }
}

void init_motor_state(motor_state_s* ms, client register_if reg, int motor_pos, 
  unsigned opening_speed, unsigned closing_speed,
  unsigned endswitches_val, unsigned motor_idx, int t) {
    ms->motor_idx = motor_idx;
    ms->actuator = BUTTON;
    ms->prev_error = NO_ERROR;
    if(motor_pos == INT_MIN) {
      ms->state = STATE_UNKNOWN;
      ms->error = POSITION_UNKNOWN;      
    } else {
      ms->state = STOPPED;
      ms->error = NO_ERROR;
    }
    ms->position = motor_pos;
    ms->target_position = ms->position;
    ms->open_button_blink_counter = 0;
    ms->close_button_blink_counter = 0;
    ms->update_flash = 0;
    ms->time_of_last_flash_update = t;

    ms->opening_speed = opening_speed;
    ms->closing_speed = closing_speed;

#if INFER_ENDSWITCHES_WITH_AC_SENSOR
    ms->AC_current_hysteresis_counter = 0;
    ms->AC_current_on = 0;
    ms->detect_endswitches_from_AC_current = 0;
#endif

#if !INFER_ENDSWITCHES_WITH_AC_SENSOR && ENDSWITCHES_CONNECTED
    // Check endswitches and compare with position read from flash
    check_endswitches_and_update_states(endswitches_val, ms, reg, 1);
#endif
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
           printf("Closing ventilation %d: current position %d exceeded Endswith position %d by %d mm. Motor slower than expected or Closed Endswitch is not working. Emergency motor stop!\n"\
            , ms->motor_idx, UM_to_MM(ms->position), UM_to_MM(ms->target_position), UM_to_MM(CLOSED_TOLERANCE));
           stop_motor(ms, reg);
           ms->error = MOTOR_TOO_SLOW;
           update_error_state(ms, reg);
           return;
         }
       } else { 
         if(ms->position >= ms->target_position) {
           printf("Closing ventilation %d is >= target position %d mm\n", ms->motor_idx, UM_to_MM(ms->target_position));
           stop_motor(ms, reg);
           return;
         }   
       }
    #else
       if(ms->position >= ms->target_position) {
         printf("Closing ventilation %d is >= target position %d mm\n", ms->motor_idx, UM_to_MM(ms->target_position));
         stop_motor(ms, reg);
         return;
       }         
    #endif 

   } 
   if(ms->state == OPENING) {
     #if ENDSWITCHES_CONNECTED
       if(ms->target_position <= OPEN_POS_ES) { // Target position is at Open Endswitch, 
         if(ms->position <= ms->target_position - OPEN_TOLERANCE) {
           printf("Opening ventilation %d: current position %d exceeded Endswith position %d by %d mm. Motor slower than expected or Open Endswitch is not working. Emergency motor stop!\n"\
            , ms->motor_idx, UM_to_MM(ms->position), UM_to_MM(ms->target_position), UM_to_MM(OPEN_TOLERANCE));
           stop_motor(ms, reg);
           ms->error = MOTOR_TOO_SLOW;
           update_error_state(ms, reg);
           return;
         } 
       } else {
         if(ms->position <= ms->target_position) {
           printf("Opening ventilation %d: current position %d <= target position %d mm\n"\
            ,ms->motor_idx, UM_to_MM(ms->position), UM_to_MM(ms->target_position));
           stop_motor(ms, reg);
           return;
         }  
       }
    #else
       if(ms->position <= ms->target_position) {
         printf("Opening ventilation %d: current position %d <= target position %d mm\n"\
          ,ms->motor_idx, UM_to_MM(ms->position), UM_to_MM(ms->target_position));
         stop_motor(ms, reg);
         return;
       }     
    #endif
   }
 
   if(!position_in_range(ms->position)) {
      ms->error = POSITION_UNKNOWN;
      update_error_state(ms, reg); 
   }

}

// Update the Read only registers
void update_position_regs(motor_state_s* ms, client register_if reg) {
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  // Causes server control to take over after I2C control: 
  int browser_pos = ms->position;

  // correct the positions for the browser display if required.
  // Related errors (MOTOR_TOO_SLOW, MOTOR_TOO_FAST) are flagged separately through error register
  if(browser_pos >= CLOSED_POS_ES) {
    // Browser cannot display positions > CLOSED_POS_ES
    browser_pos = CLOSED_POS_ES; 
  } else if(browser_pos <= OPEN_POS_ES) {
    // Browser cannot display positions < OPEN_POS_ES
    // This also avoids negative values which are invalid beause registers are uint8_t
    browser_pos = OPEN_POS_ES; 
  }

  reg.set_register(base_reg+MOTOR_TARGET_POS_REG_OFFSET,  UM_to_CM(ms->target_position));
  reg.set_register(base_reg+MOTOR_CURRENT_POS_REG_OFFSET, UM_to_CM(browser_pos));
}

void init_regs(motor_state_s* ms, client register_if reg) {
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  // Causes server control to take over after I2C control: 
  reg.set_register(base_reg, ms->state); 
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

  if(ms->error == NO_ERROR) { // Only init if there was no error
    reg.set_register(base_reg+MOTOR_TARGET_POS_REG_OFFSET,  UM_to_CM(ms->target_position));
    reg.set_register(base_reg+MOTOR_CURRENT_POS_REG_OFFSET, UM_to_CM(ms->position));
  }
}

void check_motor_state_after_endswitch_triggered(motor_state_s* ms, int actual_position) {
  // Todo: Check OPEN_TOLERANCE, CLOSED_TOLERANCE, motor state

  // check the actual_position as per the triggered Endswitch against the computed motor position
  // Cases:
  // actual_position is from the wrong Endswich -> Fatal Error -> Set POSITION_UNKNOWN
  // actual_position is ahead of ms->position: Actual motor speed faster than estimate
  // ms->position is ahead of actual_position: Actual motor speed slower than estimate

  if(ms->state == OPENING) {
    if(actual_position == CLOSED_POS_ES) {
      printf("Fatal Error. ENDSWITCH_CLOSED triggered when Motor %d is OPENING\n", ms->motor_idx);
      ms->error = POSITION_UNKNOWN;
    } 
    else if(actual_position - ms->position > OPEN_TOLERANCE) {
       printf("OPENING speed slower than estimate. Position is at %d mm when it should be at OPEN_POS_ES %d\n", UM_to_MM(ms->position), OPEN_POS_ES);
       ms->error = MOTOR_TOO_SLOW; // Motor slower than estimation
    } 
    else if(ms->position - actual_position > OPEN_TOLERANCE) {
       printf("OPENING speed faster than estimate. Position is at %d mm when it should be at OPEN_POS_ES %d\n", UM_to_MM(ms->position), OPEN_POS_ES);
       ms->error = MOTOR_TOO_FAST; // Motor faster than estimation
    }
  }
  if(ms->state == CLOSING) {
    if(actual_position == OPEN_POS_ES) {
      printf("Fatal Error. ENDSWITCH_OPEN triggered when Motor %d is CLOSING\n", ms->motor_idx);
      ms->error = POSITION_UNKNOWN;
    } 
    else if(actual_position - ms->position > CLOSED_TOLERANCE) {
       printf("CLOSING speed faster than estimate. Position is at %d mm when it should be at CLOSED_POS_ES %d\n", UM_to_MM(ms->position), CLOSED_POS_ES);
       ms->error = MOTOR_TOO_FAST; // Motor slower than estimation
    }   
    else if(ms->position - actual_position > CLOSED_TOLERANCE) {
      printf("CLOSING speed slower than estimate. Position is at %d mm when it should be at CLOSED_POS_ES %d\n", UM_to_MM(ms->position), CLOSED_POS_ES);
       ms->error = MOTOR_TOO_SLOW; // Motor slower than estimation
    } 
  } 
}


void update_pushbutton_leds(motor_state_s* ms) {
   int led_on_mask; // set bit one where LED is on
   if(ms->state == OPENING) {
     led_on_mask = (1 << MOTOR_OPEN_BUTTON_GREEN_LED_IDX);
   } else if(ms->state == CLOSING) {
     led_on_mask = (1 << MOTOR_CLOSE_BUTTON_BLUE_LED_IDX);
   } else if(ms->state == STOPPED) {
     led_on_mask = 0; // off
   };

   // blinking overrides above
   if(ms->open_button_blink_counter > 0) {
      // toggle the LED
      if(ms->open_button_blink_counter & 1) {
        // switch on for odd numbers
        led_on_mask = (1 << MOTOR_OPEN_BUTTON_GREEN_LED_IDX);
      } else {
        led_on_mask = 0;
      }
      // update the time
      ms->open_button_blink_counter -= 1;
   }
   if(ms->close_button_blink_counter > 0) {
      // toggle the LED
      if(ms->close_button_blink_counter & 1) {
        // switch on for odd numbers
        led_on_mask = (1 << MOTOR_CLOSE_BUTTON_BLUE_LED_IDX);
      } else {
        led_on_mask = 0;
      }
      ms->close_button_blink_counter -= 1;
   }

   // Error overrides above
   if(ms->state == STATE_UNKNOWN) {
     // notify the user that there is some error state
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

    update_pushbutton_leds(ms);
    update_position_regs(ms, reg);
    // register stop event
    int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
    reg.set_register(base_reg, ms->state); 
    reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, STOP);
    reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

    // trigger flash write
    ms->update_flash = 1;

    printf("Stopped Motor %u at position %d mm\n", ms->motor_idx, UM_to_MM(ms->position));

}

int start_motor(motor_state_s* ms, client register_if reg, actuator_t actuator) {
  timer tmr;

  tmr :> ms->start_time;

#if INFER_ENDSWITCHES_WITH_AC_SENSOR
  // reset the flag
  ms->detect_endswitches_from_AC_current = 0;
#endif 

  ms->actuator = actuator;

  // Update regs and other state straight away to avoid races
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  reg.set_register(base_reg, ms->state); 
  reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, START);
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position
  update_pushbutton_leds(ms);
  update_position_regs(ms, reg);

  // Do the switching
  unsigned dir_val;
  if(ms->state == OPENING) {
    dir_val = MOTOR_OPENING_DIR;
    printf("Starting Motor %u at position %d mm in opening direction\n", ms->motor_idx, UM_to_MM(ms->position));
  } else if(ms->state == CLOSING) {
    dir_val = MOTOR_CLOSING_DIR;
    printf("Starting Motor %u at position %d mm in closing direction\n", ms->motor_idx, UM_to_MM(ms->position));
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

#if INFER_ENDSWITCHES_WITH_AC_SENSOR
void update_motor_current_state(motor_state_s* ms) {
  int32_t rms_current_q16 = get_rms_current(ms->motor_idx);
  //printf("update_motor_current_state: Motor %d, AC_current_on %d, rms_current %.2f A\n", ms->motor_idx, ms->AC_current_on, F16(rms_current_q16));
  if(ms->AC_current_on) {
    if(rms_current_q16 <= MOTOR_CURRENT_OFF_THRESHOLD) {
       ms->AC_current_hysteresis_counter++;
       if(ms->AC_current_hysteresis_counter > MOTOR_CURRENT_HYSTERESIS_PERIODS) {
          ms->AC_current_on = 0; 
          printf("Changing AC current state to OFF for Motor %d\n", ms->motor_idx);
          ms->AC_current_hysteresis_counter = 0;
       }
    } else if(rms_current_q16 >= MOTOR_CURRENT_ON_THRESHOLD) {
       ms->AC_current_hysteresis_counter = 0; 
    }
  } else if(!ms->AC_current_on) {
    if(rms_current_q16 >= MOTOR_CURRENT_ON_THRESHOLD) {
       ms->AC_current_hysteresis_counter++;
       if(ms->AC_current_hysteresis_counter > MOTOR_CURRENT_HYSTERESIS_PERIODS) {
          ms->AC_current_on = 1;  
          printf("Changing AC current state to ON for Motor %d\n", ms->motor_idx);
          ms->AC_current_hysteresis_counter = 0;
       }
    } else if(rms_current_q16 <= MOTOR_CURRENT_OFF_THRESHOLD) {
       ms->AC_current_hysteresis_counter = 0; 
    }
  }
}

void monitor_motor_current(motor_state_s* ms, client register_if reg) {
   timer tmr;
   int current_time;
   tmr :> current_time;

   if((current_time - ms->start_time)/POS_UPDATE_PERIOD_CYCLES > MOTOR_CURRENT_ON_PERIODS) {
      if(ms->AC_current_on) ms->detect_endswitches_from_AC_current = 1;

      if(!ms->detect_endswitches_from_AC_current) printf("WARNING: Motor current sensor detected no current in Motor %d after it was switched on\n", ms->motor_idx);
   }

   if(ms->detect_endswitches_from_AC_current) {
     // current was detected after motor was switched on.
     // Now detect when Endswitch is switching current back off
     if(ms->state == OPENING && !ms->AC_current_on) {
        printf("Motor %d Current was switched off whilst Opening -> Open Endswitch triggered\n", ms->motor_idx);
        unsigned mimick_endswitch_value = ES_TRIGGERED == 1 ? ENDSWITCH_OPEN : (~ENDSWITCH_OPEN)&0x3; 
        check_endswitches_and_update_states(mimick_endswitch_value, ms, reg, 0);
     } else if(ms->state == CLOSING && !ms->AC_current_on) {
        printf("Motor %d Current was switched off whilst Closing -> Closed Endswitch triggered\n", ms->motor_idx);
        unsigned mimick_endswitch_value = ES_TRIGGERED == 1 ? ENDSWITCH_CLOSED : (~ENDSWITCH_CLOSED)&0x3; 
        check_endswitches_and_update_states(mimick_endswitch_value, ms, reg, 0);
     }
   }
}
#endif

// global shared memory
motor_state_s state_m0, state_m1;

void mc_control(client register_if reg, chanend flash_c) {

    timer tmr_pos;
    timer tmr_dbc; // debounce tiemr for p_control_buttons
    timer tmr_dbc_es;  

    int t_dbc, t_dbc_es, t_pos;  // time variables

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


#if ENABLE_INTERNAL_PULLS
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
#endif

    int m1_pos = INT_MIN; // invalid
    int m0_pos = INT_MIN; // ivalid

    char command = 1;
    char result;
    flash_c <: command;
    flash_c :> result;
    if(result != 0) {
       printf("Flash access Error\n");
    } else {
      // get the data
      char byte_buffer[FLASH_DATA_BYTES];
      for(int i=0; i<FLASH_DATA_BYTES; ++i) {
        flash_c :> byte_buffer[i];
      }
      // Init Motor 0 position

      if(byte_buffer[0] == FLASH_DATA_VALID_BYTE) {
        memcpy(&m0_pos, &byte_buffer[1], sizeof(int));
        if(position_in_range(m0_pos)) {
          printf("Found valid position for Motor 0: %d um\n", m0_pos);          
        } else {
          printf("Error: Position in flash is out of range for Motor 0: %d um\n", m0_pos);
          m0_pos = INT_MIN;
        }

      } 
      #if !ENDSWITCHES_CONNECTED || INFER_ENDSWITCHES_WITH_AC_SENSOR
      else {
        m0_pos = CLOSED_POS_ES;
        printf("Endswitches not connected. Initialising Motor 0 to arbitrary position %d mm\n", UM_to_MM(m0_pos));
      }
      #endif
    
      // Init Motor 1 position
      if(byte_buffer[MOTOR_FLASH_AREA_SIZE] == FLASH_DATA_VALID_BYTE) {
        memcpy(&m1_pos, &byte_buffer[MOTOR_FLASH_AREA_SIZE+1], sizeof(int));
        if(position_in_range(m1_pos)) {
          printf("Found valid position for Motor 1: %d um\n", m1_pos);          
        } else {
          printf("Error: Position in flash is out of range for Motor 1: %d um\n", m1_pos);
          m1_pos = INT_MIN;
        }
      } 
      #if !ENDSWITCHES_CONNECTED || INFER_ENDSWITCHES_WITH_AC_SENSOR
      else {
        m1_pos = CLOSED_POS_ES;
        printf("Endswitches not connected. Initialising Motor 1 to arbitrary position %d mm\n", UM_to_MM(m1_pos));
      }
      #endif
    }


    // Is this needed??
    p_control_buttons :> prev_control_buttons_val;
    p_endswitches :> prev_endswitches_val;

    unsigned endswitches_m0 = prev_endswitches_val & 0b11;
    unsigned endswitches_m1 = (prev_endswitches_val >> 2) & 0b11;

    // Init!!
    tmr_pos :> t_pos;  // init position update time

    init_motor_state(&state_m0, reg, m0_pos, 15800, 15600, endswitches_m0, 0, t_pos);
    init_regs(&state_m0, reg);
    init_motor_state(&state_m1, reg, m1_pos, 15800, 15600, endswitches_m1, 1, t_pos);
    init_regs(&state_m1, reg);

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
                  unsigned pos_um = CM_to_UM(value);
                  state_m0.target_position = pos_um;
                  printf("I2C command setting new target_position %d mm for Motor 0\n", UM_to_MM(pos_um));
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
                  unsigned pos_um = CM_to_UM(value);
                  state_m1.target_position = pos_um;
                  printf("I2C command setting new target_position %d mm for Motor 1\n", UM_to_MM(pos_um));
                  break;
                }
              }
              break;
            
            // Monitor control buttons
            case (!buttons_changed) => p_control_buttons when pinsneq(prev_control_buttons_val) :> prev_control_buttons_val:
              // Port value changed which means some button was pressed or released
              if(DEBUG_BUTTON_LOGIC) printf("p_control_buttons port value changed to 0x%x\n", prev_control_buttons_val);
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

              } else {
                if(DEBUG_BUTTON_LOGIC) printf("p_control_buttons value change ignored. Value 0x%x didn't persist after DEBOUNCE_TIME and was a glith\n", prev_control_buttons_val);
              }
              break;

#if !INFER_ENDSWITCHES_WITH_AC_SENSOR            
            // Endswitches are connected with a cable
            // Monitor Endswitches
            case (!endswitches_changed) => p_endswitches when pinsneq(prev_endswitches_val) :> prev_endswitches_val:
              // Port value changed which means some switch was pressed or released
              printf("Endswitches port changed event. new value 0x%x\n", prev_endswitches_val);
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
#endif

            // Position estimation
            case tmr_pos when timerafter(t_pos+POS_UPDATE_PERIOD_CYCLES) :> t_pos:
#if INFER_ENDSWITCHES_WITH_AC_SENSOR
              // Endswitch value is inferred using AC Sensor
              //for(unsigned c=0; c<NUM_ADC_CHANNELS; ++c) {
              //    printf("ADC channel %d value: 0x%x\n", c, get_rms_current(c));
              //}
              update_motor_current_state(&state_m0);
              update_motor_current_state(&state_m1);
#endif
              
#if INFER_ENDSWITCHES_WITH_AC_SENSOR
              if(state_m0.state == OPENING || state_m0.state == CLOSING) {
                  monitor_motor_current(&state_m0, reg);
              }
              // Note: monitor_motor_current can change motor state
#endif

              if(state_m0.state == OPENING || state_m0.state == CLOSING) {    
                  int pos_change;
                  if(state_m0.state == OPENING) {
                    pos_change = - (int64_t) state_m0.opening_speed * POS_UPDATE_PERIOD_CYCLES / (XS1_TIMER_HZ); 
                    //state_m0.position -= pos_change;
                    printf("Motor 0 is opening. ");
                  } else {
                    pos_change =  (int64_t) state_m0.opening_speed * POS_UPDATE_PERIOD_CYCLES / (XS1_TIMER_HZ); 
                    printf("Motor 0 is closing. ");
                  }
                  state_m0.position += pos_change;
                  printf("Updated position estimate to %d mm using pos_change %d um\n", UM_to_MM(state_m0.position), pos_change);
                  update_position_regs(&state_m0, reg);
                  check_and_handle_new_pos(&state_m0, reg);

                  if(t_pos - state_m0.time_of_last_flash_update >= FLASH_UPDATE_PERIOD_CYCLES) {
                     state_m0.update_flash = 1;
                     state_m0.time_of_last_flash_update = t_pos;
                  }
              }; 
#if INFER_ENDSWITCHES_WITH_AC_SENSOR
              if(state_m1.state == OPENING || state_m1.state == CLOSING) {
                  monitor_motor_current(&state_m1, reg);
              }
              // Note: monitor_motor_current can change motor state
#endif
              if(state_m1.state == OPENING || state_m1.state == CLOSING) {  
                  int pos_change;
                  if(state_m1.state == OPENING) {
                    pos_change = - (int64_t) state_m1.opening_speed * POS_UPDATE_PERIOD_CYCLES / (XS1_TIMER_HZ); 
                    //state_m1.position -= pos_change;
                    printf("Motor 1 is opening. ");
                  } else {
                    pos_change =  (int64_t) state_m1.opening_speed * POS_UPDATE_PERIOD_CYCLES / (XS1_TIMER_HZ); 
                    printf("Motor 1 is closing. ");
                  }
                  state_m1.position += pos_change;
                  printf("Updated position estimate to %d mm using pos_change %d\n", UM_to_MM(state_m1.position), pos_change);
                  update_position_regs(&state_m1, reg);
                  check_and_handle_new_pos(&state_m1, reg);

                  if(t_pos - state_m1.time_of_last_flash_update >= FLASH_UPDATE_PERIOD_CYCLES) {
                     state_m1.update_flash = 1;
                     state_m1.time_of_last_flash_update = t_pos;
                  }
              }; 
              // Use this timer event to update LEDs
              // toggle LED for to show motor controller is active and display the position update frequency
              led_val = 1-led_val;
              p_led <: (led_val << 3); // Green LED is on P4F3
              // Update pushbutton LEDs. 
              update_pushbutton_leds(&state_m0);
              update_pushbutton_leds(&state_m1);
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
  chan flash_c;
  //debug_printf("Starting I2C enabled Motoro Controller\n");

#if ACCESS_ADC_VIA_SPI
  interface spi_master_if i_spi[1];
#endif

  par {
    on MC_TILE : i2c_control(i_reg);
    on MC_TILE : mc_control(i_reg, flash_c);
    on MC_TILE : flash_server(flash_c);

#if ACCESS_ADC_VIA_SPI
    on MC_TILE: spi_app(i_spi[0]);
    on MC_TILE: spi_master(i_spi, 1,
                         p_sclk, p_mosi, p_miso, p_ss, 1,
                         null);
#endif
  }

  return 0;
}
