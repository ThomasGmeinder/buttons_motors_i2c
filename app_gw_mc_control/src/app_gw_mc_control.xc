/*
 * app_gw_mc_control.xc
 *
 *  Created on: 12 Mar 2017
 *      Author: thomas
 */

// Control Specification
// Inputs and actions
// - User presses open button whilst motors stopped. A: start_motor OPENING with target position OPEN_POS
// - User presses open button again whilst motors run. A: stop_motor
// - User presses close button once: A: start_motor CLOSING with target position CLOSED_POS
// - User presses close button again whilst motors run. A: stop_motor
// - Web server sends close command with a new target position. A: start_motor CLOSING with target position X
// - Web server sends open command with a new target position. A: start_motor OPENING with target position X
// Note: When a I2C command comes in from the server whilst the motor is running, then ignore it


// Todo:
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
#include <i2c.h>
#include "i2c_app.h"
#include "debug_print.h"

#define TEST_MODE 1

#define MOTOR_SPEED  10 // in mm/s
#define MAX_POS 100 // max position in mm
#define OPEN_POS 0
#define OPEN_TOLERANCE 10 
#define CLOSED_TOLERANCE 10 
#define CLOSED_POS MAX_POS
#define DEBOUNCE_TIME XS1_TIMER_HZ/50  // 20ms
#define POS_UPDATE_PERIOD XS1_TIMER_HZ/10 // 100ms
#define DEBOUNCE 1

#define ES_TRIGGERED 1 // End Switch triggered
#define BUTTON_PRESSED 1 // End Switch triggered
#define MOTOR_CLOSING_DIR 1
#define MOTOR_OPENING_DIR (!MOTOR_CLOSING_DIR)

#define MC_TILE tile[0]

// Login raspiviv
// raspiviv
// nX9NypBk


/** Inputs **/
// Motor 1 closed endswitch
on MC_TILE : in port p_es_m1_closed = XS1_PORT_1L;  // 
// Motor 2 closed endswitch
on MC_TILE : in port p_es_m2_closed = XS1_PORT_1K;  // 

// Motor 1 open endswitch
on MC_TILE : in port p_es_m1_open = XS1_PORT_1J;  // 
// Motor 1 open endswitch
on MC_TILE : in port p_es_m2_open = XS1_PORT_1I;  // 

// Todo: Move this to a 4-bit port. There are no more 1-bit ports 
// Button to close the ventilation (both motors)
on MC_TILE : in port p_close_button = XS1_PORT_1H;  // Button to close ventilation (move up)
// Button to open the ventilation (both motors)
on MC_TILE : in port p_open_button = XS1_PORT_1G;  // Button to close ventilation (move down)

/** Outputs **/
// Motor 1 on/off
on MC_TILE : out port p_m1_on = XS1_PORT_1F;  // Switch motor 1 on or off // X0D13
// Motor 1 direction
on MC_TILE : out port p_m1_dir = XS1_PORT_1E;  // Select motor 1 direction // X0D12

// Motor 2 on/off
on MC_TILE : out port p_m2_on = XS1_PORT_1P;  // Switch motor 2 on or off // X0D39
// Motor 2 direction
on MC_TILE : out port p_m2_dir = XS1_PORT_1O;  // Select motor 2 direction // X0D38

on MC_TILE : port p_slave_scl = XS1_PORT_1M; // X0D36 // connect to GPIO 3 on rPI
on MC_TILE : port p_slave_sda = XS1_PORT_1N; // X0D37 // connect to GPIO 2 on rPI

on MC_TILE : port p_led = XS1_PORT_4F;

typedef enum {
    OPENING,
    CLOSING,
    STOPPED,
    POS_UNKNOWN,
    ERROR
} motor_state_t;

typedef enum {
    I2C = 0,
    BUTTON = 1,
} trigger_t;

// struct for motor state
typedef struct {
    int position; // in mm from 0 (open) to MAX_POS (closed)
    int target_position; // the target position
    unsigned motor_idx;
    trigger_t trigger;

    motor_state_t state;
} motor_state_s;

void init_motor_state(motor_state_s* ms, unsigned es_open_val, unsigned es_closed_val, unsigned motor_idx) {
    ms->motor_idx = motor_idx;
    ms->trigger = BUTTON;

    // Determine state and position based on the End switches
    if(es_open_val == ES_TRIGGERED && es_closed_val == ES_TRIGGERED) {
       #if TEST_MODE
       ms->state = STOPPED;
       ms->position = CLOSED_POS; // invalid
       ms->target_position = CLOSED_POS; // safer to have it closed
       #else
       // Both switches active is eror state
       printf("ERROR: Both endswitches are on -> invalid\n");
       ms->state = ERROR;
       ms->position = -1; // invalid
       ms->target_position = CLOSED_POS; // safer to have it closed
       #endif
    } else if(es_open_val == !ES_TRIGGERED && es_closed_val == !ES_TRIGGERED) {
       #if TEST_MODE
       ms->state = STOPPED;
       ms->position = CLOSED_POS; // invalid
       ms->target_position = CLOSED_POS; // safer to have it closed
       #else 
       printf("ERROR: Both endswitches are off -> unknown position somewhere between OPEN_POS and CLOSED_POS\n");
       ms->state = POS_UNKNOWN;
       // Todo: Handle this case:
       ms->position = -1; // invalid
       ms->target_position = CLOSED_POS; // safer to have it closed
       #endif
    } else if(es_open_val == ES_TRIGGERED) {
       ms->position = OPEN_POS;
       ms->target_position = OPEN_POS;
       ms->state = STOPPED;
    } else if(es_closed_val == ES_TRIGGERED) {
       ms->position = CLOSED_POS;
      ms->target_position = CLOSED_POS;
       ms->state = STOPPED;
    } 
}

int target_pos_reached(motor_state_s* ms) {
   if(ms->state == CLOSING) {
     if(ms->position >= ms->target_position) {
       printf("Closing ventilation %d reached target position %d\n", ms->motor_idx, ms->target_position);
       return 1;
     } 
   } 
   if(ms->state == OPENING) {
     if(ms->position <= ms->target_position) {
       printf("Opening ventilation %d reached target position %d\n", ms->motor_idx, ms->target_position);
       return 1;
     } 
   }
   return 0;
}

int pos_below_min(motor_state_s* ms) {
    if(ms->position <= OPEN_POS-OPEN_TOLERANCE) {
       printf("WARNING: position estimate %d mm of Motor %d is below minimum %d mm\n", ms->position, ms->motor_idx, OPEN_POS-OPEN_TOLERANCE);
       return 1;
    }
    return 0;
}
int pos_above_max(motor_state_s* ms) {
    if(ms->position >= CLOSED_POS+CLOSED_TOLERANCE) {
       printf("WARNING: position estimate %d mm of Motor %d is above max %d mm\n", ms->position, ms->motor_idx, CLOSED_POS+CLOSED_TOLERANCE);
       return 1;
    }
    return 0;
}

// Update the Read only registers
void update_regs(motor_state_s* ms, client register_if reg) {
  int base_reg = ms->motor_idx * 4;
  reg.set_register(base_reg+1, ms->target_position);
  reg.set_register(base_reg+2, ms->position);
  reg.set_register(base_reg+3, ms->trigger);  // BUTTON == 1 which means server_changed_motor_position
}

int stop_motor(out port motor_on, motor_state_s* ms, client register_if reg) {
    motor_on <: 0;
    ms->state = STOPPED;
    update_regs(ms, reg);
    printf("Stopped Motor %u at position %d mm\n", ms->motor_idx, ms->position);
}

int start_motor(motor_state_s* ms, client register_if reg, trigger_t trigger) {

  ms->trigger = trigger;

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
    p_m1_dir <: dir_val;
    p_m1_on <: 1;
  } else {
    p_m2_dir <: dir_val;
    p_m2_on <: 1;  
  }

  update_regs(ms, reg);
  return 0;
}

int motor_moved_by_button(motor_state_s* ms, client register_if reg) {
  if(ms->state == OPENING || ms->state == CLOSING) {
    unsigned status_reg = ms->motor_idx*4 + 3;
    trigger_t trigger = (trigger_t) reg.get_register(status_reg);
    if(trigger == BUTTON) {
      return 1;
    }
  }
  return 0; 
}

void mc_control(client register_if reg) {

    timer tmr_pos;
    timer tmr_dbc;  // debounce tiemr for p_close_button
    timer tmr_dbo;  // debounce timer for p_open_button
    int t_dbc, t_dbo, t_pos;
    motor_state_s state_m0, state_m1;
    unsigned close_button_pressed=0, open_button_pressed=0;
    unsigned prev_open_button_val, prev_close_button_val;
    unsigned prev_es_m1_open_val, prev_es_m1_closed_val, prev_es_m2_open_val, prev_es_m2_closed_val;
    unsigned led_val = 0;

    printf("Starting Greenhouse Motor Control Application\n\n");
    printf("Motor speed is set to %u mm/s\n", MOTOR_SPEED);

    // read the endswitches and init motor states
    // Note: Had to move endswitch detection to another task to workaround compiler bug
    unsigned es_m1_open_val, es_m1_closed_val, es_m2_open_val, es_m2_closed_val;

    p_es_m1_open :> es_m1_open_val;
    p_es_m1_closed :> es_m1_closed_val;
    p_es_m2_open :> es_m2_open_val;
    p_es_m2_closed :> es_m2_closed_val;

    init_motor_state(&state_m0, es_m1_open_val, es_m1_closed_val, 0);
    update_regs(&state_m0, reg);
    init_motor_state(&state_m1, es_m2_open_val, es_m2_closed_val, 1);
    update_regs(&state_m1, reg);

    tmr_pos :> t_pos;  // init position update time

    while(1) {
        // input from all input ports and store in prev values
        // prev values are instrumental in the case guards to detect the desired edge
        p_close_button :> prev_close_button_val;
        p_open_button :> prev_open_button_val;
        p_es_m1_open :> prev_es_m1_open_val;
        p_es_m1_closed :> prev_es_m1_closed_val;
        p_es_m2_open :> prev_es_m2_open_val;
        p_es_m2_closed :> prev_es_m2_closed_val;

        // event handlers
        select {
            // I2C register changed logic.
            // Note: This does not cause ET_ILLEGAL_RESOURCE!
            case reg.register_changed():
              unsigned regnum = reg.get_changed_regnum();
              unsigned value = reg.get_register(regnum);

              switch(regnum) {
                case 0: {
                  // To avoid race condition, ignore I2C command from remote client if motor_moved_by_button
                  if(!motor_moved_by_button(&state_m0, reg)) { 
                    state_m0.state = (value & 0x7);
                    start_motor(&state_m0, reg, I2C);
                  } else {
                    printf("Ignoring command from remote client whilst motor 0 is operated by button\n");
                  }
                  break;
                }
                case 1: {
                  state_m0.target_position = value;
                  break;
                }
                case 4: {
                  // To avoid race condition, ignore I2C command from remote client if motor_moved_by_button
                  if(!motor_moved_by_button(&state_m1, reg)) { // Ignore I2C command if motor_moved_by_button
                    state_m1.state = (value & 0x7);
                    start_motor(&state_m1, reg, I2C);
                  } else {
                    printf("Ignoring command from remote client whilst motor 1 is operated by button\n");
                  }
                  break;
                }
                case 5: {
                  state_m1.target_position = value;
                  break;
                }
              }
              break;
            
            // rising edge on p_close_button 
            case (prev_close_button_val != BUTTON_PRESSED) => p_close_button when pinseq(BUTTON_PRESSED) :> prev_close_button_val:
              close_button_pressed = 1;
              tmr_dbc :> t_dbc;
#if DEBOUNCE
              break;

            // debounce p_close_button
            //case close_button_pressed => tmr_dbc when timerafter(t_dbc+DEBOUNCE_TIME) :> void:
            case close_button_pressed => tmr_dbc when timerafter(t_dbc+DEBOUNCE_TIME) :> t_dbc:
#endif
              unsigned close_button_val;
              p_close_button :> close_button_val;
              if(close_button_val == BUTTON_PRESSED) { // button is still pressed!
                if(state_m0.state == CLOSING) {
                  // close button pressed again whilst closing -> switch off
                  printf("p_close_button was pressed whilst Motor 1 was already closing -> Stop Motor 1\n");
                  state_m0.target_position = state_m0.position;
                  state_m0.trigger = BUTTON; // This is key to update the client if caused start_motor via I2C
                  stop_motor(p_m1_on, &state_m0, reg);
                } else if(state_m0.state == STOPPED && state_m0.position == CLOSED_POS) {
                  printf("Motor 1 is already in closed position\n");  
                  // do nothing
                } else {
                  printf("p_close_button was pressed first time -> Switch Motor 1 on in closing direction from position %d mm\n", state_m0.position);
                  state_m0.state = CLOSING;
                  state_m0.target_position = CLOSED_POS;
                  start_motor(&state_m0, reg, BUTTON);
                }
                if(state_m1.state == CLOSING) {
                  // close button pressed again whilst closing -> switch off
                  printf("p_close_button was pressed whilst Motor 2 was already closing -> Stop Motor 2\n");
                  state_m1.target_position = state_m1.position;
                  state_m1.trigger = BUTTON; // This is key to update the client if caused start_motor via I2C
                  stop_motor(p_m2_on, &state_m1, reg);

                } else if(state_m1.state == STOPPED && state_m1.position == CLOSED_POS) {
                  printf("Motor 2 is already in closed position\n");  
                  // do nothing
                } else {
                  printf("p_close_button was pressed first time -> Switch Motor 2 on in closing direction from position %d mm\n", state_m1.position);
                  state_m1.state = CLOSING;
                  state_m1.target_position = CLOSED_POS;
                  start_motor(&state_m1, reg, BUTTON);
                }
              }
              close_button_pressed = 0; // reset to re-activate case !close_button_pressed =>  
              break;

            // rising edge on p_open_button
            //case prev_open_button_val != BUTTON_PRESSED => p_open_button when pinseq(BUTTON_PRESSED) :> prev_open_button_val:
            case (prev_open_button_val != BUTTON_PRESSED) => p_open_button when pinseq(BUTTON_PRESSED) :> prev_open_button_val:
              // rising edge detected
              open_button_pressed = 1;
              tmr_dbo :> t_dbo;
#if DEBOUNCE
              break;

            // debounce p_open_button
            case open_button_pressed => tmr_dbo when timerafter(t_dbo+DEBOUNCE_TIME) :> void:
#endif
              unsigned open_button_val;
              p_open_button :> open_button_val;
              if(open_button_val == BUTTON_PRESSED) { // button is still pressed!
                if(state_m0.state == OPENING) {
                  // open button pressed again whilst closing -> switch off
                  printf("p_open_button was pressed whilst Motor 1 was already opening -> Stop Motor 1\n");
                  state_m0.target_position = state_m0.position;
                  state_m0.trigger = BUTTON; // This is key to update the client if caused start_motor via I2C
                  stop_motor(p_m1_on, &state_m0, reg);
                } else if(state_m0.state == STOPPED && state_m0.position == OPEN_POS) {
                  printf("Motor 1 is already in open position\n");  
                  // do nothing
                } else {
                  printf("p_open_button was pressed first time -> Switch Motor 1 on in opening direction from position %d mm\n", state_m0.position);
                  state_m0.state = OPENING;
                  state_m0.target_position = OPEN_POS;
                  start_motor(&state_m0, reg, BUTTON);
                }

                if(state_m1.state == OPENING) {
                  // open button pressed again whilst closing -> switch off
                  printf("p_open_button was pressed whilst Motor 2 was already opening -> Stop Motor 2\n");
                  state_m1.target_position = state_m1.position;
                  state_m1.trigger = BUTTON; // This is key to update the client if caused start_motor via I2C
                  stop_motor(p_m2_on, &state_m1, reg);
                } else if(state_m1.state == STOPPED && state_m1.position == OPEN_POS) {
                  printf("Motor 2 is already in open position\n");  
                  // do nothing
                } else {
                  printf("p_open_button was pressed first time -> Switch Motor 2 on in opening direction from position %d mm\n", state_m1.position);
                  state_m1.state = OPENING;
                  state_m1.target_position = OPEN_POS;
                  start_motor(&state_m1, reg, BUTTON);
                }
              }
              open_button_pressed = 0; // reset to re-activate case !open_button_pressed =>  
              break;


            case tmr_pos when timerafter(t_pos+POS_UPDATE_PERIOD) :> t_pos:
              unsigned pos_change =  MOTOR_SPEED * POS_UPDATE_PERIOD / XS1_TIMER_HZ; 

              if(state_m0.state == OPENING || state_m0.state == CLOSING) {
                  if(state_m0.state == OPENING) state_m0.position -= pos_change;
                  else state_m0.position += pos_change;
                  update_regs(&state_m0, reg);
                  //printf("Motor 2 is opening. Updated position estimate to %d mm\n", state_m0.position);
                  if(target_pos_reached(&state_m0)) {
                    stop_motor(p_m1_on, &state_m0, reg);
                  }
              }; 
              if(state_m1.state == OPENING || state_m1.state == CLOSING) {
                  if(state_m1.state == OPENING) state_m1.position -= pos_change;
                  else state_m1.position += pos_change;
                  update_regs(&state_m1, reg);
                  //printf("Motor 2 is opening. Updated position estimate to %d mm\n", state_m1.position);
                  if(target_pos_reached(&state_m1)) {
                    stop_motor(p_m2_on, &state_m1, reg);
                  }
              }; 
              //Todo: if(pos_out_of_range(state_m1.position)) {

              // Use this timer event to toggle LED for activity detection
              led_val = 1-led_val;
              p_led <: (led_val << 3); // Green LED is on P4F3
              break;


            // Todo: Debounce and re-activate. Without deboune they will constantly trigger
            // Todo: This has to be edge sensitive. Only transition 0 to 1 must trigger
            // p_es_m1_open triggered
            case (prev_es_m1_open_val != ES_TRIGGERED) => p_es_m1_open when pinseq(ES_TRIGGERED) :> void:
              state_m0.position = OPEN_POS;
              stop_motor(p_m1_on, &state_m0, reg);
              break;

            // p_es_m1_closed triggered
            case (prev_es_m1_closed_val != ES_TRIGGERED) => p_es_m1_closed when pinseq(ES_TRIGGERED) :> void:
              state_m0.position = CLOSED_POS;
              stop_motor(p_m1_on, &state_m0, reg);
              break;

            // p_es_m2_open triggered
            case (prev_es_m2_open_val != ES_TRIGGERED) => p_es_m2_open when pinseq(ES_TRIGGERED) :> void:
              state_m1.position = OPEN_POS;
              stop_motor(p_m2_on, &state_m1, reg);
              break;

            // p_es_m2_closed triggered
            case (prev_es_m2_closed_val != ES_TRIGGERED) => p_es_m2_closed when pinseq(ES_TRIGGERED) :> void:
              state_m1.position = CLOSED_POS;
              stop_motor(p_m2_on, &state_m1, reg);
              break;


        }
    }
}


uint8_t device_addr = 0x3c;

int main() {

  i2c_slave_callback_if i_i2c;
  register_if i_reg;
 
  //debug_printf("Starting I2C enabled Motoro Controller\n");

    par {
        on MC_TILE : i2c_slave_register_file(i_i2c, i_reg);
        on MC_TILE : i2c_slave(i_i2c, p_slave_scl, p_slave_sda, device_addr);

        on MC_TILE : mc_control(i_reg);

    }
    return 0;
}
