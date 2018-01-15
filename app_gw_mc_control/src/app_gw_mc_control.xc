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
#include <i2c.h>
#include "i2c_app.h"
#include "debug_print.h"

#define TEST_MODE 1

#define MOTOR_SPEED 10 // in mm/s
#define MAX_POS 100 // max position in mm
#define OPEN_POS 0
#define OPEN_TOLERANCE 10 
#define CLOSED_TOLERANCE 10 
#define CLOSED_POS MAX_POS
#define DEBOUNCE_TIME XS1_TIMER_HZ/50  // 20ms
#define POS_UPDATE_PERIOD XS1_TIMER_HZ/10 // 100ms
#define DEBOUNCE 1

#define ES_TRIGGERED 1 // End Switch triggered. Connects between brown and blue from Pin to VCC
#define BUTTON_PRESSED 1 // Button pressed. 

// Position index of buttons on the 4-bit port
#define CLOSE_BUTTON_0_IDX 0
#define OPEN_BUTTON_0_IDX 1
#define CLOSE_BUTTON_1_IDX 2
#define OPEN_BUTTON_1_IDX 3

// Position index of buttons on the 4-bit port
#define ENDSWITCH_0_OPEN_IDX 0
#define ENDSWITCH_0_CLOSED_IDX 1
#define ENDSWITCH_1_OPEN_IDX 2
#define ENDSWITCH_1_CLOSED_IDX 3

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
// Motor 1 closed endswitch
on MC_TILE : in port p_es_m1_closed = XS1_PORT_1L;  // X0D35
// Motor 2 closed endswitch
on MC_TILE : in port p_es_m2_closed = XS1_PORT_1K;  // X0D34

// Motor 1 open endswitch
on MC_TILE : in port p_es_m1_open = XS1_PORT_1J;    // X0D25
// Motor 2 open endswitch
on MC_TILE : in port p_es_m2_open = XS1_PORT_1I;    // X0D24

on MC_TILE : in port p_endswitches = XS1_PORT_4C;   // X0D14 (pin 0), X0D15, X0D20, X0D21 (pin 3)
//on MC_TILE : in port p_endswitches = XS1_PORT_4E;  

// Todo: Move this to a 4-bit port. There are no more 1-bit ports 
// Button to open and close the ventilations
on MC_TILE : in port p_control_buttons = XS1_PORT_4D;  
//on MC_TILE : in port p_control_buttons = XS1_PORT_4E;  

/** Outputs **/
// Motor 1 on/off
on MC_TILE : out port p_m1_on = XS1_PORT_1F;  // X0D13
// Motor 1 direction
on MC_TILE : out port p_m1_dir = XS1_PORT_1E;  // X0D12

// Motor 2 on/off
on MC_TILE : out port p_m2_on = XS1_PORT_1P;  // X0D39
// Motor 2 direction
on MC_TILE : out port p_m2_dir = XS1_PORT_1O;  // X0D38

on MC_TILE : port p_slave_scl = XS1_PORT_1M; // X0D36 // connect to GPIO 3 on rPI
on MC_TILE : port p_slave_sda = XS1_PORT_1N; // X0D37 // connect to GPIO 2 on rPI

on MC_TILE : port p_led = XS1_PORT_4F;

on MC_TILE : port p_m1_pushbutton_leds = XS1_PORT_4A;
on MC_TILE : port p_m2_pushbutton_leds = XS1_PORT_4E;

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
} actuator_t;

// struct for motor state
typedef struct {
    int position; // in mm from 0 (open) to MAX_POS (closed)
    int target_position; // the target position
    unsigned motor_idx;
    actuator_t actuator;

    motor_state_t state;
} motor_state_s;

void delay_us(unsigned time_us) {
  int t;
  timer tmr;
  tmr :> t;
  tmr when timerafter(t+time_us*100) :> t;
}

void init_motor_state(motor_state_s* ms, unsigned es_open_val, unsigned es_closed_val, unsigned motor_idx) {
    ms->motor_idx = motor_idx;
    ms->actuator = BUTTON;

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
  reg.set_register(base_reg+MOTOR_TARGET_POS_REG_OFFSET,  ms->target_position);
  reg.set_register(base_reg+MOTOR_CURRENT_POS_REG_OFFSET, ms->position);
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position
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
     p_m1_pushbutton_leds <: led_on_mask;
   } else {
     p_m2_pushbutton_leds <: led_on_mask;
   }
}
int stop_motor(out port motor_on, motor_state_s* ms, client register_if reg) {
    motor_on <: MOTOR_OFF;
    ms->state = STOPPED;

    upate_pushbutton_leds(ms);
    update_position_regs(ms, reg);
    // register stop event
    int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
    reg.set_register(base_reg, ms->state); 
    reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, STOP);
    reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

    printf("Stopped Motor %u at position %d mm\n", ms->motor_idx, ms->position);
}

void check_motor_state(motor_state_s* ms, motor_state_t state, int actual_position) {
   // Todo: Check OPEN_TOLERANCE, CLOSED_TOLERANCE, motor state
   //int diff = ms.position - actual_position;
   //if(diff < 0) diff = -diff;
   //if(diff > tolerance) // update error register so it can be read by Server
}

void check_endswitches(unsigned endswitches_val) {
   printf("Checking endswitches portval 0x%x\n", endswitches_val);  
   // Todo: Check that open and close endswitches are not triggered simultaneously, etc.
}

int start_motor(motor_state_s* ms, client register_if reg, actuator_t actuator) {

  ms->actuator = actuator;

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
    delay_us(10000); // delay 10ms to make sure the big capacitor is connected when motor is switched on
    p_m1_on <: MOTOR_ON;
  } else {
    p_m2_dir <: dir_val;
    delay_us(10000); // delay 10ms to make sure the big capacitor is connected when motor is switched on
    p_m2_on <: MOTOR_ON;  
  }

  upate_pushbutton_leds(ms);
  update_position_regs(ms, reg);
  // register start event
  int base_reg = ms->motor_idx * NUM_REGS_PER_MOTOR;
  reg.set_register(base_reg, ms->state); 
  reg.set_register(base_reg+MOTOR_EVENT_REG_OFFSET, START);
  reg.set_register(base_reg+MOTOR_ACTUATOR_REG_OFFSET, ms->actuator);  // BUTTON == 1 which means server_changed_motor_position

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

unsigned button_pressed(unsigned button_index, unsigned portval) {
  return ((portval >> button_index) & 1) == BUTTON_PRESSED;
}

unsigned endswitch_triggered(unsigned endswitch_index, unsigned portval) {
  return ((portval >> endswitch_index) & 1) == ES_TRIGGERED;
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
    init_regs(&state_m0, reg);
    init_motor_state(&state_m1, es_m2_open_val, es_m2_closed_val, 1);
    init_regs(&state_m1, reg);

    // Init!!
    tmr_pos :> t_pos;  // init position update time

    // Is this needed??
    p_control_buttons :> prev_control_buttons_val;
    //p_endswitches :> prev_endswitches_val;

    while(1) {
        // input from all input ports and store in prev values
        // prev values are instrumental in the case guards to detect the desired edge

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
                if(button_pressed(CLOSE_BUTTON_0_IDX, control_buttons_val)) {
                  if(state_m0.state == CLOSING) {
                    // close button pressed again whilst closing -> switch off
                    printf("p_close_button was pressed whilst Motor 1 was already closing -> Stop Motor 1\n");
                    state_m0.target_position = state_m0.position;
                    state_m0.actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
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
                // use else if to give close button the priority
                } else if(button_pressed(OPEN_BUTTON_0_IDX, control_buttons_val)) {
                  if(state_m0.state == OPENING) {
                    // open button pressed again whilst closing -> switch off
                    printf("p_open_button was pressed whilst Motor 1 was already opening -> Stop Motor 1\n");
                    state_m0.target_position = state_m0.position;
                    state_m0.actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
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
                }

                if(button_pressed(CLOSE_BUTTON_1_IDX, control_buttons_val)) {  
                  if(state_m1.state == CLOSING) {
                    // close button pressed again whilst closing -> switch off
                    printf("p_close_button was pressed whilst Motor 2 was already closing -> Stop Motor 2\n");
                    state_m1.target_position = state_m1.position;
                    state_m1.actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
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
                // use else if to give close button the priority
                } else if(button_pressed(OPEN_BUTTON_1_IDX, control_buttons_val)) {
                  if(state_m1.state == OPENING) {
                    // open button pressed again whilst closing -> switch off
                    printf("p_open_button was pressed whilst Motor 2 was already opening -> Stop Motor 2\n");
                    state_m1.target_position = state_m1.position;
                    state_m1.actuator = BUTTON; // This is key to update the client if caused start_motor via I2C
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
                check_endswitches(endswitches_val); // Check that Open and Closed are not triggered at the same time
                if(endswitch_triggered(ENDSWITCH_0_OPEN_IDX, endswitches_val)) {
                  check_motor_state(&state_m0, OPENING, OPEN_POS);
                  printf("Motor 1 open endswitch triggered\n");
                  state_m0.position = OPEN_POS;
                  state_m0.target_position = state_m0.position;
                  state_m0.actuator = BUTTON; 
                  stop_motor(p_m1_on, &state_m0, reg);
                }
                if(endswitch_triggered(ENDSWITCH_0_CLOSED_IDX, endswitches_val)) {
                  check_motor_state(&state_m0, CLOSING, CLOSED_POS);
                  printf("Motor 1 closed endswitch triggered\n");
                  state_m0.position = OPEN_POS;
                  state_m0.target_position = state_m0.position;
                  state_m0.actuator = BUTTON; 
                  stop_motor(p_m1_on, &state_m0, reg);
                }
                if(endswitch_triggered(ENDSWITCH_1_OPEN_IDX, endswitches_val)) {
                  check_motor_state(&state_m1, OPENING, OPEN_POS);
                  printf("Motor 2 open endswitch triggered\n");
                  state_m1.position = OPEN_POS;
                  state_m1.target_position = state_m1.position;
                  state_m1.actuator = BUTTON; 
                  stop_motor(p_m2_on, &state_m1, reg);
                }
                if(endswitch_triggered(ENDSWITCH_1_CLOSED_IDX, endswitches_val)) {
                  check_motor_state(&state_m1, CLOSING, CLOSED_POS);
                  printf("Motor 2 closed endswitch triggered\n");
                  state_m1.position = OPEN_POS;
                  state_m1.target_position = state_m1.position;
                  state_m1.actuator = BUTTON; 
                  stop_motor(p_m2_on, &state_m1, reg);
                }
              }
              break;


            // Position estimation
            case tmr_pos when timerafter(t_pos+POS_UPDATE_PERIOD) :> t_pos:
              unsigned pos_change =  MOTOR_SPEED * POS_UPDATE_PERIOD / XS1_TIMER_HZ; 

              if(state_m0.state == OPENING || state_m0.state == CLOSING) {
                  if(state_m0.state == OPENING) state_m0.position -= pos_change;
                  else state_m0.position += pos_change;
                  update_position_regs(&state_m0, reg);
                  //printf("Motor 2 is opening. Updated position estimate to %d mm\n", state_m0.position);
                  if(target_pos_reached(&state_m0)) {
                    stop_motor(p_m1_on, &state_m0, reg);
                  }
              }; 
              if(state_m1.state == OPENING || state_m1.state == CLOSING) {
                  if(state_m1.state == OPENING) state_m1.position -= pos_change;
                  else state_m1.position += pos_change;
                  update_position_regs(&state_m1, reg);
                  //printf("Motor 2 is opening. Updated position estimate to %d mm\n", state_m1.position);
                  if(target_pos_reached(&state_m1)) {
                    stop_motor(p_m2_on, &state_m1, reg);
                  }
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


#if ENDSWITCHES_ON_ONEBIT_PORTS
            // Todo: Add a check that the estimated position is close to the endswitch position
            // Todo: Add a check to make sure that when open endswitch was triggered the state was OPENING  
            // Set state to error in both cases 
            // p_es_m1_open triggered
            case (prev_es_m1_open_val != ES_TRIGGERED) => p_es_m1_open when pinseq(ES_TRIGGERED) :> void:
              printf("Motor 1 open endswitch triggered\n");
              check_motor_state(state_m0, OPENING, OPEN_POS);
              state_m0.position = OPEN_POS;
              state_m0.target_position = state_m0.position;
              state_m0.actuator = BUTTON; 
              stop_motor(p_m1_on, &state_m0, reg);
              break;

            // p_es_m1_closed triggered
            case (prev_es_m1_closed_val != ES_TRIGGERED) => p_es_m1_closed when pinseq(ES_TRIGGERED) :> void:
              printf("Motor 1 closed endswitch triggered\n");  
              check_motor_state(state_m0, OPENING, OPEN_POS);
              state_m0.position = CLOSED_POS;
              state_m0.target_position = state_m0.position;              
              state_m0.actuator = BUTTON; 
              stop_motor(p_m1_on, &state_m0, reg);
              break;

            // p_es_m2_open triggered
            case (prev_es_m2_open_val != ES_TRIGGERED) => p_es_m2_open when pinseq(ES_TRIGGERED) :> void:
              printf("Motor 2 open endswitch triggered\n");  
              state_m1.position = OPEN_POS;
              state_m1.target_position = state_m1.position;
              state_m1.actuator = BUTTON; 
              stop_motor(p_m2_on, &state_m1, reg);
              break;

            // p_es_m2_closed triggered
            case (prev_es_m2_closed_val != ES_TRIGGERED) => p_es_m2_closed when pinseq(ES_TRIGGERED) :> void:
              printf("Motor 2 closed endswitch triggered\n");  
              state_m1.position = CLOSED_POS;
              state_m1.actuator = BUTTON; 
              stop_motor(p_m2_on, &state_m1, reg);
              break;
#endif

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
