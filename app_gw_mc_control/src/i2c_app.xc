// Copyright (c) 2014-2016, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <syscall.h>

#include "i2c.h"
#include "debug_print.h"
#include "string.h"
#include "i2c_app.h"
#include "common.h"

// Todo: Add Error type and value register for each motor
// Errors: UNKNOWN_POSITION, SPEED_ESTIMATION, 
// Todo: Store the last known Motor position in flash.

reg_access_t register_access[NUM_REGISTERS] = {
   RW, // 0    Motor 1 state
   RW, // 1    Motor 1 target position
   R,  // 2    Motor 1 current position
   R,  // 3    Motor 1 actuator
   R,  // 4    Motor 1 event reg
   R,  // 5    Motor 1 error reg
   RW, // 6    Motor 2 state
   RW, // 7    Motor 2 target position
   R,  // 8    Motor 2 current position
   R,  // 9    Motor 2 actuator
   R,  // 10   Motor 2 event reg
   R,  // 11   Motor 2 error reg
   R,  // 12   ID Register
   RW, // 13   Test Register
   R,  // 14   SW Version Register
};


#define STATUS_REG_CHANGED_BIT_IDX 6

unsigned get_motor_reg_idx(unsigned regnum) {
  return regnum % NUM_REGS_PER_MOTOR;
}

[[distributable]]
void i2c_slave_register_file(server i2c_slave_callback_if i2c,
                             server register_if app)
{
  uint8_t registers[NUM_REGISTERS];
  memset(registers, 0, NUM_REGISTERS*sizeof(uint8_t)); // Init regs to 0 to avoid errors from random values
  // Register,  I2C Access, Function

  registers[SW_VERSION_REGISTER] = SW_VERSION;

  // Note: Status register is used to enable handshake between remote client and this program.
  // It is cleared only after the remote client read it to ensure that the event that the motor position was changed locally is not missed

  // This variable is set to -1 if no current register has been selected.
  // If the I2C master does a write transaction to select the register then
  // the variable will be updated to the register the master wants to
  // read/update.
  int current_regnum = -1;
  int changed_regnum = -1;
  while (1) {
    select {

    // Handle application requests to get/set register values.
    case app.set_register(int regnum, uint8_t data):
      if (regnum >= 0 && regnum < NUM_REGISTERS) {
        debug_printf("REGFILE update from appliation: reg[%d] <- %d\n", regnum, data);
        if(get_motor_reg_idx(regnum) == 3) // it is an actuator register
        {
          // store prev actuator in upper 4 bits
          //data |= (registers[regnum] << 4) & 0xF0; 
          //debug_printf("REGFILE updating data to store previous actuator 0x%x\n", data);
          //debug_printf("REGFILE setting changed bit in status register %d\n", regnum);
          //data |= 1<<STATUS_REG_CHANGED_BIT_IDX; // set changed bit in status register
        }
        registers[regnum] = data;
      }
      break;
    case app.get_register(int regnum) -> uint8_t data:
      if (regnum >= 0 && regnum < NUM_REGISTERS) {
        data = registers[regnum];
      } else {
        data = 0;
      }
      break;
    case app.get_changed_regnum() -> unsigned regnum:
      regnum = changed_regnum;
      break;

    // Handle I2C slave transactions
    case i2c.start_read_request(void):
      break;
    case i2c.ack_read_request(void) -> i2c_slave_ack_t response:
      // If no register has been selected using a previous write
      // transaction the NACK, otherwise ACK
      if (current_regnum == -1) {
        response = I2C_SLAVE_NACK;
      } else {
        response = I2C_SLAVE_ACK;
      }
      break;
    case i2c.start_write_request(void):
      break;
    case i2c.ack_write_request(void) -> i2c_slave_ack_t response:
      // Write requests are always accepted
      response = I2C_SLAVE_ACK;
      break;
    case i2c.start_master_write(void):
      break;
    case i2c.master_sent_data(uint8_t data) -> i2c_slave_ack_t response:
      // The master is trying to write, which will either select a register
      // or write to a previously selected register
      if (current_regnum != -1) {
        if(register_access[current_regnum] == RW) {
          registers[current_regnum] = data;
          debug_printf("REGFILE: reg[%d] <- %d\n", current_regnum, data);
  
          // Inform the user application that the register has changed
          changed_regnum = current_regnum;
          app.register_changed();

          response = I2C_SLAVE_ACK;
        } else {
          debug_printf("REGFILE: I2C tried to write Read only register reg[%d]. Responding wtih I2C_SLAVE_NACK\n", current_regnum);
          response = I2C_SLAVE_NACK;
        }
      }
      else {
        if (data < NUM_REGISTERS) {
          current_regnum = data;
          debug_printf("REGFILE: select reg[%d]\n", current_regnum);
          response = I2C_SLAVE_ACK;
        } else {
          response = I2C_SLAVE_NACK;
        }
      }
      break;
    case i2c.start_master_read(void):
      break;
    case i2c.master_requires_data() -> uint8_t data:
      // The master is trying to read, if a register is selected then
      // return the value (other return 0).
      if (current_regnum != -1) {
        data = registers[current_regnum];
        debug_printf("REGFILE: reg[%d] -> %d\n", current_regnum, data);

        if(get_motor_reg_idx(current_regnum) == 5) {
          // clear error register
          registers[current_regnum] = NO_ERROR;
          debug_printf("REGFILE clearing Error event register %d after read\n", current_regnum);

        }
        if(get_motor_reg_idx(current_regnum) == 4)  // Button event register
          { 
            registers[current_regnum] = NO_EVENT; 
            debug_printf("REGFILE clearing button event register %d after read\n", current_regnum);
          }
      } else {
        data = 0;
      }
      break;
    case i2c.stop_bit():
      // The I2C transaction has completed, clear the regnum
      debug_printf("REGFILE: stop_bit\n");
      current_regnum = -1;
      break;
    } // select
  }
}

void slave_application(client register_if reg)
{
  // Invert the data of any register that is written
  while (1) {
    select {
      case reg.register_changed():
        unsigned regnum = reg.get_changed_regnum();
        unsigned value = reg.get_register(regnum);
        if(regnum == 1) {

        }
        debug_printf("SLAVE: Change register %d value from %d to %d\n",
          regnum, value, ~value & 0xff);
        reg.set_register(regnum, ~value);
        break;
    }
  }
}


