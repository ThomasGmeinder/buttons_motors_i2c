// Copyright (c) 2014-2016, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <syscall.h>

#include "i2c.h"
#include "debug_print.h"
#include "i2c_app.h"

typedef enum {
    RW, // Read and Write via I2C
    R   // Read only from I2C
} reg_access_t;

reg_access_t register_access[NUM_REGISTERS] = {
   RW,
   RW,
   R,
   R, 
   RW,
   RW,
   R,
   R
};

[[distributable]]
void i2c_slave_register_file(server i2c_slave_callback_if i2c,
                             server register_if app)
{
  uint8_t registers[NUM_REGISTERS];
  // Register,  I2C Access, Function
  // 0          RW          Motor 1 trigger, states
  // 1          RW          Motor 1 target position
  // 2          R           Motor 1 current position
  // 3          R           Motor 1 status
  // 4          RW          Motor 2 trigger, states
  // 5          RW          Motor 2 target position
  // 6          R           Motor 2 current position
  // 7          R           Motor 2 status

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

        if(current_regnum == 3 || current_regnum == 7) {
          // clear status register
          registers[current_regnum] = 0;
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


