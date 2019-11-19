#ifndef _i2c_app_h_
#define _i2c_app_h_

#include "i2c.h"

typedef enum {
    RW, // Read and Write via I2C
    R   // Read only from I2C
} reg_access_t;

#define NUM_REGS_PER_MOTOR 6
#define NUM_MOTORS 2
#define NUM_MOTOR_REGISTERS (NUM_REGS_PER_MOTOR * NUM_MOTORS)
#define NUM_REGISTERS NUM_MOTOR_REGISTERS+3 // +3 System Registers

#define MOTOR_STATE_REG_OFFSET 0
#define MOTOR_TARGET_POS_REG_OFFSET 1
#define MOTOR_CURRENT_POS_REG_OFFSET 2
#define MOTOR_ACTUATOR_REG_OFFSET 3
#define MOTOR_EVENT_REG_OFFSET 4
#define MOTOR_ERROR_REG_OFFSET 5
#define SYSTEM_ID_REG_OFFSET NUM_MOTOR_REGISTERS 
#define SW_VERSION_REGISTER NUM_MOTOR_REGISTERS+2

/*
 * Interface definition between user application and I2C slave register file
 */
typedef interface register_if {
  /* Set a register value
   */
  void set_register(int regnum, uint8_t data);

  /* Get a register value.
   */
  uint8_t get_register(int regnum);

  /* Get the number of the register that has changed
   * This will also clear the notification
   */
  [[clears_notification]]
  unsigned get_changed_regnum();

  /* Notification from the register file to the application that a register
   * value has changed
   */
  [[notification]]
  slave void register_changed();
} register_if;

[[distributable]]
void i2c_slave_register_file(server i2c_slave_callback_if i2c, server register_if app);

void slave_application(client register_if reg);



#endif