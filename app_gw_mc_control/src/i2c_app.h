#ifndef _i2c_app_h_
#define _i2c_app_h_

#include "i2c.h"

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

#define NUM_REGISTERS 10

[[distributable]]
void i2c_slave_register_file(server i2c_slave_callback_if i2c, server register_if app);

void slave_application(client register_if reg);



#endif