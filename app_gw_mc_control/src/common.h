#ifndef _common_h_
#define _common_h_

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

// Todo: Maybe separate SPEED_TOO_SLOW and SPEED_TOO_FAST into a warning register
typedef enum {
    NO_ERROR,
    POSITION_UNKNOWN,
    BOTH_ENDSWITCHES_ON,
    SPEED_TOO_SLOW, 
    SPEED_TOO_FAST,     
} motor_error_t;

typedef enum {
    NO_EVENT = 0,
    START,
    STOP,
} motor_event_t;


// struct for motor state
typedef struct {
    int position; // in mm from 0 (open) to MAX_POS (closed)
    int target_position; // the target position
    unsigned motor_idx;
    actuator_t actuator;

    motor_state_t state;
    motor_error_t error;
    motor_error_t prev_error;

    int open_button_blink_counter; 
    int close_button_blink_counter;

    unsigned update_flash; // shared memory flag to trigger flash write

} motor_state_s;

#define FLASH_DATA_VALID_BYTE 0x3C // arbitrary value
#define FLASH_DATA_BYTES 4


#endif