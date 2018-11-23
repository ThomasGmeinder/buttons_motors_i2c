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

// Todo: Maybe separate MOTOR_TOO_SLOW and MOTOR_TOO_FAST into a warning register
typedef enum {
    NO_ERROR,
    POSITION_UNKNOWN,
    BOTH_ENDSWITCHES_ON,
    MOTOR_TOO_SLOW, 
    MOTOR_TOO_FAST,     
} motor_error_t;

typedef enum {
    NO_EVENT = 0,
    START,
    STOP,
} motor_event_t;


// struct for motor state
typedef struct {
    int position; // in um from 0 (open) to MAX_POS (closed)
    int target_position; // the target position
    unsigned motor_idx;
    actuator_t actuator;

    motor_state_t state;
    motor_error_t error;
    motor_error_t prev_error;

    int open_button_blink_counter; 
    int close_button_blink_counter;

    int time_of_last_flash_update;
    unsigned update_flash; // shared memory flag to trigger flash write

} motor_state_s;

#define FLASH_DATA_VALID_BYTE 0x3C // arbitrary value
#define MOTOR_FLASH_AREA_SIZE 5 // one byte marker, 4 bytes position
#define FLASH_DATA_BYTES MOTOR_FLASH_AREA_SIZE*2

// length translation macros
#define MM_to_UM(mm) (mm*1000)
#define UM_to_MM(um) (um/1000)
#define CM_to_UM(cm) (cm*10000)
#define UM_to_CM(um) (um/10000)

#endif
