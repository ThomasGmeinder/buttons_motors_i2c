#ifndef _common_h_
#define _common_h_


#if INFER_ENDSWITCHES_WITH_AC_SENSOR
#define SW_VERSION 20  // version 2.0
#else
#define SW_VERSION 13  // version 1.3
#endif

#define POS_UPDATE_PERIOD_CYCLES (XS1_TIMER_HZ/10) // update every 0.1 seconds

#ifndef INFER_ENDSWITCHES_WITH_AC_SENSOR
#define INFER_ENDSWITCHES_WITH_AC_SENSOR 0
#endif

#if INFER_ENDSWITCHES_WITH_AC_SENSOR
#define ENDSWITCHES_ACTIVE 1 // Endswitches are active in the system but not connected directly
#define ENDSWITCHES_CONNECTED 0
#define ACCESS_ADC_VIA_SPI 1
#define NUM_ADC_CHANNELS 2
#define MOTOR_CURRENT_OFF_THRESHOLD Q16(0.1) // 100 mA
#define MOTOR_CURRENT_ON_THRESHOLD Q16(0.2) // 200 mA
#define MOTOR_CURRENT_HYSTERESIS_PERIODS 2 // 2 * 100ms = 0.2s
// Todo: Fix time to ref clock cycles not update periods
#define MOTOR_CURRENT_SWITCH_PERIODS MOTOR_CURRENT_HYSTERESIS_PERIODS * 8 // time until mutor current on/off must be detected
#endif

#ifndef ENDSWITCHES_CONNECTED
#define ENDSWITCHES_CONNECTED 1
#endif 

#if ENDSWITCHES_CONNECTED
#define ENDSWITCHES_ACTIVE 1 // Endswitches are active in the system
#endif

// GPIO value when Endswitch is triggered. Note: GPIO has pulldown
#define ES_TRIGGERED 0 // Endswitch disconnects from VCC.
//#define ES_TRIGGERED 1 // Endswitch connects to VCC.
#define CONTROL_BUTTON_PRESSED 1 // Button pressed value
#define ERROR_BUTTON_PRESSED 0 // Button pressed value

#define ENABLE_INTERNAL_PULLS 1

#define CLEAR_ERROR_BIT 0x40

typedef enum {
    OPENING,
    CLOSING,
    STOPPED,
} motor_state_t;

typedef enum {
    I2C = 0,
    BUTTON = 1,
} actuator_t;

// Todo: Maybe separate MOTOR_TOO_SLOW and MOTOR_TOO_FAST into a warning register
// Errors with severity
typedef enum {
    NO_ERROR, 
    MOTOR_TOO_SLOW, 
    MOTOR_TOO_FAST, 
    POSITION_UNKNOWN,
    OUT_OF_RANGE,
    BOTH_ENDSWITCHES_ON, 
    CLOSED_ES_WHILST_OPENING, // Motor is running in wrong direction or not at all !
    OPEN_ES_WHILST_CLOSING, // Motor is running in wrong direction or not at all!
    NUM_ERRORS
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

    int start_time;
    int stop_time;

    motor_state_t state;
    motor_error_t error;
    motor_error_t prev_error;

    // speed in um/s to have right resolution
    unsigned opening_speed;
    unsigned closing_speed;

    int open_button_blink_counter; 
    int close_button_blink_counter;

    int time_of_last_flash_update;
    unsigned update_flash; // shared memory flag to trigger flash write

#if INFER_ENDSWITCHES_WITH_AC_SENSOR
    unsigned AC_current_hysteresis_counter;
    unsigned AC_current_on;
    unsigned AC_current_on_flag;
#endif

} motor_state_s;

#define FLASH_DATA_VALID_BYTE 0x3C // arbitrary value
#define MOTOR_FLASH_AREA_SIZE 6 // one byte marker, 4 bytes position, one byte error
#define FLASH_DATA_BYTES (MOTOR_FLASH_AREA_SIZE*2)

// length translation macros
#define MM_to_UM(mm) (mm*1000)
#define UM_to_MM(um) (um/1000)
#define CM_to_UM(cm) (cm*10000)
#define UM_to_CM(um) (um/10000)

#endif
