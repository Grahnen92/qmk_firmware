#include "joystick.h"
//#include "pointing_device.h"
#include "transactions.h"
#include "quantum/split_common/transactions.h"
#include "split_util.h"
#include "quantum.h"
#include "quantum/keymap_introspection.h"
//include "analog_joystick.h"
#include "analog.h"
#include "gpio.h"
#include "keymap_swedish.h"
#include "timer.h"
#include <math.h>

joystick_config_t joystick_axes[JOYSTICK_AXIS_COUNT] = {
    [0] = JOYSTICK_AXIS_IN(ANALOG_JOYSTICK_X_AXIS_PIN, 1023, 512, 0),
    [1] = JOYSTICK_AXIS_IN(ANALOG_JOYSTICK_Y_AXIS_PIN, 1023, 512, 0),
    [2] = JOYSTICK_AXIS_VIRTUAL, // RT
    [3] = JOYSTICK_AXIS_VIRTUAL, //left x
    [4] = JOYSTICK_AXIS_VIRTUAL, //left y
    [5] = JOYSTICK_AXIS_VIRTUAL //LT
};

// report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
//     if (layer_state_is(4)) {
//         // Ignore mouse reports when in joystick mode
//         mouse_report.x = 0;
//         mouse_report.y = 0; 
//     }

//      return mouse_report;
// }

// Mouse velocity configuration (pixels per second)
#define MOUSE_VELOCITY_MIN 64.0f    // Minimum velocity at input magnitude 1
#define MOUSE_VELOCITY_MAX 3072.0f  // Maximum velocity at input magnitude 254 (127+127)
#define MOUSE_MAGNITUDE_MAX 254.0f  // Maximum input magnitude from combined joysticks

// Mouse velocity control state
typedef struct {
    float accumulated_x;
    float accumulated_y;
    uint16_t last_update_time;
    float velocity;  // pixels per second
} mouse_velocity_state_t;

static mouse_velocity_state_t velocity_state = {
    .accumulated_x = 0.0f,
    .accumulated_y = 0.0f,
    .last_update_time = 0,
    .velocity = 12.0f  // Default velocity (not used with dynamic velocity calculation)
};

// Apply velocity control to mouse movement
// Input: raw movement values (-254 to 254 from combined joysticks, with 0 being no movement)
// Output: velocity-controlled movement with sub-pixel precision
// Velocity scales from MOUSE_VELOCITY_MIN to MOUSE_VELOCITY_MAX based on input magnitude
void apply_mouse_velocity(int16_t *x, int16_t *y) {
    uint16_t current_time = timer_read();
    
    // Initialize timer on first call
    if (velocity_state.last_update_time == 0) {
        velocity_state.last_update_time = current_time;
        *x = 0;
        *y = 0;
        return;
    }
    
    // Calculate elapsed time in seconds
    uint16_t elapsed_ms = timer_elapsed(velocity_state.last_update_time);
    velocity_state.last_update_time = current_time;
    
    // If no time has passed, don't update
    if (elapsed_ms == 0) {
        *x = 0;
        *y = 0;
        return;
    }
    
    float delta_time = elapsed_ms / 1000.0f;
    
    // Get input values
    float input_x = (float)(*x);
    float input_y = (float)(*y);
    
    // Calculate magnitude of input
    float magnitude = sqrtf(input_x * input_x + input_y * input_y);
    
    if (magnitude > 0.01f) {
        // Normalize direction
        float dir_x = input_x / magnitude;
        float dir_y = input_y / magnitude;
        
        // Calculate velocity based on input magnitude
        // Map magnitude [1, MOUSE_MAGNITUDE_MAX] to velocity [MOUSE_VELOCITY_MIN, MOUSE_VELOCITY_MAX] pixels/second
        float velocity;
        if (magnitude < 1.0f) {
            // Below minimum threshold, use minimum velocity
            velocity = MOUSE_VELOCITY_MIN;
        } else if (magnitude > MOUSE_MAGNITUDE_MAX) {
            // Above maximum, clamp to max velocity
            velocity = MOUSE_VELOCITY_MAX;
        } else {
            // Linear interpolation: velocity = MIN + (magnitude - 1) * (MAX - MIN) / (MOUSE_MAGNITUDE_MAX - 1)
            velocity = MOUSE_VELOCITY_MIN + (magnitude - 1.0f) * ((MOUSE_VELOCITY_MAX - MOUSE_VELOCITY_MIN) / (MOUSE_MAGNITUDE_MAX - 1.0f));
        }
        
        // Apply velocity: movement = velocity * time
        float move_distance = velocity * delta_time;
        
        // Calculate movement for this frame
        velocity_state.accumulated_x += dir_x * move_distance;
        velocity_state.accumulated_y += dir_y * move_distance;
    }
    
    // Extract integer movement and keep fractional part
    int16_t out_x = (int16_t)velocity_state.accumulated_x;
    int16_t out_y = (int16_t)velocity_state.accumulated_y;
    
    // Clamp to valid range (-127 to 127 for mouse report)
    if (out_x < -127) out_x = -127;
    if (out_x > 127) out_x = 127;
    if (out_y < -127) out_y = -127;
    if (out_y > 127) out_y = 127;
    
    // Update accumulated values (keep fractional part)
    velocity_state.accumulated_x -= out_x;
    velocity_state.accumulated_y -= out_y;
    
    // Prevent accumulator from drifting excessively (allow larger values for high velocities)
    if (velocity_state.accumulated_x > 200.0f) velocity_state.accumulated_x = 200.0f;
    if (velocity_state.accumulated_x < -200.0f) velocity_state.accumulated_x = -200.0f;
    if (velocity_state.accumulated_y > 200.0f) velocity_state.accumulated_y = 200.0f;
    if (velocity_state.accumulated_y < -200.0f) velocity_state.accumulated_y = -200.0f;
    
    *x = out_x;
    *y = out_y;
}

// Optional: Function to change velocity at runtime
void set_mouse_velocity(float new_velocity) {
    // Clamp to min/max range
    if (new_velocity < MOUSE_VELOCITY_MIN) new_velocity = MOUSE_VELOCITY_MIN;
    if (new_velocity > MOUSE_VELOCITY_MAX) new_velocity = MOUSE_VELOCITY_MAX;
    velocity_state.velocity = new_velocity;
}

report_mouse_t pointing_device_task_combined_user(report_mouse_t left_report, report_mouse_t right_report)
{
    
     if (layer_state_is(4)) {
        // Ignore mouse reports when in joystick mode
        left_report.x = 0;
        left_report.y = 0; 
    }
    else
    {
        // Combine left and right reports
        // Apply axis inversions: left X inverted, both Y inverted
        // Use int16_t to handle combined range (can be up to ±254)
        int16_t combined_x = -left_report.x + right_report.y;
        int16_t combined_y = -(left_report.y + right_report.x);
        
        // Apply velocity control
        apply_mouse_velocity(&combined_x, &combined_y);

        dprintf("mv: %d %d mi: %d %d\n", combined_x, combined_y, left_report.x, left_report.y);
        
        left_report.x = (int8_t)combined_x;
        left_report.y = (int8_t)combined_y;

    }

    return left_report;
}


// void pointing_device_driver_task(void) {
//     if (true) {
//         // Send joystick reports here
//         // e.g., joystick_set_axis(...)
//         uint16_t raw_x2 = analogReadPin(GP26);
//         uint16_t raw_y2 = analogReadPin(GP27);
//         joystick_set_axis(0, raw_x2);
//         joystick_set_axis(1, raw_y2);

//     }
// }

typedef struct {
    int x;
    int y;
} JoystickData;

JoystickData joystickData;

typedef struct _master_to_slave_t {
    int m2s_data;
} master_to_slave_t;

typedef struct _slave_to_master_t {
    int s2m_data;
    int x;
    int y;
} slave_to_master_t;

void user_sync_a_slave_handler(uint8_t in_buflen, const void* in_data, uint8_t out_buflen, void* out_data) {
    const master_to_slave_t *m2s = (const master_to_slave_t*)in_data;
    slave_to_master_t *s2m = (slave_to_master_t*)out_data;
    s2m->s2m_data = m2s->m2s_data + 5; // whatever comes in, add 5 so it can be sent back
    s2m->x = joystickData.x;
    s2m->y = joystickData.y;
}

void keyboard_post_init_user(void) {

    debug_enable=true;
    debug_matrix=true;
    debug_keyboard=true;

    transaction_register_rpc(USER_SYNC_A, user_sync_a_slave_handler);

   // pointing_device_set_cpi_on_side(true, 1000); //Set cpi on left side to a low value for slower scrolling.
    //pointing_device_set_cpi_on_side(false, 1000); //Set cpi on right side to a reasonable value for mousing.
}

int kScaleInputMax = 1024;
int kScaleOutputRange = 32768;

int16_t Scale_0_Max_to_NegRange_Range(int Value)
{
    // Clamp input to [0, kScaleInputMax]
    if (Value < 0)
        Value = 0;
    else if (Value > kScaleInputMax)
        Value = kScaleInputMax;

    // Perform scaling: 0 -> -kScaleOutputRange, kScaleInputMax -> kScaleOutputRange
    // Formula: scaled = ((Value * (2 * kScaleOutputRange)) / kScaleInputMax) - kScaleOutputRange
    int16_t scaled = ((Value * (2 * kScaleOutputRange)) + (kScaleInputMax / 2)) / kScaleInputMax - kScaleOutputRange; // rounding

    return scaled;
}

void housekeeping_task_user(void) 
{
    if (layer_state_is(4))
    {
        if(!joystick_axis_is_enabled(0))
        {
            joystick_set_axis_enabled((uint8_t) 0, true);
            joystick_set_axis_enabled((uint8_t) 1, true);
            joystick_set_axis_enabled((uint8_t) 2, true);
            joystick_set_axis_enabled((uint8_t) 3, true);
            joystick_set_axis_enabled((uint8_t) 4, true);
            joystick_set_axis_enabled((uint8_t) 5, true);
        }

        if(is_keyboard_master())
        {
            {
                {
                    master_to_slave_t m2s = {6};
                    slave_to_master_t s2m = {0, 0, 0};
                    if(transaction_rpc_exec(USER_SYNC_A, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) 
                    {
                        const int16_t slaveX = Scale_0_Max_to_NegRange_Range(s2m.x);
                        const int16_t slaveY = Scale_0_Max_to_NegRange_Range(s2m.y);
                        //dprintf("Slave value: %d %d\n", slaveX, slaveY);

                        joystick_set_axis((uint8_t) 2, 0);
                        joystick_set_axis((uint8_t) 3, slaveX);
                        joystick_set_axis((uint8_t) 4, slaveY);
                        joystick_set_axis((uint8_t) 5, 0);


                    } else 
                    {
                        //dprint("Slave sync failed!\n");
                    }
                }
            }
        }
        else
        {
            joystickData.x = (int) analogReadPin(ANALOG_JOYSTICK_X_AXIS_PIN); // Read and convert joystick X-axis data
            joystickData.y = (int) analogReadPin(ANALOG_JOYSTICK_Y_AXIS_PIN); // Read and convert joystick Y-axis data
            
        }
    }
    else
    {
        if(joystick_axis_is_enabled(0))
        {
            joystick_set_axis((uint8_t) 0, 0);
            joystick_set_axis((uint8_t) 1, 0);
            joystick_set_axis((uint8_t) 2, 0);
            joystick_set_axis((uint8_t) 3, 0);
            joystick_set_axis((uint8_t) 4, 0);
            joystick_set_axis((uint8_t) 5, 0);

            joystick_set_axis_enabled((uint8_t) 0, false);
            joystick_set_axis_enabled((uint8_t) 1, false);
            joystick_set_axis_enabled((uint8_t) 2, false);
            joystick_set_axis_enabled((uint8_t) 3, false);
            joystick_set_axis_enabled((uint8_t) 4, false);
            joystick_set_axis_enabled((uint8_t) 5, false);
        }
    }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
	switch (keycode) 
    {
		// case JS_10:
		// 	return false;
        // case KC_FNX:                                    
        //     if(record->event.pressed){
        //         fnx_layer_timer = timer_read();
        //         register_mod(MOD_LSFT);
        //     } else {
        //         unregister_mod(MOD_LSFT);
        //         if (timer_elapsed(fnx_layer_timer) < TAPPING_TERM) {  
        //             layer_invert(_SYMB);
        //         }
        //     }
        // return false;
		default:
        return true;
    }
}