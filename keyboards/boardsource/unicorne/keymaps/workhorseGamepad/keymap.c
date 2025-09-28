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

joystick_config_t joystick_axes[JOYSTICK_AXIS_COUNT] = {
    [0] = JOYSTICK_AXIS_IN(ANALOG_JOYSTICK_X_AXIS_PIN, 900, 575, 285),
    [1] = JOYSTICK_AXIS_IN(ANALOG_JOYSTICK_Y_AXIS_PIN, 900, 575, 285),
    [2] = JOYSTICK_AXIS_VIRTUAL, // RT
    [3] = JOYSTICK_AXIS_VIRTUAL, //left x
    [4] = JOYSTICK_AXIS_VIRTUAL, //left y
    [5] = JOYSTICK_AXIS_VIRTUAL //LT
};

bool sendMouseReports = false;

report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (!sendMouseReports) {
        // Ignore mouse reports when in joystick mode
        mouse_report.x = 0;
        mouse_report.y = 0;
    }
     return mouse_report;
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
    if(is_keyboard_master())
    {
        {
            //static uint32_t last_sync2 = 0;
            //if (timer_elapsed32(last_sync2) > 1) 
            {
                master_to_slave_t m2s = {6};
                slave_to_master_t s2m = {0, 0, 0};
                if(transaction_rpc_exec(USER_SYNC_A, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) 
                {
                    const int16_t slaveX = Scale_0_Max_to_NegRange_Range(s2m.x);
                    const int16_t slaveY = Scale_0_Max_to_NegRange_Range(s2m.y);
                    dprintf("Slave value: %d %d\n", slaveX, slaveY);

                    //last_sync2 = timer_read32();
                    joystick_set_axis((uint8_t) 2, 0);
                    joystick_set_axis((uint8_t) 3, slaveX);
                    joystick_set_axis((uint8_t) 4, slaveY);
                    joystick_set_axis((uint8_t) 5, 0);


                } else 
                {
                    dprint("Slave sync failed!\n");
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

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
	switch (keycode) 
    {
		case JS_10:
			if (record->event.pressed) {
				sendMouseReports = true;
			} else {
				sendMouseReports = false;
				
			}
			return false;
		default:
        return true;
    }
}