#ifndef ESC_H
#define ESC_H

#include <stdbool.h>

// PWM chip and channels for the two ESCs
// GPIO18 = pwmchip0 channel 2
// GPIO19 = pwmchip0 channel 3
#define ESC_PWM_CHIP       0
#define ESC_PWM_CHANNEL_1  2    // GPIO18 
#define ESC_PWM_CHANNEL_2  3    // GPIO19

#define ESC_DEFAULT_FREQ_HZ  200

// Standard RC pulse widths in microseconds
#define ESC_PULSE_STOP_US     1500
#define ESC_PULSE_FWD_MAX_US  1900
#define ESC_PULSE_REV_MAX_US  1100

// initialisation; pass 0 to use the default freq_hz=200.
// returns true on success
bool esc_init(int freq_hz);

// Set throttle for motor 1 (GPIO18). pct is -100.0 to +100.0, 0 = stop
void esc_set_throttle_1(double pct);

// Set throttle for motor 2 (GPIO19). pct is -100.0 to +100.0, 0 = stop
void esc_set_throttle_2(double pct);

// Set both motors at once
void esc_set_throttle_both(double pct1, double pct2);

// call this at shutdown. sends stop pulse then disables PWM cleanly
void esc_cleanup(void);

#endif
