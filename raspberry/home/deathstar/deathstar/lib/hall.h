#ifndef HALL_H
#define HALL_H

#include <stdint.h>
#include <stdbool.h>

// default GPIO pins for the two hall sensors
#define HALL_GPIO_1 20
#define HALL_GPIO_2 6

#define HALL_POLES 14
#define HALL_PULSES_PER_REV (HALL_POLES / 2)

// how many pulses we keep in the history ring for RPM averaging.
//should be >= HALL_PULSES_PER_REV and a power of 2 ideally
#define HALL_HISTORY_LEN  16

// ignore pulses that arrive faster than this (the motors have a top rpm of circa 13k for throttle 100%)
//otherwise it is noise(double triggering)
#define HALL_RPM_MAX_CREDIBLE  15000.0

// if no pulse has arrived within the inter-pulse period corresponding to this
// RPM, consider the motor stopped and return 0; to voids stale readings after the motor winds down
#define HALL_RPM_CUTOFF  140.0

// start background Hall threads for both sensors.
// gpio1 / gpio2 are the GPIO numbers (use HALL_GPIO_1 or HALL_GPIO_2)
// returns true on success
bool hall_init(int gpio1, int gpio2);

// stop threads and release GPIO resources
void hall_cleanup(void);

// copy the last pulse timestamps 
int hall_get_last_pulses(int sensor, uint64_t *out_buf);

// RPM based on the gap between the two most recent pulses. 
// returns 0.0 if fewer than 2 pulses seen.
double hall_get_rpm_instant(int sensor);

// RPM averaged over the last n pulses (maximum length is HALL_HISTORY_LEN)
// returns 0.0 if not enough data yet
double hall_get_rpm_avg(int sensor, int n);

#endif
