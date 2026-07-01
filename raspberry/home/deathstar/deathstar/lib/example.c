#include <stdio.h>
#include <unistd.h>

#include "esc.h"
#include "hall.h"
#include "gpio_outputs.h"

int main(void)
{
    // init everything
    if (!gpio_outputs_init()) return 1;
    if (!esc_init(0))  return 1;   // 0 = use default 200 Hz
    if (!hall_init(HALL_GPIO_1, HALL_GPIO_2)) return 1;

    // blink the LED
    led_set(true);
    usleep(500000);
    led_set(false);

    // up motor 1 at 30% 
    esc_set_throttle_1(30.0);
    printf("motor 1 at 30%%\n");

    // meas RPM for 5 seconds
    for (int i = 0; i < 50; i++) {
        usleep(100000);   // 100 ms
        double rpm1 = hall_get_rpm_avg(0, HALL_PULSES_PER_REV);
        double rpm2 = hall_get_rpm_avg(1, HALL_PULSES_PER_REV);
        printf("RPM  sensor0=%.1f  sensor1=%.1f\n", rpm1, rpm2);
    }

    esc_set_throttle_1(0.0);

    // toggle thruster 1 
    thruster_set(1, true);
    usleep(200000);
    thruster_set(1, false);

    // flash the laser
    laser_set(true);
    usleep(100000);
    laser_set(false);

    // cleanup in reverse order
    hall_cleanup();
    esc_cleanup();
    gpio_outputs_cleanup();

    return 0;
}
