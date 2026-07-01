#ifndef GPIO_OUTPUTS_H
#define GPIO_OUTPUTS_H

#include <stdbool.h>

#define LASER_GPIO 12
#define THRUSTER_1_GPIO 24
#define THRUSTER_2_GPIO 25
#define THRUSTER_3_GPIO 16
#define THRUSTER_4_GPIO 26
#define LED_GPIO 23
#define ESTOP_GPIO 5

// Init - call once before anything else. All outputs default OFF.
// on_estop: optional callback, called from the estop thread when triggered.
//          pass NULL for only the default behaviour
// returns true on success
bool gpio_outputs_init(void (*on_estop)(void));

//laser (GPIO 12)
void laser_set(bool on);

//Thrusters (GPIO 24, 25, 16, 26) - index is 1-4 
void thruster_set(int thruster, bool on);
void thruster_set_all(bool on);

// LED (GPIO 23)
void led_set(bool on);

// ESTOP (GPIO 5)
bool estop_is_pressed(void); // raw read
bool estop_is_triggered(void);// check if button was pressed 

// cleanup at shutdown:to turn everything off, stop estop thread, releases GPIO
void gpio_outputs_cleanup(void);

#endif
