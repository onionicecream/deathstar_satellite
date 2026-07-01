// libgpiod v1.6 was used
// if to be update to libgpiod v2 some calls might differ
#include "gpio_outputs.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <gpiod.h>
#include <signal.h>

#define ESTOP_POLL_MS 50

static struct gpiod_chip *g_chip  = NULL;

static struct gpiod_line *g_laser = NULL;
static struct gpiod_line *g_thr[4]= {NULL};
static struct gpiod_line *g_led = NULL;
static struct gpiod_line *g_estop = NULL;

static const int THRUSTER_GPIOS[4] = {
    THRUSTER_1_GPIO, THRUSTER_2_GPIO,
    THRUSTER_3_GPIO, THRUSTER_4_GPIO,
};

// estop thread state
static pthread_t g_estop_thread;
static atomic_bool g_estop_triggered = false;
static void (*g_estop_cb)(void) = NULL;

// internal functions

static struct gpiod_line *request_output(int gpio, const char *name)
{
    struct gpiod_line *line = gpiod_chip_get_line(g_chip, (unsigned int)gpio);
    if (!line) {
        fprintf(stderr, "[gpio_outputs] cannot get GPIO %d: %s\n",
                gpio, strerror(errno));
        return NULL;
    }
    if (gpiod_line_request_output(line, name, 0) < 0) {
        fprintf(stderr, "[gpio_outputs] cannot request GPIO %d as output: %s\n",
                gpio, strerror(errno));
        return NULL;
    }
    return line;
}

static struct gpiod_line *request_input(int gpio, const char *name)
{
    struct gpiod_line *line = gpiod_chip_get_line(g_chip, (unsigned int)gpio);
    if (!line) {
        fprintf(stderr, "[gpio_outputs] cannot get GPIO %d: %s\n",
                gpio, strerror(errno));
        return NULL;
    }
    if (gpiod_line_request_input(line, name) < 0) {
        fprintf(stderr, "[gpio_outputs] cannot request GPIO %d as input: %s\n",
                gpio, strerror(errno));
        return NULL;
    }
    return line;
}

static void default_estop_handler(void)
{
    fprintf(stderr, "[gpio_outputs] ESTOP pressed - shutting down\n");
    raise(SIGTERM);
}

static void *estop_poll_thread(void *arg)
{
    (void)arg;
    struct timespec ts = { 0, ESTOP_POLL_MS * 1000 * 1000 };

    while (!g_estop_triggered) {
        if (estop_is_pressed()) {
            g_estop_triggered = true;
            g_estop_cb();    
            break;
        }
        nanosleep(&ts, NULL);
    }
    return NULL;
}

//public commnads

bool gpio_outputs_init(void (*on_estop)(void))
{
    g_chip = gpiod_chip_open_by_number(0);
    if (!g_chip) {
        fprintf(stderr, "[gpio_outputs] cannot open gpiochip0: %s\n",
                strerror(errno));
        return false;
    }

    g_laser = request_output(LASER_GPIO, "laser");
    if (!g_laser) return false;

    for (int i = 0; i < 4; i++) {
        g_thr[i] = request_output(THRUSTER_GPIOS[i], "thruster");
        if (!g_thr[i]) return false;
    }

    g_led = request_output(LED_GPIO, "led");
    if (!g_led) return false;

    g_estop = request_input(ESTOP_GPIO, "estop");
    if (!g_estop) return false;

    g_estop_cb = on_estop ? on_estop : default_estop_handler;

    if (pthread_create(&g_estop_thread, NULL, estop_poll_thread, NULL) != 0) {
        fprintf(stderr, "[gpio_outputs] failed to start estop thread: %s\n",
                strerror(errno));
        return false;
    }

    fprintf(stderr, "[gpio_outputs] init done, estop polling started\n");
    return true;
}

void laser_set(bool on)
{
    if (!g_laser) return;
    gpiod_line_set_value(g_laser, on ? 1 : 0);
}

void thruster_set(int thruster, bool on)
{
    if (thruster < 1 || thruster > 4) {
        fprintf(stderr, "[gpio_outputs] thruster index %d out of range (1-4)\n",
                thruster);
        return;
    }
    if (!g_thr[thruster - 1]) return;
    gpiod_line_set_value(g_thr[thruster - 1], on ? 1 : 0);
}

void thruster_set_all(bool on)
{
    for (int i = 0; i < 4; i++)
        if (g_thr[i]) gpiod_line_set_value(g_thr[i], on ? 1 : 0);
}

void led_set(bool on)
{
    if (!g_led) return;
    gpiod_line_set_value(g_led, on ? 1 : 0);
}

bool estop_is_pressed(void)
{
    if (!g_estop) return false;
    int val = gpiod_line_get_value(g_estop);
    if (val < 0) {
        fprintf(stderr, "[gpio_outputs] estop read error: %s\n",
                strerror(errno));
        return false;
    }
    return val == 0;   //LOW = pressed (pull-up resistor)
}

bool estop_is_triggered(void)
{
    return g_estop_triggered;
}

void gpio_outputs_cleanup(void)
{
    // stop estop thread first
    g_estop_triggered = true;
    pthread_join(g_estop_thread, NULL);

    // turn everything off 
    laser_set(false);
    thruster_set_all(false);
    led_set(false);

    // release lines 
    if (g_laser)  { gpiod_line_release(g_laser); g_laser  = NULL; }
    for (int i = 0; i < 4; i++)
        if (g_thr[i]) { gpiod_line_release(g_thr[i]); g_thr[i] = NULL; }
    if (g_led)    { gpiod_line_release(g_led); g_led    = NULL; }
    if (g_estop)  { gpiod_line_release(g_estop); g_estop  = NULL; }

    if (g_chip)   { gpiod_chip_close(g_chip);g_chip   = NULL; }

    fprintf(stderr, "[gpio_outputs] cleanup done\n");
}
