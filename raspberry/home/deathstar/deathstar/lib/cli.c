#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include "esc.h"
#include "hall.h"
#include "gpio_outputs.h"

static volatile int g_stream_1  = 0;
static volatile int g_stream_2  = 0;
static volatile int g_rpm_running = 1;
static pthread_t g_rpm_thread;

static void *rpm_stream_thread(void *arg)
{
    (void)arg;
    while (g_rpm_running) {
        if (g_stream_1 || g_stream_2) {
            if (g_stream_1 && g_stream_2) {
                printf("[rpm]  s1: %7.1f rpm    s2: %7.1f rpm\n",
                       hall_get_rpm_avg(0, HALL_PULSES_PER_REV),
                       hall_get_rpm_avg(1, HALL_PULSES_PER_REV));
            } else if (g_stream_1) {
                printf("[rpm]  s1: %7.1f rpm\n",
                       hall_get_rpm_avg(0, HALL_PULSES_PER_REV));
            } else {
                printf("[rpm]  s2: %7.1f rpm\n",
                       hall_get_rpm_avg(1, HALL_PULSES_PER_REV));
            }
            fflush(stdout);
        }
        usleep(200000);   // 5 Hz 
    }
    return NULL;
}

static void shutdown_all(void)
{
    printf("\ncleaning up...\n");

    g_rpm_running = 0;
    g_stream_1 = g_stream_2 = 0;
    pthread_join(g_rpm_thread, NULL);

    esc_set_throttle_both(0.0, 0.0);

    hall_cleanup();
    esc_cleanup();
    gpio_outputs_cleanup();

    printf("bye\n");
}


//state tracked for toggling
static bool g_laser_on = false;
static bool g_led_on = false;
static bool g_thr_on[4] = {false,};

static void sig_handler(int sig)
{
    (void)sig;
    shutdown_all();
    exit(0);
}

static void print_help(void)
{
    printf("\n"
           "  h            show this help\n"
           "  t1 <val>     set ESC 1 throttle  (-100 to 100, 0 = stop)\n"
           "  t2 <val>     set ESC 2 throttle  (-100 to 100, 0 = stop)\n"
           "  l            toggle laser\n"
           "  s1           start/stop RPM stream for sensor 1\n"
           "  s2           start/stop RPM stream for sensor 2\n"
           "  s            stop all RPM streams\n"
           "  i            toggle LED indicator\n"
           "  q  /  x      stop motors, cleanup, quit\n"
           "  thr <val>     toggle  thruster\n"
           "  Ctrl+C       same as q\n"
           "\n");
}

static bool handle_command(char *line)
{
    // take out trailing newline/whitespace 
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'
                    || line[len-1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) return true;

    // t1 / t2 - throttle with a value 
    if ((line[0] == 't' || line[0] == 'T') &&
        (line[1] == '1' || line[1] == '2') &&
        line[2] == ' ') {

        double val = atof(line + 3);
        if (line[1] == '1') {
            esc_set_throttle_1(val);
            printf("ESC1 throttle → %.1f%%\n", val);
        } else {
            esc_set_throttle_2(val);
            printf("ESC2 throttle → %.1f%%\n", val);
        }
        return true;
    }

    // s1 / s2 - toggle individual stream 
    if ((line[0] == 's' || line[0] == 'S') && line[1] == '1' && line[2] == '\0') {
        g_stream_1 = !g_stream_1;
        printf("RPM stream sensor 1: %s\n", g_stream_1 ? "ON" : "OFF");
        return true;
    }
    if ((line[0] == 's' || line[0] == 'S') && line[1] == '2' && line[2] == '\0') {
        g_stream_2 = !g_stream_2;
        printf("RPM stream sensor 2: %s\n", g_stream_2 ? "ON" : "OFF");
        return true;
    }
    
        // thr thruster toggle
    if ((line[0] == 't' || line[0] == 'T') &&
		(line[1] == 'h' || line[1] == 'H') &&
		(line[2] == 'r' || line[2] == 'R') &&
        line[3] == ' ') {

        int val = atoi(line + 4);
        if (val < 5 && val > 0 ) {
			g_thr_on[val-1] = !g_thr_on[val-1];
            thruster_set(val, g_thr_on[val-1]);
            printf("thruster %d is:  %s\n", val, g_thr_on[val-1] ? "ON" : "OFF");
        } else {
		printf("thruster val should be  1,2,3,or4\n");
		}
        return true;
    }

    // single-character commands
    if (len == 1 || (len > 1 && line[1] == '\0')) {
        switch (line[0]) {

        case 'h': case 'H':
            print_help();
            break;

        case 'l': case 'L':
            g_laser_on = !g_laser_on;
            laser_set(g_laser_on);
            printf("laser: %s\n", g_laser_on ? "ON" : "OFF");
            break;

        case 's': case 'S':
            g_stream_1 = g_stream_2 = 0;
            printf("RPM streams stopped\n");
            break;

        case 'i': case 'I':
            g_led_on = !g_led_on;
            led_set(g_led_on);
            printf("LED: %s\n", g_led_on ? "ON" : "OFF");
            break;

        case 'q': case 'Q':
        case 'x': case 'X':
            return false;   // caller will shutdown 

        default:
            printf("unknown command '%s' - type h for help\n", line);
            break;
        }
        return true;
    }

    printf("unknown command '%s' - type h for help\n", line);
    return true;
}


int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    printf("initialising hardware...\n");
    //if (!gpio_outputs_init(NULL)) { //NULL->built-in estop handler
        //fprintf(stderr, "gpio_outputs_init failed\n");
        //return 1;
    //}
    if (!gpio_outputs_init(NULL)) { //NULL->built-in estop handler
        fprintf(stderr, "gpio_outputs_init failed\n");
        return 1;
    }
    if (!esc_init(0)) {
        fprintf(stderr, "esc_init failed\n");
        gpio_outputs_cleanup();
        return 1;
    }
    if (!hall_init(HALL_GPIO_1, HALL_GPIO_2)) {
        fprintf(stderr, "hall_init failed\n");
        esc_cleanup();
        gpio_outputs_cleanup();
        return 1;
    }

    //start the RPM printing thread 
    if (pthread_create(&g_rpm_thread, NULL, rpm_stream_thread, NULL) != 0) {
        fprintf(stderr, "failed to start rpm thread: %s\n", strerror(errno));
        return 1;
    }

    printf("hello! type h for help.\n\n");

    char line[256];
    while (1) {
        printf("> ");
        fflush(stdout);
        
        if (!fgets(line, sizeof(line), stdin)) break;

        if (!handle_command(line)) break;
    }

    shutdown_all();
    return 0;
}
