#include "hall.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <poll.h>
#include <time.h>
#include <gpiod.h>

// internal structs

// buffer for timestamps
typedef struct {
    uint64_t ts[HALL_HISTORY_LEN];// ring list, newest at [head-1]
    int head; // next write position
    int count;  //how many entries are valid
    pthread_mutex_t lock;
} PulseHistory;

typedef struct {
    int gpio;
    int sensor_id;//o or 1
    PulseHistory history;
    pthread_t thread;
    volatile int running;
} HallSensor;

static HallSensor g_hall[2];
static bool g_initialised = false;

// get time-stamp time
static inline uint64_t now_us_raw(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// push a new timestamp into the history ring
static void history_push(PulseHistory *h, uint64_t ts_us)
{
    h->ts[h->head] = ts_us;
    h->head = (h->head + 1) % HALL_HISTORY_LEN;
    if (h->count < HALL_HISTORY_LEN) h->count++;
}


// background thread - one per sensor
// written for libgpiod v1.6.3 (gpiod_line_*, not gpiod_chip_request_lines which is for the newer versions in v2.x)
static void *hall_thread(void *arg)
{
    HallSensor *s = (HallSensor *)arg;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset); //set to core 3
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    // try real-time priority fifo (use 'sudo' when running); it can still run without as well
    struct sched_param sp = { .sched_priority = 80 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "[hall%d] SCHED_FIFO not available (not root?)\n",
                s->sensor_id);

    struct gpiod_chip *chip = gpiod_chip_open_by_number(0);
    if (!chip) {
        fprintf(stderr, "[hall%d] cannot open gpiochip0: %s\n",
                s->sensor_id, strerror(errno));
        s->running = 0;
        return NULL;
    }

    // request the line with falling-edge events and internal pull-up 
    struct gpiod_line *line = gpiod_chip_get_line(chip, (unsigned int)s->gpio);
    if (!line) {
        fprintf(stderr, "[hall%d] cannot get line %d: %s\n",
                s->sensor_id, s->gpio, strerror(errno));
        gpiod_chip_close(chip);
        s->running = 0;
        return NULL;
    }

    struct gpiod_line_request_config cfg = {
        .consumer = "hall_sensor",
        .request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE,
        .flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,
    };
    if (gpiod_line_request(line, &cfg, 0) < 0) {
        fprintf(stderr, "[hall%d] line request failed: %s\n",
                s->sensor_id, strerror(errno));
        gpiod_chip_close(chip);
        s->running = 0;
        return NULL;
    }

    fprintf(stderr, "[hall%d] thread ready on GPIO %d\n",
            s->sensor_id, s->gpio);

    // noise gate - minimum micro seconds between valid pulses
    // (if it happens quicker than this credible speed, discard reading as it is prob due to double/misreading)
    uint64_t min_gap_us = (uint64_t)(
        60000000.0 / (HALL_RPM_MAX_CREDIBLE * HALL_PULSES_PER_REV));

    uint64_t last_us = 0;
    int fd = gpiod_line_event_get_fd(line);

    struct pollfd pfd = { .fd = fd, .events = POLLIN };

    while (s->running) {
        int ret = poll(&pfd, 1, 500); //500ms timeout
        if (ret < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[hall%d] poll error: %s\n",
                    s->sensor_id, strerror(errno));
            break;
        }
        if (ret == 0) continue;   /* timeout, loop back and check running */

        struct gpiod_line_event ev;
        if (gpiod_line_event_read(line, &ev) < 0) continue;

        uint64_t t = now_us_raw();

        // noise gate 
        if (last_us > 0 && (t - last_us) < min_gap_us) continue;

        last_us = t;

        pthread_mutex_lock(&s->history.lock);
        history_push(&s->history, t);
        pthread_mutex_unlock(&s->history.lock);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
    fprintf(stderr, "[hall%d] thread exited\n", s->sensor_id);
    return NULL;
}

// --- public commands ---
bool hall_init(int gpio1, int gpio2)
{
    if (g_initialised) return true;

    int gpios[2] = { gpio1, gpio2 };

    for (int i = 0; i < 2; i++) {
        HallSensor *s = &g_hall[i];
        memset(s, 0, sizeof(*s));
        s->gpio = gpios[i];
        s->sensor_id = i;
        s->running = 1;
        pthread_mutex_init(&s->history.lock, NULL);

        if (pthread_create(&s->thread, NULL, hall_thread, s) != 0) {
            fprintf(stderr, "[hall] failed to create thread %d: %s\n",i, strerror(errno));
            return false;
        }
    }

    g_initialised = true;
    return true;
}

void hall_cleanup(void)
{
    if (!g_initialised) return;
    for (int i = 0; i < 2; i++) {
        g_hall[i].running = 0;
        pthread_join(g_hall[i].thread, NULL);
        pthread_mutex_destroy(&g_hall[i].history.lock);
    }
    g_initialised = false;
}

// this is if maybe other kind of filtering has to be donme in the main loop on the read pulses
static int snapshot_history(int sensor, uint64_t *out, int *out_count)
{
    if (sensor < 0 || sensor > 1) return 0;
    PulseHistory *h = &g_hall[sensor].history;

    pthread_mutex_lock(&h->lock);
    int count = h->count;
    // copy in chronological order: oldest first
    for (int i = 0; i < count; i++) {
        // head points to the next write slot, so head-count is the oldest
        int idx = ((h->head - count + i) + HALL_HISTORY_LEN) % HALL_HISTORY_LEN;
        out[i] = h->ts[idx];
    }
    pthread_mutex_unlock(&h->lock);

    *out_count = count;
    return count;
}

int hall_get_last_pulses(int sensor, uint64_t *out_buf)
{
    int count = 0;
    snapshot_history(sensor, out_buf, &count);
    return count;
}

// How long (microseconds) a single inter-pulse period is at the cutoff RPM.
// if the last pulse is older than this we treat the motor as stopped.
//(otherwise history would remain blocked on the last readings)
static inline uint64_t cutoff_period_us(void)
{
    //period_s = 60 / (RPM * pulses_per_rev), convert to micro sec
    return (uint64_t)(60.0 / (HALL_RPM_CUTOFF * HALL_PULSES_PER_REV) * 1e6);
}

//returns the timestamp of the most recent pulse, or 0 if no pulses yet
static uint64_t last_pulse_ts(int sensor)
{
    if (sensor < 0 || sensor > 1) return 0;
    PulseHistory *h = &g_hall[sensor].history;
    pthread_mutex_lock(&h->lock);
    uint64_t ts = 0;
    if (h->count > 0) {
        int idx = ((h->head - 1) + HALL_HISTORY_LEN) % HALL_HISTORY_LEN;
        ts = h->ts[idx];
    }
    pthread_mutex_unlock(&h->lock);
    return ts;
}

double hall_get_rpm_instant(int sensor)
{
    uint64_t buf[HALL_HISTORY_LEN];
    int count = 0;
    snapshot_history(sensor, buf, &count);
    if (count < 2) return 0.0;

    // if the last pulse is too old, the motor has stopped 
    uint64_t age_us = now_us_raw() - buf[count - 1];
    if (age_us > cutoff_period_us()) return 0.0;

    uint64_t gap_us = buf[count - 1] - buf[count - 2];
    if (gap_us == 0) return 0.0;

    // one pulse gap -> one inter-pulse period -> convert to RPM 
    double period_s = (double)gap_us * 1e-6;
    return 60.0 / (period_s * HALL_PULSES_PER_REV);
}

double hall_get_rpm_avg(int sensor, int n)
{
    if (n < 2) n = 2;
    if (n > HALL_HISTORY_LEN) n = HALL_HISTORY_LEN;

    uint64_t buf[HALL_HISTORY_LEN];
    int count = 0;
    snapshot_history(sensor, buf, &count);
    if (count < 2) return 0.0;

    // again, if the motor stopped, the history is still
    // full of old pulses that would give a false RPM reading
    uint64_t age_us = now_us_raw() - buf[count - 1];
    if (age_us > cutoff_period_us()) return 0.0;

    // use as many pulses as we have, up to n 
    int use = (count < n) ? count : n;

    // total time across (use-1) gaps, spanning use pulses
    uint64_t total_us = buf[count - 1] - buf[count - use];
    if (total_us == 0) return 0.0;

    int gaps = use - 1;
    double period_s = (double)total_us * 1e-6 / gaps;
    return 60.0 / (period_s * HALL_PULSES_PER_REV);
}
