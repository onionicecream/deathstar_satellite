#include "esc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

//using harware pwm
// https://raspberrypi.stackexchange.com/questions/148769/troubleshooting-pwm-via-sysfs
// https://raspberrypi.stackexchange.com/questions/148795/only-1-pwm-pin-working-when-controlling-via-sysfs/148801#148801
// https://github.com/Pioreactor/rpi_hardware_pwm/blob/main/rpi_hardware_pwm/__init__.py

// internal state - one entry per motor 
typedef struct {
    int       channel;
    char      pwm_path[128];   // this is the low-level hardware rpm e.g. /sys/class/pwm/pwmchip0/pwm2
    bool      initialised;
    long long pwm_start_us;    // timestamp when enable=1 was written 
} EscChannel;

static EscChannel g_esc[2];
static long       g_period_ns = 0;

// ------------------------------------------------------------------ 
static long long now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static void pwm_write(const char *pwm_path, const char *attr, long value)
{
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", pwm_path, attr);
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return; }
    fprintf(f, "%ld\n", value);
    fclose(f);
}

static bool pwm_channel_init(EscChannel *esc, int chip, int channel,
                              long period_ns)
{
    char base[128];
    snprintf(base,sizeof(base),"/sys/class/pwm/pwmchip%d", chip);
    snprintf(esc->pwm_path, sizeof(esc->pwm_path), "%s/pwm%d", base, channel);
    esc->channel = channel;
    esc->initialised= false;
    esc->pwm_start_us = 0;

    // unexport first just in case it was left exported from a previous run 
    if (access(esc->pwm_path, F_OK) == 0) {
        char p[256];
        snprintf(p, sizeof(p), "%s/unexport", base);
        FILE *f = fopen(p, "w");
        if (f) { fprintf(f, "%d\n", channel);fclose(f); }
        usleep(100000);
    }

    char export_path[256];
    snprintf(export_path, sizeof(export_path), "%s/export", base);
    FILE *f = fopen(export_path, "w");
    if (!f) {perror("pwm export"); return false;}
    fprintf(f, "%d\n", channel);
    fclose(f);
    usleep(150000);

    pwm_write(esc->pwm_path,"period",period_ns);
    pwm_write(esc->pwm_path, "duty_cycle", (long)ESC_PULSE_STOP_US * 1000);

    // Record init timestamp to allow shutdown/disabling at safe OFF-portion of period
    // still if the progamr has run for some time (several minutes) it can 
    // happen that there was usfieicint jitter in the rpi to lose syncronisation in this tracking
    esc->pwm_start_us = now_us();
    pwm_write(esc->pwm_path, "enable", 1);

    esc->initialised = true;
    fprintf(stderr, "[esc] channel %d ready  period=%ld ns\n", channel, period_ns);
    return true;
}

static void pwm_set_us(EscChannel *esc, double us)
{
    if (!esc->initialised) return;
    long ns = (long)(us * 1000.0);
    if (ns > g_period_ns) ns = g_period_ns;
    if (ns < 0)           ns = 0;
    pwm_write(esc->pwm_path, "duty_cycle", ns);
}

// wait until  it is safely inside the OFF portion of the PWM period.
 // OFF window: [duty_us + margin, period_us - margin)

static void wait_for_off_window(EscChannel *esc)
{
    const long period_us = g_period_ns / 1000;
    const long duty_us = ESC_PULSE_STOP_US; // neutral, written just before
    const long margin_us = 200;// micro secs clear of edges in case of jitter
    const long safe_on_us = duty_us  + margin_us; // OFF starts here
    const long safe_end_us = period_us - margin_us;// OFF ends here  

    long long elapsed = now_us() - esc->pwm_start_us;
    long      phase   = (long)(elapsed % (long long)period_us);

    long wait = 0;
    if (phase >= safe_on_us && phase < safe_end_us) {
        // already in the safe OFF window => no wait needed
        return;
    } else if (phase < safe_on_us) {
        // still in or just after ON => wait until safe_on_us
        wait = safe_on_us - phase;
    } else {
        // too close to the next rising edge => skip to OFF window of next period
        wait = (period_us - phase) + safe_on_us;
    }

    usleep((useconds_t)wait);
}

static void pwm_channel_close(EscChannel *esc)
{
    if (!esc->initialised) return;

    // Write neutral duty cycle 
    pwm_write(esc->pwm_path, "duty_cycle", (long)ESC_PULSE_STOP_US * 1000);

    //let neutral settle for several periods so the ESC accepts it
    usleep(25000);   //5 periods at 200Hz

    //wait until it is in the OFF window before disabling
    wait_for_off_window(esc);

    //disable
    pwm_write(esc->pwm_path, "enable", 0);

    // unexport
    char base[128];
    snprintf(base, sizeof(base), "/sys/class/pwm/pwmchip%d", ESC_PWM_CHIP);
    char p[256];
    snprintf(p, sizeof(p), "%s/unexport", base);
    FILE *f = fopen(p, "w");
    if (f) {fprintf(f, "%d\n", esc->channel); fclose(f);}

    esc->initialised  = false;
    esc->pwm_start_us = 0;
    fprintf(stderr, "[esc] channel %d closed\n", esc->channel);
}

// Throttle percentage -> pulse width conv.
static double throttle_to_us(double pct)
{
    if (pct >  100.0) pct = 100.0;
    if (pct < -100.0) pct = -100.0;

    if (pct >= 0.0)
        return ESC_PULSE_STOP_US
             + (pct / 100.0) * (ESC_PULSE_FWD_MAX_US - ESC_PULSE_STOP_US);
    else
        return ESC_PULSE_STOP_US
             + (pct / 100.0) * (ESC_PULSE_STOP_US - ESC_PULSE_REV_MAX_US);
}

// public commands
bool esc_init(int freq_hz)
{
    if (freq_hz <= 0) freq_hz = ESC_DEFAULT_FREQ_HZ;
    g_period_ns = 1000000000L / freq_hz;

    bool ok = true;
    ok &= pwm_channel_init(&g_esc[0], ESC_PWM_CHIP, ESC_PWM_CHANNEL_1, g_period_ns);
    ok &= pwm_channel_init(&g_esc[1], ESC_PWM_CHIP, ESC_PWM_CHANNEL_2, g_period_ns);
    return ok;
}

void esc_set_throttle_1(double pct)
{
    pwm_set_us(&g_esc[0], throttle_to_us(pct));
}

void esc_set_throttle_2(double pct)
{
    pwm_set_us(&g_esc[1], throttle_to_us(pct));
}

void esc_set_throttle_both(double pct1, double pct2)
{
    esc_set_throttle_1(pct1);
    esc_set_throttle_2(pct2);
}

void esc_cleanup(void)
{
    pwm_channel_close(&g_esc[0]);
    pwm_channel_close(&g_esc[1]);
}
