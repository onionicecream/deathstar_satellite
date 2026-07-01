// this is the main code that prints the command line interface (CLI) and interfaces with the controller and constellation_sensor
//
// what threads it has:
// - camera_thread -> ConstellationSensor::update() -> g_cam_data
// - imu_thread -> RTIMULib at ~200 Hz -> g_imu_data
// - control_thread -> 10 Hz loop, updateController  -> ESC
// - main -> has the CLI
//
// esc: esc_set_throttle_1 = pitch, esc_set_throttle_2 = yaw
// throttle scale throughout: percent [0, 100]
// ------------------------------------------------------------------------
#include "RTIMULib.h"
#include "constellation_sensor.hpp"
#include "controller.hpp"

extern "C" {
    #include "esc.h"
    #include "hall.h"
    #include "gpio_outputs.h"
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <cmath>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <opencv2/opencv.hpp>

static constexpr double CTRL_HZ = 10.0;
static constexpr double CTRL_DT = 1.0 / CTRL_HZ;
static constexpr long CTRL_SLEEP_US = (long)(CTRL_DT * 1e6);
static constexpr double PI = M_PI;

// ------------------------------------------------------------------------
//  pattern tracking + slew interpolation
// ------------------------------------------------------------------------

//target point offsets; must match TARGET_POINTS in constellation_sensor.cpp
static const double TARGET_LOCAL_X[] = { 0.00,  0.061, -0.061,  0.00,   0.00  };
static const double TARGET_LOCAL_Y[] = { 0.00,  0.00,   0.00,   0.031, -0.031 };
static const int NUM_TARGETS = 5;

// tracking sequence {index of point; how much time to stay at point;}
struct TrackStep { int point_index; double dwell_seconds; };
static const TrackStep TRACK_SEQUENCE[] = {
    {0, 5.0},
    {1, 5.0},
    {0, 5.0},
    {2, 5.0},
    // {0, 5.0},
    // {3, 2.0},
    // {0, 5.0},
    // {4, 2.0},
      // {0, 7.0},
      // {10, 7.0},
};
static const int NUM_TRACK_STEPS = (int)(sizeof(TRACK_SEQUENCE) / sizeof(TRACK_SEQUENCE[0]));


struct SlewState {
    double cur_x = 0.0; // current interpolated position [m]
    double cur_y = 0.0;
    double step_x = 0.0; // per-tick increment during transition
    double step_y = 0.0;
    int steps_left = 0;
    double speed_x_cm_s = 2.2; // slew speed: x (yaw) axis [cm/s]
    double speed_y_cm_s = 4.5; // slew speed: y (pitch) axis [cm/s]
    // sequencer state
    int seq_step = 0; // current index into TRACK_SEQUENCE
    double dwell_elapsed = 0.0; // time spent on-target at current step [s]
    bool arrived = false; // laser is within arrival_norm_cm of the target
    double arrival_norm_cm = 1.0; // threshold to declare arrival [cm]
                                //(if below this from point consider pointing arrived;
                                // it is a quite generous, just so the slew parameters are turned off
                                //soon enough; then the pointing pid will bring it to the exact point
                                // also for dwell timer (how much time to stay at point)
};

// ------------------------------------------------------------------------
//  shared state
// ------------------------------------------------------------------------

struct SharedCam {
    SensorData data;
    uint64_t stamp_us = 0;
};

struct SharedImu {
    double omega_x = 0.0; // rad/s
    double omega_y = 0.0;
    double omega_z = 0.0;
};

// snapshot written by control_thread, read by camera_thread for the status window
struct StatusSnap {
    SharedCam cam;
    SharedImu imu;
    ActuatorOutputs out;
    double error_sum_pitch = 0.0;
    double error_sum_yaw = 0.0;
    bool is_slewing = false;
    bool pattern_on = false;
    int seq_step = 0;
    double dwell_elapsed = 0.0;
    bool arrived = false;
    int steps_left = 0;
    bool valid = false;
};

static SharedCam g_cam_data;
static pthread_mutex_t g_cam_mutex = PTHREAD_MUTEX_INITIALIZER;

static SharedImu g_imu_data;
static pthread_mutex_t g_imu_mutex = PTHREAD_MUTEX_INITIALIZER;

static ControllerParams g_params;
static ControllerState g_state;
static pthread_mutex_t g_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;

static ActuatorOutputs g_last_out;

static StatusSnap g_status_snap;
static pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;

static volatile bool g_running = true;
static volatile bool g_laser_on = true; //  laser on at startup but the range check below is applied
static volatile double g_laser_max_deg = 11.0; // laser off if |yaw| or |pitch| exceeds this
static volatile bool g_laser_active = false; // actual physical laser state
static volatile bool g_status_win_on = false;

// csv recording
static volatile bool g_record_on = false;
static FILE* g_record_file = nullptr;
static pthread_mutex_t g_record_mutex = PTHREAD_MUTEX_INITIALIZER;

// pattern tracking
static SlewState g_slew;

static ConstellationSensor* g_sensor = nullptr;

// ------------------------------------------------------------------------
//  utilities
// ------------------------------------------------------------------------

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

static double now_s()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// ------------------------------------------------------------------------
//  camera thread
// ------------------------------------------------------------------------

static void* camera_thread(void* arg)
{
    ConstellationSensor* sensor = static_cast<ConstellationSensor*>(arg);

    static constexpr const char* STATUS_WIN = "Status";

    while (g_running) {
        SensorData d = sensor->update(200);

        pthread_mutex_lock(&g_cam_mutex);
        g_cam_data.data = d;
        g_cam_data.stamp_us = now_us();
        pthread_mutex_unlock(&g_cam_mutex);

        // render status window if enabled (all opencv gui from this thread)
        if (g_status_win_on) {
            StatusSnap snap;
            pthread_mutex_lock(&g_status_mutex);
            snap = g_status_snap;
            pthread_mutex_unlock(&g_status_mutex);

            cv::Mat win(340, 500, CV_8UC3, cv::Scalar(0, 0, 0));

            auto txt = [&](const char* s, int y, cv::Scalar col = {200, 200, 200}) {
                cv::putText(win, s, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, col, 1);
            };

            char buf[128];
            int y = 30;
            txt("--- sensor ---", y); y += 28;

            SensorData& cd = snap.cam.data;
            std::snprintf(buf, sizeof(buf), "cam: %s  dist:%.1fcm  fps:%.1f",
                          cd.valid ? "ok" : "---", cd.dist_cm, cd.fps);
            txt(buf, y, cd.valid ? cv::Scalar{0, 255, 100} : cv::Scalar{0, 80, 255}); y += 28;

            if (cd.valid) {
                std::snprintf(buf, sizeof(buf), "yaw:%+.2f  pitch:%+.2f deg",
                              cd.yaw_deg, cd.pitch_deg);
                txt(buf, y); y += 28;
            }

            if (cd.laser_valid) {
                std::snprintf(buf, sizeof(buf), "laser dx:%.2fcm  dy:%.2fcm  idx:%d",
                              cd.laser_dx_m * 100.0, cd.laser_dy_m * 100.0,
                              cd.active_target_index);
                txt(buf, y, {0, 255, 200}); y += 28;
            } else {
                txt("laser: no dot", y, {100, 100, 100}); y += 28;
            }

            y += 5;
            txt("--- controller ---", y); y += 28;

            std::snprintf(buf, sizeof(buf), "thr_p:%.1f  thr_y:%.1f",
                          snap.out.throttle1 * 100.0, snap.out.throttle2 * 100.0);
            txt(buf, y); y += 28;

            std::snprintf(buf, sizeof(buf), "imu wy:%.3f  wz:%.3f rad/s",
                          snap.imu.omega_y, snap.imu.omega_z);
            txt(buf, y); y += 28;

            std::snprintf(buf, sizeof(buf), "int_p:%.3f  int_y:%.3f",
                          snap.error_sum_pitch, snap.error_sum_yaw);
            txt(buf, y); y += 28;

            if (snap.pattern_on) {
                int si = snap.seq_step;
                double dwell_s = (si >= 0 && si < NUM_TRACK_STEPS)
                                 ? TRACK_SEQUENCE[si].dwell_seconds : 0.0;
                int pt = (si >= 0 && si < NUM_TRACK_STEPS)
                         ? TRACK_SEQUENCE[si].point_index : 0;
                if (snap.steps_left > 0) {
                    std::snprintf(buf, sizeof(buf), "slew->pt%d  steps:%d", pt, snap.steps_left);
                    txt(buf, y, cv::Scalar{0, 200, 255});
                } else if (!snap.arrived) {
                    std::snprintf(buf, sizeof(buf), "pt%d: waiting arrival", pt);
                    txt(buf, y, cv::Scalar{100, 200, 255});
                } else {
                    std::snprintf(buf, sizeof(buf), "pt%d: dwell %.1f/%.1fs", pt,
                                  snap.dwell_elapsed, dwell_s);
                    txt(buf, y, cv::Scalar{0, 255, 150});
                }
            } else {
                std::snprintf(buf, sizeof(buf), "slew: %s", snap.is_slewing ? "YES" : "no");
                txt(buf, y, snap.is_slewing ? cv::Scalar{0, 200, 255} : cv::Scalar{200, 200, 200});
            }

            cv::imshow(STATUS_WIN, win);
        }

        cv::waitKey(1);
    }

    if (g_status_win_on)
        cv::destroyWindow(STATUS_WIN);

    return nullptr;
}

// ------------------------------------------------------------------------
//  imu thread
// ------------------------------------------------------------------------

static void* imu_thread(void* /*arg*/)
{
    RTIMUSettings* settings = new RTIMUSettings("RTIMULib");
    RTIMU* imu = RTIMU::createIMU(settings);

    if (!imu || imu->IMUType() == RTIMU_TYPE_NULL) {
        fprintf(stderr, "no imu found\n");
        g_running = false;
        delete settings;
        return nullptr;
    }

    imu->IMUInit();
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);

    while (g_running) {
        usleep(5000); // drain at  200 Hz
        while (imu->IMURead()) {
            RTIMU_DATA data = imu->getIMUData();
            pthread_mutex_lock(&g_imu_mutex);
            g_imu_data.omega_x = data.gyro.x();
            g_imu_data.omega_y = data.gyro.y();
            g_imu_data.omega_z = data.gyro.z();
            pthread_mutex_unlock(&g_imu_mutex);
        }
    }

    delete imu;
    delete settings;
    return nullptr;
}

// ------------------------------------------------------------------------
//  status line: termial display with refresh
// ------------------------------------------------------------------------

static volatile bool g_status_first = true;

static void print_status_line(const SensorData& cam, const SharedImu& imu,
                               const ActuatorOutputs& out,
                               const ControllerState& state)
{
    if (!g_status_first)
        // move cursor up 3 lines to overwrite previous status,
        //so that it looks like updating text
        printf("\033[3A");
    g_status_first = false;

    char pid_str[32], ff_str[32];
    snprintf(pid_str, sizeof(pid_str), "pid[%c%c]",
             state.pid_pitch_on ? 'p' : '-', state.pid_yaw_on ? 'y' : '-');
    snprintf(ff_str, sizeof(ff_str), "ff[%c%c]",
             state.ff_pitch_on ? 'p' : '-', state.ff_yaw_on ? 'y' : '-');

    printf("cam:%s yaw:%+6.2f pit:%+6.2f  laser:%s dx:%+5.2fcm dy:%+5.2fcm\033[K\n",
           cam.valid ? "ok " : "---",
           cam.valid ? cam.yaw_deg : 0.0,
           cam.valid ? cam.pitch_deg : 0.0,
           cam.laser_valid ? "ok " : "---",
           cam.laser_valid ? cam.laser_dx_m * 100.0 : 0.0,
           cam.laser_valid ? cam.laser_dy_m * 100.0 : 0.0);

    printf("thr_p:%5.1f thr_y:%5.1f  %s %s  %s  %s  rec:%s\033[K\n",
           out.throttle1 * 100.0, out.throttle2 * 100.0,
           pid_str, ff_str,
           state.controllers_enabled ? "ON " : "OFF",
           state.is_slewing ? "SLEW" : "----",
           g_record_on ? "ON " : "off");

    printf("imu wy:%+6.3f wz:%+6.3f rad/s  int_p:%+5.3f int_y:%+5.3f\033[K\n",
           imu.omega_y, imu.omega_z,
           state.error_sum_pitch, state.error_sum_yaw);

    fflush(stdout);
}

// ------------------------------------------------------------------------
//  slew tick: sequencer for waypoint creation, called once per control tick.
//  dwell timer starts only after laser is within arrival_norm_cm of target.
//  returns true when a new target is triggered
// ------------------------------------------------------------------------

static bool tick_slew(SlewState& s, ConstellationSensor* sensor,
                      bool laser_valid, double laser_dx_m, double laser_dy_m)
{
    // check arrival once slew is done; latch until next step
    if (s.steps_left == 0 && !s.arrived && laser_valid) {
        double err_m = std::sqrt(laser_dx_m*laser_dx_m + laser_dy_m*laser_dy_m);
        if (err_m < s.arrival_norm_cm * 0.01)
            s.arrived = true;
    }

    // accumulate dwell only after arrival
    if (s.arrived)
        s.dwell_elapsed += CTRL_DT;

    // increment sequence when dwell complete
    bool triggered = false;
    if (s.arrived && s.dwell_elapsed >= TRACK_SEQUENCE[s.seq_step].dwell_seconds) {
        s.seq_step = (s.seq_step + 1) % NUM_TRACK_STEPS;
        s.dwell_elapsed = 0.0;
        s.arrived = false;

        int ni = TRACK_SEQUENCE[s.seq_step].point_index;
        if (ni < 0 || ni >= NUM_TARGETS) ni = 0;
        int oi = sensor->activeTargetIndex();
        if (oi < 0 || oi >= NUM_TARGETS) oi = 0;
        double dx = TARGET_LOCAL_X[ni] - TARGET_LOCAL_X[oi];
        double dy = TARGET_LOCAL_Y[ni] - TARGET_LOCAL_Y[oi];
        double step_xm = s.speed_x_cm_s * 0.01 * CTRL_DT;
        double step_ym = s.speed_y_cm_s * 0.01 * CTRL_DT;
        int sx = (std::fabs(dx) > 1e-9) ? std::max(1, (int)std::ceil(std::fabs(dx) / step_xm)) : 0;
        int sy = (std::fabs(dy) > 1e-9) ? std::max(1, (int)std::ceil(std::fabs(dy) / step_ym)) : 0;
        int steps = std::max(1, std::max(sx, sy));
        s.step_x = dx / steps;
        s.step_y = dy / steps;
        s.steps_left = steps;
        sensor->setActiveTargetIndex(ni);
        triggered = true;
    }

    // increment interpolation one step per tick
    if (s.steps_left > 0) {
        s.cur_x += s.step_x;
        s.cur_y += s.step_y;
        s.steps_left--;
        if (s.steps_left == 0) {
            int ai = sensor->activeTargetIndex();
            if (ai >= 0 && ai < NUM_TARGETS) {
                s.cur_x = TARGET_LOCAL_X[ai];
                s.cur_y = TARGET_LOCAL_Y[ai];
            }
        }
    }

    return triggered;
}

// ------------------------------------------------------------------------
//  control thread
// ------------------------------------------------------------------------

static void* control_thread(void* /*arg*/)
{
    uint64_t last_status_us = 0;

    while (g_running) {
        usleep(CTRL_SLEEP_US);

        // inputs
        SharedCam cam;
        pthread_mutex_lock(&g_cam_mutex);
        cam = g_cam_data;
        pthread_mutex_unlock(&g_cam_mutex);

        SharedImu imu;
        pthread_mutex_lock(&g_imu_mutex);
        imu = g_imu_data;
        pthread_mutex_unlock(&g_imu_mutex);

        // camera check (>500 ms = invalid)
        uint64_t age = now_us() - cam.stamp_us;
        if (age > 500000) cam.data.valid = false;

        // gate laser: only when camera valid and within angular range
        {
            bool in_range = cam.data.valid &&
                            std::fabs(cam.data.yaw_deg) <= g_laser_max_deg &&
                            std::fabs(cam.data.pitch_deg) <= g_laser_max_deg;
            bool want = g_laser_on && in_range;
            if (want != g_laser_active) {
                laser_set(want);
                g_laser_active = want;
            }
        }

        // hall sensor wheel speeds
        double rpm1 = hall_get_rpm_avg(0, 8);
        double rpm2 = hall_get_rpm_avg(1, 8);
        double omega1_act = rpm1 * 2.0 * PI / 60.0;
        double omega2_act = rpm2 * 2.0 * PI / 60.0;

        // pattern tracking: advance slew interpolation and update active target
        {
            pthread_mutex_lock(&g_ctrl_mutex);
            bool pat = g_state.pattern_enabled;
            pthread_mutex_unlock(&g_ctrl_mutex);
            if (pat && g_sensor) {
                if (tick_slew(g_slew, g_sensor,
                              cam.data.laser_valid, cam.data.laser_dx_m, cam.data.laser_dy_m)) {
                    pthread_mutex_lock(&g_ctrl_mutex);
                    g_state.slew_requested = true;
                    pthread_mutex_unlock(&g_ctrl_mutex);
                }
            }
        }
        if (g_sensor)
            g_sensor->setInterimPoint(g_slew.cur_x, g_slew.cur_y);

        // build SensorInputs
        SensorInputs sensors = {};
        sensors.cam_valid = cam.data.valid;
        sensors.theta_pitch_cam = cam.data.valid ? cam.data.pitch_deg * PI / 180.0 : 0.0;
        sensors.theta_yaw_cam = cam.data.valid ? cam.data.yaw_deg * PI / 180.0 : 0.0;

        sensors.laser_valid = cam.data.laser_valid;
        double tz_m = cam.data.tz_cm / 100.0;
        if (cam.data.laser_valid && tz_m > 0.01) {
            sensors.theta_yaw_laser = std::atan2(-cam.data.laser_dx_m, tz_m);
            sensors.theta_pitch_laser = std::atan2(-cam.data.laser_dy_m, tz_m);
        }

        sensors.omega_pitch = imu.omega_y;
        sensors.omega_yaw = imu.omega_z;
        sensors.omega_yaw_des   = (g_slew.steps_left > 0 && tz_m > 0.1)
                                ?  g_slew.step_x / (tz_m * CTRL_DT) : 0.0;
        sensors.omega_pitch_des = (g_slew.steps_left > 0 && tz_m > 0.1)
                                ?  g_slew.step_y / (tz_m * CTRL_DT) : 0.0;
        sensors.omega_body = Eigen::Vector3d(imu.omega_x, imu.omega_y, imu.omega_z);

        sensors.Omega1_act = omega1_act;
        sensors.Omega2_act = omega2_act;

        sensors.alpha_des = Eigen::Vector3d::Zero();

        // absolute setpoint angle from slew position in target plane.
        // always computed when camera is valid, regardless of laser validity.
        // used by the feedforward (gravity compensation at commanded pose) and
        // by the PID in no-laser mode (drives camera angle to match setpoint).
        // in laser mode the PID uses 0.0 as its desired (laser-error space),
        // so theta_pitch/yaw_des being nonzero here does not affect the PID.
        sensors.theta_pitch_des = 0.0;
        sensors.theta_yaw_des = 0.0;
        if (tz_m > 0.1) {
            sensors.theta_yaw_des   = std::atan2( g_slew.cur_x, tz_m);
            sensors.theta_pitch_des = std::atan2(-g_slew.cur_y, tz_m);
        }

        // run controller
        pthread_mutex_lock(&g_ctrl_mutex);
        ActuatorOutputs out = updateController(sensors, g_params, g_state, CTRL_DT);
        int log_active_idx = g_sensor ? g_sensor->activeTargetIndex() : 0;
        double log_cam_off_pitch = g_state.cam_offset_pitch;
        double log_cam_off_yaw = g_state.cam_offset_yaw;
        bool log_slew_active = g_state.is_slewing;
        double log_kp_pitch = g_params.Kp_pitch;
        double log_ki_pitch = g_params.Ki_pitch;
        double log_kd_pitch = g_params.Kd_pitch;
        double log_kp_yaw = g_params.Kp_yaw;
        double log_ki_yaw = g_params.Ki_yaw;
        double log_kd_yaw = g_params.Kd_yaw;
        double log_error_sum_pitch = g_state.error_sum_pitch;
        double log_error_sum_yaw = g_state.error_sum_yaw;
        double log_Omega1_des = g_state.Omega1_des;
        double log_Omega2_des = g_state.Omega2_des;
        bool log_pid_pitch_on = g_state.pid_pitch_on;
        bool log_pid_yaw_on = g_state.pid_yaw_on;
        bool log_ff_pitch_on = g_state.ff_pitch_on;
        bool log_ff_yaw_on = g_state.ff_yaw_on;
        pthread_mutex_unlock(&g_ctrl_mutex);

        // drive ESC (controller [0,1], esc functions expect [0,100])
        esc_set_throttle_1(out.throttle1 * 100.0); // pitch
        esc_set_throttle_2(out.throttle2 * 100.0); // yaw
        g_last_out = out;

        // csv recording
        pthread_mutex_lock(&g_record_mutex);
        bool rec = g_record_on;
        FILE* rf = g_record_file;
        pthread_mutex_unlock(&g_record_mutex);

        if (rec && rf) {
            double ts = now_s();
            if (cam.data.valid)
                fprintf(rf, "%.4f,%.5f,%.5f,", ts,
                        sensors.theta_pitch_cam, sensors.theta_yaw_cam);
            else
                fprintf(rf, "%.4f,nan,nan,", ts);

            if (cam.data.laser_valid)
                fprintf(rf, "%.5f,%.5f,%.5f,%.5f,",
                        sensors.theta_pitch_laser, sensors.theta_yaw_laser,
                        cam.data.laser_dx_m, cam.data.laser_dy_m);
            else
                fprintf(rf, "nan,nan,nan,nan,");

            double log_e_pitch = sensors.theta_pitch_des - (cam.data.laser_valid
                ? sensors.theta_pitch_laser : sensors.theta_pitch_cam + log_cam_off_pitch);
            double log_e_yaw = sensors.theta_yaw_des - (cam.data.laser_valid
                ? sensors.theta_yaw_laser : sensors.theta_yaw_cam + log_cam_off_yaw);
            fprintf(rf, "%.5f,%.5f,%.5f,%.2f,%.2f,",
                    imu.omega_x, imu.omega_y, imu.omega_z,
                    out.throttle1, out.throttle2);
            fprintf(rf, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.3f,%.5f,%.5f,",
                    g_slew.cur_x, g_slew.cur_y,
                    sensors.theta_pitch_des, sensors.theta_yaw_des,
                    log_cam_off_pitch, log_cam_off_yaw,
                    log_slew_active ? 1 : 0,
                    cam.data.valid ? cam.data.dist_cm : (double)NAN,
                    sensors.omega_pitch_des, sensors.omega_yaw_des);
            fprintf(rf, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                    log_kp_pitch, log_ki_pitch, log_kd_pitch,
                    log_kp_yaw, log_ki_yaw, log_kd_yaw);
            fprintf(rf, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%d,%d,%d,",
                    log_error_sum_pitch, log_error_sum_yaw,
                    omega1_act, omega2_act,
                    out.u_ff1, out.u_ff2,
                    out.u_fb1, out.u_fb2,
                    log_e_pitch, log_e_yaw,
                    log_Omega1_des, log_Omega2_des,
                    cam.data.valid ? 1 : 0,
                    cam.data.laser_valid ? 1 : 0,
                    log_active_idx,
                    log_pid_pitch_on ? 1 : 0,
                    log_pid_yaw_on ? 1 : 0,
                    log_ff_pitch_on ? 1 : 0,
                    log_ff_yaw_on ? 1 : 0);

            // absolute laser position and angles (board-local, origin = board center)
            int safe_ai = (log_active_idx >= 0 && log_active_idx < NUM_TARGETS) ? log_active_idx : 0;
            double tgt_x = TARGET_LOCAL_X[safe_ai];
            double tgt_y = TARGET_LOCAL_Y[safe_ai];
            if (cam.data.laser_valid) {
                double lax = cam.data.laser_abs_x_m;
                double lay = cam.data.laser_abs_y_m;
                double lab_yaw   = (tz_m > 0.01) ? std::atan2( lax, tz_m) : (double)NAN;
                double lab_pitch = (tz_m > 0.01) ? std::atan2(-lay, tz_m) : (double)NAN;
                fprintf(rf, "%.5f,%.5f,%.5f,%.5f,",
                        lax, lay, lab_pitch, lab_yaw);
            } else {
                fprintf(rf, "nan,nan,nan,nan,");
            }
            double tgt_yaw   = (tz_m > 0.01) ? std::atan2( tgt_x, tz_m) : (double)NAN;
            double tgt_pitch = (tz_m > 0.01) ? std::atan2(-tgt_y, tz_m) : (double)NAN;
            fprintf(rf, "%.5f,%.5f,%.5f,%.5f,%d\n",
                    tgt_x, tgt_y, tgt_pitch, tgt_yaw,
                    g_slew.arrived ? 1 : 0);
            fflush(rf);
        }

        // update status snapshot for status window
        {
            pthread_mutex_lock(&g_status_mutex);
            g_status_snap.cam = cam;
            g_status_snap.imu = imu;
            g_status_snap.out = out;
            pthread_mutex_lock(&g_ctrl_mutex);
            g_status_snap.error_sum_pitch = g_state.error_sum_pitch;
            g_status_snap.error_sum_yaw = g_state.error_sum_yaw;
            g_status_snap.is_slewing = g_state.is_slewing;
            g_status_snap.pattern_on = g_state.pattern_enabled;
            pthread_mutex_unlock(&g_ctrl_mutex);
            g_status_snap.seq_step = g_slew.seq_step;
            g_status_snap.dwell_elapsed = g_slew.dwell_elapsed;
            g_status_snap.arrived = g_slew.arrived;
            g_status_snap.steps_left = g_slew.steps_left;
            g_status_snap.valid = true;
            pthread_mutex_unlock(&g_status_mutex);
        }

        // terminal status line at ~2 Hz
        uint64_t now = now_us();
        if (now - last_status_us > 500000) {
            pthread_mutex_lock(&g_ctrl_mutex);
            ControllerState snap_state = g_state;
            pthread_mutex_unlock(&g_ctrl_mutex);
            print_status_line(cam.data, imu, out, snap_state);
            last_status_us = now;
        }
    }

    esc_set_throttle_1(0.0);
    esc_set_throttle_2(0.0);
    return nullptr;
}

// ------------------------------------------------------------------------
//  threads
// ------------------------------------------------------------------------

static pthread_t g_thr_cam;
static pthread_t g_thr_imu;
static pthread_t g_thr_ctrl;

// ------------------------------------------------------------------------
//  shutdown
// ------------------------------------------------------------------------

static void shutdown_all()
{
    printf("\n\ncleaning up...\n");

    g_running = false;

    pthread_mutex_lock(&g_ctrl_mutex);
    g_state.controllers_enabled = false;
    pthread_mutex_unlock(&g_ctrl_mutex);

    pthread_join(g_thr_ctrl, nullptr);
    pthread_join(g_thr_cam, nullptr);
    pthread_join(g_thr_imu, nullptr);

    esc_set_throttle_both(0.0, 0.0);

    if (g_sensor) g_sensor->shutdown();

    laser_set(false);

    pthread_mutex_lock(&g_record_mutex);
    if (g_record_file) { fclose(g_record_file); g_record_file = nullptr; }
    pthread_mutex_unlock(&g_record_mutex);

    hall_cleanup();
    esc_cleanup();
    gpio_outputs_cleanup();

    pthread_mutex_lock(&g_ctrl_mutex);
    printf("\n// pid gains\n");
    printf("kp_pitch=%.4f ki_pitch=%.4f kd_pitch=%.4f\n",
           g_params.Kp_pitch, g_params.Ki_pitch, g_params.Kd_pitch);
    printf("kp_yaw=%.4f   ki_yaw=%.4f   kd_yaw=%.4f\n",
           g_params.Kp_yaw, g_params.Ki_yaw, g_params.Kd_yaw);
    pthread_mutex_unlock(&g_ctrl_mutex);

    printf("bye\n");
}

static void sig_handler(int /*sig*/)
{
    shutdown_all();
    exit(0);
}

// ------------------------------------------------------------------------
//  cli
// ------------------------------------------------------------------------

static void print_help()
{
    printf("\n"
           "  h                    this help\n"
           "\n"
           "  on pid               enable pitch + yaw pid\n"
           "  on pid p             enable pitch pid only\n"
           "  on pid y             enable yaw pid only\n"
           "  on ff                enable pitch + yaw feedforward\n"
           "  on ff p              enable pitch ff only\n"
           "  on ff y              enable yaw ff only\n"
           "  off / s              disable all control (pid + ff)\n"
           "\n"
           "  enable / e           arm controllers (begin startup ramp)\n"
           "  disable / d          disarm controllers\n"
           "\n"
           "  pat           start/stop tracking pattern loop\n"
           "\n"
           "  rec         start/stop csv logging to logs/\n"
           "\n"
           "  p kp <v>  p ki <v>  p kd <v>      pitch pid gains\n"
           "  y kp <v>  y ki <v>  y kd <v>      yaw pid gains\n"
           "  p kis <v>  y kis <v>       integral gain during slewing (0=blocked)\n"
           "  p drain <v>    y drain <v>          integral drain rate per axis\n"
           "  p offset <v>   y offset <v>         throttle offset percent\n"
           "  p pid / y pid                       show pid params\n"
           "\n"
           "  slew speed <v>        slew speed cm/s (both axes)\n"
           "  slew speed p <v>     slew speed pitch axis cm/s\n"
           "  slew speed y <v>     slew speed yaw axis cm/s\n"
           "  slew norm <v>        arrival threshold in cm; starts dwell timer\n"
           "\n"
           "  com <v>              set CoM offset to v mm below pivot (-z axis)\n"
           "\n"
           "  calib                set current cam error as zero\n"
           "  laser                toggle laser on/off intent\n"
           "  laser range <v>      laser off if |yaw| or |pitch| exceed v deg\n"
           "  disp              toggle camera display window\n"
           "  swin                 toggle status window\n"
           "  stat               full status dump\n"
           "\n"
           "  q / x                stop motors, cleanup, quit\n"
           "\n");
}

static void start_recording()
{
    struct stat st;
    if (stat("logs", &st) != 0) mkdir("logs", 0755);

    time_t t = time(nullptr);
    struct tm* tm = localtime(&t);
    char fname[64];
    strftime(fname, sizeof(fname), "logs/bep_ctrl4_%Y%m%d_%H%M%S.csv", tm);

    FILE* f = fopen(fname, "w");
    if (!f) { printf("failed to open %s\n", fname); return; }

    // header of log
    fprintf(f, "timestamp_s,"
               "cam_pitch_rad,cam_yaw_rad,"
               "laser_pitch_rad,laser_yaw_rad,"
               "laser_dx_m,laser_dy_m,"
               "imu_omega_x_rad_s,imu_omega_y_rad_s,imu_omega_z_rad_s,"
               "throttle_pitch,throttle_yaw,"
               "setpoint_laser_dx_m,setpoint_laser_dy_m,"
               "cam_setpoint_pitch_rad,cam_setpoint_yaw_rad,"
               "cam_offset_pitch_rad,cam_offset_yaw_rad,"
               "mode_slew,dist_cm,"
               "omega_des_pitch_rad_s,omega_des_yaw_rad_s,"
               "kp_pitch,ki_pitch,kd_pitch,kp_yaw,ki_yaw,kd_yaw,"
               "error_sum_pitch,error_sum_yaw,"
               "omega1_act_rad_s,omega2_act_rad_s,"
               "u_ff_pitch,u_ff_yaw,u_fb_pitch,u_fb_yaw,"
               "e_pitch_rad,e_yaw_rad,"
               "Omega1_des_rad_s,Omega2_des_rad_s,"
               "cam_valid,laser_valid,active_target_idx,"
               "pid_pitch_on,pid_yaw_on,ff_pitch_on,ff_yaw_on,"
               "laser_abs_x_m,laser_abs_y_m,laser_abs_pitch_rad,laser_abs_yaw_rad,"
               "target_abs_x_m,target_abs_y_m,target_abs_pitch_rad,target_abs_yaw_rad,"
               "arrived\n");
    fflush(f);

    pthread_mutex_lock(&g_record_mutex);
    if (g_record_file) fclose(g_record_file);
    g_record_file = f;
    g_record_on = true;
    pthread_mutex_unlock(&g_record_mutex);

    printf("recording to %s\n", fname);
}

static void stop_recording()
{
    pthread_mutex_lock(&g_record_mutex);
    g_record_on = false;
    if (g_record_file) { fclose(g_record_file); g_record_file = nullptr; }
    pthread_mutex_unlock(&g_record_mutex);
    printf("recording stopped\n");
}

static bool handle_command(char* line)
{
    // strip trailing whitespace
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r' || line[len-1] == ' '))
        line[--len] = '\0';
    if (len == 0) return true;

    // quit
    if (strcasecmp(line, "q") == 0 || strcasecmp(line, "x") == 0)
        return false;

    // help
    if (strcasecmp(line, "h") == 0 || strcasecmp(line, "help") == 0) {
        print_help();
        return true;
    }

    // disable all control (keep motors at offset, just disable pid/ff)
    if (strcasecmp(line, "off") == 0 || strcasecmp(line, "s") == 0) {
        pthread_mutex_lock(&g_ctrl_mutex);
        g_state.pid_pitch_on = false;
        g_state.pid_yaw_on = false;
        g_state.ff_pitch_on = false;
        g_state.ff_yaw_on = false;
        pthread_mutex_unlock(&g_ctrl_mutex);
        printf("pid + ff disabled\n");
        return true;
    }

    // arm / disarm
    if (strcasecmp(line, "enable") == 0 || strcasecmp(line, "e") == 0) {
        pthread_mutex_lock(&g_ctrl_mutex);
        if (!g_state.controllers_enabled) {
            g_state.controllers_enabled = true;
            g_state.startup_done = false;
            g_state.startup_ramp_progress = 0.0;
            g_state.homing_done = false;
            g_state.homing_yaw_accumulated = 0.0;
            g_state.ff_pitch_initialized = false;
            g_state.ff_yaw_initialized = false;
            g_state.u_ff1_integrated = 0.0;
            g_state.u_ff2_integrated = 0.0;
        }
        pthread_mutex_unlock(&g_ctrl_mutex);
        printf("controllers armed (startup ramp will begin)\n");
        return true;
    }
    if (strcasecmp(line, "disable") == 0 || strcasecmp(line, "d") == 0) {
        pthread_mutex_lock(&g_ctrl_mutex);
        g_state.controllers_enabled = false;
        pthread_mutex_unlock(&g_ctrl_mutex);
        printf("controllers disarmed\n");
        return true;
    }

    // on pid / on ff
    if (strncasecmp(line, "on ", 3) == 0) {
        const char* sub = line + 3;
        pthread_mutex_lock(&g_ctrl_mutex);

        if (strcasecmp(sub, "pid") == 0) {
            g_state.pid_pitch_on = true;
            g_state.pid_yaw_on = true;
            g_state.error_sum_pitch = 0.0;
            g_state.error_sum_yaw = 0.0;
            printf("pid on (pitch + yaw)\n");
        } else if (strcasecmp(sub, "pid p") == 0) {
            g_state.pid_pitch_on = true;
            g_state.error_sum_pitch = 0.0;
            printf("pitch pid on\n");
        } else if (strcasecmp(sub, "pid y") == 0) {
            g_state.pid_yaw_on = true;
            g_state.error_sum_yaw = 0.0;
            printf("yaw pid on\n");
        } else if (strcasecmp(sub, "ff") == 0) {
            g_state.ff_pitch_on = true;
            g_state.ff_yaw_on = true;
            printf("feedforward on (pitch + yaw)\n");
        } else if (strcasecmp(sub, "ff p") == 0) {
            g_state.ff_pitch_on = true;
            printf("pitch feedforward on\n");
        } else if (strcasecmp(sub, "ff y") == 0) {
            g_state.ff_yaw_on = true;
            printf("yaw feedforward on\n");
        } else {
            printf("unknown: %s\n", line);
        }
        pthread_mutex_unlock(&g_ctrl_mutex);
        return true;
    }

    // pat toggle
    if (strcasecmp(line, "pat") == 0) {
        pthread_mutex_lock(&g_ctrl_mutex);
        bool on = !g_state.pattern_enabled;
        g_state.pattern_enabled = on;
        pthread_mutex_unlock(&g_ctrl_mutex);
        if (on) {
            int first_pt = TRACK_SEQUENCE[0].point_index;
            if (first_pt < 0 || first_pt >= NUM_TARGETS) first_pt = 0;
            g_slew.cur_x = TARGET_LOCAL_X[first_pt];
            g_slew.cur_y = TARGET_LOCAL_Y[first_pt];
            g_slew.step_x = 0.0;
            g_slew.step_y = 0.0;
            g_slew.steps_left = 0;
            g_slew.seq_step = 0;
            g_slew.dwell_elapsed = 0.0;
            g_slew.arrived = false;
            if (g_sensor) g_sensor->setActiveTargetIndex(first_pt);
            printf("tracking pattern started (waiting for arrival at pt%d)\n", first_pt);
        } else {
            g_slew.cur_x = 0.0;
            g_slew.cur_y = 0.0;
            g_slew.steps_left = 0;
            if (g_sensor) g_sensor->setActiveTargetIndex(0);
            printf("tracking pattern stopped, target index 0\n");
        }
        return true;
    }

    // slew speed (per-axis or both)
    if (strncasecmp(line, "slew speed ", 11) == 0) {
        const char* sub = line + 11;
        if (strncasecmp(sub, "p ", 2) == 0) {
            double v = atof(sub + 2);
            if (v > 0.0) { g_slew.speed_y_cm_s = v; printf("slew speed pitch: %.2f cm/s\n", v); }
            else printf("speed must be > 0\n");
        } else if (strncasecmp(sub, "y ", 2) == 0) {
            double v = atof(sub + 2);
            if (v > 0.0) { g_slew.speed_x_cm_s = v; printf("slew speed yaw: %.2f cm/s\n", v); }
            else printf("speed must be > 0\n");
        } else {
            double v = atof(sub);
            if (v > 0.0) {
                g_slew.speed_x_cm_s = v;
                g_slew.speed_y_cm_s = v;
                printf("slew speed: %.2f cm/s (both axes)\n", v);
            } else printf("speed must be > 0\n");
        }
        return true;
    }

    // slew arrival norm
    if (strncasecmp(line, "slew norm ", 10) == 0) {
        double v = atof(line + 10);
        if (v > 0.0) { g_slew.arrival_norm_cm = v; printf("slew norm: %.2f cm\n", v); }
        else printf("norm must be > 0\n");
        return true;
    }

    // rec toggle
    if (strcasecmp(line, "rec") == 0) { g_record_on ? stop_recording() : start_recording(); return true; }

    // laser
    if (strcasecmp(line, "laser") == 0) {
        g_laser_on = !g_laser_on;
        printf("laser intent: %s (physical: gated by camera+range)\n",
               g_laser_on ? "on" : "off");
        return true;
    }

    // laser angular range threshold
    if (strncasecmp(line, "laser range ", 12) == 0) {
        double v = atof(line + 12);
        if (v > 0.0) { g_laser_max_deg = v; printf("laser range: %.1f deg\n", v); }
        else printf("range must be > 0\n");
        return true;
    }

    // disp toggle
    if (strcasecmp(line, "disp") == 0) {
        if (g_sensor) {
            bool next = !g_sensor->displayEnabled();
            g_sensor->enableDisplay(next);
            printf("camera display %s\n", next ? "on" : "off");
        }
        return true;
    }

    // status window toggle
    if (strcasecmp(line, "swin") == 0) {
        g_status_win_on = !g_status_win_on;
        printf("status window %s\n", g_status_win_on ? "on" : "off");
        return true;
    }

    // calibration: reset integrators
    if (strcasecmp(line, "calib") == 0) {
        printf("point at target center and press space...\n");
        int c;
        do { c = getchar(); } while (c != ' ');
        while ((c = getchar()) != '\n' && c != EOF);
        printf("calibration: use camera zero offsets by setting theta_des to current cam error\n");
        pthread_mutex_lock(&g_ctrl_mutex);
        g_state.error_sum_pitch = 0.0;
        g_state.error_sum_yaw = 0.0;
        pthread_mutex_unlock(&g_ctrl_mutex);
        printf("integrators reset\n");
        return true;
    }

    // full status dump
    if (strcasecmp(line, "stat") == 0) {
        SharedCam cam;
        pthread_mutex_lock(&g_cam_mutex);
        cam = g_cam_data;
        pthread_mutex_unlock(&g_cam_mutex);

        SharedImu imu;
        pthread_mutex_lock(&g_imu_mutex);
        imu = g_imu_data;
        pthread_mutex_unlock(&g_imu_mutex);

        pthread_mutex_lock(&g_ctrl_mutex);
        ControllerState s = g_state;
        ControllerParams p = g_params;
        pthread_mutex_unlock(&g_ctrl_mutex);

        printf("\n");
        printf("  cam: %s  dist:%.1fcm  fps:%.1f  markers:%d\n",
               cam.data.valid ? "VALID" : "NO POSE",
               cam.data.dist_cm, cam.data.fps, cam.data.num_markers);
        printf("  cam yaw:%+.2fdeg  pitch:%+.2fdeg\n",
               cam.data.yaw_deg, cam.data.pitch_deg);
        printf("  laser: %s", cam.data.laser_valid ? "VALID" : "no dot");
        if (cam.data.laser_valid)
            printf("  dx:%.2fcm  dy:%.2fcm  yaw:%.2fdeg  pitch:%.2fdeg",
                   cam.data.laser_dx_m * 100.0, cam.data.laser_dy_m * 100.0,
                   cam.data.laser_yaw_deg, cam.data.laser_pitch_deg);
        printf("\n");
        printf("  imu: wy=%+.3f wz=%+.3f rad/s\n",
               imu.omega_y, imu.omega_z);
        printf("  throttle: thr_p=%.1f  thr_y=%.1f\n",
               g_last_out.throttle1 * 100.0, g_last_out.throttle2 * 100.0);
        printf("  controller: %s  homing:%s  startup:%s\n",
               s.controllers_enabled ? "ARMED" : "DISARMED",
               s.homing_done ? "done" : "pending",
               s.startup_done ? "done" : "ramp");
        printf("  pid p: [%s] kp=%.3f ki=%.3f kd=%.3f kis=%.3f drain=%.6f  int=%.4f\n",
               s.pid_pitch_on ? "ON " : "off",
               p.Kp_pitch, p.Ki_pitch, p.Kd_pitch,
               p.Ki_slew_pitch, p.integral_drain_rate_pitch, s.error_sum_pitch);
        printf("  pid y: [%s] kp=%.3f ki=%.3f kd=%.3f kis=%.3f drain=%.6f  int=%.4f\n",
               s.pid_yaw_on ? "ON " : "off",
               p.Kp_yaw, p.Ki_yaw, p.Kd_yaw,
               p.Ki_slew_yaw, p.integral_drain_rate_yaw, s.error_sum_yaw);
        printf("  ff p:[%s] ff y:[%s]  offset_p=%.1f offset_y=%.1f\n",
               s.ff_pitch_on ? "ON" : "off", s.ff_yaw_on ? "ON" : "off",
               p.offset_pitch, p.offset_yaw);
        printf("  pattern: %s  step:%d  arrived:%s  dwell:%.1f/%.1fs\n",
               s.pattern_enabled ? "ON" : "off",
               g_slew.seq_step, g_slew.arrived ? "yes" : "no",
               g_slew.dwell_elapsed,
               (g_slew.seq_step < NUM_TRACK_STEPS)
                   ? TRACK_SEQUENCE[g_slew.seq_step].dwell_seconds : 0.0);
        printf("  slew: speed_y:%.2f speed_p:%.2f cm/s  norm:%.2fcm  steps_left:%d  record:%s\n",
               g_slew.speed_x_cm_s, g_slew.speed_y_cm_s, g_slew.arrival_norm_cm,
               g_slew.steps_left, g_record_on ? "ON" : "off");
        printf("  laser: intent:%s  active:%s  range:%.1fdeg\n",
               g_laser_on ? "on" : "off", g_laser_active ? "on" : "off",
               (double)g_laser_max_deg);
        printf("  com offset: %.2f mm (-z body axis)\n\n",
               -g_params.r_CoM.z() * 1000.0);
        return true;
    }

    // com offset: "com <v>" where v is offset in mm along -z body axis
    if (strncasecmp(line, "com ", 4) == 0) {
        double v = atof(line + 4);
        pthread_mutex_lock(&g_ctrl_mutex);
        g_params.r_CoM.z() = -v / 1000.0;
        pthread_mutex_unlock(&g_ctrl_mutex);
        printf("com offset: %.2f mm  (r_CoM.z = %.5f m)\n", v, g_params.r_CoM.z());
        return true;
    }

    // axis commands: "p ..." or "y ..."
    if (len >= 3 && line[1] == ' ') {
        char axis = tolower((unsigned char)line[0]);
        if (axis != 'y' && axis != 'p') goto unknown;
        const char* sub = line + 2;

        pthread_mutex_lock(&g_ctrl_mutex);
        double* kp = (axis == 'y') ? &g_params.Kp_yaw : &g_params.Kp_pitch;
        double* ki = (axis == 'y') ? &g_params.Ki_yaw : &g_params.Ki_pitch;
        double* kd = (axis == 'y') ? &g_params.Kd_yaw : &g_params.Kd_pitch;
        double* ki_slew = (axis == 'y') ? &g_params.Ki_slew_yaw : &g_params.Ki_slew_pitch;
        double* drain = (axis == 'y') ? &g_params.integral_drain_rate_yaw
                                      : &g_params.integral_drain_rate_pitch;
        double* off = (axis == 'y') ? &g_params.offset_yaw : &g_params.offset_pitch;
        double* isum = (axis == 'y') ? &g_state.error_sum_yaw : &g_state.error_sum_pitch;
        const char* ax = (axis == 'y') ? "YAW" : "PITCH";

        if (strncasecmp(sub, "kp ", 3) == 0) {
            *kp = atof(sub + 3);
            printf("%s kp=%.4f\n", ax, *kp);
        } else if (strncasecmp(sub, "ki ", 3) == 0) {
            *ki = atof(sub + 3);
            *isum = 0.0;
            printf("%s ki=%.4f (integrator reset)\n", ax, *ki);
        } else if (strncasecmp(sub, "kd ", 3) == 0) {
            *kd = atof(sub + 3);
            printf("%s kd=%.4f\n", ax, *kd);
        } else if (strncasecmp(sub, "kis ", 4) == 0) {
            *ki_slew = atof(sub + 4);
            printf("%s kis=%.4f\n", ax, *ki_slew);
        } else if (strncasecmp(sub, "drain ", 6) == 0) {
            *drain = atof(sub + 6);
            printf("%s drain=%.6f\n", ax, *drain);
        } else if (strncasecmp(sub, "offset ", 7) == 0) {
            *off = atof(sub + 7);
            printf("%s offset=%.4f\n", ax, *off);
        } else if (strcasecmp(sub, "pid") == 0) {
            printf("%s kp=%.4f ki=%.4f kd=%.4f kis=%.4f drain=%.6f int=%.4f\n",
                   ax, *kp, *ki, *kd, *ki_slew, *drain, *isum);
        } else {
            printf("unknown %s command '%s'\n", ax, sub);
        }
        pthread_mutex_unlock(&g_ctrl_mutex);
        return true;
    }

unknown:
    printf("unknown command '%s' -- type h for help\n", line);
    return true;
}

// ------------------------------------------------------------------------
//  main
// ------------------------------------------------------------------------

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    g_params = make_default_params();
    g_state = make_initial_state();

    printf("initialising hardware...\n");

    if (!gpio_outputs_init(nullptr)) {
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

    // laser managed by control_thread; gated by camera valid and range
    laser_set(false);

    ConstellationSensor::Config cam_cfg;
    cam_cfg.display = false;
    ConstellationSensor sensor(cam_cfg);
    g_sensor = &sensor;

    if (!sensor.init()) {
        fprintf(stderr, "sensor init failed\n");
        esc_cleanup();
        gpio_outputs_cleanup();
        return 1;
    }

    if (pthread_create(&g_thr_cam, nullptr, camera_thread, &sensor) != 0 ||
        pthread_create(&g_thr_imu, nullptr, imu_thread, nullptr) != 0 ||
        pthread_create(&g_thr_ctrl, nullptr, control_thread, nullptr) != 0) {
        fprintf(stderr, "failed to start threads: %s\n", strerror(errno));
        return 1;
    }

    printf("\n\n\n");
    printf("ready. type 'h' for help list.\n\n");

    char line[256];
    while (true) {
        printf("> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;
        if (!handle_command(line)) break;
        g_status_first = true;
        printf("\n\n\n");
        fflush(stdout);
    }

    shutdown_all();
    return 0;
}
