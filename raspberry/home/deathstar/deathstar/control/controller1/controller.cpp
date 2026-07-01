// all controller logic: feedforward, pid, homing, startup ramp
// ------------------------------------------------------------------------
#include "controller.hpp"
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>

static constexpr double PI = M_PI;

// max throttle change per tick at 10 hz
// to prevent to abrupt change in throttle command (so that RPi doesn't brown-out)
static constexpr double RAMP_MAX_STEP = 0.25;

static double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static double ramp_limit(double target, double last)
{
    double d = clamp(target - last, -RAMP_MAX_STEP, RAMP_MAX_STEP);
    return last + d;
}

ControllerParams make_default_params()
{
    ControllerParams p;
	//for feedforward:
    p.J_total <<
        0.0325938780796586, -0.00192528711213483, -0.00214221992111931,
       -0.00192528711213483,  0.0314521408239867,  0.00200183889657857,
       -0.00214221992111931,  0.00200183889657857,  0.0375049296630568;

    p.Jw_spin = 2.4932e-5;

    p.s1 = Eigen::Vector3d(0.0, -1.0, 0.0);
    p.s2 = Eigen::Vector3d(0.0,  0.0, -1.0);

    p.B_pinv <<
        0.0, -1.0,  0.0,
        0.0,  0.0, -1.0;

    p.m_total = 3.66;
    p.r_CoM = Eigen::Vector3d(0.0, 0.0, -0.007); // CoM below pivot on -z body axis

    // Kff: approx maps wheel speed [rad/s] to ESC throttle [0,1].
    // The physical calibration is 1/14 rad/s -> percent [0,100];
    // divide by 100 to match the internal [0,1] throttle scale used everywhere.
    p.Kff = 1.0 / 14.0 / 100.0;

	//for feedback:
    // **********params for yaw pattern************** //

    p.Kp_pitch = 7.0;
    p.Ki_pitch = 6.0;
    p.Kd_pitch = 0.5;
    
    p.Kp_yaw = 5.0;
    p.Ki_yaw = 4.0;
    p.Kd_yaw = 1.3;

    p.Ki_slew_pitch = 5.0;
    p.Ki_slew_yaw = 0.5;

    p.offset_pitch = 0.55;
    p.offset_yaw = 0.55;

    p.stable_margin = 0.04; // [rad]
    p.integral_drain_rate_pitch = 0.00; // [1/s]
    p.integral_drain_rate_yaw = 0.00;
    p.integrator_clamp = 0.5;

    p.startup_ramp_rate = 0.05; // throttle/s

    p.homing_throttle = 0.15;
    p.homing_timeout_s = 30.0;

    p.throttle_min = 0.10;
    p.throttle_max = 1.00;

    p.slew_rate_threshold_pitch = 0.9; // [rad/s]
    p.slew_rate_threshold_yaw   = 0.9; // [rad/s]

    p.cam_offset_epsilon = 0.07; // [rad]

    // **********params for pitch pattern  **********//
// 
//     p.Kp_pitch = 7.0;
//     p.Ki_pitch = 6.0;
//     p.Kd_pitch = 0.5;
//     
//     p.Kp_yaw = 6.0;
//     p.Ki_yaw = 5.0;
//     p.Kd_yaw = 1.0;
// 
//     p.Ki_slew_pitch = 5.0;
//     p.Ki_slew_yaw = 0.5;
// 
//     p.offset_pitch = 0.55;
//     p.offset_yaw = 0.55;
// 
//     p.stable_margin = 0.04; // [rad]
//     p.integral_drain_rate_pitch = 0.00; // [1/s]
//     p.integral_drain_rate_yaw = 0.00;
//     p.integrator_clamp = 0.5;
// 
//     p.startup_ramp_rate = 0.05; // throttle/s
// 
//     p.homing_throttle = 0.15;
//     p.homing_timeout_s = 30.0;
// 
//     p.throttle_min = 0.10;
//     p.throttle_max = 1.00;
// 
//     p.slew_rate_threshold_pitch = 0.9; // [rad/s]
//     p.slew_rate_threshold_yaw   = 0.9; // [rad/s]
// 
//     p.cam_offset_epsilon = 0.2; // [rad]

    return p;
}

ControllerState make_initial_state()
{
    ControllerState s;
    s.error_sum_pitch = 0.0;
    s.error_sum_yaw = 0.0;
    s.Omega1_des = 0.0;
    s.Omega2_des = 0.0;
    s.startup_ramp_progress = 0.0;
    s.startup_done = false;
    s.homing_done = false;
    s.homing_yaw_accumulated = 0.0;
    s.homing_elapsed_s = 0.0;
    s.controllers_enabled = false;
    s.pattern_enabled = false;
    s.pid_pitch_on = false;
    s.pid_yaw_on = false;
    s.ff_pitch_on = false;
    s.ff_yaw_on = false;
    s.slew_requested = false;
    s.is_slewing = false;
    s.last_throttle1 = 0.0;
    s.last_throttle2 = 0.0;
    s.R_body = Eigen::Matrix3d::Identity();
    s.cam_offset_pitch = 0.0;
    s.cam_offset_yaw = 0.0;
    s.ff_pitch_initialized = false;
    s.ff_yaw_initialized = false;
    s.u_ff1_integrated = 0.0;
    s.u_ff2_integrated = 0.0;
    return s;
}

ActuatorOutputs updateController(
    const SensorInputs& sensors,
    const ControllerParams& params,
    ControllerState& state,
    double dt)
{
    ActuatorOutputs out;
    out.throttle1 = 0.0;
    out.throttle2 = 0.0;
    out.u_ff1 = 0.0;
    out.u_ff2 = 0.0;
    out.u_fb1 = 0.0;
    out.u_fb2 = 0.0;

    if (!state.controllers_enabled) {
        out.throttle1 = ramp_limit(0.0, state.last_throttle1);
        out.throttle2 = ramp_limit(0.0, state.last_throttle2);
        state.last_throttle1 = out.throttle1;
        state.last_throttle2 = out.throttle2;
        return out;
    }

    // startup ramp: spin wheels up to offset before enabling control
    if (!state.startup_done) {
        state.startup_ramp_progress += params.startup_ramp_rate * dt;
        double target = params.offset_pitch;
        if (state.startup_ramp_progress >= target) {
            state.startup_ramp_progress = target;
            state.startup_done = true;
        }
        double thr = state.startup_ramp_progress;
        bool use_pitch = state.pid_pitch_on || state.ff_pitch_on;
        bool use_yaw = state.pid_yaw_on || state.ff_yaw_on;
        out.throttle1 = ramp_limit(use_pitch ? thr : 0.0, state.last_throttle1);
        out.throttle2 = ramp_limit(use_yaw ? thr : 0.0, state.last_throttle2);
        state.last_throttle1 = out.throttle1;
        state.last_throttle2 = out.throttle2;
        return out;
    }

    // homing: sweep yaw until camera finds target
    if (!state.homing_done && !sensors.cam_valid) {
        state.homing_yaw_accumulated += sensors.omega_yaw * dt;
        state.homing_elapsed_s += dt;

        bool timed_out = state.homing_elapsed_s >= params.homing_timeout_s;
        bool full_circle = std::fabs(state.homing_yaw_accumulated) >= 2.0 * PI;

        if (timed_out || full_circle) {
            state.controllers_enabled = false;
            out.throttle1 = ramp_limit(0.0, state.last_throttle1);
            out.throttle2 = ramp_limit(0.0, state.last_throttle2);
            state.last_throttle1 = out.throttle1;
            state.last_throttle2 = out.throttle2;
            return out;
        }

        out.throttle1 = params.offset_pitch;
        out.throttle2 = params.homing_throttle;
        out.throttle1 = clamp(out.throttle1, params.throttle_min, params.throttle_max);
        out.throttle2 = clamp(out.throttle2, params.throttle_min, params.throttle_max);
        state.last_throttle1 = out.throttle1;
        state.last_throttle2 = out.throttle2;
        return out;
    }

    // camera just became valid: end homing
    if (!state.homing_done && sensors.cam_valid) {
        state.homing_done = true;
        state.homing_yaw_accumulated = 0.0;
    }

    // (old: R_body  via IMU - replaced by direct angle measurement below
    // {
    //     double angle = sensors.omega_body.norm() * dt;
    //     if (angle > 1e-12) {
    //         Eigen::AngleAxisd dR(angle, sensors.omega_body.normalized());
    //         state.R_body = state.R_body * dR.toRotationMatrix();
    //     }
    // }
    // Eigen::Vector3d g_world(0.0, 0.0, -9.81);
    // Eigen::Vector3d g_body = state.R_body.transpose() * g_world;
    // Eigen::Vector3d tau_g = params.m_total * params.r_CoM.cross(g_body);

    if (state.slew_requested &&
        std::fabs(sensors.omega_pitch) < params.slew_rate_threshold_pitch &&
        std::fabs(sensors.omega_yaw)   < params.slew_rate_threshold_yaw)
        state.slew_requested = false;

    bool slewing_pitch = state.slew_requested && (std::fabs(sensors.omega_pitch) > params.slew_rate_threshold_pitch);
    bool slewing_yaw   = state.slew_requested && (std::fabs(sensors.omega_yaw)   > params.slew_rate_threshold_yaw);
    state.is_slewing = slewing_pitch || slewing_yaw;

    // --- feedforward: gravity compensation ---
    double theta_pitch_ff = sensors.theta_pitch_des;
    // yaw axis is balanced; gravity FF term is zero.

    // (old: full Euler + gyroscopic dynamics feedforward via B_pinv)
    // Eigen::Vector3d tau_ff_b = Eigen::Vector3d::Zero();
    // if (state.ff_pitch_on || state.ff_yaw_on) {
    //     Eigen::Vector3d omega = sensors.omega_body;
    //     Eigen::Vector3d tau_ff_b_dyn =
    //         params.J_total * sensors.alpha_des
    //         + omega.cross(params.J_total * omega)
    //         + omega.cross(params.Jw_spin * sensors.Omega1_act * params.s1)
    //         + omega.cross(params.Jw_spin * sensors.Omega2_act * params.s2);
    //     tau_ff_b = slewing ? (tau_ff_b_dyn - tau_g) : -tau_g;
    // }
    // if (!state.ff_pitch_on) tau_ff_b.y() = 0.0;
    // if (!state.ff_yaw_on)   tau_ff_b.z() = 0.0;
    // Eigen::Vector2d tau_wheels = params.B_pinv * tau_ff_b;
    // double Omega1_dot_des_old = tau_wheels[0] / params.Jw_spin;
    // double Omega2_dot_des_old = tau_wheels[1] / params.Jw_spin;

    double tau_g_pitch = params.m_total * 9.81
        * std::fabs(params.r_CoM.z())
        * std::sin(theta_pitch_ff);

    double Omega1_dot_des = state.ff_pitch_on ? (tau_g_pitch / params.Jw_spin) : 0.0;
    double Omega2_dot_des = 0.0; // yaw gravity = 0

    // - implementation: A (active): speed error tracking with Hall sensor feedback
    // Omega1_des is initialised from the actual wheel speed on each FF enable so the
    // error starts at zero; as Omega1_des ramps ahead of Omega1_act, the nonzero
    // speed error keeps the ESC continuously accelerating the wheel, which is what
    // produces the sustained torque needed to hold a non-zero pitch angle.
//     if (state.ff_pitch_on && !state.ff_pitch_initialized) {
//         state.Omega1_des = sensors.Omega1_act;
//         state.ff_pitch_initialized = true;
//     }
//     if (!state.ff_pitch_on) {
//         // track actual while off so the next enable starts smoothly
//         state.ff_pitch_initialized = false;
//         state.Omega1_des = sensors.Omega1_act;
//     }
// 
//     state.Omega1_des += Omega1_dot_des * dt;
//     state.Omega2_des += Omega2_dot_des * dt;
// 
//     // clamp desired speed to [0, 1/Kff] which maps to throttle [0, 1]
//     double omega_max = 1.0 / params.Kff;
//     state.Omega1_des = clamp(state.Omega1_des, 0.0, omega_max);
//     state.Omega2_des = clamp(state.Omega2_des, 0.0, omega_max);
// 
//     // speed error -> feedforward throttle; the lag of Omega1_act behind Omega1_des
//     // is a constant that sustains the wheel acceleration and thus the torque.
//     double u_ff1 = state.ff_pitch_on
//         ? (state.Omega1_des - sensors.Omega1_act) * params.Kff
//         : 0.0;
//     double u_ff2 = 0.0; // yaw FF disabled

    // -implementation B: integrate torque directly in throttle space
    // Equivalent to impl A but without wheel speed feedback;
    // Reset to zero on FF disable so next enable ramps cleanly from offset alone.
    if (!state.ff_pitch_on) {
        state.u_ff1_integrated = 0.0;
    } else {
        // throttle rate = (tau_g / Jw) * Kff  [throttle/s]
        double u_ff_dot = (tau_g_pitch / params.Jw_spin) * params.Kff;
        state.u_ff1_integrated += u_ff_dot * dt;
        state.u_ff1_integrated = clamp(state.u_ff1_integrated,
            0.0, params.throttle_max - params.offset_pitch);
    }
    double u_ff1 = state.u_ff1_integrated;
    double u_ff2 = 0.0;

    // feedback pid
    // update camera offset when laser is near center and camera is valid
    // this is in case there is frame where laser is not detected such that there is a fallback
    if (sensors.laser_valid && sensors.cam_valid &&
        std::fabs(sensors.theta_pitch_laser) < params.cam_offset_epsilon &&
        std::fabs(sensors.theta_yaw_laser) < params.cam_offset_epsilon) {
        state.cam_offset_pitch = sensors.theta_pitch_laser - sensors.theta_pitch_cam;
        state.cam_offset_yaw = sensors.theta_yaw_laser - sensors.theta_yaw_cam;
    }

    double theta_pitch_meas = sensors.laser_valid ? sensors.theta_pitch_laser
                                                   : sensors.theta_pitch_cam + state.cam_offset_pitch;
    double theta_yaw_meas = sensors.laser_valid ? sensors.theta_yaw_laser
                                                 : sensors.theta_yaw_cam + state.cam_offset_yaw;

    // PID desired angle: 0.0 in laser mode (laser error is already relative to active target,
    // so "on target" = 0). In no-laser mode, use the absolute setpoint from the slew position.
    // theta_pitch/yaw_des now carries the absolute setpoint for the FF;
    double e_pitch = (sensors.laser_valid ? 0.0 : sensors.theta_pitch_des) - theta_pitch_meas;
    double e_yaw   = (sensors.laser_valid ? 0.0 : sensors.theta_yaw_des)   - theta_yaw_meas;

    if (!slewing_pitch) {
        bool aw_p = (state.last_throttle1 >= params.throttle_max && e_pitch > 0.0) ||
                    (state.last_throttle1 <= params.throttle_min && e_pitch < 0.0);
        if (!aw_p) state.error_sum_pitch += e_pitch * dt;
    } else if (params.Ki_slew_pitch > 0.0) {
        state.error_sum_pitch += e_pitch * dt;
    }

    if (!slewing_yaw) {
        bool aw_y = (state.last_throttle2 >= params.throttle_max && e_yaw > 0.0) ||
                    (state.last_throttle2 <= params.throttle_min && e_yaw < 0.0);
        if (!aw_y) state.error_sum_yaw += e_yaw * dt;
    } else if (params.Ki_slew_yaw > 0.0) {
        state.error_sum_yaw += e_yaw * dt;
    }

    state.error_sum_pitch = clamp(state.error_sum_pitch,
        -params.integrator_clamp, params.integrator_clamp);
    state.error_sum_yaw = clamp(state.error_sum_yaw,
        -params.integrator_clamp, params.integrator_clamp);

    double u_fb_pitch = 0.0;
    double u_fb_yaw = 0.0;

    if (state.pid_pitch_on) {
        double ki_p = slewing_pitch ? params.Ki_slew_pitch : params.Ki_pitch;
        u_fb_pitch = params.Kp_pitch * e_pitch
                   + ki_p * state.error_sum_pitch
                   + params.Kd_pitch * (sensors.omega_pitch_des + sensors.omega_pitch);
    }
    if (state.pid_yaw_on) {
        double ki_y = slewing_yaw ? params.Ki_slew_yaw : params.Ki_yaw;
        u_fb_yaw = params.Kp_yaw * e_yaw
                 + ki_y * state.error_sum_yaw
                 + params.Kd_yaw * (sensors.omega_yaw_des - sensors.omega_yaw);
    }

    bool on_offset_pitch = state.pid_pitch_on || state.ff_pitch_on;
    bool on_offset_yaw = state.pid_yaw_on || state.ff_yaw_on;

    double u1 = u_ff1 + u_fb_pitch + on_offset_pitch * params.offset_pitch;
    double u2 = u_ff2 + u_fb_yaw + on_offset_yaw * params.offset_yaw;

    u1 = on_offset_pitch ? clamp(u1, params.throttle_min, params.throttle_max) : 0.0;
    u2 = on_offset_yaw ? clamp(u2, params.throttle_min, params.throttle_max) : 0.0;

    out.u_ff1 = u_ff1;
    out.u_ff2 = u_ff2;
    out.u_fb1 = u_fb_pitch;
    out.u_fb2 = u_fb_yaw;
    out.throttle1 = ramp_limit(u1, state.last_throttle1);
    out.throttle2 = ramp_limit(u2, state.last_throttle2);

    state.last_throttle1 = out.throttle1;
    state.last_throttle2 = out.throttle2;

    return out;
}
