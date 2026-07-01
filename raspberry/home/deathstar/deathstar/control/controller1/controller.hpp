// controller: structs, params, and updateController declaration
// the tunable controller variables are in controller.cpp
// ------------------------------------------------------------------------
#pragma once

#include <Eigen/Dense>

struct ControllerParams {
    // feedforward inertia
    Eigen::Matrix3d J_total;
    double Jw_spin; // 2.4932e-5 [kg*m^2]
    Eigen::Vector3d s1; // pitch wheel axis: (0, -1, 0)
    Eigen::Vector3d s2; // yaw wheel axis:   (0,  0, -1)
    Eigen::Matrix<double, 2, 3> B_pinv; // pseudoinverse of wheel axis matrix

    // gravity torque
    double m_total; // 3.6 [kg]
    Eigen::Vector3d r_CoM; // center of mass offset [m]

    // actuator mapping: throttle = omega_des * Kff
    double Kff;

    // pid gains (outputs in throttle [0, 1])
    double Kp_pitch, Ki_pitch, Kd_pitch;
    double Kp_yaw, Ki_yaw, Kd_yaw;

    // integral gain during slewing; 0 = blocked completely
    double Ki_slew_pitch;
    double Ki_slew_yaw;

    // baseline throttle [0, 1]
    double offset_pitch;
    double offset_yaw;

    // integral desaturation
    double stable_margin; // error threshold to start draining integrator [rad]
    double integral_drain_rate_pitch; // drain rate per axis [1/s]
    double integral_drain_rate_yaw;

    // anti-windup clamp on integrator
    double integrator_clamp;

    // startup ramp
    double startup_ramp_rate; // throttle/s

    // homing
    double homing_throttle; // fixed yaw throttle during sweep [0, 1]
    double homing_timeout_s;

    // throttle limits [0, 1]
    double throttle_min;
    double throttle_max;

    // slew mode: active when flag is set AND per-axis rate exceeds threshold
    double slew_rate_threshold_pitch; // [rad/s]
    double slew_rate_threshold_yaw; // [rad/s]

    // camera offset: update when laser is within this distance from center [rad]
    double cam_offset_epsilon;
};

struct SensorInputs {
    // from camera marker detection
    double theta_pitch_cam; // [rad], positive = target above center
    double theta_yaw_cam; // [rad], positive = target to the right
    bool cam_valid;

    // from laser (angle-error relative to active target point, same sign as camera)
    double theta_pitch_laser; // [rad]
    double theta_yaw_laser; // [rad]
    bool laser_valid;

    // from imu (body angular rates)
    double omega_pitch; // [rad/s]
    double omega_yaw; // [rad/s]
    double omega_pitch_des; // [rad/s] desired rate (0 for pointing, nonzero for slew)
    double omega_yaw_des;
    Eigen::Vector3d omega_body; // full 3-axis [rad/s]

    // from hall sensors (actual wheel speeds)
    double Omega1_act; // pitch wheel [rad/s]
    double Omega2_act; // yaw wheel [rad/s]

    // desired trajectory
    double theta_pitch_des; // [rad] desired pitch angle
    double theta_yaw_des; // [rad] desired yaw angle
    Eigen::Vector3d alpha_des; // desired angular acceleration [rad/s^2]
};

struct ControllerState {
    double error_sum_pitch;
    double error_sum_yaw;
    double Omega1_des; // integrated desired wheel speed pitch [rad/s]
    double Omega2_des; // integrated desired wheel speed yaw [rad/s]
    double startup_ramp_progress; // current ramp throttle value [0..offset]
    bool startup_done;
    bool homing_done;
    double homing_yaw_accumulated; // track rotation during homing [rad]
    double homing_elapsed_s; // time spent in homing
    bool controllers_enabled;
    bool pattern_enabled;

    // per-axis enable flags
    bool pid_pitch_on;
    bool pid_yaw_on;
    bool ff_pitch_on;
    bool ff_yaw_on;

    // set by main when pattern switches target point; combined with rate check for slew
    bool slew_requested;
    bool is_slewing; // true when slew_requested and body rate above threshold

    // throttle from last tick for ramp limiting
    double last_throttle1;
    double last_throttle2;

    // body attitude tracked via imu integration (body-to-world rotation)
    Eigen::Matrix3d R_body;

    // camera attitude offset: cam + offset approximates laser attitude
    double cam_offset_pitch;
    double cam_offset_yaw;

    // feedforward initialization: Omega_des is re-seeded from Omega_act on each FF enable
    bool ff_pitch_initialized;
    bool ff_yaw_initialized;

    // throttle accumulator for gravity FF impl B
    double u_ff1_integrated;
    double u_ff2_integrated;
};

struct ActuatorOutputs {
    double throttle1; // pitch wheel [0, 1]
    double throttle2; // yaw wheel [0, 1]
    double u_ff1; // feedforward throttle pitch
    double u_ff2; // feedforward throttle yaw
    double u_fb1; // feedback throttle pitch
    double u_fb2; // feedback throttle yaw
};

ControllerParams make_default_params();
ControllerState make_initial_state();

ActuatorOutputs updateController(
    const SensorInputs& sensors,
    const ControllerParams& params,
    ControllerState& state,
    double dt
);
