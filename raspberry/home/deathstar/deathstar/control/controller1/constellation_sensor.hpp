// camera: aruco pose estimation + laser dot detection
// ------------------------------------------------------------------------
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <libcamera/control_ids.h>

#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <string>

// point on the target plane in target-local coordinates
struct TargetPoint {
    int index;
    double local_x; // meters, positive = right in target plane
    double local_y; // meters, positive = up in target plane
};

struct SensorData {
    bool valid = false;

    // pose
    double dist_cm = 0.0;
    double yaw_deg = 0.0; // positive = target to right
    double pitch_deg = 0.0; // positive = target up
    double roll_deg = 0.0;

    // translation in camera frame [cm]
    double tx_cm = 0.0;
    double ty_cm = 0.0;
    double tz_cm = 0.0;

    int num_markers = 0;
    std::vector<int> marker_ids;
    double fps = 0.0;

    // laser dot fields (valid only when laser_valid = true)
    bool laser_valid = false;
    double laser_dx_m = 0.0; // x error in target plane [m], relative to active target
    double laser_dy_m = 0.0; // y error in target plane [m], relative to active target
    double laser_abs_x_m = 0.0; // absolute laser x in board-local frame [m] (origin = board center)
    double laser_abs_y_m = 0.0; // absolute laser y in board-local frame [m]
    double laser_yaw_deg = 0.0; // laser dot yaw in camera frame [deg] (for display)
    double laser_pitch_deg = 0.0; // laser dot pitch in camera frame [deg] (for display)
    int active_target_index = 0;

    std::string str() const;
    void print() const;
};

class ConstellationSensor {
public:
    struct Config {
        //res.1296x972
        uint32_t frame_w = 1296;
        uint32_t frame_h = 972;
        bool display = false;

        cv::Mat cam_matrix;
        cv::Mat dist_coeffs;

        double aruco_side = 0.07;
        double board_w = 0.122; //dist between aruco markers width
        double board_h = 0.209; //dist between aruco markers height

        cv::aruco::PredefinedDictionaryType dict_type = cv::aruco::DICT_4X4_250;
        // marker centers in board frame: id -> (cx, cy, 0). filled with defaults if empty
        std::map<int, cv::Vec3d> marker_centers;

        Config();
    };

    explicit ConstellationSensor(Config cfg = Config{});
    ~ConstellationSensor();

    bool init();
    void shutdown();

    // blocks until a new frame is available (or timeout_ms exceeded)
    SensorData update(int timeout_ms = 500);

    void enableDisplay(bool on);
    bool displayEnabled() const { return display_on_; }

    void setActiveTargetIndex(int idx) { active_target_idx_ = idx; }
    int activeTargetIndex() const { return active_target_idx_; }
    // interim slew position in target-local coords [m]; drives display line
    void setInterimPoint(double x, double y) { interim_x_ = x; interim_y_ = y; }

    bool isValid() const { return last_.valid; }
    double distCm() const { return last_.dist_cm; }
    double yawDeg() const { return last_.yaw_deg; }
    double pitchDeg() const { return last_.pitch_deg; }
    double rollDeg() const { return last_.roll_deg; }
    double fps() const { return last_.fps; }

    void printData() const { last_.print(); }

private:
    struct FrameRaw {
        std::vector<uint8_t> bytes;
    };

    void onRequestCompleted(libcamera::Request* req);

    static double rollFromRvec(const cv::Mat& rvec);

    void renderFrame(cv::Mat& frame, const SensorData& d,
                     const std::vector<std::vector<cv::Point2f>>& corners,
                     const std::vector<int>& ids,
                     const cv::Mat& rvec_final,
                     const cv::Mat& tvec_final,
                     cv::Point2f dot_pos, bool dot_found,
                     cv::Point2f active_target_proj,
                     cv::Point2f interim_proj,
                     cv::Point2f bound_center, float bound_radius);

    Config cfg_;
    std::map<int, std::vector<cv::Point3f>> obj_points_;

    libcamera::CameraManager cam_mgr_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> cam_cfg_;
    libcamera::FrameBufferAllocator* allocator_ = nullptr;
    libcamera::Stream* stream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    cv::aruco::Dictionary dict_;
    cv::aruco::DetectorParameters params_;
    cv::aruco::ArucoDetector detector_;

    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::queue<FrameRaw> frame_queue_;

    std::chrono::steady_clock::time_point prev_time_;

    bool display_on_ = false;
    static constexpr const char* WIN_NAME = "Constellation Sensor";

    SensorData last_;
    uint32_t stride_ = 0;
    bool initialized_ = false;
    int active_target_idx_ = 0;
    double interim_x_ = 0.0;
    double interim_y_ = 0.0;
};
