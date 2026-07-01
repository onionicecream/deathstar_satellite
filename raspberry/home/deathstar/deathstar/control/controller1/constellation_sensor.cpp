// camera implementation: aruco pose + laser dot with 3d intersection
// ------------------------------------------------------------------------
#include "constellation_sensor.hpp"

#include <sys/mman.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <iomanip>

using namespace libcamera;

// target points on the board in target-local coordinates (meters)
// index 0 is the constellation center
// static const TargetPoint TARGET_POINTS[] = {
//     {0,  0.00,  0.00}, // center
//     {1,  0.061,  0.00}, // 6.1 cm right (yaw direction)
//     {2, -0.061,  0.00}, // 3 cm left  (yaw direction)
//     {3, 0.0,  0.031}, // 3.1 cm up
//     {4, 0.0,  -0.031}, // 3.1 cm down
//     {5, -0.061,  0.031}, // corner
//     {6, 0.061,  0.031}, // corner
//     {7, -0.061,  -0.031}, // corner
//     {8, 0.061,  -0.031}, // corner
// };
static const TargetPoint TARGET_POINTS[] = {
    {0,  0.00,  0.00}, // center
    {1,  0.061,  0.00}, // 6.1 cm right (yaw direction)
    {2, -0.061,  0.00}, // 6.1 cm left (yaw direction)
    {3, 0.0,  0.031}, // 3.1 cm up
    {4, 0.0,  -0.031}, // 3.1 cm down
    {5, -0.061,  0.031}, // corner
    {6, 0.061,  0.031}, // corner
    {7, -0.061,  -0.031}, // corner
	{8, 0.061,  -0.031}, // corner
	{10, 0.0,  0.0155}, // halfway up
};

static const int NUM_TARGET_POINTS = (int)(sizeof(TARGET_POINTS) / sizeof(TARGET_POINTS[0]));

// ------------------------------------------------------------------------
//  SensorData printing(on display preview)
// ------------------------------------------------------------------------
std::string SensorData::str() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    if (!valid) {
        ss << "[sensor] no markers  fps:" << fps;
        return ss.str();
    }
    ss << "[sensor] markers:" << num_markers
       << " dist:" << dist_cm << "cm"
       << " yaw:" << yaw_deg << "deg"
       << " pitch:" << pitch_deg << "deg"
       << " fps:" << fps;
    if (laser_valid) {
        ss << " | laser dx:" << laser_dx_m * 100.0 << "cm"
           << " dy:" << laser_dy_m * 100.0 << "cm"
           << " yaw:" << laser_yaw_deg << "deg"
           << " pitch:" << laser_pitch_deg << "deg";
    }
    return ss.str();
}

void SensorData::print() const
{
    std::cout << str() << "\n";
}

// ------------------------------------------------------------------------
//  Config defaults
// ------------------------------------------------------------------------
// this is calib for new cam (model v1.3) - on res.1296x972
ConstellationSensor::Config::Config()
{
    cam_matrix = (cv::Mat_<double>(3, 3) <<
        1177.57545, 0.0,        652.166382,
        0.0,       1176.83529,  442.615443,
        0.0,       0.0,         1.0);

    dist_coeffs = (cv::Mat_<double>(1, 5) <<
        0.15314514, -0.50238314, -0.00563695, -0.0019991, 0.60596155);
}

// ------------------------------------------------------------------------
//  Constructor / Destructor
// ------------------------------------------------------------------------
ConstellationSensor::ConstellationSensor(Config cfg)
    : cfg_(std::move(cfg))
    , dict_(cv::aruco::getPredefinedDictionary(cfg_.dict_type))
    , params_{}
    , detector_(dict_, params_)
{
    display_on_ = cfg_.display;

    //this is the definition of the constellation

    const double half_side = cfg_.aruco_side / 2.0;
    const double w_half = cfg_.board_w / 2.0;
    const double h_half = cfg_.board_h / 2.0;

    if (cfg_.marker_centers.empty()) {
        // X = leftward, Y = downward convention
        // marker 4: top-left, 3: top-right, 2: bottom-right, 1: bottom-left (aruco id and location on paper)
        cfg_.marker_centers = {
            {4, { w_half, -h_half, 0.0}},
            {3, {-w_half, -h_half, 0.0}},
            {2, {-w_half,  h_half, 0.0}},
            {1, { w_half,  h_half, 0.0}},
        };
    }

    for (auto& [mid, c] : cfg_.marker_centers) {
        double cx = c[0], cy = c[1];
        obj_points_[mid] = {
            {float(cx - half_side), float(cy + half_side), 0.f},
            {float(cx + half_side), float(cy + half_side), 0.f},
            {float(cx + half_side), float(cy - half_side), 0.f},
            {float(cx - half_side), float(cy - half_side), 0.f},
        };
    }
}

ConstellationSensor::~ConstellationSensor()
{
    shutdown();
}

// ------------------------------------------------------------------------
//  init()
// ------------------------------------------------------------------------
bool ConstellationSensor::init()
{
    if (initialized_) return true;

    cam_mgr_.start();
    if (cam_mgr_.cameras().empty()) {
        std::cerr << "[sensor] no cameras found\n";
        return false;
    }

    camera_ = cam_mgr_.cameras()[0];
    if (camera_->acquire() < 0) {
        std::cerr << "[sensor] failed to acquire camera\n";
        return false;
    }

    cam_cfg_ = camera_->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration& sc = cam_cfg_->at(0);
    sc.pixelFormat = formats::RGB888;
    sc.size = {cfg_.frame_w, cfg_.frame_h};
    sc.bufferCount = 6;

    if (cam_cfg_->validate() == CameraConfiguration::Invalid) {
        std::cerr << "[sensor] camera configuration invalid\n";
        return false;
    }
    camera_->configure(cam_cfg_.get());
    stride_ = cam_cfg_->at(0).stride;

    allocator_ = new FrameBufferAllocator(camera_);
    stream_ = cam_cfg_->at(0).stream();
    allocator_->allocate(stream_);

    for (const auto& buf : allocator_->buffers(stream_)) {
        auto req = camera_->createRequest();
        req->addBuffer(stream_, buf.get());
        requests_.push_back(std::move(req));
    }

    ControlList controls;
    // controls.set(controls::AfMode, controls::AfModeManual); // fixed focus, no autofocus this was on old cam
    std::array<int64_t, 2> fd = {10000LL, 10000LL}; // 100 fps max
    controls.set(controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(fd));

    camera_->requestCompleted.connect(this, &ConstellationSensor::onRequestCompleted);
    camera_->start(&controls);
    for (auto& req : requests_)
        camera_->queueRequest(req.get());

    prev_time_ = std::chrono::steady_clock::now();
    initialized_ = true;

    if (display_on_)
        cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);

    return true;
}

// ------------------------------------------------------------------------
//  shutdown()
// ------------------------------------------------------------------------
void ConstellationSensor::shutdown()
{
    if (!initialized_) return;

    camera_->stop();
    allocator_->free(stream_);
    delete allocator_;
    allocator_ = nullptr;
    camera_->release();
    cam_mgr_.stop();

    if (display_on_)
        cv::destroyWindow(WIN_NAME);

    initialized_ = false;
}

// ------------------------------------------------------------------------
//  camera callback (libcamera thread)
// ------------------------------------------------------------------------
void ConstellationSensor::onRequestCompleted(Request* req)
{
    if (req->status() == Request::RequestCancelled) return;

    const FrameBuffer* fb = req->buffers().at(stream_);
    const FrameBuffer::Plane& plane = fb->planes()[0];

    void* data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED,
                      plane.fd.get(), plane.offset);
    if (data != MAP_FAILED) {
        FrameRaw fr;
        fr.bytes.assign(static_cast<uint8_t*>(data),
                        static_cast<uint8_t*>(data) + plane.length);
        munmap(data, plane.length);

        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            while (frame_queue_.size() > 1) frame_queue_.pop();
            frame_queue_.push(std::move(fr));
        }
        frame_cv_.notify_one();
    }

    req->reuse(Request::ReuseBuffers);
    camera_->queueRequest(req);
}

// ------------------------------------------------------------------------
//  update()
// ------------------------------------------------------------------------
SensorData ConstellationSensor::update(int timeout_ms)
{
    SensorData result;
    result.active_target_index = active_target_idx_;

    if (!initialized_) {
        std::cerr << "[sensor] not initialized\n";
        return result;
    }

    FrameRaw fr;
    {
        std::unique_lock<std::mutex> lk(frame_mutex_);
        bool got = frame_cv_.wait_for(lk,
            std::chrono::milliseconds(timeout_ms),
            [this] { return !frame_queue_.empty(); });
        if (!got) return result;
        fr = std::move(frame_queue_.front());
        frame_queue_.pop();
    }

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - prev_time_).count();
    prev_time_ = now;
    result.fps = 1.0 / std::max(dt, 1e-6);

    // convert raw RGB -> BGR
    cv::Mat rgb(cfg_.frame_h, cfg_.frame_w, CV_8UC3, fr.bytes.data(), stride_);
    cv::Mat processing_frame;
    cv::cvtColor(rgb, processing_frame, cv::COLOR_RGB2BGR);
    cv::Mat display_frame = processing_frame.clone();

    // aruco detection on processing_frame
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    detector_.detectMarkers(processing_frame, corners, ids, rejected);

    cv::Mat rvec_final, tvec_final;

    if (!ids.empty()) {
        // filter to known board markers
        std::vector<std::pair<int, std::vector<cv::Point2f>>> valid;
        for (size_t i = 0; i < ids.size(); ++i)
            if (obj_points_.count(ids[i]))
                valid.push_back({ids[i], corners[i]});

        // stack all visible marker corners into a single solvePnP call so the
        // pose is always expressed relative to the same constellation-centre origin
        std::vector<cv::Point3f> stacked_obj;
        std::vector<cv::Point2f> stacked_img;
        for (auto& [mid, imgPts] : valid) {
            for (auto& pt : obj_points_.at(mid)) stacked_obj.push_back(pt);
            for (auto& pt : imgPts)              stacked_img.push_back(pt);
        }

        if (!stacked_obj.empty() &&
            cv::solvePnP(stacked_obj, stacked_img,
                         cfg_.cam_matrix, cfg_.dist_coeffs,
                         rvec_final, tvec_final, false, cv::SOLVEPNP_SQPNP)) {

            double tx = tvec_final.at<double>(0);
            double ty = tvec_final.at<double>(1);
            double tz = tvec_final.at<double>(2);

            result.valid = true;
            result.dist_cm = cv::norm(tvec_final) * 100.0;
            result.yaw_deg = std::atan2(tx, tz) * 180.0 / M_PI;
            result.pitch_deg = std::atan2(-ty, tz) * 180.0 / M_PI;
            result.roll_deg = rollFromRvec(rvec_final);
            result.tx_cm = tx * 100.0;
            result.ty_cm = ty * 100.0;
            result.tz_cm = tz * 100.0;
            result.num_markers = static_cast<int>(valid.size());
            for (auto& [mid, _] : valid) result.marker_ids.push_back(mid);
            std::sort(result.marker_ids.begin(), result.marker_ids.end());
        }
    }

    // laser dot detection (only useful when markers are found for 3d intersection)
    cv::Point2f dot_pos(0.f, 0.f);
    bool dot_found = false;
    cv::Point2f active_target_proj(0.f, 0.f);
    cv::Point2f interim_proj(0.f, 0.f);
    cv::Point2f bound_center(0.f, 0.f);
    float bound_radius = 0.f;

    if (result.valid) {
        // project target center (tvec) to 2d screen position
        std::vector<cv::Point3f> board_center_pt = {{
            float(tvec_final.at<double>(0)),
            float(tvec_final.at<double>(1)),
            float(tvec_final.at<double>(2))}};
        std::vector<cv::Point2f> board_center_2d;
        cv::projectPoints(board_center_pt,
                          cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                          cfg_.cam_matrix, cfg_.dist_coeffs, board_center_2d);
        cv::Point2f cx_b = board_center_2d[0];

        // dynamic bounding radius from first marker corner + marker width
        float R_bound = 28.0f; // fallback
        if (!corners.empty() && !corners[0].empty()) {
            float d = cv::norm(cx_b - corners[0][0]);
            float w = cv::norm(corners[0][1] - corners[0][0]);
            R_bound = d - w/3.0;
        }

        bound_center = cx_b;
        bound_radius = R_bound;

        // hsv threshold for overexposed white core
        cv::Mat hsv;
        cv::cvtColor(processing_frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 250), cv::Scalar(180, 30, 255), mask);

        // morphology: erode 1, dilate 3
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
        cv::erode(mask, mask, kernel, {-1, -1}, 1);
        cv::dilate(mask, mask, kernel, {-1, -1}, 3);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float best_redness = -15.0f;

        for (auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < 5.0 || area > 2000.0) continue;

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(cnt, center, radius);

            if (cv::norm(center - cx_b) > R_bound) continue;

            // ring mask for redness sampling
            cv::Mat ring = cv::Mat::zeros(processing_frame.size(), CV_8UC1);
            cv::circle(ring, cv::Point(int(center.x), int(center.y)),
                       int(radius + 10), 255, -1);
            cv::circle(ring, cv::Point(int(center.x), int(center.y)),
                       int(radius), 0, -1);

            // sample rgb in ring, compute redness = mean(R - (G+B)/2)
            float redness_sum = 0.0f;
            int redness_cnt = 0;
            int rx0 = std::max(0, int(center.x - radius - 11));
            int rx1 = std::min(processing_frame.cols - 1, int(center.x + radius + 11));
            int ry0 = std::max(0, int(center.y - radius - 11));
            int ry1 = std::min(processing_frame.rows - 1, int(center.y + radius + 11));

            for (int ry = ry0; ry <= ry1; ry++) {
                for (int rx = rx0; rx <= rx1; rx++) {
                    if (ring.at<uint8_t>(ry, rx) == 0) continue;
                    auto bgr = processing_frame.at<cv::Vec3b>(ry, rx);
                    float R = bgr[2], G = bgr[1], B = bgr[0];
                    redness_sum += R - (G + B) / 2.0f;
                    redness_cnt++;
                }
            }

            if (redness_cnt == 0) continue;
            float redness = redness_sum / redness_cnt;

            if (redness > best_redness) {
                best_redness = redness;
                dot_pos = center;
                dot_found = true;
            }
        }

        // 3d intersection: project laser pixel onto target plane
        if (dot_found) {
            double fx = cfg_.cam_matrix.at<double>(0, 0);
            double fy = cfg_.cam_matrix.at<double>(1, 1);
            double cx_cam = cfg_.cam_matrix.at<double>(0, 2);
            double cy_cam = cfg_.cam_matrix.at<double>(1, 2);

            // ray direction (not normalized; z=1 convention)
            Eigen::Vector3d ray;
            ray.x() = (dot_pos.x - cx_cam) / fx;
            ray.y() = (dot_pos.y - cy_cam) / fy;
            ray.z() = 1.0;

            // rotation matrix from averaged rvec
            cv::Mat R_cv;
            cv::Rodrigues(rvec_final, R_cv);
            Eigen::Matrix3d R_mat;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    R_mat(r, c) = R_cv.at<double>(r, c);

            Eigen::Vector3d tvec_eig(
                tvec_final.at<double>(0),
                tvec_final.at<double>(1),
                tvec_final.at<double>(2));

            // plane normal = z-axis of target frame in camera space
            Eigen::Vector3d plane_normal = R_mat.col(2);
            double denom = ray.dot(plane_normal);

            if (std::fabs(denom) > 1e-8) {
                double scale = tvec_eig.dot(plane_normal) / denom;
                Eigen::Vector3d laser_3d_cam = scale * ray;

                // into target local frame
                Eigen::Vector3d laser_local = R_mat.transpose() * (laser_3d_cam - tvec_eig);

                int ai = active_target_idx_;
                if (ai < 0 || ai >= NUM_TARGET_POINTS) ai = 0;

                // error relative to current interim slew position (not the final target)
                double error_x = laser_local.x() - interim_x_;
                double error_y = laser_local.y() - interim_y_;

                result.laser_valid = true;
                result.laser_abs_x_m = laser_local.x();
                result.laser_abs_y_m = laser_local.y();
                result.laser_dx_m = error_x;
                result.laser_dy_m = error_y;
                // angular error relative to active target point (same sign as controller)
                // 0 when laser is on target; changes proportionally to lateral offset
                result.laser_yaw_deg = -1.0 * std::atan2(-error_x, tvec_eig.z()) * 180.0 / M_PI;
                result.laser_pitch_deg = -1.0 * std::atan2(-error_y, tvec_eig.z()) * 180.0 / M_PI;
                //-1 needed to match camera impl.

                // project active target (endpoint) to screen
                double ax = TARGET_POINTS[ai].local_x;
                double ay = TARGET_POINTS[ai].local_y;
                Eigen::Vector3d target_cam = R_mat * Eigen::Vector3d(ax, ay, 0.0) + tvec_eig;
                std::vector<cv::Point3f> ap = {{float(target_cam.x()),
                                                float(target_cam.y()),
                                                float(target_cam.z())}};
                std::vector<cv::Point2f> ap2d;
                cv::projectPoints(ap,
                                  cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                                  cfg_.cam_matrix, cfg_.dist_coeffs, ap2d);
                active_target_proj = ap2d[0];

                // project current slew interim position (may differ during transition)
                Eigen::Vector3d int_cam = R_mat * Eigen::Vector3d(interim_x_, interim_y_, 0.0) + tvec_eig;
                std::vector<cv::Point3f> ip = {{float(int_cam.x()), float(int_cam.y()), float(int_cam.z())}};
                std::vector<cv::Point2f> ip2d;
                cv::projectPoints(ip,
                                  cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                                  cfg_.cam_matrix, cfg_.dist_coeffs, ip2d);
                interim_proj = ip2d[0];
            }
        }
    }

    if (display_on_) {
        renderFrame(display_frame, result, corners, ids,
                    rvec_final, tvec_final,
                    dot_pos, dot_found, active_target_proj, interim_proj,
                    bound_center, bound_radius);
        cv::Mat preview;
        cv::resize(display_frame, preview, cv::Size(), 0.5, 0.5);
        cv::cvtColor(preview, preview, cv::COLOR_BGR2RGB); cv::imshow(WIN_NAME, preview);
        cv::imshow(WIN_NAME, preview);
        cv::waitKey(1);
    }

    last_ = result;
    return result;
}

// ------------------------------------------------------------------------
//  enableDisplay()
// ------------------------------------------------------------------------
void ConstellationSensor::enableDisplay(bool on)
{
    if (on == display_on_) return;
    display_on_ = on;
    if (on)
        cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
    else
        cv::destroyWindow(WIN_NAME);
}

// ------------------------------------------------------------------------
//  renderFrame() - overlay info on display_frame
// ------------------------------------------------------------------------
void ConstellationSensor::renderFrame(
    cv::Mat& frame, const SensorData& d,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    const cv::Mat& rvec, const cv::Mat& tvec,
    cv::Point2f dot_pos, bool dot_found,
    cv::Point2f active_target_proj,
    cv::Point2f interim_proj,
    cv::Point2f bound_center, float bound_radius)
{
    auto label = [&](const std::string& txt, cv::Point pos, cv::Scalar col) {
        cv::putText(frame, txt, pos, cv::FONT_HERSHEY_SIMPLEX, 0.65, col, 2);
    };

    char buf[128];
    int y = 30;

    if (!d.valid) {
        label("no targets detected", {10, y}, {0, 0, 255});
        y += 35;
        std::snprintf(buf, sizeof(buf), "fps: %.1f", d.fps);
        label(buf, {10, y}, {0, 255, 0});
        return;
    }

    cv::aruco::drawDetectedMarkers(frame, corners, ids);
    cv::drawFrameAxes(frame, cfg_.cam_matrix, cfg_.dist_coeffs,
                      rvec, tvec, float(cfg_.aruco_side));

    // draw laser check bounding circle (yellow)
    if (bound_radius > 0.f) {
        cv::circle(frame,
                   cv::Point(int(bound_center.x), int(bound_center.y)),
                   int(bound_radius),
                   cv::Scalar(0, 220, 220), 1);
    }

    // top-left: tracking status, fps, distance
    std::snprintf(buf, sizeof(buf), "markers: %d", d.num_markers);
    label(buf, {10, y}, {200, 200, 0});
    y += 30;

    std::snprintf(buf, sizeof(buf), "fps: %.1f", d.fps);
    label(buf, {10, y}, {0, 255, 0});
    y += 30;

    std::snprintf(buf, sizeof(buf), "dist: %.1f cm", d.dist_cm);
    label(buf, {10, y}, {0, 180, 255});
    y += 30;

    std::snprintf(buf, sizeof(buf), "yaw: %+.2f deg  pitch: %+.2f deg",
                  d.yaw_deg, d.pitch_deg);
    label(buf, {10, y}, {255, 100, 100});

    // bottom-left: laser info
    int h = frame.rows;
    int by = h - 150;

    if (d.laser_valid) {
        std::snprintf(buf, sizeof(buf), "laser dX: %+.2f cm", d.laser_dx_m * 100.0);
        label(buf, {10, by}, {0, 255, 200});
        by += 28;
        std::snprintf(buf, sizeof(buf), "laser dY: %+.2f cm", d.laser_dy_m * 100.0);
        label(buf, {10, by}, {0, 255, 200});
        by += 28;
        std::snprintf(buf, sizeof(buf), "laser yaw: %+.2f deg", d.laser_yaw_deg);
        label(buf, {10, by}, {0, 200, 255});
        by += 28;
        std::snprintf(buf, sizeof(buf), "laser pitch: %+.2f deg", d.laser_pitch_deg);
        label(buf, {10, by}, {0, 200, 255});
        by += 28;
        std::snprintf(buf, sizeof(buf), "target idx: %d", d.active_target_index);
        label(buf, {10, by}, {200, 200, 200});

        // green circle around laser dot
        cv::circle(frame, cv::Point(int(dot_pos.x), int(dot_pos.y)),
                   20, {0, 255, 0}, 2);

        // line from laser dot to current interim slew position
        cv::line(frame,
                 cv::Point(int(dot_pos.x), int(dot_pos.y)),
                 cv::Point(int(interim_proj.x), int(interim_proj.y)),
                 {255, 0, 255}, 2);

        // yellow dot at the final target endpoint
        cv::circle(frame, cv::Point(int(active_target_proj.x), int(active_target_proj.y)),
                   8, {0, 215, 255}, -1);
    } else if (dot_found) {
        // dot found but couldn't compute 3d (shouldn't happen if markers valid)
        cv::circle(frame, cv::Point(int(dot_pos.x), int(dot_pos.y)), 20, {0, 255, 0}, 2);
        label("laser: no 3d", {10, by}, {0, 128, 255});
    } else {
        label("laser: no dot", {10, by}, {128, 128, 128});
    }
}

// ------------------------------------------------------------------------
//  rotation funct
// ------------------------------------------------------------------------
double ConstellationSensor::rollFromRvec(const cv::Mat& rvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double roll = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    // wrap to (-pi,pi] centred on zero
    if (roll > 0.0)
        roll = M_PI - roll;
    else
        roll = -(M_PI + roll);
    return roll * 180.0 / M_PI;
}
