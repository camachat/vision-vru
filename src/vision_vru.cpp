// =============================================================================
//  vision_vru.cpp
//  Real-time VRU detector with real-world ground-plane coordinates and heading
//  • No "using namespace" anywhere
//  • One publish call per frame with std::vector<TrackedObject>
//  • Works on Raspberry Pi 4 & 5 (Linux + FreeBSD)
// =============================================================================

#include <filesystem>
#include <fstream>
#ifdef __FreeBSD__
#include <sys/time.h>
#include <sys/types.h>
#endif

#include "trackedobject.hpp"
#include <chrono>
#include <cmath>
#include <format>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

// =============================================================================
//  YOUR EVENT PUBLISHER — ONE CALL PER FRAME
// =============================================================================
void publishVruEvent(const std::vector<TrackedObject> &objects) {}

// =============================================================================
//  TRACKER (SORT-style)
// =============================================================================
struct Tracker {
    int id = -1;
    cv::Rect2d bbox;
    cv::Point2f center_px;
    cv::Point2f velocity_px_s;
    int class_id = -1;
    float confidence = 0.0f;
    int hits = 0;
    int misses = 0;
};

class ObjectTracker
{
    std::vector<Tracker> trackers;
    int next_id = 0;
    double fps = 30.0;

    double IoU(const cv::Rect2d &a, const cv::Rect2d &b) const
    {
        auto inter = a & b;
        auto uni = a | b;
        return uni.area() == 0 ? 0.0 : inter.area() / (double)uni.area();
    }

  public:
    void update(const std::vector<cv::Rect2d> &boxes,
                const std::vector<int> &classes,
                const std::vector<float> &confidences, double current_fps)
    {
        fps = current_fps > 1.0 ? current_fps : 30.0;

        if (trackers.empty()) {
            for (size_t i = 0; i < boxes.size(); ++i) {
                cv::Point2f c = (boxes[i].tl() + boxes[i].br()) * 0.5f;
                trackers.push_back({next_id++,
                                    boxes[i],
                                    c,
                                    {0, 0},
                                    classes[i],
                                    confidences[i],
                                    1,
                                    0});
            }
            return;
        }

        std::vector<int> assignment(boxes.size(), -1);
        std::vector<bool> used(trackers.size(), false);

        for (int iter = 0; iter < 10; ++iter) {
            double best_iou = 0.3;
            int best_t = -1, best_d = -1;
            for (size_t t = 0; t < trackers.size(); ++t) {
                if (used[t] || trackers[t].misses > 10)
                    continue;
                for (size_t d = 0; d < boxes.size(); ++d) {
                    if (assignment[d] != -1)
                        continue;
                    double iou_val = IoU(trackers[t].bbox, boxes[d]);
                    if (iou_val > best_iou) {
                        best_iou = iou_val;
                        best_t = t;
                        best_d = d;
                    }
                }
            }
            if (best_t == -1)
                break;
            used[best_t] = true;
            assignment[best_d] = best_t;
        }

        // Update matched trackers
        for (size_t d = 0; d < boxes.size(); ++d) {
            if (assignment[d] == -1)
                continue;
            int t = assignment[d];
            cv::Point2f new_c = (boxes[d].tl() + boxes[d].br()) * 0.5f;
            cv::Point2f delta = new_c - trackers[t].center_px;
            trackers[t].velocity_px_s = delta * static_cast<float>(fps);
            trackers[t].center_px = new_c;
            trackers[t].bbox = boxes[d];
            trackers[t].class_id = classes[d];
            trackers[t].confidence = confidences[d];
            trackers[t].hits++;
            trackers[t].misses = 0;
        }

        // Increase misses for unmatched
        for (size_t t = 0; t < trackers.size(); ++t) {
            if (!used[t])
                trackers[t].misses++;
        }

        // Add new detections
        for (size_t d = 0; d < boxes.size(); ++d) {
            if (assignment[d] == -1) {
                cv::Point2f c = (boxes[d].tl() + boxes[d].br()) * 0.5f;
                trackers.push_back({next_id++,
                                    boxes[d],
                                    c,
                                    {0, 0},
                                    classes[d],
                                    confidences[d],
                                    1,
                                    0});
            }
        }

        // Remove dead trackers
        trackers.erase(
            std::remove_if(trackers.begin(), trackers.end(),
                           [](const Tracker &t) { return t.misses > 15; }),
            trackers.end());
    }

    std::vector<Tracker> getActive() const
    {
        std::vector<Tracker> active;
        for (const auto &t : trackers) {
            if (t.hits >= 3)
                active.push_back(t);
        }
        return active;
    }
};

// =============================================================================
//  STEREO CALIBRATION & GROUND PLANE PROJECTION
// =============================================================================
cv::Mat map1l, map2l, map1r, map2r;
double focal_px = 600.0;
double baseline_m = 0.08;

void loadCalibration(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["map1l"] >> map1l;
        fs["map2l"] >> map2l;
        fs["map1r"] >> map1r;
        fs["map2r"] >> map2r;
        fs["focal"] >> focal_px;
        fs["baseline"] >> baseline_m;
        fs.release();
        std::cout << "[OK] Loaded calibration: focal=" << focal_px
                  << "px, baseline=" << baseline_m << "m\n";
    } else {
        std::cout << "[WARN] No stereo_calib.xml — using defaults (depth will "
                     "be inaccurate)\n";
    }
}

bool pixelToGroundPlane(const cv::Point2f &pt_norm, float depth_m,
                        float &forward_m, float &lateral_m)
{
    if (depth_m < 0.3f)
        return false;

    const float w = 640.0f, h = 480.0f;
    float u = pt_norm.x * w;
    float v = pt_norm.y * h;
    float cx = w / 2.0f, cy = h / 2.0f;

    // Camera at ground level looking horizontally
    // Xc = lateral (left/right), Zc = forward (depth)
    float Xc = (u - cx) * depth_m / focal_px;
    float Zc = depth_m;

    if (std::abs(Zc) < 0.01f)
        return false;

    forward_m = Zc;
    lateral_m = Xc;

    return true;
}

// =============================================================================
//  Confituration
// =============================================================================
bool isGuiAvailable()
{
    // Try to create a dummy window to test GUI availability
    try {
        cv::Mat dummy(1, 1, CV_8UC3);
        cv::namedWindow("__gui_test__", cv::WINDOW_NORMAL);
        cv::destroyWindow("__gui_test__");
        return true;
    } catch (...) {
        return false;
    }
}

std::string timestamp_string()
{
    auto now_tp = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now_tp.time_since_epoch());
    auto now_s = std::chrono::duration_cast<std::chrono::seconds>(now_ms);
    std::time_t now_time = now_s.count();
    int ms = now_ms.count() % 1000;
    std::tm tm_buf;
    localtime_r(&now_time, &tm_buf);
    return std::format("{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}_{:03d}: ",
                       tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                       tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, ms);
}
// =============================================================================
//  MAIN
// =============================================================================
#include "config.hpp"
AppConfig appConfig{};
int main(int argc, char **argv)
{
    // YOLO model input/output resolution (must match blobFromImage size)
    constexpr int YOLO_INPUT_SIZE = 640;

    appConfig = loadConfig(argc, argv);

    // Check if display is actually available (override config if not)
    if (appConfig.enable_display && !isGuiAvailable()) {
        std::printf("[WARN] Display requested but GUI not available - running "
                    "headless\n");
        appConfig.enable_display = false;
    }

    std::printf("Using YOLO model: %s\n", appConfig.yolo_model_path.c_str());
    std::printf("Using stereo calib: %s\n",
                appConfig.stereo_calib_path.c_str());
    std::printf("Display mode: %s\n",
                appConfig.enable_display ? "enabled" : "headless");

    loadCalibration(appConfig.stereo_calib_path);

    // Put this before the while(true) loop, after loadCalibration()
    if (appConfig.enable_display) {
        cv::namedWindow("VRU Detector", cv::WINDOW_NORMAL); // ← important!
        cv::resizeWindow("VRU Detector", 1280, 960); // initial size (optional)
        // or cv::setWindowProperty("VRU Detector", cv::WND_PROP_FULLSCREEN,
        // cv::WINDOW_FULLSCREEN);
    }
    auto net = cv::dnn::readNetFromONNX(
        appConfig.yolo_model_path); // or yolo11n.onnx, yolov12n.onnx
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    auto stereo = cv::StereoSGBM::create(0, 128, 5);
    stereo->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    //cv::VideoCapture capL(appConfig.left_camera_id, cv::CAP_V4L2),
    //    capR(appConfig.right_camera_id, cv::CAP_V4L2);
    cv::VideoCapture capL(appConfig.left_camera_id),
        capR(appConfig.right_camera_id);

    if (!capL.isOpened() || !capR.isOpened()) {
        std::cerr << "ERROR: Cannot open cameras\n";
        return -1;
    }

    // Set MJPEG format for better USB bandwidth efficiency
    capL.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capR.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    capL.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capL.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capR.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capR.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    capL.set(cv::CAP_PROP_FPS, 30);
    capR.set(cv::CAP_PROP_FPS, 30);

    // Reduce buffer size to minimize latency
    capL.set(cv::CAP_PROP_BUFFERSIZE, 2);
    capR.set(cv::CAP_PROP_BUFFERSIZE, 2);

    ObjectTracker tracker;
    cv::Mat frameL, frameR, rectL, rectR, grayL, grayR, disparity, blob;
    auto prev_time = std::chrono::steady_clock::now();
    double fps = 30.0;

    std::map<int, std::pair<cv::Point2f, int64_t>> pos_history;

    while (true) {
        capL >> frameL;
        capR >> frameR;
        if (frameL.empty() || frameR.empty())
            break;

        // Flip if needed
        if (appConfig.is_image_mirrored) {
            cv::flip(frameL, frameL, 1); // 1 = horizontal flip
            cv::flip(frameR, frameR, 1); // 1 = horizontal flip
        }

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(
                        now - prev_time)
                        .count() /
                    1e6;
        if (dt > 0)
            fps = 0.9 * fps + 0.1 / dt;
        prev_time = now;

        // Stereo rectification
        cv::remap(frameL, rectL, map1l, map2l, cv::INTER_LINEAR);
        cv::remap(frameR, rectR, map1r, map2r, cv::INTER_LINEAR);
        cv::cvtColor(rectL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rectR, grayR, cv::COLOR_BGR2GRAY);
        stereo->compute(grayL, grayR, disparity);
        disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);

        // YOLO inference on left image
        cv::dnn::blobFromImage(frameL, blob, 1 / 255.0,
                               cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE),
                               cv::Scalar(), true, false);
        net.setInput(blob);
        std::vector<cv::Mat> outs;
        net.forward(outs, net.getUnconnectedOutLayersNames());

        std::vector<cv::Rect2d> boxes;
        std::vector<int> classes;
        std::vector<float> confidences;

        // YOLOv11 output format: [1, 84, 8400]
        // 84 = 4 bbox coords + 80 class scores (no objectness score)
        cv::Mat output = outs[0];

        // Transpose if needed: [1, 84, 8400] -> [8400, 84]
        if (output.size[1] == 84) {
            cv::Mat output_t(output.size[2], output.size[1], CV_32F);
            for (int i = 0; i < output.size[2]; ++i) {
                for (int j = 0; j < output.size[1]; ++j) {
                    output_t.at<float>(i, j) = output.at<float>(0, j, i);
                }
            }
            output = output_t;
        }

        float *data = (float *)output.data;
        int num_detections = output.rows;

        for (int i = 0; i < num_detections; ++i, data += 84) {
            // YOLO11: first 4 are bbox, next 80 are class scores
            float cx = data[0], cy = data[1], w = data[2], h = data[3];

            // Find max class score
            float max_score = 0.0f;
            int max_class = -1;
            for (int c = 0; c < 80; ++c) {
                if (data[4 + c] > max_score) {
                    max_score = data[4 + c];
                    max_class = c;
                }
            }

            // Filter: confidence > 0.5
            if (max_score > 0.5f) {
                // Scale from YOLO's output space back to actual frame
                // dimensions
                int x = static_cast<int>((cx - w / 2) * frameL.cols /
                                         YOLO_INPUT_SIZE);
                int y = static_cast<int>((cy - h / 2) * frameL.rows /
                                         YOLO_INPUT_SIZE);
                int ww = static_cast<int>(w * frameL.cols / YOLO_INPUT_SIZE);
                int hh = static_cast<int>(h * frameL.rows / YOLO_INPUT_SIZE);
                boxes.emplace_back(x, y, ww, hh);
                classes.push_back(max_class);
                confidences.push_back(max_score);
            }
        }

        std::vector<int> nms_idx;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5f, 0.4f, nms_idx);

        std::vector<cv::Rect2d> final_boxes;
        std::vector<int> final_classes;
        std::vector<float> final_confs;
        for (int i : nms_idx) {
            final_boxes.push_back(boxes[i]);
            final_classes.push_back(classes[i]);
            final_confs.push_back(confidences[i]);
        }

        tracker.update(final_boxes, final_classes, final_confs, fps);

        // Build current frame objects
        std::vector<TrackedObject> current_objects;
        int64_t frame_ts =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();

        for (const auto &t : tracker.getActive()) {
            cv::Point2f cnorm(t.center_px.x / frameL.cols,
                              t.center_px.y / frameL.rows);
            int cy = static_cast<int>(t.center_px.y);
            int cx = static_cast<int>(t.center_px.x);

            // Convert disparity to actual depth using stereo formula: Z = (f *
            // B) / d
            float disp_value = disparity.empty() || cy < 0 ||
                                       cy >= disparity.rows || cx < 0 ||
                                       cx >= disparity.cols
                                   ? -1.0f
                                   : disparity.at<float>(cy, cx);

            float depth = -1.0f;
            if (disp_value > 1.0f) // Avoid division by zero/small values
            {
                depth =
                    static_cast<float>((focal_px * baseline_m) / disp_value);
            }

            float forward = -999.0f, lateral = -999.0f;
            pixelToGroundPlane(cnorm, depth, forward, lateral);

            // Speed & heading from history
            float speed = 0.0f, heading = 0.0f;
            if (pos_history.count(t.id)) {
                auto [prev_pos, prev_t] = pos_history[t.id];
                float dt_s = (frame_ts - prev_t) / 1000.0f;
                if (dt_s > 0.05f) {
                    float vx = (forward - prev_pos.x) / dt_s;
                    float vy = (lateral - prev_pos.y) / dt_s;
                    speed = std::sqrt(vx * vx + vy * vy);
                    heading = std::atan2(vy, vx) * 180.0f / CV_PI;
                    if (heading < 0)
                        heading += 360.0f;
                }
            }
            pos_history[t.id] = {{forward, lateral}, frame_ts};

            TrackedObject obj;
            obj.timestamp_ms = frame_ts;
            obj.id = t.id;
            obj.class_id = t.class_id;
            obj.confidence = t.confidence;
            obj.cx_norm = cnorm.x;
            obj.cy_norm = cnorm.y;
            obj.bbox_x_px = static_cast<float>(t.bbox.x);
            obj.bbox_y_px = static_cast<float>(t.bbox.y);
            obj.bbox_w_px = static_cast<float>(t.bbox.width);
            obj.bbox_h_px = static_cast<float>(t.bbox.height);
            obj.forward_m = forward;
            obj.lateral_m = lateral;
            obj.speed_m_s = speed;
            obj.heading_deg = heading;

            current_objects.push_back(obj);
        }

        // =====================================================================
        //  YOUR EVENT PUBLISHING — ONE CALL PER FRAME
        // =====================================================================
        publishVruEvent(current_objects);

        // Optional: keep JSON on stdout for debugging
        if (!current_objects.empty()) {
            nlohmann::json jarr = nlohmann::json::array();
            for (const auto &o : current_objects) {
                jarr.push_back(
                    {{"ts", o.timestamp_ms},
                     {"id", o.id},
                     {"class_id", o.class_id},
                     {"class", o.className()},
                     {"conf", o.confidence},
                     {"cx", o.cx_norm},
                     {"cy", o.cy_norm},
                     {"bbox",
                      {o.bbox_x_px, o.bbox_y_px, o.bbox_w_px, o.bbox_h_px}},
                     {"forward_m", o.forward_m},
                     {"lateral_m", o.lateral_m},
                     {"speed_m_s", o.speed_m_s},
                     {"heading_deg", o.heading_deg}});
            }

            std::cout << timestamp_string() << jarr.dump() << '\n';
            std::cout.flush();
        }

        // Visualization
        if (appConfig.enable_display) {
            // Check if frames are valid
            if (frameL.empty() || frameR.empty()) {
                std::cerr << "ERROR: Frame is empty\n";
                break;
            }
            for (const auto &o : current_objects) {
                cv::Rect r(o.bbox_x_px, o.bbox_y_px, o.bbox_w_px, o.bbox_h_px);
                cv::rectangle(frameL, r, cv::Scalar(0, 255, 0), 2);
                std::string label = std::format(
                    "#{} {} {:.1f}m {:.1f}m/s {:.0f}°", o.id, o.className(),
                    o.forward_m, o.speed_m_s, o.heading_deg);
                cv::putText(frameL, label, cv::Point2f(r.x, r.y - 8), 0, 0.6,
                            cv::Scalar(0, 255, 0), 2);
            }
            try {
                cv::imshow("VRU Detector", frameL);
            } catch (const cv::Exception &e) {
                std::cerr << "OpenCV error: " << e.what() << '\n';
            }
        }
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;
        if (key == 'f' || key == 'F') { // F → toggle fullscreen
            static bool is_fullscreen = false;
            is_fullscreen = !is_fullscreen;
            cv::setWindowProperty("VRU Detector", cv::WND_PROP_FULLSCREEN,
                                  is_fullscreen ? cv::WINDOW_FULLSCREEN
                                                : cv::WINDOW_NORMAL);
        }
    }

    return 0;
}
