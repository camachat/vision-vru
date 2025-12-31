#include "config.hpp"
#include <chrono>
#include <filesystem>
#include <format>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <poll.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

AppConfig appConfig{};

bool checkStdinInput()
{
    pollfd fds{};
    fds.fd = STDIN_FILENO;
    fds.events = POLLIN;

    return poll(&fds, 1, 0) > 0;
}

bool runCalibration(const std::vector<std::string> &left_files,
                    const std::vector<std::string> &right_files,
                    float measured_baseline, cv::Size board_size,
                    float square_size)
{
    if (left_files.size() < 10) {
        std::cout << "Need at least 10 image pairs for calibration (have "
                  << left_files.size() << ")\n";
        return false;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points_l, image_points_r;

    // Precompute ideal 3D points
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_size.height; ++i)
        for (int j = 0; j < board_size.width; ++j)
            objp.push_back(cv::Point3f(j * square_size, i * square_size, 0.0f));

    // Find corners in all image pairs
    int valid_pairs = 0;
    for (size_t i = 0; i < left_files.size(); ++i) {
        cv::Mat img_l = cv::imread(left_files[i], cv::IMREAD_COLOR);
        cv::Mat img_r = cv::imread(right_files[i], cv::IMREAD_COLOR);
        if (img_l.empty() || img_r.empty())
            continue;

        cv::Mat gray_l, gray_r;
        cv::cvtColor(img_l, gray_l, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img_r, gray_r, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners_l, corners_r;
        bool found_l = cv::findChessboardCorners(
            gray_l, board_size, corners_l,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found_r = cv::findChessboardCorners(
            gray_r, board_size, corners_r,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found_l && found_r) {
            cv::cornerSubPix(gray_l, corners_l, cv::Size(11, 11),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS +
                                                  cv::TermCriteria::COUNT,
                                              30, 0.001));
            cv::cornerSubPix(gray_r, corners_r, cv::Size(11, 11),
                             cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS +
                                                  cv::TermCriteria::COUNT,
                                              30, 0.001));

            object_points.push_back(objp);
            image_points_l.push_back(corners_l);
            image_points_r.push_back(corners_r);
            ++valid_pairs;
        }
    }

    if (valid_pairs < 10) {
        std::cout << "Only found " << valid_pairs
                  << " valid pairs - need at least 10\n";
        return false;
    }

    std::cout << "\nRunning calibration with " << valid_pairs
              << " valid pairs...\n";

    // Individual camera calibration
    cv::Mat K_l, D_l, K_r, D_r;
    std::vector<cv::Mat> rvecs_l, tvecs_l, rvecs_r, tvecs_r;

    cv::calibrateCamera(object_points, image_points_l, cv::Size(640, 480), K_l,
                        D_l, rvecs_l, tvecs_l);
    cv::calibrateCamera(object_points, image_points_r, cv::Size(640, 480), K_r,
                        D_r, rvecs_r, tvecs_r);

    // Stereo calibration
    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        object_points, image_points_l, image_points_r, K_l, D_l, K_r, D_r,
        cv::Size(640, 480), R, T, E, F, cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
                         1e-5));

    // Rectification
    cv::Mat Rl, Rr, Pl, Pr, Q_mat, map1l, map2l, map1r, map2r;
    cv::Rect validRoi[2];

    cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(640, 480), R, T, Rl, Rr, Pl,
                      Pr, Q_mat, cv::CALIB_ZERO_DISPARITY, -1, cv::Size(),
                      &validRoi[0], &validRoi[1]);

    cv::initUndistortRectifyMap(K_l, D_l, Rl, Pl, cv::Size(640, 480), CV_32FC1,
                                map1l, map2l);
    cv::initUndistortRectifyMap(K_r, D_r, Rr, Pr, cv::Size(640, 480), CV_32FC1,
                                map1r, map2r);

    // Calculate baseline from calibration and compare with measured value
    double calculated_baseline = cv::norm(T);
    double baseline_error_percent =
        std::abs(calculated_baseline - measured_baseline) / measured_baseline *
        100.0;

    std::cout << "\n========== Calibration Results ==========\n";
    std::cout << "RMS error: " << rms
              << " (lower = better, <1.0 is excellent)\n";
    std::cout << "Focal length: " << K_l.at<double>(0, 0) << " pixels\n";
    std::cout << "\n--- Baseline Validation ---\n";
    std::cout << "Measured baseline (hardware):   " << measured_baseline
              << " m\n";
    std::cout << "Calculated baseline (stereo):   " << calculated_baseline
              << " m\n";
    std::cout << "Error: " << baseline_error_percent << "%\n";

    // Save calibration
    std::filesystem::create_directories(
        std::filesystem::path(appConfig.stereo_calib_path).parent_path());
    cv::FileStorage fs(appConfig.stereo_calib_path, cv::FileStorage::WRITE);
    fs << "K_l" << K_l << "D_l" << D_l;
    fs << "K_r" << K_r << "D_r" << D_r;
    fs << "R" << R << "T" << T;
    fs << "Q" << Q_mat;
    fs << "map1l" << map1l << "map2l" << map2l;
    fs << "map1r" << map1r << "map2r" << map2r;
    fs << "focal" << K_l.at<double>(0, 0);
    fs << "baseline" << measured_baseline;
    fs.release();

    std::cout << "\n✓ Saved calibration to: " << appConfig.stereo_calib_path
              << "\n";

    bool calibration_good = baseline_error_percent <= 10.0;

    if (calibration_good) {
        std::cout << "✓ Baseline validation passed - calibration looks good!\n";
    } else {
        std::cout << "\n⚠️  WARNING: Baseline error > 10%!\n";
        std::cout << "Possible causes:\n";
        std::cout << "  • Wrong chessboard square size (currently "
                  << square_size * 1000 << " mm)\n";
        std::cout << "  • Poor calibration image quality\n";
        std::cout << "  • Incorrect physical measurement\n";
        std::cout << "  • Camera moved during calibration image capture\n";
        std::cout << "\nCapture more images from different angles or check "
                     "your parameters.\n";
    }
    std::cout << "==========================================\n\n";

    return calibration_good;
}

int main(int argc, char **argv)
{
    appConfig = loadConfig(argc, argv);
    CheckerboadConfig &cbConfig = appConfig.checkerboard_calibration;

    // Get measured baseline from command line or use default
    float measured_baseline = cbConfig.baseline; // Default from config
    if (measured_baseline <= 0.0f || measured_baseline > 2.0f) {
        std::cerr << "Error: Baseline must be between 0 and 2.0 meters.\n";
        return -1;
    }

    // Chessboard parameters - can be passed as command line args
    cv::Size board_size(cbConfig.board_width,
                        cbConfig.board_height); // 8×6 inner corners
    float square_size = cbConfig.square_size_meters;

    std::cout << "Interactive Stereo Calibration Tool\n";
    std::cout << "====================================\n";
    std::cout << "Calibration file: " << appConfig.stereo_calib_path << "\n";
    std::cout << "Measured baseline: " << measured_baseline << " m\n";
    std::cout << "Chessboard: " << board_size.width << "x" << board_size.height
              << " inner corners\n";
    std::cout << "Square size: " << square_size * 1000 << " mm\n\n";

    // Create output directory
    fs::create_directories("calib_images");

    // Force V4L2 backend instead of GStreamer
    cv::VideoCapture left(appConfig.left_camera_id, cv::CAP_V4L2);
    cv::VideoCapture right(appConfig.right_camera_id, cv::CAP_V4L2);

    // Set MJPEG format for better performance
    left.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    right.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Try 640x480 first to reduce USB bandwidth
    left.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    left.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    right.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    right.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Set FPS
    left.set(cv::CAP_PROP_FPS, 30);
    right.set(cv::CAP_PROP_FPS, 30);

    // Set buffer size to reduce latency
    left.set(cv::CAP_PROP_BUFFERSIZE, 1);
    right.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!left.isOpened() || !right.isOpened()) {
        std::cerr << "ERROR: Failed to open cameras\n";
        return 1;
    }

    // Warm up cameras - discard first few frames
    std::cout << "Warming up cameras...\n";
    cv::Mat dummy;
    for (int i = 0; i < 10; ++i) {
        left.read(dummy);
        right.read(dummy);
        std::this_thread::sleep_for(100ms);
    }

    std::cout << "Cameras opened successfully\n"
              << "Press ENTER to capture image pair, q to quit\n"
              << "Will capture up to 30 pairs\n\n";

    std::cout << "Cameras opened successfully\n"
              << "Commands:\n"
              << "  ENTER - Capture image pair\n"
              << "  c     - Run calibration with current images\n"
              << "  q     - Quit\n\n";

    int count = 0;
    int error_count = 0;
    constexpr int MAX_ERRORS = 5;
    bool calibration_complete = false;
    std::vector<std::string> left_files, right_files;

    while (!calibration_complete) {
        cv::Mat frameLeft, frameRight;
        bool retL = left.read(frameLeft);
        bool retR = right.read(frameRight);

        if (!retL || !retR) {
            ++error_count;
            if (error_count > MAX_ERRORS) {
                std::cerr << "Too many read errors, exiting...\n";
                break;
            }
            std::this_thread::sleep_for(500ms);
            continue;
        }

        error_count = 0; // Reset on successful read

        // Flip images horizontally to undo webcam mirroring
        if (appConfig.is_image_mirrored) {
            cv::flip(frameLeft, frameLeft, 1);
            cv::flip(frameRight, frameRight, 1);
        }

        // Non-blocking input check
        if (checkStdinInput()) {
            std::string line;
            std::getline(std::cin, line);

            if (line == "q") {
                std::cout << "Quitting...\n";
                break;
            } else if (line == "c") {
                // Run calibration with captured images
                std::cout << "\nRunning calibration...\n";
                calibration_complete =
                    runCalibration(left_files, right_files, measured_baseline,
                                   board_size, square_size);

                if (calibration_complete) {
                    std::cout << "\n✓ Calibration successful! You can quit now "
                                 "(q) or capture more images.\n\n";
                } else {
                    std::cout << "\nCapture more images and try again (c), or "
                                 "quit (q).\n\n";
                }
            } else { // ENTER pressed - capture image
                std::string leftPath =
                    std::format("calib_images/left_{:02d}.jpg", count);
                std::string rightPath =
                    std::format("calib_images/right_{:02d}.jpg", count);

                cv::imwrite(leftPath, frameLeft);
                cv::imwrite(rightPath, frameRight);

                left_files.push_back(leftPath);
                right_files.push_back(rightPath);

                std::cout << std::format("Saved pair {} (total: {})\n", count,
                                         left_files.size());
                ++count;

                // Auto-run calibration after every 5 images (starting at 10)
                if (left_files.size() >= 10 && left_files.size() % 5 == 0) {
                    std::cout << "\nAuto-running calibration with "
                              << left_files.size() << " pairs...\n";
                    calibration_complete = runCalibration(
                        left_files, right_files, measured_baseline, board_size,
                        square_size);

                    if (calibration_complete) {
                        std::cout << "\n✓ Calibration successful! Press 'q' to "
                                     "quit or capture more images.\n\n";
                    }
                }
            }
        }

        std::this_thread::sleep_for(50ms); // Small delay to avoid busy loop
    }

    left.release();
    right.release();
    std::cout << std::format("\nCaptured {} image pairs total\n", count);

    return calibration_complete ? 0 : 1;
}
