#pragma once
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

struct CheckerboadConfig {
    int board_width = 8;
    int board_height = 6;
    float square_size_meters = 0.03f;
    float baseline = 0.10f;
};

struct AppConfig {
    int left_camera_id = 0;
    int right_camera_id = 1;
    std::string yolo_model_path = "configs/models/yolo11n.onnx";
    std::string stereo_calib_path = "configs/cfg/stereo_calib.xml";
    bool enable_display = false; // Set false for headless operation
    bool is_image_mirrored = true;
    CheckerboadConfig checkerboard_calibration{};
};

AppConfig loadConfig(int argc, char **argv)
{
    std::string config_path = (argc > 1 && std::filesystem::exists(argv[1]))
                                  ? argv[1]
                                  : "/etc/edge/vision-vru/vision_vru.json";
    AppConfig cfg;
    if (std::filesystem::exists(config_path)) {
        nlohmann::json jcfg;
        std::ifstream ifs(config_path);
        ifs >> jcfg;
        ifs.close();
        if (jcfg.contains("left_camera_id"))
            cfg.left_camera_id = jcfg["left_camera_id"].get<int>();
        if (jcfg.contains("right_camera_id"))
            cfg.right_camera_id = jcfg["right_camera_id"].get<int>();
        if (jcfg.contains("yolo_model_path"))
            cfg.yolo_model_path = jcfg["yolo_model_path"].get<std::string>();
        if (jcfg.contains("stereo_calib_path"))
            cfg.stereo_calib_path =
                jcfg["stereo_calib_path"].get<std::string>();
        if (jcfg.contains("enable_display"))
            cfg.enable_display = jcfg["enable_display"].get<bool>();
        if (jcfg.contains("is_image_mirrored"))
            cfg.is_image_mirrored = jcfg["is_image_mirrored"].get<bool>();
        if (jcfg.contains("checkerboard_calibration")) {
            const auto &jcb = jcfg["checkerboard_calibration"];
            if (jcb.contains("baseline"))
                cfg.checkerboard_calibration.baseline =
                    jcb["baseline"].get<float>();
            if (jcb.contains("board_width"))
                cfg.checkerboard_calibration.board_width =
                    jcb["board_width"].get<int>();
            if (jcb.contains("board_height"))
                cfg.checkerboard_calibration.board_height =
                    jcb["board_height"].get<int>();
            if (jcb.contains("square_size_meters"))
                cfg.checkerboard_calibration.square_size_meters =
                    jcb["square_size_meters"].get<float>();
        }
    }
    return cfg;
}
