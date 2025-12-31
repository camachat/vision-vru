#pragma once

#include <cstdint>
#include "coco_classes.hpp"

// =============================================================================
//  TRACKEDOBJECT — YOUR DATA STRUCTURE (switch-friendly class_id)
// =============================================================================
struct TrackedObject {
    int64_t timestamp_ms = 0; // Unix epoch in milliseconds

    int id = -1;       // Persistent tracker ID
    int class_id = -1; // 0=pedestrian, 1=cyclist, 2=motorcyclist
    float confidence = 0.0f;

    float cx_norm = 0.0f; // [0.0,1.0] — left→right
    float cy_norm = 0.0f; // [0.0,1.0] — top→bottom

    float bbox_x_px = 0.0f;
    float bbox_y_px = 0.0f;
    float bbox_w_px = 0.0f;
    float bbox_h_px = 0.0f;

    float forward_m = -999.0f; // meters ahead of camera
    float lateral_m = -999.0f; // meters right of centerline

    float speed_m_s = 0.0f;
    float heading_deg = 0.0f; // [0,360): 0°=oncoming, 90°=right→left

    const char *className() const
    {
        if (class_id < coco::NUM_CLASSES) {
            return coco::get_class_name(class_id);
        }
        return "unknown";
    }
};
