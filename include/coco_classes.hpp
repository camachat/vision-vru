// coco_classes.hpp
// Generated: List of 80 COCO classes used in YOLOv5/v8/v10, Ultralytics, etc.
//            Indices match the official COCO evaluation order (0–79)

#ifndef COCO_CLASSES_H
#define COCO_CLASSES_H

#include <string>
#include <array>

namespace coco {

inline constexpr int NUM_CLASSES = 80;

// Array of class names (index 0 = "person", ..., index 79 = "toothbrush")
inline constexpr std::array<const char*, NUM_CLASSES> CLASS_NAMES = {{
    "person",         "bicycle",       "car",              "motorcycle",     "airplane",
    "bus",            "train",         "truck",            "boat",           "traffic light",
    "fire hydrant",   "stop sign",     "parking meter",    "bench",          "bird",
    "cat",            "dog",           "horse",            "sheep",          "cow",
    "elephant",       "bear",          "zebra",            "giraffe",        "backpack",
    "umbrella",       "handbag",       "tie",              "suitcase",       "frisbee",
    "skis",           "snowboard",     "sports ball",      "kite",           "baseball bat",
    "baseball glove", "skateboard",    "surfboard",        "tennis racket",  "bottle",
    "wine glass",     "cup",           "fork",             "knife",          "spoon",
    "bowl",           "banana",        "apple",            "sandwich",       "orange",
    "broccoli",       "carrot",        "hot dog",          "pizza",          "donut",
    "cake",           "chair",         "couch",            "potted plant",   "bed",
    "dining table",   "toilet",        "tv",               "laptop",         "mouse",
    "remote",         "keyboard",      "cell phone",       "microwave",      "oven",
    "toaster",        "sink",          "refrigerator",     "book",           "clock",
    "vase",           "scissors",      "teddy bear",       "hair drier",     "toothbrush"
}};

// Optional: human-readable version with nicer formatting (may contain spaces)
inline constexpr std::array<const char*, NUM_CLASSES> CLASS_NAMES_READABLE = {{
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush"
}};

// Helper function to get class name by index (with bounds checking in debug mode)
inline const char* get_class_name(int class_id) {
    if (class_id >= 0 && class_id < NUM_CLASSES) {
        return CLASS_NAMES[class_id];
    }
    return "invalid_class";
}

// Same but using the readable version
inline const char* get_class_name_readable(int class_id) {
    if (class_id >= 0 && class_id < NUM_CLASSES) {
        return CLASS_NAMES_READABLE[class_id];
    }
    return "invalid_class";
}

} // namespace coco

#endif // COCO_CLASSES_H