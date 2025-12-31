// simple_webcam_test.cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv) {
    int webcamId = 0;
    if (argc > 1) {
      webcamId = atoi(argv[1]);
    }
    std::cout << "Use webcam: " << webcamId << "\n";

    // Try to open the default webcam (index 0)
    // You can change to 1, 2, ... if you have multiple cameras
    //cv::VideoCapture cap(webcamId, cv::CAP_V4L2);  // Use CAP_V4L2 on Linux/FreeBSD
    //cv::VideoCapture cap(webcamId, cv::CAP_AVFOUNDATION);  // Use CAP_AVFOUNDATION on macOS
    cv::VideoCapture cap(webcamId);  // Automatic

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open webcam (index " << webcamId << ")\n";
        return -1;
    }

    // Optional: try to set a reasonable resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);           // often ignored by driver
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);           // often ignored by driver

    std::cout << "Webcam opened successfully\n";
    std::cout << "Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";
    std::cout << "Press ESC or 'q' to quit\n";

    cv::Mat frame;
    int skip_count = 0;
    const int SKIP_FRAMES = 0;// or try 5-30
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: blank frame grabbed\n";
            break;
        }

        skip_count++;
        if (skip_count < SKIP_FRAMES) {
          std::cout << "Skipping initial frame " << skip_count << "\n";
          continue;
        }

        // Optional: flip if camera is mirrored
        // cv::flip(frame, frame, 1);

        // Show simple text overlay
        cv::putText(frame, "Webcam Test - ESC/q to quit",
                    cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2);

        cv::imshow("Webcam Test", frame);

        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC or q
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    std::cout << "Webcam test finished.\n";
    return 0;
}
