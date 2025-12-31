Here are the **exact kinds of images** you need for accurate stereo calibration on Raspberry Pi 5 (or any stereo system).
These are real-world examples that work perfectly with the OpenCV calibration code I gave you.

### Required Checkerboard Pattern
Print this pattern (or any of these) on a flat, rigid surface (A4 paper glued to cardboard works great):

To list all cameras:

```sh
v4l2-ctl -A
```

Use **ffmpeg** Verify left and right camera, for example

```sh
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test_video0.jpg
```

### How to Capture 15–25 Perfect Image Pairs

```bash
# Create text files with image paths
ls calib_images/left_*.jpg  > left_images.txt
ls calib_images/right_*.jpg > right_images.txt

# Run calibration
./calibrator configs/vision_vru.json
```

After success, your `main_stevision_vrureo.cpp` detector will read `stereo_calib.xml` and give you **real meter-accurate distances** — no more guessing!
