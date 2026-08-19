# RealSense D455 stereo capture (C++)

Capture the Intel RealSense D455 stereo module **IR Left / IR Right** streams, save them as `left.png` and `right.png`, then compute an OpenCV SGBM **disparity map** and histogram.

This C++ build links `librealsense2` directly, so it does not need `pyrealsense2`.

## Dependencies (Jetson)

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake pkg-config \
  libopencv-dev \
  librealsense2-dev librealsense2-utils
```

If you built librealsense from source instead of apt, `librealsense2-dev` is not required; `sudo make install` is enough. You still need OpenCV (`libopencv-dev` or JetPack OpenCV).

## Build

```bash
cd /path/to/stereo
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(($(nproc)-1))"
```

## Usage

During preview, press **Space** to save and **Q** or **Esc** to quit:

```bash
./build/capture_stereo
```

Common options:

```bash
# Output directory and resolution
./build/capture_stereo --out captures --width 1280 --height 800 --fps 30

# Save 10 pairs per Space press
./build/capture_stereo --count 10

# Also save RGB (color camera, not the stereo pair)
./build/capture_stereo --rgb

# Enable the IR dot projector (usually leave this off for passive stereo matching)
./build/capture_stereo --emitter

# SGBM search range (must be a multiple of 16). Larger = farther near objects, slower
./build/capture_stereo --num-disp 192 --block-size 5

# Save images only, skip disparity
./build/capture_stereo --no-disp
```

If `1280x800` fails to start, try `--width 640 --height 480`.

If CMake cannot find RealSense:

```bash
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

If `rs-enumerate-devices` works but this program cannot open the camera, unplug other RealSense apps (including `realsense-viewer`) and retry. On Jetson USB issues, rebuild librealsense with `-DFORCE_RSUSB_BACKEND=ON`.

Output layout:

```
captures/
  20260819_151230_123/
    left.png
    right.png
    rgb.png               # only with --rgb
    disparity.png         # colorized disparity map
    disparity_hist.png    # disparity histogram
    disparity_raw.png     # 16-bit SGBM output (value = disparity * 16)
```

## Notes

- Left/right images come from the **stereo infrared cameras**, not a split RGB image.
- For SGBM / depth estimation, keep the **emitter off** (the default) so the dot pattern does not interfere with texture.
- Headless SSH: OpenCV windows need a display. Use a desktop session, or `ssh -X`, or run on the device monitor.
