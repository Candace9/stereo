# RealSense D455 stereo capture (C++)

Three programs:

1. **`capture_stereo`** — capture D455 IR **left/right** and **RGB** (no disparity).
2. **`compute_disparity`** — run OpenCV SGBM on folders that contain `left.png` and `right.png`.
3. **`compute_depth`** — convert `disparity_raw.png` to metric depth using `camera_paramter/D455_depth.json`.

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

## 1. Capture images

Click the OpenCV window first, then **Space** / **S** to save, **Q** / **Esc** to quit. Keys also work in the terminal. Left-click the preview to save.

RGB is saved by default.

```bash
./build/capture_stereo
./build/capture_stereo --out captures --width 1280 --height 800 --fps 30
./build/capture_stereo --count 10
./build/capture_stereo --no-rgb
./build/capture_stereo --emitter
```

If `1280x800` fails to start, try `--width 640 --height 480`.

## 2. Compute disparity

`PATH` can be one pair folder, or a parent folder of many pair folders:

```bash
./build/compute_disparity captures
./build/compute_disparity captures/20260819_151230_123
./build/compute_disparity --num-disp 192 --block-size 5 captures
./build/compute_disparity --show captures
```

`--show` opens preview windows (needs a display). Without it, the tool can run headless.

## 3. Compute depth

Requires `disparity_raw.png` from step 2. Uses \(Z = f_x B / d\) from the calib JSON.

```bash
./build/compute_depth captures
./build/compute_depth captures/20260819_151230_123
./build/compute_depth --calib camera_paramter/D455_depth.json --max-z 8 captures
./build/compute_depth --show captures
```

`--max-z` clips the color preview and the point cloud. Metric `depth_m.tiff` / `depth_mm.png` still keep all valid depths.

Point cloud: `cloud.ply` (binary, meters, OpenCV camera frame: X right, Y down, Z forward). Color comes from `rgb.png` if present, otherwise `left.png`. Open in CloudCompare or MeshLab.

```bash
./build/compute_depth --max-z 8 captures
./build/compute_depth --stride 2 captures      # lighter cloud
./build/compute_depth --no-cloud captures      # depth images only
```

## Output layout

```
captures/
  20260819_151230_123/
    left.png
    right.png
    rgb.png               # unless --no-rgb
    disparity.png         # after compute_disparity
    disparity_hist.png
    disparity_raw.png     # 16-bit SGBM output (value = disparity * 16)
    depth.png             # colorized depth (near = warm)
    depth_mm.png          # 16-bit depth in millimeters (0 = invalid)
    depth_m.tiff          # 32-bit float depth in meters (0 = invalid)
    cloud.ply             # XYZRGB point cloud (meters)
```

## Notes

- Left/right images come from the **stereo infrared cameras**, not a split RGB image.
- For SGBM, keep the **emitter off** (the default) so the dot pattern does not interfere with texture.
- If CMake cannot find RealSense: `export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH`
- If `rs-enumerate-devices` works but capture cannot open the camera, close `realsense-viewer` and retry. On Jetson USB issues, rebuild librealsense with `-DFORCE_RSUSB_BACKEND=ON`.
- Headless SSH: OpenCV windows need a display. Capture needs a desktop / `ssh -X`. `compute_disparity` without `--show` does not.
