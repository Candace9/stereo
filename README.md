# RealSense D455 stereo capture

Capture the Intel RealSense D455 stereo module **IR Left / IR Right** streams and save them as `left.png` and `right.png`.

## Setup

1. Install [Intel RealSense SDK 2.0](https://github.com/IntelRealSense/librealsense/releases) (on Windows, use the official installer).
2. Plug in the D455 and confirm the camera appears in Device Manager.
3. Python 3.10+:

```powershell
cd d:\stereo
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Usage

During preview, press **Space** to save and **Q** or **Esc** to quit:

```powershell
python capture_stereo.py
```

Common options:

```powershell
# Output directory and resolution
python capture_stereo.py --out captures --width 1280 --height 800 --fps 30

# Save 10 pairs per Space press
python capture_stereo.py --count 10

# Also save RGB (color camera, not the stereo pair)
python capture_stereo.py --rgb

# Enable the IR dot projector (usually leave this off for passive stereo matching)
python capture_stereo.py --emitter
```

Output layout:

```
captures/
  20260819_151230_123/
    left.png
    right.png
    rgb.png          # only with --rgb
```

## Notes

- Left/right images come from the **stereo infrared cameras**, not a split RGB image.
- For SGBM / depth estimation, keep the **emitter off** (the default) so the dot pattern does not interfere with texture.
- If `1280x800` fails to start, try `--width 640 --height 480`.
