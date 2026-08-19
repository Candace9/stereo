"""Capture and save RealSense D455 left/right stereo images.

The D455 stereo module is a pair of infrared cameras (IR Left / IR Right).
This script saves that pair by default. Use --rgb to also save the color image.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and save RealSense D455 left/right stereo images"
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("captures"),
        help="Output directory (default: captures)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="IR width (default: 1280; D455 commonly uses 1280x800 or 640x480)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=800,
        help="IR height (default: 800)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate (default: 30)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of pairs to save after each Space press (default: 1)",
    )
    parser.add_argument(
        "--emitter",
        action="store_true",
        help="Enable the IR projector (dot pattern). Leave off for passive stereo matching",
    )
    parser.add_argument(
        "--rgb",
        action="store_true",
        help="Also save the RGB color image",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=15,
        help="Frames to discard before preview so auto-exposure can settle (default: 15)",
    )
    return parser.parse_args()


def ir_to_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def save_pair(
    out_dir: Path,
    left: np.ndarray,
    right: np.ndarray,
    rgb: np.ndarray | None,
) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    pair_dir = out_dir / stamp
    pair_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(pair_dir / "left.png"), left)
    cv2.imwrite(str(pair_dir / "right.png"), right)
    if rgb is not None:
        cv2.imwrite(str(pair_dir / "rgb.png"), rgb)
    return pair_dir


def main() -> None:
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps
    )
    config.enable_stream(
        rs.stream.infrared, 2, args.width, args.height, rs.format.y8, args.fps
    )
    if args.rgb:
        config.enable_stream(
            rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
        )

    profile = pipeline.start(config)
    device = profile.get_device()

    depth_sensor = device.first_depth_sensor()
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1.0 if args.emitter else 0.0)

    try:
        for _ in range(args.warmup):
            pipeline.wait_for_frames()

        print("Preview: press Space to save, Q or Esc to quit")
        if args.emitter:
            print("IR projector: on")
        else:
            print("IR projector: off (recommended for passive stereo matching)")

        saved = 0
        pending = 0

        while True:
            frames = pipeline.wait_for_frames()
            left_frame = frames.get_infrared_frame(1)
            right_frame = frames.get_infrared_frame(2)
            if not left_frame or not right_frame:
                continue

            left = np.asanyarray(left_frame.get_data())
            right = np.asanyarray(right_frame.get_data())
            rgb = None
            if args.rgb:
                color_frame = frames.get_color_frame()
                if color_frame:
                    rgb = np.asanyarray(color_frame.get_data())

            vis = np.hstack((ir_to_bgr(left), ir_to_bgr(right)))
            scale = 960 / vis.shape[1]
            if scale < 1:
                vis = cv2.resize(vis, None, fx=scale, fy=scale)
            cv2.putText(
                vis,
                "SPACE: save  |  Q/ESC: quit",
                (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("D455 stereo  L | R", vis)
            if rgb is not None:
                cv2.imshow("D455 RGB", rgb)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == 32:
                pending = args.count

            if pending > 0:
                pair_dir = save_pair(args.out, left, right, rgb)
                saved += 1
                pending -= 1
                print(f"Saved {pair_dir}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"Saved {saved} pair(s)")


if __name__ == "__main__":
    main()
