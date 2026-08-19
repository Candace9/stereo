// Capture and save RealSense D455 left/right stereo images.
// The stereo module is IR Left / IR Right. Use --rgb to also save color.

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

struct Options {
  fs::path out_dir{"captures"};
  int width{1280};
  int height{800};
  int fps{30};
  int count{1};
  int warmup{15};
  bool emitter{false};
  bool rgb{false};
};

void print_usage(const char* argv0) {
  std::cout
      << "Capture and save RealSense D455 left/right stereo images\n\n"
      << "Usage: " << argv0 << " [options]\n\n"
      << "  --out DIR       Output directory (default: captures)\n"
      << "  --width N       IR width (default: 1280)\n"
      << "  --height N      IR height (default: 800)\n"
      << "  --fps N         Frame rate (default: 30)\n"
      << "  --count N       Pairs to save after each Space press (default: 1)\n"
      << "  --warmup N      Frames to discard before preview (default: 15)\n"
      << "  --emitter       Enable IR projector (dot pattern)\n"
      << "  --rgb           Also save the RGB color image\n"
      << "  -h, --help      Show this help\n";
}

int parse_int(const char* text, const char* flag) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    throw std::runtime_error(std::string("Invalid integer for ") + flag + ": " + text);
  }
  return static_cast<int>(value);
}

Options parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + flag);
      }
      return argv[++i];
    };

    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--out") {
      opt.out_dir = need_value("--out");
    } else if (arg == "--width") {
      opt.width = parse_int(need_value("--width"), "--width");
    } else if (arg == "--height") {
      opt.height = parse_int(need_value("--height"), "--height");
    } else if (arg == "--fps") {
      opt.fps = parse_int(need_value("--fps"), "--fps");
    } else if (arg == "--count") {
      opt.count = parse_int(need_value("--count"), "--count");
    } else if (arg == "--warmup") {
      opt.warmup = parse_int(need_value("--warmup"), "--warmup");
    } else if (arg == "--emitter") {
      opt.emitter = true;
    } else if (arg == "--rgb") {
      opt.rgb = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }
  return opt;
}

std::string timestamp_ms() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto t = system_clock::to_time_t(now);
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S_") << std::setw(3) << std::setfill('0')
      << ms.count();
  return oss.str();
}

cv::Mat ir_to_bgr(const cv::Mat& ir) {
  cv::Mat bgr;
  cv::cvtColor(ir, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

cv::Mat frame_to_mat(const rs2::video_frame& frame, int type) {
  return cv::Mat(frame.get_height(), frame.get_width(), type,
                 const_cast<void*>(frame.get_data()))
      .clone();
}

fs::path save_pair(const fs::path& out_dir, const cv::Mat& left, const cv::Mat& right,
                   const cv::Mat& rgb) {
  const fs::path pair_dir = out_dir / timestamp_ms();
  fs::create_directories(pair_dir);
  cv::imwrite((pair_dir / "left.png").string(), left);
  cv::imwrite((pair_dir / "right.png").string(), right);
  if (!rgb.empty()) {
    cv::imwrite((pair_dir / "rgb.png").string(), rgb);
  }
  return pair_dir;
}

int main(int argc, char** argv) {
  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }

  fs::create_directories(opt.out_dir);

  try {
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_INFRARED, 1, opt.width, opt.height, RS2_FORMAT_Y8,
                         opt.fps);
    config.enable_stream(RS2_STREAM_INFRARED, 2, opt.width, opt.height, RS2_FORMAT_Y8,
                         opt.fps);
    if (opt.rgb) {
      config.enable_stream(RS2_STREAM_COLOR, opt.width, opt.height, RS2_FORMAT_BGR8,
                           opt.fps);
    }

    const rs2::pipeline_profile profile = pipeline.start(config);
    const rs2::device device = profile.get_device();
    const std::vector<rs2::sensor> sensors = device.query_sensors();
    for (const rs2::sensor& sensor : sensors) {
      if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, opt.emitter ? 1.f : 0.f);
      }
    }

    for (int i = 0; i < opt.warmup; ++i) {
      pipeline.wait_for_frames();
    }

    std::cout << "Preview: press Space to save, Q or Esc to quit\n";
    if (opt.emitter) {
      std::cout << "IR projector: on\n";
    } else {
      std::cout << "IR projector: off (recommended for passive stereo matching)\n";
    }

    int saved = 0;
    int pending = 0;

    while (true) {
      const rs2::frameset frames = pipeline.wait_for_frames();
      const rs2::video_frame left_frame = frames.get_infrared_frame(1);
      const rs2::video_frame right_frame = frames.get_infrared_frame(2);
      if (!left_frame || !right_frame) {
        continue;
      }

      const cv::Mat left = frame_to_mat(left_frame, CV_8UC1);
      const cv::Mat right = frame_to_mat(right_frame, CV_8UC1);
      cv::Mat rgb;
      if (opt.rgb) {
        const rs2::video_frame color_frame = frames.get_color_frame();
        if (color_frame) {
          rgb = frame_to_mat(color_frame, CV_8UC3);
        }
      }

      cv::Mat vis;
      cv::hconcat(ir_to_bgr(left), ir_to_bgr(right), vis);
      const double scale = 960.0 / vis.cols;
      if (scale < 1.0) {
        cv::resize(vis, vis, cv::Size(), scale, scale);
      }
      cv::putText(vis, "SPACE: save  |  Q/ESC: quit", cv::Point(16, 28),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA);
      cv::imshow("D455 stereo  L | R", vis);
      if (!rgb.empty()) {
        cv::imshow("D455 RGB", rgb);
      }

      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 'Q' || key == 27) {
        break;
      }
      if (key == 32) {
        pending = opt.count;
      }

      if (pending > 0) {
        const fs::path pair_dir = save_pair(opt.out_dir, left, right, rgb);
        ++saved;
        --pending;
        std::cout << "Saved " << pair_dir.string() << "\n";
      }
    }

    pipeline.stop();
    cv::destroyAllWindows();
    std::cout << "Saved " << saved << " pair(s)\n";
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  return 0;
}
