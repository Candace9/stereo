// Capture RealSense D455 left/right IR images and optional RGB.

#include "sgbm.hpp"

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <conio.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

class TerminalKeys {
 public:
  TerminalKeys() {
#if !defined(_WIN32)
    if (!isatty(STDIN_FILENO)) {
      return;
    }
    if (tcgetattr(STDIN_FILENO, &old_) != 0) {
      return;
    }
    termios raw = old_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      return;
    }
    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    active_ = true;
#endif
  }

  ~TerminalKeys() {
#if !defined(_WIN32)
    if (!active_) {
      return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    if (old_flags_ >= 0) {
      fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }
#endif
  }

  int poll() {
#if defined(_WIN32)
    if (_kbhit()) {
      return _getch();
    }
    return -1;
#else
    if (!active_) {
      return -1;
    }
    unsigned char c = 0;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      return static_cast<int>(c);
    }
    return -1;
#endif
  }

 private:
#if !defined(_WIN32)
  termios old_{};
  int old_flags_{-1};
  bool active_{false};
#endif
};

bool is_quit_key(int key) {
  if (key < 0) {
    return false;
  }
  const int c = key & 0xFF;
  return c == 'q' || c == 'Q' || c == 27;
}

bool is_save_key(int key) {
  if (key < 0) {
    return false;
  }
  const int c = key & 0xFF;
  if (c == 255) {
    return false;
  }
  return c == ' ' || c == 's' || c == 'S' || c == '\r' || c == '\n';
}

void on_mouse_save(int event, int, int, int, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    *static_cast<std::atomic<bool>*>(userdata) = true;
  }
}

struct Options {
  fs::path out_dir{"captures"};
  int width{1280};
  int height{800};
  int fps{30};
  int count{1};
  int warmup{15};
  bool emitter{false};
  bool rgb{true};
};

void print_usage(const char* argv0) {
  std::cout
      << "Capture RealSense D455 left/right IR images and RGB\n\n"
      << "Usage: " << argv0 << " [options]\n\n"
      << "  --out DIR         Output directory (default: captures)\n"
      << "  --width N         IR/RGB width (default: 1280)\n"
      << "  --height N        IR/RGB height (default: 800)\n"
      << "  --fps N           Frame rate (default: 30)\n"
      << "  --count N         Pairs to save after each Space press (default: 1)\n"
      << "  --warmup N        Frames to discard before preview (default: 15)\n"
      << "  --emitter         Enable IR projector (dot pattern)\n"
      << "  --no-rgb          Do not save the RGB color image\n"
      << "  -h, --help        Show this help\n\n"
      << "Keys (OpenCV window OR this terminal): Space/S save, Q/Esc quit.\n"
      << "Click the preview window first, or type in the terminal.\n"
      << "Disparity is computed separately with compute_disparity.\n";
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
      opt.width = parse_int_arg(need_value("--width"), "--width");
    } else if (arg == "--height") {
      opt.height = parse_int_arg(need_value("--height"), "--height");
    } else if (arg == "--fps") {
      opt.fps = parse_int_arg(need_value("--fps"), "--fps");
    } else if (arg == "--count") {
      opt.count = parse_int_arg(need_value("--count"), "--count");
    } else if (arg == "--warmup") {
      opt.warmup = parse_int_arg(need_value("--warmup"), "--warmup");
    } else if (arg == "--emitter") {
      opt.emitter = true;
    } else if (arg == "--no-rgb") {
      opt.rgb = false;
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

    cv::startWindowThread();
    cv::namedWindow("D455 stereo  L | R", cv::WINDOW_NORMAL);
    cv::resizeWindow("D455 stereo  L | R", 1280, 480);
    std::atomic<bool> mouse_save{false};
    cv::setMouseCallback("D455 stereo  L | R", on_mouse_save, &mouse_save);

    TerminalKeys terminal_keys;

    std::cout << "Preview: Space/S save, Q or Esc quit\n"
              << "Use the OpenCV window (click it first) OR this terminal.\n"
              << "Left-click the preview to save if keys still do not work.\n"
              << "RGB: " << (opt.rgb ? "on" : "off") << "\n";
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
      cv::putText(vis, "SPACE/S: save  |  Q/ESC: quit  |  click: save", cv::Point(16, 28),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA);
      cv::imshow("D455 stereo  L | R", vis);
      if (!rgb.empty()) {
        cv::namedWindow("D455 RGB", cv::WINDOW_NORMAL);
        cv::imshow("D455 RGB", rgb);
      }

      const int gui_key = cv::waitKeyEx(30);
      bool quit = is_quit_key(gui_key);
      bool save = is_save_key(gui_key);
      for (;;) {
        const int term_key = terminal_keys.poll();
        if (term_key < 0) {
          break;
        }
        quit = quit || is_quit_key(term_key);
        save = save || is_save_key(term_key);
      }
      if (cv::getWindowProperty("D455 stereo  L | R", cv::WND_PROP_VISIBLE) < 1) {
        quit = true;
      }
      if (quit) {
        break;
      }
      if (save || mouse_save.exchange(false)) {
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
