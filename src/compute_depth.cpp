// Convert disparity_raw.png to metric depth using D455_depth.json.

#include "sgbm.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct DepthCalib {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double baseline_m{0.0};
    double fx_times_baseline_m{0.0};
    double sgbm_scale{16.0};
};

struct Options {
    fs::path input;
    fs::path calib{"camera_paramter/D455_depth.json"};
    double max_z{10.0};
    int stride{1};
    bool show{false};
    bool no_cloud{false};
};

void print_usage(const char* argv0) {
    std::cout
        << "Compute metric depth from disparity_raw.png\n\n"
        << "Usage: " << argv0 << " [options] PATH\n\n"
        << "PATH is either:\n"
        << "  a pair folder containing disparity_raw.png, or\n"
        << "  a parent folder (each subfolder with disparity_raw.png is processed)\n\n"
        << "  --calib FILE      Camera JSON (default: camera_paramter/D455_depth.json)\n"
        << "  --max-z M         Clip color map and point cloud at this range in meters (default: 10)\n"
        << "  --stride N        Keep every Nth pixel in the point cloud (default: 1)\n"
        << "  --no-cloud        Do not write cloud.ply\n"
        << "  --show            Show OpenCV preview windows\n"
        << "  -h, --help        Show this help\n";
}

double parse_double_arg(const char* text, const char* flag) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (end == text || *end != '\0') {
        throw std::runtime_error(std::string("Invalid number for ") + flag + ": " + text);
    }
    return value;
}

double json_number(const std::string& text, const std::string& key) {
    const std::string pat = "\"" + key + "\"";
    const std::size_t k = text.find(pat);
    if (k == std::string::npos) {
        throw std::runtime_error("JSON key not found: " + key);
    }
    const std::size_t colon = text.find(':', k + pat.size());
    if (colon == std::string::npos) {
        throw std::runtime_error("JSON value missing for: " + key);
    }
    char* end = nullptr;
    const double value = std::strtod(text.c_str() + colon + 1, &end);
    if (end == text.c_str() + colon + 1) {
        throw std::runtime_error("JSON value is not a number: " + key);
    }
    return value;
}

DepthCalib load_calib(const fs::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open calib JSON: " + path.string());
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();

    DepthCalib calib;
    calib.fx = json_number(text, "fx");
    calib.fy = json_number(text, "fy");
    calib.cx = json_number(text, "cx");
    calib.cy = json_number(text, "cy");
    calib.baseline_m = json_number(text, "baseline_m");
    calib.sgbm_scale = json_number(text, "sgbm_output_scale");
    try {
        calib.fx_times_baseline_m = json_number(text, "fx_times_baseline_m");
    } catch (const std::exception&) {
        calib.fx_times_baseline_m = calib.fx * calib.baseline_m;
    }
    if (!(calib.fx_times_baseline_m > 0.0) || !(calib.sgbm_scale > 0.0) || !(calib.fx > 0.0) ||
        !(calib.fy > 0.0)) {
        throw std::runtime_error("Invalid camera parameters in " + path.string());
    }
    return calib;
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
        } else if (arg == "--calib") {
            opt.calib = need_value("--calib");
        } else if (arg == "--max-z") {
            opt.max_z = parse_double_arg(need_value("--max-z"), "--max-z");
        } else if (arg == "--stride") {
            opt.stride = parse_int_arg(need_value("--stride"), "--stride");
        } else if (arg == "--no-cloud") {
            opt.no_cloud = true;
        } else if (arg == "--show") {
            opt.show = true;
        } else if (!arg.empty() && arg[0] == '-') {
            throw std::runtime_error("Unknown argument: " + arg);
        } else if (opt.input.empty()) {
            opt.input = arg;
        } else {
            throw std::runtime_error("Unexpected extra path: " + arg);
        }
    }
    if (opt.input.empty()) {
        throw std::runtime_error("Missing PATH");
    }
    if (!(opt.max_z > 0.0)) {
        throw std::runtime_error("--max-z must be > 0");
    }
    if (opt.stride < 1) {
        throw std::runtime_error("--stride must be >= 1");
    }
    return opt;
}

bool is_depth_dir(const fs::path& dir) {
    return fs::exists(dir / "disparity_raw.png");
}

std::vector<fs::path> collect_dirs(const fs::path& input) {
    if (!fs::exists(input)) {
        throw std::runtime_error("Path does not exist: " + input.string());
    }
    if (!fs::is_directory(input)) {
        throw std::runtime_error("PATH must be a directory: " + input.string());
    }

    std::vector<fs::path> dirs;
    if (is_depth_dir(input)) {
        dirs.push_back(input);
        return dirs;
    }
    for (const fs::directory_entry& entry : fs::directory_iterator(input)) {
        if (entry.is_directory() && is_depth_dir(entry.path())) {
            dirs.push_back(entry.path());
        }
    }
    std::sort(dirs.begin(), dirs.end());
    return dirs;
}

cv::Mat colorize_depth(const cv::Mat& depth_m, double max_z) {
    cv::Mat valid = (depth_m > 0.0f) & (depth_m <= static_cast<float>(max_z));
    double min_v = 0.0;
    double max_v = 0.0;
    cv::minMaxLoc(depth_m, &min_v, &max_v, nullptr, nullptr, valid);
    if (!(max_v > min_v)) {
        return cv::Mat::zeros(depth_m.size(), CV_8UC3);
    }

    // Invert so nearer (smaller Z) maps to warmer JET colors, like disparity.
    cv::Mat inv;
    depth_m.convertTo(inv, CV_32F, -1.0, max_v);
    cv::Mat norm8;
    inv.convertTo(norm8, CV_8U, 255.0 / (max_v - min_v));
    norm8.setTo(0, ~valid);

    cv::Mat color;
    cv::applyColorMap(norm8, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(0, 0, 0), ~valid);
    return color;
}

#pragma pack(push, 1)
struct PlyVertex {
    float x;
    float y;
    float z;
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
};
#pragma pack(pop)

cv::Mat load_cloud_color(const fs::path& dir, const cv::Size& size) {
    cv::Mat bgr;
    const fs::path rgb_path = dir / "rgb.png";
    const fs::path left_path = dir / "left.png";
    if (fs::exists(rgb_path)) {
        bgr = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
    } else if (fs::exists(left_path)) {
        const cv::Mat gray = cv::imread(left_path.string(), cv::IMREAD_GRAYSCALE);
        if (!gray.empty()) {
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        }
    }
    if (bgr.empty()) {
        bgr = cv::Mat(size, CV_8UC3, cv::Scalar(200, 200, 200));
        return bgr;
    }
    if (bgr.size() != size) {
        cv::resize(bgr, bgr, size, 0, 0, cv::INTER_LINEAR);
    }
    return bgr;
}

std::size_t write_cloud_ply(const fs::path& path, const cv::Mat& depth_m, const cv::Mat& bgr,
                            const DepthCalib& calib, double max_z, int stride) {
    std::vector<PlyVertex> points;
    points.reserve(static_cast<std::size_t>(depth_m.rows * depth_m.cols) / static_cast<std::size_t>(stride * stride));

    for (int y = 0; y < depth_m.rows; y += stride) {
        const float* zrow = depth_m.ptr<float>(y);
        const cv::Vec3b* crow = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < depth_m.cols; x += stride) {
            const float z = zrow[x];
            if (!(z > 0.0f) || z > static_cast<float>(max_z)) {
                continue;
            }
            PlyVertex v{};
            v.x = static_cast<float>((static_cast<double>(x) - calib.cx) * z / calib.fx);
            v.y = static_cast<float>((static_cast<double>(y) - calib.cy) * z / calib.fy);
            v.z = z;
            v.b = crow[x][0];
            v.g = crow[x][1];
            v.r = crow[x][2];
            points.push_back(v);
        }
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Cannot write " + path.string());
    }
    out << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "element vertex " << points.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "end_header\n";
    out.write(reinterpret_cast<const char*>(points.data()),
              static_cast<std::streamsize>(points.size() * sizeof(PlyVertex)));
    if (!out) {
        throw std::runtime_error("Failed while writing " + path.string());
    }
    return points.size();
}

bool process_dir(const fs::path& dir, const DepthCalib& calib, const Options& opt) {
    const fs::path raw_path = dir / "disparity_raw.png";
    cv::Mat raw = cv::imread(raw_path.string(), cv::IMREAD_UNCHANGED);
    if (raw.empty()) {
        throw std::runtime_error("Failed to read " + raw_path.string());
    }
    if (raw.channels() != 1) {
        throw std::runtime_error("disparity_raw.png must be single-channel: " + raw_path.string());
    }

    cv::Mat disp_px;
    raw.convertTo(disp_px, CV_32F, 1.0 / calib.sgbm_scale);

    cv::Mat depth_m;
    cv::divide(calib.fx_times_baseline_m, disp_px, depth_m);
    depth_m.setTo(0, disp_px <= 0);
    cv::patchNaNs(depth_m, 0.0);

    cv::Mat depth_mm;
    depth_m.convertTo(depth_mm, CV_16U, 1000.0);
    depth_mm.setTo(0, depth_m <= 0);

    const cv::Mat depth_color = colorize_depth(depth_m, opt.max_z);
    cv::imwrite((dir / "depth_m.tiff").string(), depth_m);
    cv::imwrite((dir / "depth_mm.png").string(), depth_mm);
    cv::imwrite((dir / "depth.png").string(), depth_color);

    if (!opt.no_cloud) {
        const cv::Mat bgr = load_cloud_color(dir, depth_m.size());
        const std::size_t n =
            write_cloud_ply(dir / "cloud.ply", depth_m, bgr, calib, opt.max_z, opt.stride);
        std::cout << "  cloud.ply  " << n << " points\n";
    }

    if (opt.show) {
        cv::imshow("Depth", depth_color);
        cv::waitKey(1);
    }
    return true;
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

    try {
        const DepthCalib calib = load_calib(opt.calib);
        const std::vector<fs::path> dirs = collect_dirs(opt.input);
        if (dirs.empty()) {
            std::cerr << "No folders with disparity_raw.png under " << opt.input.string() << "\n"
                      << "Run compute_disparity first.\n";
            return 1;
        }

        std::cout << "Z = " << calib.fx_times_baseline_m << " / (raw / " << calib.sgbm_scale
                  << ") meters\n"
                  << "calib: " << opt.calib.string() << "\n";

        int ok = 0;
        for (size_t i = 0; i < dirs.size(); ++i) {
            std::cout << "[" << (i + 1) << "/" << dirs.size() << "] " << dirs[i].string() << "\n";
            if (process_dir(dirs[i], calib, opt)) {
                ++ok;
            }
        }

        if (opt.show) {
            std::cout << "Press any key in an OpenCV window to close\n";
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
        std::cout << "Wrote depth for " << ok << " pair(s)\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
