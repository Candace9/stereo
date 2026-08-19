// Compute SGBM disparity for left.png / right.png under a given path.

#include "sgbm.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Options {
  fs::path input;
  int num_disp{128};
  int block_size{5};
  bool show{false};
};

void print_usage(const char* argv0) {
  std::cout
      << "Compute SGBM disparity for captured stereo pairs\n\n"
      << "Usage: " << argv0 << " [options] PATH\n\n"
      << "PATH is either:\n"
      << "  a pair folder containing left.png and right.png, or\n"
      << "  a parent folder (each subfolder with left.png/right.png is processed)\n\n"
      << "  --num-disp N      SGBM max disparity, multiple of 16 (default: 128)\n"
      << "  --block-size N    SGBM block size, odd number >= 3 (default: 5)\n"
      << "  --show            Show OpenCV preview windows\n"
      << "  -h, --help        Show this help\n";
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
    } else if (arg == "--num-disp") {
      opt.num_disp = parse_int_arg(need_value("--num-disp"), "--num-disp");
    } else if (arg == "--block-size") {
      opt.block_size = parse_int_arg(need_value("--block-size"), "--block-size");
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
    throw std::runtime_error("Missing PATH (folder of captures or one pair folder)");
  }
  normalize_sgbm_params(opt.num_disp, opt.block_size);
  return opt;
}

bool is_pair_dir(const fs::path& dir) {
  return fs::exists(dir / "left.png") && fs::exists(dir / "right.png");
}

std::vector<fs::path> collect_pair_dirs(const fs::path& input) {
  if (!fs::exists(input)) {
    throw std::runtime_error("Path does not exist: " + input.string());
  }
  if (!fs::is_directory(input)) {
    throw std::runtime_error("PATH must be a directory: " + input.string());
  }

  std::vector<fs::path> dirs;
  if (is_pair_dir(input)) {
    dirs.push_back(input);
    return dirs;
  }

  for (const fs::directory_entry& entry : fs::directory_iterator(input)) {
    if (entry.is_directory() && is_pair_dir(entry.path())) {
      dirs.push_back(entry.path());
    }
  }
  std::sort(dirs.begin(), dirs.end());
  return dirs;
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
    const std::vector<fs::path> dirs = collect_pair_dirs(opt.input);
    if (dirs.empty()) {
      std::cerr << "No folders with left.png and right.png under " << opt.input.string()
                << "\n";
      return 1;
    }

    std::cout << "SGBM numDisp=" << opt.num_disp << " blockSize=" << opt.block_size << "\n";
    int ok = 0;
    for (size_t i = 0; i < dirs.size(); ++i) {
      std::cout << "[" << (i + 1) << "/" << dirs.size() << "] " << dirs[i].string() << "\n";
      if (process_pair_dir(dirs[i], opt.num_disp, opt.block_size, opt.show)) {
        ++ok;
      }
    }

    if (opt.show) {
      std::cout << "Press any key in an OpenCV window to close\n";
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
    std::cout << "Wrote disparity for " << ok << " pair(s)\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
