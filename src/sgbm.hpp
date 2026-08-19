#pragma once

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

inline int parse_int_arg(const char* text, const char* flag) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        throw std::runtime_error(std::string("Invalid integer for ") + flag + ": " + text);
    }
    return static_cast<int>(value);
}

inline void normalize_sgbm_params(int& num_disp, int& block_size) {
    if (num_disp < 16) {
        throw std::runtime_error("--num-disp must be >= 16");
    }
    if (num_disp % 16 != 0) {
        num_disp = ((num_disp + 15) / 16) * 16;
        std::cout << "Rounded --num-disp up to " << num_disp << "\n";
    }
    if (block_size < 3) {
        block_size = 3;
    }
    if (block_size % 2 == 0) {
        ++block_size;
    }
}

inline cv::Mat to_gray(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }
    if (image.channels() == 1) {
        return image;
    }
    cv::Mat gray;
    if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

inline cv::Mat compute_disparity(const cv::Mat& left, const cv::Mat& right, int num_disp,
                                int block_size) {
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, num_disp, block_size);
    const int cn = 1;
    sgbm->setP1(8 * cn * block_size * block_size);
    sgbm->setP2(32 * cn * block_size * block_size);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(num_disp);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setPreFilterCap(63);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    cv::Mat disp16;
    sgbm->compute(left, right, disp16);
    return disp16;
}

inline cv::Mat colorize_disparity(const cv::Mat& disp16) {
    cv::Mat disp32;
    disp16.convertTo(disp32, CV_32F, 1.0 / 16.0);

    cv::Mat valid = disp32 > 0.0f;
    double min_v = 0.0;
    double max_v = 0.0;
    cv::minMaxLoc(disp32, &min_v, &max_v, nullptr, nullptr, valid);
    if (!(max_v > min_v)) {
        return cv::Mat::zeros(disp16.size(), CV_8UC3);
    }

    cv::Mat norm8;
    disp32.convertTo(norm8, CV_8U, 255.0 / (max_v - min_v), -min_v * 255.0 / (max_v - min_v));
    norm8.setTo(0, ~valid);

    cv::Mat color;
    cv::applyColorMap(norm8, color, cv::COLORMAP_JET);
    color.setTo(cv::Scalar(0, 0, 0), ~valid);
    return color;
}

inline cv::Mat disparity_histogram(const cv::Mat& disp16, int num_disp) {
    cv::Mat disp32;
    disp16.convertTo(disp32, CV_32F, 1.0 / 16.0);
    cv::Mat valid_mask = disp32 > 0.0f;

    const int bins = std::max(16, num_disp);
    const float range[] = {0.f, static_cast<float>(num_disp)};
    const float* ranges[] = {range};
    cv::Mat hist;
    cv::calcHist(&disp32, 1, nullptr, valid_mask, hist, 1, &bins, ranges, true, false);

    double max_count = 1.0;
    cv::minMaxLoc(hist, nullptr, &max_count);
    max_count = std::max(max_count, 1.0);

    const int width = 640;
    const int height = 240;
    const int margin = 40;
    cv::Mat graph(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
    const int plot_w = width - 2 * margin;
    const int plot_h = height - 2 * margin;
    const float bin_w = static_cast<float>(plot_w) / static_cast<float>(bins);

    cv::rectangle(graph, cv::Point(margin, margin),
                  cv::Point(width - margin, height - margin), cv::Scalar(60, 60, 60));

    for (int i = 0; i < bins; ++i) {
        const float count = hist.at<float>(i);
        const int h = static_cast<int>((count / max_count) * plot_h);
        const int x0 = margin + static_cast<int>(i * bin_w);
        const int x1 = margin + static_cast<int>((i + 1) * bin_w) - 1;
        cv::rectangle(graph, cv::Point(x0, height - margin - h),
                      cv::Point(std::max(x0, x1), height - margin), cv::Scalar(0, 200, 255),
                      cv::FILLED);
    }

    cv::putText(graph, "Disparity histogram (px)", cv::Point(margin, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(graph, "0", cv::Point(margin, height - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    cv::putText(graph, std::to_string(num_disp), cv::Point(width - margin - 28, height - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    return graph;
}

inline void write_disparity(const std::filesystem::path& pair_dir, const cv::Mat& disp16,
                            const cv::Mat& disp_color, const cv::Mat& disp_hist) {
    cv::Mat disp16u;
    disp16.convertTo(disp16u, CV_16U);
    disp16u.setTo(0, disp16 <= 0);
    cv::imwrite((pair_dir / "disparity_raw.png").string(), disp16u);
    cv::imwrite((pair_dir / "disparity.png").string(), disp_color);
    cv::imwrite((pair_dir / "disparity_hist.png").string(), disp_hist);
}

inline bool process_pair_dir(const std::filesystem::path& pair_dir, int num_disp, int block_size,
                             bool show) {
    const std::filesystem::path left_path = pair_dir / "left.png";
    const std::filesystem::path right_path = pair_dir / "right.png";
    if (!std::filesystem::exists(left_path) || !std::filesystem::exists(right_path)) {
        return false;
    }

    const cv::Mat left = to_gray(cv::imread(left_path.string(), cv::IMREAD_UNCHANGED));
    const cv::Mat right = to_gray(cv::imread(right_path.string(), cv::IMREAD_UNCHANGED));
    if (left.empty() || right.empty()) {
        throw std::runtime_error("Failed to read " + pair_dir.string());
    }
    if (left.size() != right.size()) {
        throw std::runtime_error("left/right size mismatch in " + pair_dir.string());
    }

    const cv::Mat disp16 = compute_disparity(left, right, num_disp, block_size);
    const cv::Mat disp_color = colorize_disparity(disp16);
    const cv::Mat disp_hist = disparity_histogram(disp16, num_disp);
    write_disparity(pair_dir, disp16, disp_color, disp_hist);

    if (show) {
        cv::imshow("Left", left);
        cv::imshow("Right", right);
        cv::imshow("Disparity", disp_color);
        cv::imshow("Disparity histogram", disp_hist);
        cv::waitKey(1);
    }
    return true;
}
