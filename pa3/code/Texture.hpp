//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv2/opencv.hpp>
class Texture {
  private:
    cv::Mat image_data;

  public:
    Texture(const std::string &name) {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        float x = u * width;
        float y = (1 - v) * height;

        int u1 = static_cast<int>(std::floor(x));
        int v1 = static_cast<int>(std::floor(y));
        int u2 = u1 + 1;
        int v2 = v1 + 1;

        u1 = std::clamp(u1, 0, width - 1);
        u2 = std::clamp(u2, 0, width - 1);
        v1 = std::clamp(v1, 0, height - 1);
        v2 = std::clamp(v2, 0, height - 1);

        float s = x - std::floor(x);
        float t = y - std::floor(y);

        auto color11 = image_data.at<cv::Vec3b>(v1, u1);
        auto color12 = image_data.at<cv::Vec3b>(v1, u2);
        auto color21 = image_data.at<cv::Vec3b>(v2, u1);
        auto color22 = image_data.at<cv::Vec3b>(v2, u2);

        auto color1 = (1 - s) * color11 + s * color12;
        auto color2 = (1 - s) * color21 + s * color22;
        auto color = (1 - t) * color1 + t * color2;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
