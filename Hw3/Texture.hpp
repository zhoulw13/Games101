//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getBilinearColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u_left = int(u_img);
        int v_top = int(v_img);
        float alpha = u_img - u_left;
        float beta = v_img - v_top;

        auto color1 = image_data.at<cv::Vec3b>(v_top, u_left);
        auto color2 = image_data.at<cv::Vec3b>(v_top+1, u_left);
        auto color3 = image_data.at<cv::Vec3b>(v_top, u_left+1);
        auto color4 = image_data.at<cv::Vec3b>(v_top+1, u_left+1);

        auto final_color = (color1 * (1-beta)+ color2 * beta) * (1-alpha) + (color3 * (1-beta)+ color4 * beta) * alpha;
        return Eigen::Vector3f(final_color[0], final_color[1], final_color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
