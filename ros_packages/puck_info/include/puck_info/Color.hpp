//
// Created by previato on 01/08/18.
//

#ifndef PUCK_INFO_COLOR_HPP
#define PUCK_INFO_COLOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using cv::Scalar;

enum class Color_e {
    yellow = 0,
    green = 2,
    red = 4,
    none = -1
};

std::string to_string(Color_e color) {
    switch (color){
        case Color_e::yellow:
            return std::string("yellow");
        case Color_e::green:
            return std::string("green");
        case Color_e::red:
            return std::string("red");
        case Color_e::none:
            return std::string("none");
    }
}

struct Color {
    Scalar min;
    Scalar max;
    Color_e color;

    Color(int a, int b, int c, int d, int e, int f, Color_e color) : color(color) {
        min = {a, b, c};
        max = {d, e, f};
    }
};

#endif //PUCK_INFO_COLOR_HPP
