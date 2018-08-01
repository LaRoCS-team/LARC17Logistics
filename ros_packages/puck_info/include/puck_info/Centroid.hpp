//
// Created by previato on 01/08/18.
//

#ifndef PUCK_INFO_CENTROID_HPP
#define PUCK_INFO_CENTROID_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using cv::Point;

#include "Color.hpp"

struct Centroid {
    Color_e color;
    Point point;
    int index;

    Centroid(Color_e color, const Point &point, int index) : color(color),
                                                      point(point),
                                                      index(index) {}
    Centroid() : color(Color_e::none), point(Point(-1, -1)), index(0) {}
};

#endif //PUCK_INFO_CENTROID_HPP
