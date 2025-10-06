//constants.h
#pragma once
#include <opencv2/opencv.hpp>

const cv::Size IMAGE_SIZE = cv::Size(1280, 720); // or #define IMAGE_SIZE cv::Size(1000, 600); // seems to compile better than a cont cv::Size
const cv::Point IMAGE_CENTER = cv::Point(IMAGE_SIZE.width / 2, IMAGE_SIZE.height / 2);        
const float BOX_SIZE = 0.05f; // in metres

const cv::Scalar BACKGROUND_COLOR = cv::Scalar(60, 60, 60);
const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
const cv::Scalar TEXT_COLOR = WHITE;
const cv::Scalar RED = CV_RGB(255, 0, 0);
const cv::Scalar GREEN = CV_RGB(0, 255, 0);
const cv::Scalar BLUE = CV_RGB(0, 0, 255);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar CYAN = cv::Scalar(255, 0, 255);
const cv::Scalar MAGENTA = cv::Scalar(255, 255, 0);
const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
const cv::Scalar GREY = cv::Scalar(128, 128, 128);
const cv::Scalar ORANGE = cv::Scalar(0, 165, 255);
const cv::Scalar PURPLE = cv::Scalar(128, 0, 128);
const cv::Scalar PINK = cv::Scalar(203, 192, 255);
const cv::Scalar BROWN = cv::Scalar(42, 42, 165);
const cv::Scalar RUST = cv::Scalar(20, 69, 139);
const cv::Scalar BEIGE = cv::Scalar(220, 245, 245);
