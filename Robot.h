#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "CameraVirtual.h"
#include "CameraReal.h"
#include "constants.h"
#include "Test.h"

// (Optional) Avoid 'using namespace' in headers to prevent namespace pollution.
// using namespace std;
// using namespace cv;
// using namespace dnn;

class CRobot
{
public:
    CRobot();
    ~CRobot();

private:
    cv::Size _image_size;
    cv::Point _image_center;
    cv::Mat _canvas;

    // robot<cubes<8 vertexes/points>>
    std::vector<std::vector<cv::Mat>> _simple_robot;

    CCameraVirtual _virtualcam;
    CCameraReal    _realcam;

    // Builders / helpers
    std::vector<cv::Mat> createBox(float w, float h, float d);
    std::vector<cv::Mat> createCoord();

    void init();
    void transformPoints(std::vector<cv::Mat>& points, cv::Mat T);

    void drawBox(cv::Mat& im, std::vector<cv::Mat> box3d, cv::Scalar colour);
    void drawCoord(cv::Mat& im, std::vector<cv::Mat> coord3d);
    void update_settings(cv::Mat& im);
    cv::Scalar chooseColors(int idx);

    ////////////////////////////////////
    // LAB 4 (helpers; can be private)
    void drawBoxReal(cv::Mat& im,
        const std::vector<cv::Mat>& box3d,
        CCameraReal& cam,
        const cv::Scalar& color);

public:
    // Exposed so lab4() can call it
    void draw_simple_robot_on_real(cv::Mat& frame, CCameraReal& cam);

    ////////////////////////////////////
    // LAB 5

    int _do_animate; // Animation state machine

public:
    cv::Mat createHT(cv::Vec3d t, cv::Vec3d r);

    // Lab 3-style API
    void create_simple_robot();
    void draw_simple_robot();

    ////////////////////////////////////
    // Lab 4

    void draw();

    // Overload: size robot based on checkerboard square length (meters)
    void create_simple_robot(float squareLenMeters);

    ////////////////////////////////////
    // Lab 5 (Forward Kinematics) – add impl in .cpp when ready
    // cv::Mat fkine() const;

    ////////////////////////////////////
    // Lab 6 (Inverse Kinematics) – add impl in .cpp when ready
    // bool ikine(const cv::Mat& T_target);
};
