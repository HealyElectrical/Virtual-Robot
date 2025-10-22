#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "CameraVirtual.h"
#include "CameraReal.h"
#include "constants.h"
#include "Test.h"

class CRobot
{
public:
    CRobot();
    ~CRobot();

private:
    cv::Size  _image_size;
    cv::Point _image_center;
    cv::Mat   _canvas;

    // robot<cubes<8 vertexes/points>>
    std::vector<std::vector<cv::Mat>> _simple_robot;
    std::vector<std::vector<cv::Mat>> _scara_links;


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

public:
    // Draw the simple robot onto the real frame with a Z-rotation (degrees)
    void draw_rotating_robot_on_real(cv::Mat& frame, CCameraReal& cam, double angle_deg);

    ////////////////////////////////////
    // LAB 5

    int _do_animate; // Animation state machine

public:
    // NOTE: const so it can be used inside const helper functions
    cv::Mat createHT(cv::Vec3d t, cv::Vec3d r) const;

    // Lab 3-style API
    void create_simple_robot();
    void draw_simple_robot();
    


    ////////////////////////////////////
    // Lab 4
    void draw();

    // Overload: size robot based on checkerboard square length (meters)
    void create_simple_robot(float squareLenMeters);

    ////////////////////////////////////
    // Lab 5 (Forward Kinematics) – public API used from template.cpp
    cv::Mat fkine(double q1_deg, double q2_deg, double d3_m, double q4_deg) const;
    void    create_scara_templates();
    void draw_scara(double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg);
    void update_joint_controls(double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg);


    // Small helper so UI can draw onto our canvas without exposing the member
    cv::Mat& canvas();
private:
    // ---- Lab 5 helpers & geometry (used by fkine and drawing) ----
    // Tiny helpers to build transforms
    cv::Mat Rz_deg(double deg) const;
    cv::Mat Tx(double x) const;
    cv::Mat Tz(double z) const;

    // SCARA link lengths / offsets (meters) — per spec
    double L1_ = 0.15;      // link 1 length (15 cm)
    double L2_ = 0.15;      // link 2 length (15 cm)
    double pedestalZ_ = 0.135; // 13.5 cm pedestal so prismatic 0 is at 0.15 m world Z

    // Link box dimensions for drawing (meters) – 15 x 3 x 3 cm
    double Bx_ = 0.15, By_ = 0.03, Bz_ = 0.03;

    // Reusable templates for drawing links/end-effector (in local frames)
    std::vector<cv::Mat> linkTemplate_; // box pointing +X, joint at -X face
    std::vector<cv::Mat> effTemplate_;  // small tool box

    // Draw a link/effector given a template and world transform
    void draw_link(cv::Mat& im, const std::vector<cv::Mat>& templ,
        const cv::Mat& T_world, const cv::Scalar& color);

    ////////////////////////////////////
    // Lab 6 (Inverse Kinematics) – add impl in .cpp when ready
    // bool ikine(const cv::Mat& T_target);
};
