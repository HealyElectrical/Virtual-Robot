#include "stdafx.h"
#include "Robot.h"
#include <cstdio>


#include <cmath>
#include <algorithm>   // for std::max
#include "cvui.h"


// Keep using cv:: explicitly in .cpp to avoid polluting headers.

CRobot::CRobot()
{
    _image_size = IMAGE_SIZE;
    _image_center = IMAGE_CENTER;

    _canvas = cv::Mat::zeros(_image_size, CV_8UC3);
    cv::namedWindow(CANVAS_NAME);
    cvui::init(CANVAS_NAME);

    _simple_robot.clear();
    init();

    _lab3 = true;

    // NEW:
    solve_mode_ = SolveMode::FK;
    elbow_up_toggle_ = true; // start with elbow-up
    // in CRobot::CRobot()
    track_cube_ = false;       // default: off

}


CRobot::~CRobot() {}

void CRobot::init()
{
    // reset variables
    _do_animate = 0;
}

cv::Mat CRobot::createHT(cv::Vec3d t, cv::Vec3d r) const
{
    // roll-pitch-yaw (X-Y-Z) in degrees
    const double deg2rad = 3.14159265358979323846 / 180.0;
    const double a = r[2] * deg2rad; // yaw   (Z)
    const double b = r[1] * deg2rad; // pitch (Y)
    const double g = r[0] * deg2rad; // roll  (X)

    const double sa = sin(a), ca = cos(a);
    const double sb = sin(b), cb = cos(b);
    const double sg = sin(g), cg = cos(g);

    // R = Rz(a) * Ry(b) * Rx(g)
    double R00 = ca * cb;
    double R01 = ca * sb * sg - sa * cg;
    double R02 = ca * sb * cg + sa * sg;
    double R10 = sa * cb;
    double R11 = sa * sb * sg + ca * cg;
    double R12 = sa * sb * cg - ca * sg;
    double R20 = -sb;
    double R21 = cb * sg;
    double R22 = cb * cg;

    cv::Mat T = (cv::Mat1f(4, 4) <<
        (float)R00, (float)R01, (float)R02, (float)t[0],
        (float)R10, (float)R11, (float)R12, (float)t[1],
        (float)R20, (float)R21, (float)R22, (float)t[2],
        0.f, 0.f, 0.f, 1.f);
    return T;
}

std::vector<cv::Mat> CRobot::createBox(float w, float h, float d)
{
    std::vector<cv::Mat> box;

    // The 8 vertices, origin at the center of the box
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
    box.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));
    return box;
}

std::vector<cv::Mat> CRobot::createCoord()
{
    std::vector<cv::Mat> coord;
    float axis_length = 0.03f; // meters

    coord.push_back((cv::Mat1f(4, 1) << 0, 0, 0, 1));               // O
    coord.push_back((cv::Mat1f(4, 1) << axis_length, 0, 0, 1));     // X
    coord.push_back((cv::Mat1f(4, 1) << 0, axis_length, 0, 1));     // Y
    coord.push_back((cv::Mat1f(4, 1) << 0, 0, axis_length, 1));     // Z
    return coord;
}

void CRobot::transformPoints(std::vector<cv::Mat>& points, cv::Mat T)
{
    for (size_t i = 0; i < points.size(); ++i)
        points[i] = T * points[i];
}

void CRobot::drawBox(cv::Mat& im, std::vector<cv::Mat> box3d, cv::Scalar colour)
{
    std::vector<cv::Point2f> box2d;

    // The 12 lines connecting all vertices
    int draw_box1[12] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
    int draw_box2[12] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

    // Virtual Camera
    _virtualcam.transform_to_image(box3d, box2d);

    for (int i = 0; i < 12; ++i)
    {
        cv::Point pt1 = box2d.at(draw_box1[i]);
        cv::Point pt2 = box2d.at(draw_box2[i]);
        cv::line(im, pt1, pt2, colour, 1); // use 3 instead of 1

    }
}

void CRobot::drawCoord(cv::Mat& im, std::vector<cv::Mat> coord3d)
{
    cv::Point2f O, X, Y, Z;

    _virtualcam.transform_to_image(coord3d.at(0), O);
    if (_lab3)
    {
        cv::Mat flippedX = coord3d.at(1).clone();
        flippedX.at<float>(0, 0) *= -1;  // flip X direction
        _virtualcam.transform_to_image(flippedX, X);
    }
    else
        _virtualcam.transform_to_image(coord3d.at(1), X);
    _virtualcam.transform_to_image(coord3d.at(2), Y);
    _virtualcam.transform_to_image(coord3d.at(3), Z);

    cv::line(im, O, X, RED, 2);
    cv::line(im, O, Y, GREEN, 2);
    cv::line(im, O, Z, BLUE, 2);

    cv::putText(im, "X", X + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, RED, 1);
    cv::putText(im, "Y", Y + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1);
    cv::putText(im, "Z", Z + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 1);
}

void CRobot::create_simple_robot()
{
    _lab3 = true;

    _simple_robot.clear();
    float box_size = BOX_SIZE;

    std::vector<cv::Vec3d> positions = {
        { 0, 0, 0 },
        { 0, 0, 1 * box_size },
        { 0, 0, 2 * box_size },
        { 0, 0, 3 * box_size },
        { +1 * box_size, 0, 2 * box_size },
        { -1 * box_size, 0, 2 * box_size }
    };

    for (auto& pos : positions)
    {
        std::vector<cv::Mat> box = createBox(box_size, box_size, box_size);
        cv::Mat T = createHT(pos, cv::Vec3d(0, 0, 0));
        transformPoints(box, T);
        _simple_robot.push_back(box);
    }
}

void CRobot::create_simple_robot(float squareLenMeters)
{
    _simple_robot.clear();
    const float s = squareLenMeters;

    std::vector<cv::Vec3d> centers = {
        { 0.0, 0.0, 0.5 * s },
        { 0.0, 0.0, 1.5 * s },
        { 0.0, 0.0, 2.5 * s },
        { 0.0, 0.0, 3.5 * s },
        { +1.0 * s, 0.0, 2.5 * s },
        { -1.0 * s, 0.0, 2.5 * s }
    };

    for (const auto& c : centers)
    {
        std::vector<cv::Mat> box = createBox(s, s, s);
        cv::Mat T = createHT(c, cv::Vec3d(0, 0, 0));
        transformPoints(box, T);
        _simple_robot.push_back(box);
    }
}

void CRobot::draw_simple_robot()
{
    _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    _virtualcam.update_settings(_canvas);
    update_settings(_canvas);

    // draw the robot
    for (size_t i = 0; i < _simple_robot.size(); ++i)
        drawBox(_canvas, _simple_robot[i], chooseColors((int)i));

    // world axes
    auto O = createCoord();
    drawCoord(_canvas, O);

    cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////////
// LAB4
////////////////////////////////////////

// 12 cube edges
static const int E1[12] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
static const int E2[12] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

void CRobot::drawBoxReal(cv::Mat& im, const std::vector<cv::Mat>& box3d, CCameraReal& cam, const cv::Scalar& color)
{
    if (!cam.have_pose) return;

    std::vector<cv::Point2f> pts2d(8);

    for (int i = 0; i < 8; ++i) {
        float x = box3d[i].at<float>(0, 0);
        float y = box3d[i].at<float>(1, 0);
        float z = box3d[i].at<float>(2, 0);

        // flip Z only
        cv::Mat p3 = (cv::Mat_<float>(3, 1) << x, y, -z);
        cam.transform_to_image(p3, pts2d[i]);
    }

    for (int k = 0; k < 12; ++k)
        cv::line(im, pts2d[E1[k]], pts2d[E2[k]], color, 2, cv::LINE_AA);
}

void CRobot::draw_simple_robot_on_real(cv::Mat& frame, CCameraReal& cam)
{
    if (!cam.have_pose) return;

    for (size_t i = 0; i < _simple_robot.size(); ++i)
        drawBoxReal(frame, _simple_robot[i], cam, chooseColors((int)i));
}

void CRobot::draw_rotating_robot_on_real(cv::Mat& frame, CCameraReal& cam, double angle_deg)
{
    if (!cam.have_pose) return;

    cv::Mat Rz = createHT(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, angle_deg));

    for (size_t i = 0; i < _simple_robot.size(); ++i) {
        std::vector<cv::Mat> tmp = _simple_robot[i];
        transformPoints(tmp, Rz);

        std::vector<cv::Point2f> pts2d(8);
        for (int v = 0; v < 8; ++v) {
            cv::Mat p3 = (cv::Mat_<float>(3, 1)
                << tmp[v].at<float>(0, 0),
                tmp[v].at<float>(1, 0),
                -tmp[v].at<float>(2, 0));
            cam.transform_to_image(p3, pts2d[v]);
        }

        const cv::Scalar color = chooseColors((int)i);
        for (int e = 0; e < 12; ++e)
            cv::line(frame, pts2d[E1[e]], pts2d[E2[e]], color, 2, cv::LINE_AA);
    }
}

////////////////////////////////////////
// LAB5
////////////////////////////////////////

void CRobot::update_settings(cv::Mat& im)
{
    cv::Point _setting_window;

    _setting_window.x = im.size().width - 200;
    cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

    _setting_window.x += 5;
    _setting_window.y += 20;

    if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
    {
        init();
        _do_animate = 1;
    }

    if (_do_animate != 0)
    {
        if (_do_animate == 1) { _do_animate = 2; }
        else if (_do_animate == 2) { _do_animate = 3; }
        else if (_do_animate == 3) { _do_animate = 0; init(); }
    }

    cvui::update();
}

void CRobot::draw()
{
    _virtualcam.update_settings(_canvas);
    update_settings(_canvas);
    cv::imshow(CANVAS_NAME, _canvas);
}


Scalar CRobot::chooseColors(int idx, Scalar c0, Scalar c1, Scalar c2, Scalar c3, Scalar c4, Scalar c5)
{
    Scalar colors[6] = { c0, c1, c2, c3, c4, c5 };
    return colors[idx % 6]; // wrap around if idx >= 6
}

// ===== LAB 5: Helper Transforms =====
cv::Mat CRobot::Rz_deg(double deg) const {
    return createHT(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, deg));
}
cv::Mat CRobot::Tx(double x) const {
    return createHT(cv::Vec3d(x, 0, 0), cv::Vec3d(0, 0, 0));
}
cv::Mat CRobot::Tz(double z) const {
    return createHT(cv::Vec3d(0, 0, z), cv::Vec3d(0, 0, 0));
}

// LAB 5: Forward Kinematics
cv::Mat CRobot::fkine(double q1_deg, double q2_deg, double d3_m, double q4_deg) const
{
    cv::Mat T0 = Tz(pedestalZ_);
    cv::Mat T01 = Rz_deg(q1_deg) * Tx(L1_);
    cv::Mat T12 = Rz_deg(q2_deg) * Tx(L2_);
    cv::Mat T23 = Tz(d3_m);
    cv::Mat T34 = Rz_deg(q4_deg);

    return T0 * T01 * T12 * T23 * T34;
}

void CRobot::create_scara_templates()
{
    if (!linkTemplate_.empty() && !effTemplate_.empty()) return;

    // For debugging: make the link big and easy to see
    float bx = 0.30f;  // 30 cm long
    float by = 0.10f;  // 10 cm thick
    float bz = 0.10f;  // 10 cm tall

    linkTemplate_ = createBox(bx, by, bz);

    // Move so the joint is at the left end of the box
    transformPoints(linkTemplate_, Tx(bx / 2.0));

    // Small tool box (you won?t see it yet)
    effTemplate_ = createBox(0.04f, 0.02f, 0.02f);
    transformPoints(effTemplate_, Tx(0.02));
}


void CRobot::draw_link(cv::Mat& im, const std::vector<cv::Mat>& templ,
    const cv::Mat& T_world, const cv::Scalar& color)
{
    std::vector<cv::Mat> verts = templ;
    transformPoints(verts, T_world);
    drawBox(im, verts, color);
}


/*
void CRobot::draw_scara(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // === Reset canvas and update virtual camera ===
    _canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR;
    _virtualcam.update_settings(_canvas);

    std::vector<std::vector<cv::Mat>> blocks;

    // --- Dimensions (meters) ---
    const float base_height = 0.15f;
    const float arm_len1 = 0.15f;
    const float arm_len2 = 0.15f;
    const float thickness = 0.03f;
    const float prism_height = 0.15f;
    const float offset_z = 0.135f;   // 13.5 cm upward offset for prismatic

    // --- Helper to draw small axes (2 cm) ---
    auto makeSmallAxis = [&](const cv::Mat& T_world)
        {
            auto A = createCoord();
            for (auto& pt : A)
            {
                pt.at<float>(0, 0) *= 0.4f;
                pt.at<float>(1, 0) *= 0.4f;
                pt.at<float>(2, 0) *= 0.4f;
            }
            transformPoints(A, T_world);
            drawCoord(_canvas, A);
        };

    // ===== Forward-kinematic chain =====
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);

    // Base rotation at origin, then lift the pedestal
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    // Shoulder rotation (joint 2)
    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    // Elbow rotation (joint 3)
    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;

    // Prismatic translation (joint 4)
    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // ===== Block placements =====
    // 1?? Base pedestal (red)
    auto block1 = createBox(thickness, thickness, base_height);
    transformPoints(block1, T1 * Tz(-base_height / 2.0));
    blocks.push_back(block1);

    // 2?? First arm (green)
    auto block2 = createBox(arm_len1, thickness, thickness);
    transformPoints(block2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block2);

    // 3?? Second arm (blue)
    auto block3 = createBox(arm_len2, thickness, thickness);
    transformPoints(block3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block3);

    // 4?? Prismatic link (yellow)
    auto block4 = createBox(thickness, thickness, prism_height);
    transformPoints(block4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block4);

    // ===== Draw all links =====
    for (size_t i = 0; i < blocks.size(); ++i)
        drawBox(_canvas, blocks[i], chooseColors((int)i));

    // ===== Axes at each joint =====
    makeSmallAxis(createHT(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, q1_deg))); // base
    makeSmallAxis(T1);  // shoulder joint
    makeSmallAxis(T2);  // elbow joint
    makeSmallAxis(T3);  // top of prismatic
    makeSmallAxis(T4);  // bottom of prismatic

    // ===== GUI =====
    _virtualcam.update_settings(_canvas);
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);

    // ===== Display =====
    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}
*/

void CRobot::draw_scara(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    std::vector<std::vector<cv::Mat>> blocks;

    // Keep existing background
    if (_canvas.empty())
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    _lab3 = false;

    // Camera + UI (we'll update pose readouts just before the UI so sliders mirror it)
    _virtualcam.update_settings(_canvas);

    // --- Dimensions (meters) ---
    const float thickness = 0.03f;
    const float LINK_LENGTH = 0.15f;
    const float base_height = LINK_LENGTH - thickness / 2;
    const float arm_len1 = LINK_LENGTH;
    const float arm_len2 = LINK_LENGTH;
    const float prism_height = LINK_LENGTH;
    const float offset_z = 0.135f;

    // ===== Forward kinematics (virtual) =====
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;

    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // ---- Update the EE readouts using the same helper as AR ----
    update_ee_readout_from_T(T4, d3_m);

    // ---- UI (sliders mirror the just-updated pose values) ----
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);

    // ---- Advance animation in VIRTUAL mode as well ----
    if (anim_running_) {
        if (!step_anim_partB(q1_deg, q2_deg, q3_deg, d3_m)) {
            anim_running_ = false; // finished
        }
    }

    // ===== Prisms (same placement as AR) =====
    auto prism1 = createPrism(thickness, base_height, thickness);
    blocks.push_back(prism1);

    auto prism2 = createPrism(arm_len1, thickness, thickness);
    transformPoints(prism2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0 - thickness / 2.0));
    blocks.push_back(prism2);

    auto prism3 = createPrism(arm_len2, thickness, thickness);
    transformPoints(prism3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0 - thickness / 2.0));
    blocks.push_back(prism3);

    auto prism4 = createPrism(thickness, prism_height, thickness);
    transformPoints(prism4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0 - 2.0f * thickness));
    blocks.push_back(prism4);

    for (size_t i = 0; i < blocks.size(); ++i)
        drawPrism(_canvas, blocks[i], chooseColors((int)i, RED, YELLOW, GREEN, MAGENTA));

    // ===== Axes =====
    auto W = createCoord();                  // world
    drawCoord(_canvas, W);

    cv::Mat fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 0, 0));
    auto j0 = createCoord(); transformPoints(j0, fixAxes); transformPoints(j0, T1); drawCoord(_canvas, j0);
    auto j1 = createCoord(); transformPoints(j1, fixAxes); transformPoints(j1, T2); drawCoord(_canvas, j1);
    fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 90, 0));
    auto j2 = createCoord(); transformPoints(j2, fixAxes); transformPoints(j2, T3); drawCoord(_canvas, j2);
    auto E = createCoord(); transformPoints(E, fixAxes); transformPoints(E, T4); drawCoord(_canvas, E);

    // ===== Draw the target EE pose in virtual view =====
    draw_target_ee_virtual();

    // ===== Present =====
    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}


std::vector<cv::Mat> CRobot::createPrism(float w, float h, float d)
{
    std::vector<cv::Mat> prism;

    // The 8 vertices, origin at the center of the base (z=0)
    // Bottom face (z = 0)
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -d / 2, 0, 1))); // 0: left-back
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -d / 2, 0, 1))); // 1: right-back
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, d / 2, 0, 1))); // 2: right-front
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, d / 2, 0, 1))); // 3: left-front

    // Top face (z = h)
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -d / 2, h, 1))); // 4: left-back-top
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -d / 2, h, 1))); // 5: right-back-top
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, d / 2, h, 1))); // 6: right-front-top
    prism.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, d / 2, h, 1))); // 7: left-front-top

    return prism;
}

void CRobot::drawPrism(cv::Mat& im, std::vector<cv::Mat> prism3d, cv::Scalar colour)
{
    std::vector<cv::Point2f> prism2d;

    // The 12 lines connecting all vertices (same as box)
    int draw_prism1[12] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
    int draw_prism2[12] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

    // Project 3D points to 2D
    _virtualcam.transform_to_image(prism3d, prism2d);

    for (int i = 0; i < 12; ++i)
    {
        cv::Point pt1 = prism2d.at(draw_prism1[i]);
        cv::Point pt2 = prism2d.at(draw_prism2[i]);
        cv::line(im, pt1, pt2, colour, 1);
    }
}

cv::Mat& CRobot::canvas() { return _canvas; }


/*
void CRobot::update_joint_controls(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- UI panel position ---
    int panel_x = _canvas.cols - 280;
    int panel_y = 100;

    // Background panel
    cv::rectangle(_canvas,
        cv::Point(panel_x - 10, panel_y - 30),
        cv::Point(panel_x + 270, panel_y + 640),
        cv::Scalar(70, 70, 70), cv::FILLED);

    // Window frame
    cvui::window(_canvas, panel_x, panel_y, 260, 610, "SCARA Controls");
    panel_x += 10;
    panel_y += 30;

    // --- Mode toggle (AR vs Virtual) ---
    cvui::text(_canvas, panel_x, panel_y, "View Mode:");
    bool isAR = (view_mode_ == ViewMode::AR);
    if (cvui::button(_canvas, panel_x + 90, panel_y - 6, 70, 24, isAR ? "[AR]" : " AR "))   view_mode_ = ViewMode::AR;
    if (cvui::button(_canvas, panel_x + 165, panel_y - 6, 70, 24, isAR ? "VIRT" : "[VIRT]"))  view_mode_ = ViewMode::Virtual;
    panel_y += 30;

    // --- Solve direction (FK vs IK) ---
    cvui::text(_canvas, panel_x, panel_y, "Solve Direction:");
    bool isFK = (solve_mode_ == SolveMode::FK);
    if (cvui::button(_canvas, panel_x + 140, panel_y - 6, 50, 24, isFK ? "[FK]" : " FK "))     solve_mode_ = SolveMode::FK;
    if (cvui::button(_canvas, panel_x + 195, panel_y - 6, 50, 24, isFK ? " IK " : "[IK]"))     solve_mode_ = SolveMode::IK;
    panel_y += 30;

    // Helper: read-only trackbar
    auto readonly_trackbar = [&](double current, double lo, double hi, const char* label)
        {
            double tmp = current;
            cvui::text(_canvas, panel_x, panel_y - 8, label);
            cvui::trackbar(_canvas, panel_x, panel_y, 240, &tmp, lo, hi);
            panel_y += 50;
        };

    // Helper: live trackbar
    auto live_trackbar = [&](double& ref, double lo, double hi, const char* label)
        {
            cvui::text(_canvas, panel_x, panel_y - 8, label);
            cvui::trackbar(_canvas, panel_x, panel_y, 240, &ref, lo, hi);
            panel_y += 50;
        };

    // --- Joint controls (live in FK, read-only in IK) ---
    if (isFK) {
        live_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)");
        live_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)");
        live_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  (acts as wrist q4)");
        live_trackbar(d3_m, 0.0, 0.15, "d3 (m)");
    }
    else {
        readonly_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)  [read-only in IK]");
        readonly_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)  [read-only in IK]");
        readonly_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  [read-only in IK]");
        readonly_trackbar(d3_m, 0.0, 0.15, "d3 (m)    [read-only in IK]");
    }

    // --- EE pose section ---
    cvui::text(_canvas, panel_x, panel_y, isFK
        ? "End Effector Pose (mirror)"
        : "End Effector Pose (IK target)");
    panel_y += 20;

    if (isFK) {
        // FK  EE is mirror only
        readonly_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        readonly_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        readonly_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        readonly_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");
    }
    else {
        // IK -> EE is user-controlled
        live_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        live_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        live_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        live_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");

        // === Continuous IK: use slider's travel-to-extension mapping for prismatic ===
        {
            // Slider semantics: ee_z_mm_ = 150 - d3_m*1000  (150mm top -> 0mm bottom)
            double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0; // meters
            // Clamp to joint limits
            d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));

            // Convert to a world Z for ikine() (since fkine does Tz(pedestalZ_) then Tz(d3))
            const double xt = ee_x_mm_ / 1000.0;
            const double yt = ee_y_mm_ / 1000.0;
            const double zt = pedestalZ_ + d3_from_slider;  // world Z at the wrist
            const double th = ee_theta_deg_;

            double q1d, q2d, d3d, q4d;
            if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
                q1_deg = q1d;
                q2_deg = q2d;
                d3_m = d3d;           // will match d3_from_slider after ikine
                q3_deg = q4d;           // UI's q3 holds wrist (fkine's q4)
                show_applied_pose_ = true;
            }
            else {
                cv::putText(_canvas, "Unreachable",
                    cv::Point(panel_x + 120, panel_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
            }
        }
    }

    // --- IK elbow-up toggle + action button row ---
    if (cvui::checkbox(_canvas, panel_x, panel_y + 6, "Elbow up (IK)", &elbow_up_toggle_)) { /* no-op *//* }

    if (isFK) {
        // FK: take current joints -> compute EE (mirror-on-demand)
        if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Apply FK \xE2\x86\x92 EE")) {
            cv::Mat T = fkine(q1_deg, q2_deg, d3_m, q3_deg); // q3 slider == wrist (q4)
            update_ee_readout_from_T(T, d3_m);
            show_applied_pose_ = true;
        }
    }
    else {
        // IK: one-shot solve button (kept for convenience) - uses the same mapping fix
        if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Solve IK \xE2\x86\x92 Joints")) {
            double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0;
            d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));

            const double xt = ee_x_mm_ / 1000.0;
            const double yt = ee_y_mm_ / 1000.0;
            const double zt = pedestalZ_ + d3_from_slider;
            const double th = ee_theta_deg_;

            double q1d, q2d, d3d, q4d;
            if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
                q1_deg = q1d; q2_deg = q2d; d3_m = d3d; q3_deg = q4d; // q3 holds wrist
                show_applied_pose_ = true;
            }
            else {
                cv::putText(_canvas, "Unreachable",
                    cv::Point(panel_x + 120, panel_y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
            }
        }
    }
    panel_y += 40;

    // --- Animation controls ---

    // --- Animation controls (existing Part B) ---
   // --- Animation & quick actions row (two columns) ---
    {
        const int xL = panel_x;           // left column
        const int xR = panel_x + 130;     // right column
        const int y0 = panel_y;           // first row (buttons)
        const int row_h = 40;

        // Left/top: Animate Part B (or Stop)
        if (!anim_running_) {
            if (cvui::button(_canvas, xL, y0, 120, 30, "Animate Part B"))
                start_anim_partB();
        }
        else {
            if (cvui::button(_canvas, xL, y0, 120, 30, "Stop"))
                stop_anim_partB();
        }

        // Right/top: Animate Linear IK (or Stop)
        if (!lin_anim_running_) {
            if (cvui::button(_canvas, xR, y0, 120, 30, "Animate Linear IK"))
                start_linear_anim();
        }
        else {
            if (cvui::button(_canvas, xR, y0, 120, 30, "Stop Linear IK"))
                stop_linear_anim();
        }

        // Left/bottom: Reset (directly under Animate Part B)
        if (cvui::button(_canvas, xL, y0 + row_h, 120, 30, "Reset")) {
            q1_deg = q2_deg = q3_deg = 0.0;
            d3_m = 0.0;
            ee_x_mm_ = ee_y_mm_ = 0.0;
            ee_z_mm_ = 150.0;
            ee_theta_deg_ = 0.0;
            show_applied_pose_ = false;
            solve_mode_ = SolveMode::FK;
            elbow_up_toggle_ = true;
        }

        // Right/bottom: Track Cube toggle
        {
            const char* label = track_cube_ ? "Stop Tracking" : "Track Cube";
            if (cvui::button(_canvas, xR, y0 + row_h, 120, 30, label)) {
                track_cube_ = !track_cube_;
                // optional: when starting, switch to IK mode so joint sliders are read-only
                if (track_cube_) {
                    solve_mode_ = SolveMode::IK;
                }
            }
        }


        // advance panel_y past both rows
        panel_y = y0 + 2 * row_h;
    }

}
*/
/*
void CRobot::update_joint_controls(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- UI panel position ---
    int panel_x = _canvas.cols - 280;
    int panel_y = 100;

    // Background panel
    cv::rectangle(_canvas,
        cv::Point(panel_x - 10, panel_y - 30),
        cv::Point(panel_x + 270, panel_y + 660),
        cv::Scalar(70, 70, 70), cv::FILLED);

    // Window frame
    cvui::window(_canvas, panel_x, panel_y, 260, 630, "SCARA Controls");
    panel_x += 10;
    panel_y += 30;

    // --- View Mode (AR vs Virtual) ---
    cvui::text(_canvas, panel_x, panel_y, "View Mode:");
    bool isAR = (view_mode_ == ViewMode::AR);
    if (cvui::button(_canvas, panel_x + 90, panel_y - 6, 70, 24, isAR ? "[AR]" : " AR "))   view_mode_ = ViewMode::AR;
    if (cvui::button(_canvas, panel_x + 165, panel_y - 6, 70, 24, isAR ? "VIRT" : "[VIRT]")) view_mode_ = ViewMode::Virtual;
    panel_y += 30;

    // --- Solve direction (FK vs IK) ---
    cvui::text(_canvas, panel_x, panel_y, "Solve Direction:");
    bool isFK = (solve_mode_ == SolveMode::FK);
    if (cvui::button(_canvas, panel_x + 140, panel_y - 6, 50, 24, isFK ? "[FK]" : " FK "))     solve_mode_ = SolveMode::FK;
    if (cvui::button(_canvas, panel_x + 195, panel_y - 6, 50, 24, isFK ? " IK " : "[IK]"))     solve_mode_ = SolveMode::IK;
    panel_y += 30;

    // --- TRACK CUBE status badge ---
    if (track_cube_) {
        cv::rectangle(_canvas, cv::Rect(panel_x, panel_y, 240, 22), cv::Scalar(20, 60, 20), cv::FILLED);
        cv::putText(_canvas, "TRACKING marker 50", cv::Point(panel_x + 6, panel_y + 16),
            cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
        panel_y += 28;
    }

    // Helpers
    auto readonly_trackbar = [&](double current, double lo, double hi, const char* label)
        {
            double tmp = current;
            cvui::text(_canvas, panel_x, panel_y - 8, label);
            cvui::trackbar(_canvas, panel_x, panel_y, 240, &tmp, lo, hi);
            panel_y += 50;
        };
    auto live_trackbar = [&](double& ref, double lo, double hi, const char* label)
        {
            cvui::text(_canvas, panel_x, panel_y - 8, label);
            cvui::trackbar(_canvas, panel_x, panel_y, 240, &ref, lo, hi);
            panel_y += 50;
        };

    // --- Joint controls ---
    // While tracking, always show read-only to avoid fighting the tracker.
    if (track_cube_) {
        readonly_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)  [tracking]");
        readonly_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)  [tracking]");
        readonly_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  [tracking]");
        readonly_trackbar(d3_m, 0.0, 0.15, "d3 (m)    [tracking]");
    }
    else if (isFK) {
        live_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)");
        live_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)");
        live_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  (acts as wrist q4)");
        live_trackbar(d3_m, 0.0, 0.15, "d3 (m)");
    }
    else {
        readonly_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)  [read-only in IK]");
        readonly_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)  [read-only in IK]");
        readonly_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  [read-only in IK]");
        readonly_trackbar(d3_m, 0.0, 0.15, "d3 (m)    [read-only in IK]");
    }

    // --- EE pose section ---
    cvui::text(_canvas, panel_x, panel_y,
        (isFK || track_cube_) ? "End Effector Pose (mirror)" : "End Effector Pose (IK target)");
    panel_y += 20;

    if (track_cube_) {
        // Mirror only
        readonly_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)  [tracking]");
        readonly_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)  [tracking]");
        readonly_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)  [tracking]");
        readonly_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)  [tracking]");
    }
    else if (isFK) {
        // Mirror only
        readonly_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        readonly_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        readonly_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        readonly_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");
    }
    else {
        // IK target (live) with continuous IK
        live_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        live_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        live_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        live_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");

        // Continuous IK (skip while tracking is off? no, only skip if tracking ON)
        {
            // Slider semantics: ee_z_mm_ = 150 - d3_m*1000
            double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0;
            d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));

            const double xt = ee_x_mm_ / 1000.0;
            const double yt = ee_y_mm_ / 1000.0;
            const double zt = pedestalZ_ + d3_from_slider;
            const double th = ee_theta_deg_;

            double q1d, q2d, d3d, q4d;
            if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
                q1_deg = q1d; q2_deg = q2d; d3_m = d3d; q3_deg = q4d;
                show_applied_pose_ = true;
            }
            else {
                cv::putText(_canvas, "Unreachable",
                    cv::Point(panel_x + 120, panel_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
            }
        }
    }

    // --- IK elbow-up toggle ---
    if (cvui::checkbox(_canvas, panel_x, panel_y + 6, "Elbow up (IK)", &elbow_up_toggle_)) { /* no-op *//* }

    // --- FK/IK one-shot buttons ---
    if (!track_cube_) {
        if (isFK) {
            if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Apply FK -> EE")) {
                cv::Mat T = fkine(q1_deg, q2_deg, d3_m, q3_deg);
                update_ee_readout_from_T(T, d3_m);
                show_applied_pose_ = true;
            }
        }
        else {
            if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Solve IK -> Joints")) {
                double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0;
                d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));
                const double xt = ee_x_mm_ / 1000.0;
                const double yt = ee_y_mm_ / 1000.0;
                const double zt = pedestalZ_ + d3_from_slider;
                const double th = ee_theta_deg_;
                double q1d, q2d, d3d, q4d;
                if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
                    q1_deg = q1d; q2_deg = q2d; d3_m = d3d; q3_deg = q4d;
                    show_applied_pose_ = true;
                }
                else {
                    cv::putText(_canvas, "Unreachable",
                        cv::Point(panel_x + 120, panel_y + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
                }
            }
        }
    }
    panel_y += 40;

    // --- Animation & quick actions row (two columns) ---
    {
        const int xL = panel_x;           // left column
        const int xR = panel_x + 130;     // right column
        const int y0 = panel_y;           // first row (buttons)
        const int row_h = 40;

        // Animate Part B (disabled while tracking)
        if (!track_cube_) {
            if (!anim_running_) {
                if (cvui::button(_canvas, xL, y0, 120, 30, "Animate Part B"))
                    start_anim_partB();
            }
            else {
                if (cvui::button(_canvas, xL, y0, 120, 30, "Stop"))
                    stop_anim_partB();
            }
        }
        else {
            // Disabled look
            cv::rectangle(_canvas, cv::Rect(xL, y0, 120, 30), cv::Scalar(90, 90, 90), cv::FILLED);
            cv::putText(_canvas, "Animate Part B", cv::Point(xL + 6, y0 + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
        }

        // Animate Linear IK (disabled while tracking)
        if (!track_cube_) {
            if (!lin_anim_running_) {
                if (cvui::button(_canvas, xR, y0, 120, 30, "Animate Linear IK"))
                    start_linear_anim();
            }
            else {
                if (cvui::button(_canvas, xR, y0, 120, 30, "Stop Linear IK"))
                    stop_linear_anim();
            }
        }
        else {
            cv::rectangle(_canvas, cv::Rect(xR, y0, 120, 30), cv::Scalar(90, 90, 90), cv::FILLED);
            cv::putText(_canvas, "Linear IK", cv::Point(xR + 20, y0 + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
        }

        // Reset
        if (cvui::button(_canvas, xL, y0 + row_h, 120, 30, "Reset")) {
            q1_deg = q2_deg = q3_deg = 0.0;
            d3_m = 0.0;
            ee_x_mm_ = ee_y_mm_ = 0.0;
            ee_z_mm_ = 150.0;
            ee_theta_deg_ = 0.0;
            show_applied_pose_ = false;
            solve_mode_ = SolveMode::FK;
            elbow_up_toggle_ = true;
        }

        // Track Cube toggle
        {
            const char* label = track_cube_ ? "Stop Tracking" : "Track Cube";
            if (cvui::button(_canvas, xR, y0 + row_h, 120, 30, label)) {
                track_cube_ = !track_cube_;
                if (track_cube_) {
                    // Prefer IK display while tracking (joints are driven by target)
                    solve_mode_ = SolveMode::IK;
                    // Stop any running animations
                    anim_running_ = false;
                    lin_anim_running_ = false;
                }
            }
        }

        // advance panel_y past both rows
        panel_y = y0 + 2 * row_h;
    }
}
*/
void CRobot::update_joint_controls(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- place the panel at the very top-right ---
    int panel_x = _canvas.cols - 280;
    int panel_y = 10;                // was 100

    // Slightly taller background to fit tracking badge
    cv::rectangle(_canvas,
        cv::Point(panel_x - 10, panel_y - 30),
        cv::Point(panel_x + 270, panel_y + 680),
        cv::Scalar(70, 70, 70), cv::FILLED);

    cvui::window(_canvas, panel_x, panel_y, 260, 650, "SCARA Controls");
    panel_x += 10;
    panel_y += 30;

    // View mode
    cvui::text(_canvas, panel_x, panel_y, "View Mode:");
    bool isAR = (view_mode_ == ViewMode::AR);
    if (cvui::button(_canvas, panel_x + 90, panel_y - 6, 70, 24, isAR ? "[AR]" : " AR "))   view_mode_ = ViewMode::AR;
    if (cvui::button(_canvas, panel_x + 165, panel_y - 6, 70, 24, isAR ? "VIRT" : "[VIRT]")) view_mode_ = ViewMode::Virtual;
    panel_y += 30;

    // Solve mode
    cvui::text(_canvas, panel_x, panel_y, "Solve Direction:");
    bool isFK = (solve_mode_ == SolveMode::FK);
    if (cvui::button(_canvas, panel_x + 140, panel_y - 6, 50, 24, isFK ? "[FK]" : " FK "))   solve_mode_ = SolveMode::FK;
    if (cvui::button(_canvas, panel_x + 195, panel_y - 6, 50, 24, isFK ? " IK " : "[IK]"))   solve_mode_ = SolveMode::IK;
    panel_y += 30;

    // Tracking badge
    if (track_cube_) {
        cv::rectangle(_canvas, cv::Rect(panel_x, panel_y, 240, 22), cv::Scalar(20, 60, 20), cv::FILLED);
        cv::putText(_canvas, "TRACKING marker 50", cv::Point(panel_x + 6, panel_y + 16),
            cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
        panel_y += 28;
    }

    auto readonly_trackbar = [&](double current, double lo, double hi, const char* label) {
        double tmp = current;
        cvui::text(_canvas, panel_x, panel_y - 8, label);
        cvui::trackbar(_canvas, panel_x, panel_y, 240, &tmp, lo, hi);
        panel_y += 50;
        };
    auto live_trackbar = [&](double& ref, double lo, double hi, const char* label) {
        cvui::text(_canvas, panel_x, panel_y - 8, label);
        cvui::trackbar(_canvas, panel_x, panel_y, 240, &ref, lo, hi);
        panel_y += 50;
        };

    // Joints (read-only if tracking)
    if (track_cube_) {
        readonly_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)  [tracking]");
        readonly_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)  [tracking]");
        readonly_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  [tracking]");
        readonly_trackbar(d3_m, 0.0, 0.15, "d3 (m)    [tracking]");
    }
    else if (isFK) {
        live_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)");
        live_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)");
        live_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  (acts as wrist q4)");
        live_trackbar(d3_m, 0.0, 0.15, "d3 (m)");
    }
    else {
        readonly_trackbar(q1_deg, -180.0, 180.0, "q1 (deg)  [read-only in IK]");
        readonly_trackbar(q2_deg, -180.0, 180.0, "q2 (deg)  [read-only in IK]");
        readonly_trackbar(q3_deg, -180.0, 180.0, "q3 (deg)  [read-only in IK]");
        readonly_trackbar(d3_m, 0.0, 0.15, "d3 (m)    [read-only in IK]");
    }

    // EE section
    cvui::text(_canvas, panel_x, panel_y,
        (isFK || track_cube_) ? "End Effector Pose (mirror)" : "End Effector Pose (IK target)");
    panel_y += 20;

    if (track_cube_ || isFK) {
        readonly_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        readonly_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        readonly_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        readonly_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");
    }
    else {
        live_trackbar(ee_x_mm_, -300.0, 300.0, "x_e (mm)");
        live_trackbar(ee_y_mm_, -300.0, 300.0, "y_e (mm)");
        live_trackbar(ee_z_mm_, 0.0, 150.0, "z_e (mm)");
        live_trackbar(ee_theta_deg_, -180.0, 180.0, "theta_e (deg)");

        // continuous IK
        double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0;
        d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));
        const double xt = ee_x_mm_ / 1000.0, yt = ee_y_mm_ / 1000.0, zt = pedestalZ_ + d3_from_slider, th = ee_theta_deg_;
        double q1d, q2d, d3d, q4d;
        if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
            q1_deg = q1d; q2_deg = q2d; d3_m = d3d; q3_deg = q4d; show_applied_pose_ = true;
        }
        else {
            cv::putText(_canvas, "Unreachable", cv::Point(panel_x + 120, panel_y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
        }
    }

    // elbow-up
    (void)cvui::checkbox(_canvas, panel_x, panel_y + 6, "Elbow up (IK)", &elbow_up_toggle_);
    // one-shots hidden if tracking
    if (!track_cube_) {
        if (isFK) {
            if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Apply FK -> EE")) {
                cv::Mat T = fkine(q1_deg, q2_deg, d3_m, q3_deg);
                update_ee_readout_from_T(T, d3_m);
                show_applied_pose_ = true;
            }
        }
        else {
            if (cvui::button(_canvas, panel_x + 120, panel_y, 120, 30, "Solve IK -> Joints")) {
                double d3_from_slider = (150.0 - ee_z_mm_) / 1000.0;
                d3_from_slider = std::max(D3_MIN_, std::min(D3_MAX_, d3_from_slider));
                const double xt = ee_x_mm_ / 1000.0, yt = ee_y_mm_ / 1000.0, zt = pedestalZ_ + d3_from_slider, th = ee_theta_deg_;
                double q1d, q2d, d3d, q4d;
                if (ikine(xt, yt, zt, th, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
                    q1_deg = q1d; q2_deg = q2d; d3_m = d3d; q3_deg = q4d; show_applied_pose_ = true;
                }
                else {
                    cv::putText(_canvas, "Unreachable", cv::Point(panel_x + 120, panel_y + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv::LINE_AA);
                }
            }
        }
    }
    panel_y += 40;

    // bottom row: animations + reset + track toggle
    const int xL = panel_x, xR = panel_x + 130, y0 = panel_y, row_h = 40;

    // Animate buttons disabled while tracking
    if (!track_cube_) {
        if (!anim_running_) { if (cvui::button(_canvas, xL, y0, 120, 30, "Animate Part B")) start_anim_partB(); }
        else { if (cvui::button(_canvas, xL, y0, 120, 30, "Stop"))          stop_anim_partB(); }
        if (!lin_anim_running_) { if (cvui::button(_canvas, xR, y0, 120, 30, "Animate Linear IK")) start_linear_anim(); }
        else { if (cvui::button(_canvas, xR, y0, 120, 30, "Stop Linear IK"))    stop_linear_anim(); }
    }
    else {
        cv::rectangle(_canvas, cv::Rect(xL, y0, 120, 30), cv::Scalar(90, 90, 90), cv::FILLED);
        cv::putText(_canvas, "Animate Part B", cv::Point(xL + 6, y0 + 22), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
        cv::rectangle(_canvas, cv::Rect(xR, y0, 120, 30), cv::Scalar(90, 90, 90), cv::FILLED);
        cv::putText(_canvas, "Linear IK", cv::Point(xR + 20, y0 + 22), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
    }

    // Reset
    if (cvui::button(_canvas, xL, y0 + row_h, 120, 30, "Reset")) {
        q1_deg = q2_deg = q3_deg = 0.0; d3_m = 0.0;
        ee_x_mm_ = ee_y_mm_ = 0.0; ee_z_mm_ = 150.0; ee_theta_deg_ = 0.0;
        show_applied_pose_ = false; solve_mode_ = SolveMode::FK; elbow_up_toggle_ = true;
    }

    // Track Cube toggle
    const char* label = track_cube_ ? "Stop Tracking" : "Track Cube";
    if (cvui::button(_canvas, xR, y0 + row_h, 120, 30, label)) {
        track_cube_ = !track_cube_;
        if (track_cube_) {
            solve_mode_ = SolveMode::IK;
            anim_running_ = false;
            lin_anim_running_ = false;
        }
    }

    // Lab 7: joint-space trajectory Home <-> PoseB
    if (!track_cube_) {
        if (cvui::button(_canvas, xL, y0 + 2 * row_h, 240, 30,
            "Lab7: Home <-> PoseB (jtraj)"))
        {
            start_lab7_home_target_traj(q1_deg, q2_deg, q3_deg, d3_m);
        }
    }

    // Move panel_y down so later widgets (if any) are below
    panel_y = y0 + 3 * row_h;

}


void CRobot::draw_scara_on_real(cv::Mat& frame, CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (!cam.have_pose) return;

    // Reset canvas = live frame
    cv::Mat im = frame.clone();

    // Forward kinematics (same as your virtual one)
    const float base_height = 0.15f;
    const float arm_len1 = 0.15f;
    const float arm_len2 = 0.15f;
    const float thickness = 0.03f;
    const float prism_height = 0.15f;
    const float offset_z = 0.135f;

    std::vector<std::vector<cv::Mat>> blocks;
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;
    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;
    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;
    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // Build blocks
    auto block1 = createBox(thickness, thickness, base_height);
    transformPoints(block1, T1);
    blocks.push_back(block1);

    auto block2 = createBox(arm_len1, thickness, thickness);
    transformPoints(block2, T1 * Tx(arm_len1 / 2.0));
    blocks.push_back(block2);

    auto block3 = createBox(arm_len2, thickness, thickness);
    transformPoints(block3, T2 * Tx(arm_len2 / 2.0));
    blocks.push_back(block3);

    auto block4 = createBox(thickness, thickness, prism_height);
    transformPoints(block4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z));
    blocks.push_back(block4);

    // Project to image using real camera
    for (size_t i = 0; i < blocks.size(); ++i)
        drawBoxReal(im, blocks[i], cam, chooseColors((int)i));

    // Show in live feed
    cv::imshow(CANVAS_NAME, im);
}

void CRobot::set_world_anchor(const cv::Vec3d& p_WB, const cv::Vec3d& rpy_WB)
{
    // Place robot base wrt the ChArUco board/world
    T_WB_ = createHT(p_WB, rpy_WB); // meters + degrees
}

void CRobot::drawPrismReal(cv::Mat& im,
    std::vector<cv::Mat> prism3d, CCameraReal& cam, const cv::Scalar& colour)
{
    if (!cam.have_pose) return;

    // Match your BoxReal convention: flip Z before projecting
    for (auto& P : prism3d) {
        P.at<float>(2, 0) = -P.at<float>(2, 0);
    }

    std::vector<cv::Point2f> pts2d;
    cam.transform_to_image(prism3d, pts2d);

    static const int e1[12] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
    static const int e2[12] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

    if (pts2d.size() == 8) {
        for (int i = 0; i < 12; ++i) {
            cv::line(im, pts2d[e1[i]], pts2d[e2[i]], colour, 2, cv::LINE_AA);
        }
    }
}


/*void CRobot::draw_scara_world(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- caches for a "frozen" robot overlay only (not the whole frame) ---
    static cv::Mat last_robot_layer;  // BGR, only robot graphics drawn
    static cv::Mat last_robot_mask;   // 8-bit mask where robot_layer has pixels
    static bool overlay_valid = false;

    if (_canvas.empty()) _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    // ---------------- Dimensions (meters) ----------------
    const float thickness = 0.03f;
    const float LINK_LENGTH = 0.15f;
    const float base_height = LINK_LENGTH - thickness / 2.0f;
    const float arm_len1 = LINK_LENGTH;
    const float arm_len2 = LINK_LENGTH;
    const float prism_height = LINK_LENGTH;
    const float offset_z = 0.135f;

    // === Helper to (re)compute FK chain ===
    auto fk_recompute = [&](cv::Mat& T0, cv::Mat& T1, cv::Mat& T2, cv::Mat& T3, cv::Mat& T4)
        {
            T0 = T_WB_;
            cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
            T1 = T0 * T01;
            cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
            T2 = T1 * T12;
            cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
            T3 = T2 * T23;
            cv::Mat T34 = Tz(-d3_m);
            T4 = T3 * T34;
        };

    // ---------------- FK in WORLD (board) frame ----------------
    cv::Mat T0, T1, T2, T3, T4;
    fk_recompute(T0, T1, T2, T3, T4);

    // ---------------- Tracking (runs BEFORE readouts/UI) ----------------
    // If tracking toggled on and we have a fresh marker pose, update joints now.
    bool did_track = maybe_track_cube(cam, q1_deg, q2_deg, q3_deg, d3_m);
    if (did_track) {
        // Recompute FK so everything (readouts, rendering) reflects the new joints
        fk_recompute(T0, T1, T2, T3, T4);
    }

    // ---------------- Update EE readouts (mirror final T4) ----------------
    {
        const float x_m = T4.at<float>(0, 3);
        const float y_m = T4.at<float>(1, 3);
        const float R00 = T4.at<float>(0, 0);
        const float R10 = T4.at<float>(1, 0);
        const float theta_deg = static_cast<float>(std::atan2(R10, R00) * 180.0 / CV_PI);

        ee_x_mm_ = 1000.0f * x_m;
        ee_y_mm_ = 1000.0f * y_m;
        ee_theta_deg_ = theta_deg;

        const float z_travel_mm = 150.0f - static_cast<float>(d3_m * 1000.0f);
        ee_z_mm_ = std::max(0.0f, std::min(150.0f, z_travel_mm));
    }

    // ---------------- UI + optional animation advance ----------------
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);

    // If tracking is ON, skip animations so they don't fight the tracker.
    if (!track_cube_) {
        if (anim_running_) {
            if (!step_anim_partB(q1_deg, q2_deg, q3_deg, d3_m)) {
                anim_running_ = false;
            }
            else {
                // joints changed -> recompute FK and mirror readouts
                fk_recompute(T0, T1, T2, T3, T4);
                const float R00 = T4.at<float>(0, 0), R10 = T4.at<float>(1, 0);
                ee_x_mm_ = 1000.0f * T4.at<float>(0, 3);
                ee_y_mm_ = 1000.0f * T4.at<float>(1, 3);
                ee_theta_deg_ = static_cast<float>(std::atan2(R10, R00) * 180.0 / CV_PI);
                ee_z_mm_ = std::max(0.0f, std::min(150.0f, 150.0f - static_cast<float>(d3_m * 1000.0)));
            }
        }
        if (lin_anim_running_) {
            if (!step_linear_anim(q1_deg, q2_deg, q3_deg, d3_m)) {
                lin_anim_running_ = false;
            }
            else {
                // joints changed -> recompute FK and mirror readouts
                fk_recompute(T0, T1, T2, T3, T4);
                const float R00 = T4.at<float>(0, 0), R10 = T4.at<float>(1, 0);
                ee_x_mm_ = 1000.0f * T4.at<float>(0, 3);
                ee_y_mm_ = 1000.0f * T4.at<float>(1, 3);
                ee_theta_deg_ = static_cast<float>(std::atan2(R10, R00) * 180.0 / CV_PI);
                ee_z_mm_ = std::max(0.0f, std::min(150.0f, 150.0f - static_cast<float>(d3_m * 1000.0)));
            }
        }
    }

    // If pose is missing: draw NOTHING new for robot, just paste last overlay over the live frame
    if (!cam.have_pose)
    {
        if (overlay_valid && !last_robot_layer.empty() && !last_robot_mask.empty())
        {
            last_robot_layer.copyTo(_canvas, last_robot_mask);
        }
        cvui::update();
        cv::imshow(CANVAS_NAME, _canvas);
        return;
    }

    // ---------------- Have pose: render robot on a fresh overlay layer ----------------
    cv::Mat robot_layer = cv::Mat::zeros(_canvas.size(), _canvas.type());

    // Build prisms with SAME placements as virtual
    std::vector<std::vector<cv::Mat>> prisms;
    auto p1 = createPrism(thickness, base_height, thickness);
    transformPoints(p1, T1 * Tz(-base_height / 2.0));
    prisms.push_back(p1);

    auto p2 = createPrism(arm_len1, thickness, thickness);
    transformPoints(p2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0));
    prisms.push_back(p2);

    auto p3 = createPrism(arm_len2, thickness, thickness);
    transformPoints(p3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0));
    prisms.push_back(p3);

    auto p4 = createPrism(thickness, prism_height, thickness);
    transformPoints(p4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0 - 2.0f * thickness));
    prisms.push_back(p4);

    // Draw prisms into robot_layer (not _canvas)
    for (size_t i = 0; i < prisms.size(); ++i) {
        drawPrismReal(robot_layer, prisms[i], cam,
            chooseColors((int)i, RED, YELLOW, GREEN, MAGENTA));
    }

    // Draw axes into robot_layer (unchanged placement)
    auto drawAxes = [&](const std::vector<cv::Mat>& C, CCameraReal& camRef)
        {
            std::vector<cv::Mat> tmp = C;
            drawCoordReal(robot_layer, tmp, camRef);
        };

    // World/base axes
    auto W = createCoord();
    transformPoints(W, T0);
    drawAxes(W, cam);

    auto makeSmall = [&](std::vector<cv::Mat>& C)
        {
            for (auto& P : C) {
                P.at<float>(0, 0) *= 0.4f;
                P.at<float>(1, 0) *= 0.4f;
                P.at<float>(2, 0) *= 0.4f;
            }
        };

    cv::Mat fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 0, 0));
    auto J0 = createCoord(); makeSmall(J0); transformPoints(J0, fixAxes); transformPoints(J0, T1); drawAxes(J0, cam);
    auto J1 = createCoord(); makeSmall(J1); transformPoints(J1, fixAxes); transformPoints(J1, T2); drawAxes(J1, cam);
    fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 90, 0));
    auto J2 = createCoord(); makeSmall(J2); transformPoints(J2, fixAxes); transformPoints(J2, T3); drawAxes(J2, cam);
    auto E = createCoord(); makeSmall(E);  transformPoints(E, fixAxes); transformPoints(E, T4); drawAxes(E, cam);

    // Create mask where robot_layer has content
    cv::Mat gray, mask;
    cv::cvtColor(robot_layer, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);

    // Composite robot_layer over the live frame in _canvas
    robot_layer.copyTo(_canvas, mask);

    // Cache overlay for use when pose is lost
    last_robot_layer = robot_layer;
    last_robot_mask = mask;
    overlay_valid = true;

    // Present
    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}
*/
void CRobot::draw_scara_world(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- caches for a "frozen" robot overlay only (not the whole frame) ---
    static cv::Mat last_robot_layer;  // BGR, only robot graphics drawn
    static cv::Mat last_robot_mask;   // 8-bit mask where robot_layer has pixels
    static bool overlay_valid = false;

    if (_canvas.empty()) _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    // ---------------- Dimensions (meters) ----------------
    const float thickness = 0.03f;
    const float LINK_LENGTH = 0.15f;
    const float base_height = LINK_LENGTH - thickness / 2.0f;
    const float arm_len1 = LINK_LENGTH;
    const float arm_len2 = LINK_LENGTH;
    const float prism_height = LINK_LENGTH;
    const float offset_z = 0.135f;

    // === NEW: if tracking is ON, update joints from cube BEFORE FK ===
    (void)maybe_track_cube(cam, q1_deg, q2_deg, q3_deg, d3_m);

    // ---------------- FK in WORLD (board) frame ----------------
    cv::Mat T0 = T_WB_;
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;
    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;
    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;
    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // ---------------- Update EE readouts (always) ----------------
    {
        const float x_m = T4.at<float>(0, 3);
        const float y_m = T4.at<float>(1, 3);
        const float R00 = T4.at<float>(0, 0);
        const float R10 = T4.at<float>(1, 0);
        const float theta_deg = static_cast<float>(std::atan2(R10, R00) * 180.0 / CV_PI);

        ee_x_mm_ = 1000.0f * x_m;
        ee_y_mm_ = 1000.0f * y_m;
        ee_theta_deg_ = theta_deg;

        const float z_travel_mm = 150.0f - static_cast<float>(d3_m * 1000.0);
        ee_z_mm_ = std::max(0.0f, std::min(150.0f, z_travel_mm));
    }

    // ---------------- UI + optional animation advance ----------------
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);
    if (anim_running_) {
        if (!step_anim_partB(q1_deg, q2_deg, q3_deg, d3_m)) {
            anim_running_ = false;
        }
    }
    if (lin_anim_running_) {
        if (!step_linear_anim(q1_deg, q2_deg, q3_deg, d3_m)) {
            lin_anim_running_ = false;
        }
    }

    // If pose is missing: draw NOTHING new for robot, just paste last overlay over the live frame
    if (!cam.have_pose)
    {
        if (overlay_valid && !last_robot_layer.empty() && !last_robot_mask.empty()) {
            last_robot_layer.copyTo(_canvas, last_robot_mask);
        }
        cvui::update();
        cv::imshow(CANVAS_NAME, _canvas);
        return;
    }

    // ---------------- Have pose: render robot on a fresh overlay layer ----------------
    cv::Mat robot_layer = cv::Mat::zeros(_canvas.size(), _canvas.type());

    // Build prisms with SAME placements as virtual
    std::vector<std::vector<cv::Mat>> prisms;
    auto p1 = createPrism(thickness, base_height, thickness);
    transformPoints(p1, T1 * Tz(-base_height / 2.0));                       prisms.push_back(p1);
    auto p2 = createPrism(arm_len1, thickness, thickness);
    transformPoints(p2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0));   prisms.push_back(p2);
    auto p3 = createPrism(arm_len2, thickness, thickness);
    transformPoints(p3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0));   prisms.push_back(p3);
    auto p4 = createPrism(thickness, prism_height, thickness);
    transformPoints(p4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0 - 2.0f * thickness));
    prisms.push_back(p4);

    for (size_t i = 0; i < prisms.size(); ++i) {
        drawPrismReal(robot_layer, prisms[i], cam,
            chooseColors((int)i, RED, YELLOW, GREEN, MAGENTA));
    }

    // Axes into robot_layer
    auto drawAxes = [&](const std::vector<cv::Mat>& C, CCameraReal& camRef)
        {
            std::vector<cv::Mat> tmp = C;
            drawCoordReal(robot_layer, tmp, camRef);
        };
    auto W = createCoord(); transformPoints(W, T0); drawAxes(W, cam);

    auto makeSmall = [&](std::vector<cv::Mat>& C) {
        for (auto& P : C) { P.at<float>(0, 0) *= 0.4f; P.at<float>(1, 0) *= 0.4f; P.at<float>(2, 0) *= 0.4f; }
        };
    cv::Mat fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 0, 0));
    auto J0 = createCoord(); makeSmall(J0); transformPoints(J0, fixAxes); transformPoints(J0, T1); drawAxes(J0, cam);
    auto J1 = createCoord(); makeSmall(J1); transformPoints(J1, fixAxes); transformPoints(J1, T2); drawAxes(J1, cam);
    fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 90, 0));
    auto J2 = createCoord(); makeSmall(J2); transformPoints(J2, fixAxes); transformPoints(J2, T3); drawAxes(J2, cam);
    auto E = createCoord(); makeSmall(E); transformPoints(E, fixAxes); transformPoints(E, T4); drawAxes(E, cam);

    // Composite robot_layer over the live frame in _canvas
    cv::Mat gray, mask;
    cv::cvtColor(robot_layer, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
    robot_layer.copyTo(_canvas, mask);

    // Cache overlay
    last_robot_layer = robot_layer;
    last_robot_mask = mask;
    overlay_valid = true;

    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}




// === Real-camera axis drawer (projects using CCameraReal, with Z flip to match your BoxReal) ===
void CRobot::drawCoordReal(cv::Mat& im, std::vector<cv::Mat> coord3d, CCameraReal& cam)
{
    if (!cam.have_pose) return;

    // Flip Z like drawBoxReal/drawPrismReal
    auto flipped = coord3d;
    for (auto& P : flipped) {
        P.at<float>(2, 0) = -P.at<float>(2, 0);
    }

    cv::Point2f O, X, Y, Z;
    cam.transform_to_image(flipped[0], O);
    cam.transform_to_image(flipped[1], X);
    cam.transform_to_image(flipped[2], Y);
    cam.transform_to_image(flipped[3], Z);

    cv::line(im, O, X, RED, 2);
    cv::line(im, O, Y, GREEN, 2);
    cv::line(im, O, Z, BLUE, 2);

    cv::putText(im, "X", X + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, RED, 1);
    cv::putText(im, "Y", Y + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1);
    cv::putText(im, "Z", Z + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 1);
}
void CRobot::draw_target_ee_world(CCameraReal& cam)
{
    if (!cam.have_pose) return;

    // Convert mm to meters
    const double ex = ee_x_mm_ / 1000.0;
    const double ey = ee_y_mm_ / 1000.0;
    const double ez = ee_z_mm_ / 1000.0;

    // Target pose in WORLD (board) frame
    cv::Mat T_target_W = T_WB_ * createHT(cv::Vec3d(ex, ey, ez),
        cv::Vec3d(0.0, 0.0, ee_theta_deg_));

    // Draw a small coordinate frame at the target pose
    auto A = createCoord();
    // Optionally scale a bit larger for visibility:
    for (auto& P : A) {
        P.at<float>(0, 0) *= 1.2f;
        P.at<float>(1, 0) *= 1.2f;
        P.at<float>(2, 0) *= 1.2f;
    }
    transformPoints(A, T_target_W);
    drawCoordReal(_canvas, A, cam);

    // Label
    cv::putText(_canvas, "EE target",
        cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2);
}
void CRobot::start_anim_partB()
{
    anim_running_ = true;
    anim_phase_ = 0;
}

void CRobot::stop_anim_partB()
{
    anim_running_ = false;
    anim_phase_ = 0;
}

// Returns true while running, false when finished
bool CRobot::step_anim_partB(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (!anim_running_) return false;

    switch (anim_phase_)
    {
        // ---- Joint 1: 0 -> +180 ----
    case 0:
        q1_deg += anim_step_deg_;
        if (q1_deg >= 180.0) { q1_deg = 180.0; anim_phase_ = 1; }
        break;

        // ---- Joint 1: +180 -> -180 ----
    case 1:
        q1_deg -= anim_step_deg_;
        if (q1_deg <= -180.0) { q1_deg = -180.0; anim_phase_ = 2; }
        break;

        // ---- Joint 1: -180 -> 0 ----
    case 2:
        if (q1_deg < 0.0) q1_deg = std::min(0.0, q1_deg + anim_step_deg_);
        else              q1_deg = std::max(0.0, q1_deg - anim_step_deg_);
        if (std::abs(q1_deg) <= anim_step_deg_) { q1_deg = 0.0; anim_phase_ = 3; }
        break;

        // ---- Joint 2: 0 -> +180 ----
    case 3:
        q2_deg += anim_step_deg_;
        if (q2_deg >= 180.0) { q2_deg = 180.0; anim_phase_ = 4; }
        break;

        // ---- Joint 2: +180 -> -180 ----
    case 4:
        q2_deg -= anim_step_deg_;
        if (q2_deg <= -180.0) { q2_deg = -180.0; anim_phase_ = 5; }
        break;

        // ---- Joint 2: -180 -> 0 ----
    case 5:
        if (q2_deg < 0.0) q2_deg = std::min(0.0, q2_deg + anim_step_deg_);
        else              q2_deg = std::max(0.0, q2_deg - anim_step_deg_);
        if (std::abs(q2_deg) <= anim_step_deg_) { q2_deg = 0.0; anim_phase_ = 6; }
        break;

        // ---- Joint 3: 0 -> 360 ----
    case 6:
        q3_deg += anim_step_deg_;
        if (q3_deg >= 360.0) { q3_deg = 360.0; anim_phase_ = 7; }
        break;

        // ---- Prismatic: 0 m -> +0.15 m ----
    case 7:
        d3_m += anim_step_m_;
        if (d3_m >= 0.15) { d3_m = 0.15; anim_phase_ = 8; }
        break;

    default:
        stop_anim_partB();
        return false;
    }

    return true;
}

/*
void CRobot::draw_scara_dispatch(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (_canvas.empty())
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    if (view_mode_ == ViewMode::AR)
    {
        // 1) Grab a fresh camera frame
        cv::Mat frame;
        cam.get_image(frame);

        if (!frame.empty()) {
            last_live_frame_ = frame;
        }
        else if (last_live_frame_.empty()) {
            _canvas = cv::Mat::zeros(_image_size, CV_8UC3);
            cv::imshow(CANVAS_NAME, _canvas);
            cvui::update();
            return;
        }

        // 2) Keep one CLEAN copy for pose estimation ONLY (no overlays)
        cv::Mat detect_scratch = last_live_frame_.clone();

        // 3) Estimate board pose on the CLEAN copy (avoid contaminating future draws)
        cam.detectBoardPose(detect_scratch);  // throws its own axes onto detect_scratch, which we discard

        // 4) Start our display canvas from the raw camera frame (still clean)
        _canvas = last_live_frame_.clone();

        // 5) Draw the cube (it does its own marker detect) on the canvas
        const float kMarkerLen = 0.02042f;  // meters
        const float kCubeH = 0.030f;    // meters
        (void)cam.draw_cube_on_marker(_canvas, 50, kMarkerLen, kCubeH);

        // 6) Now draw the robot in WORLD coords (uses cam.have_pose set in step 3)
        draw_scara_world(cam, q1_deg, q2_deg, q3_deg, d3_m);

        // 7) Optional: EE target label
        if (show_applied_pose_) {
            draw_target_ee_world(cam);
        }

        // 8) Optional HUD showing marker pose in board frame
        {
            cv::Vec3d rvec_BM, tvec_BM;
            if (cam.get_marker_pose_in_board(50, rvec_BM, tvec_BM)) {
                cv::Mat Rbm; cv::Rodrigues(rvec_BM, Rbm);
                double yaw_rad = std::atan2(Rbm.at<double>(1, 0), Rbm.at<double>(0, 0));
                double yaw_deg = yaw_rad * 180.0 / 3.14159265358979323846;

                double x_mm = 1000.0 * tvec_BM[0];
                double y_mm = 1000.0 * tvec_BM[1];
                double z_mm = 1000.0 * tvec_BM[2];

                char buf[160];
                std::snprintf(buf, sizeof(buf),
                    "Marker 50 in Board: x=%.1f mm, y=%.1f mm, z=%.1f mm, yaw=%.1f deg",
                    x_mm, y_mm, z_mm, yaw_deg);

                int baseline = 0;
                cv::Size ts = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
                cv::Rect bg(10, 10, ts.width + 12, ts.height + 12);
                cv::rectangle(_canvas, bg, cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(_canvas, buf, cv::Point(16, 10 + ts.height + 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
        }

        cv::imshow(CANVAS_NAME, _canvas);
        cvui::update();
    }
    else
    {
        // Virtual view path unchanged
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR;
        _virtualcam.update_settings(_canvas);
        draw_scara(q1_deg, q2_deg, q3_deg, d3_m);

        if (show_applied_pose_) {
            auto A = createCoord();
            for (auto& P : A) {
                P.at<float>(0, 0) *= 1.2f;
                P.at<float>(1, 0) *= 1.2f;
                P.at<float>(2, 0) *= 1.2f;
            }
            cv::Mat T_target = createHT(
                cv::Vec3d(ee_x_mm_ / 1000.0, ee_y_mm_ / 1000.0, ee_z_mm_ / 1000.0),
                cv::Vec3d(0, 0, ee_theta_deg_));
            transformPoints(A, T_target);
            drawCoord(_canvas, A);
        }

        cv::imshow(CANVAS_NAME, _canvas);
        cvui::update();
    }
}
*/

void CRobot::draw_scara_dispatch(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (_canvas.empty())
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    if (view_mode_ == ViewMode::AR)
    {
        // --- AR branch (unchanged setup) ---
        cv::Mat frame;
        cam.get_image(frame);
        if (!frame.empty()) { last_live_frame_ = frame; }
        else if (last_live_frame_.empty()) {
            _canvas = cv::Mat::zeros(_image_size, CV_8UC3);
            cv::imshow(CANVAS_NAME, _canvas);
            cvui::update();
            return;
        }

        cv::Mat detect_scratch = last_live_frame_.clone();
        cam.detectBoardPose(detect_scratch);

        _canvas = last_live_frame_.clone();

        const float kMarkerLen = 0.02042f;
        const float kCubeH = 0.030f;
        (void)cam.draw_cube_on_marker(_canvas, 50, kMarkerLen, kCubeH);

        // AR-only button: move to Marker 50 using joint-space jtraj
        if (cvui::button(_canvas, 10, 50, 220, 30,
            "Lab7: jtraj to marker 50"))
        {
            start_traj_to_marker(cam, 50, q1_deg, q2_deg, q3_deg, d3_m);
        }

        // Advance any running animations (including jtraj)
        tick_animations(q1_deg, q2_deg, q3_deg, d3_m);


        // draw robot in world (uses pose computed above)
        draw_scara_world(cam, q1_deg, q2_deg, q3_deg, d3_m);

        if (show_applied_pose_) {
            draw_target_ee_world(cam);
        }

        cv::imshow(CANVAS_NAME, _canvas);
        cvui::update();
    }
    else
    {
        // --- VIRTUAL branch ---
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR;

        // NEW: advance any running animations here too
        tick_animations(q1_deg, q2_deg, q3_deg, d3_m);

        // left-panel virtual camera UI
        _virtualcam.update_settings(_canvas);

        // draw the robot in pure virtual space
        draw_scara(q1_deg, q2_deg, q3_deg, d3_m);

        if (show_applied_pose_) {
            // visualize desired EE pose in virtual view
            auto A = createCoord();
            for (auto& P : A) {
                P.at<float>(0, 0) *= 1.2f;
                P.at<float>(1, 0) *= 1.2f;
                P.at<float>(2, 0) *= 1.2f;
            }
            cv::Mat T_target = createHT(
                cv::Vec3d(ee_x_mm_ / 1000.0, ee_y_mm_ / 1000.0, ee_z_mm_ / 1000.0),
                cv::Vec3d(0, 0, ee_theta_deg_));
            transformPoints(A, T_target);
            drawCoord(_canvas, A);
            cv::putText(_canvas, "EE target (virtual)", cv::Point(12, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2);
        }

        cv::imshow(CANVAS_NAME, _canvas);
        cvui::update();
    }
}


// Put this in Robot.cpp (and declare in Robot.h if you want), used by both AR + Virtual
void CRobot::update_ee_readout_from_T(const cv::Mat& T4, double d3_m)
{
    // Position (x,y) in meters -> mm
    const float x_m = T4.at<float>(0, 3);
    const float y_m = T4.at<float>(1, 3);

    // Orientation about Z (theta) from rotation matrix first column
    const float R00 = T4.at<float>(0, 0);
    const float R10 = T4.at<float>(1, 0);
    const float theta_deg = static_cast<float>(std::atan2(R10, R00) * 180.0 / CV_PI);

    ee_x_mm_ = 1000.0f * x_m;
    ee_y_mm_ = 1000.0f * y_m;
    ee_theta_deg_ = theta_deg;

    // Z readout spec: 150 mm at top (d3 = 0) -> 0 mm at bottom as d3 increases
    const float z_travel_mm = 150.0f - static_cast<float>(d3_m * 1000.0);
    ee_z_mm_ = std::max(0.0f, std::min(150.0f, z_travel_mm));
}

// Draw the desired EE pose in VIRTUAL view (uses _virtualcam via drawCoord)
void CRobot::draw_target_ee_virtual()
{
    // Convert mm -> m
    const double ex = ee_x_mm_ / 1000.0;
    const double ey = ee_y_mm_ / 1000.0;
    const double ez = ee_z_mm_ / 1000.0;

    // Target pose in virtual WORLD frame (origin at virtual world base)
    cv::Mat T_target = createHT(cv::Vec3d(ex, ey, ez),
        cv::Vec3d(0.0, 0.0, ee_theta_deg_));

    // Slightly larger axis for visibility
    auto A = createCoord();
    for (auto& P : A) {
        P.at<float>(0, 0) *= 1.2f;
        P.at<float>(1, 0) *= 1.2f;
        P.at<float>(2, 0) *= 1.2f;
    }
    transformPoints(A, T_target);
    drawCoord(_canvas, A); // virtual camera path

    cv::putText(_canvas, "EE target (virtual)",
        cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2);
}

/*
//inverse kinematics lab 6 functions

static inline double rad2deg(double r) { return r * 180.0 / CV_PI; }
static inline double deg2rad(double d) { return d * CV_PI / 180.0; }

bool CRobot::ikine(double x_m, double y_m, double z_m, double theta_deg,
    double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg,
    bool elbow_up) const
{
    // ---------- Step 1: planar 2R for (x, y) ----------
    // r^2 and law of cosines for q2
    const double r2 = x_m * x_m + y_m * y_m;
    const double L1 = L1_;
    const double L2 = L2_;

    // cos(q2) from law of cosines; clamp to handle tiny numeric overshoot
    double c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (c2 > 1.0) c2 = 1.0;
    if (c2 < -1.0) c2 = -1.0;

    double s2 = std::sqrt(std::max(0.0, 1.0 - c2 * c2));
    if (!elbow_up) s2 = -s2; // choose branch

    const double q2 = std::atan2(s2, c2);

    // q1 from geometry (Khatib form)
    const double k1 = L1 + L2 * c2;
    const double k2 = L2 * s2;
    const double q1 = std::atan2(y_m, x_m) - std::atan2(k2, k1);

    // ---------- Step 2: prismatic from Z ----------
    // Your fkine does: T0 = Tz(pedestalZ_), then T23 = Tz(d3_m)
    // so z_e = pedestalZ_ + d3_m  ?  d3_m = z_m - pedestalZ_
    d3_m = z_m - pedestalZ_;
    // clamp to joint limits
    if (d3_m < D3_MIN_) d3_m = D3_MIN_;
    if (d3_m > D3_MAX_) d3_m = D3_MAX_;

    // ---------- Step 3: wrist angle as residual ----------
    // With your fkine chain T = ... Rz(q1) * ... * Rz(q2) * ... * Rz(q4),
    // the end-effector yaw ? ? q1 + q2 + q4  (no extra constant twists)
    const double q4 = deg2rad(theta_deg) - (q1 + q2);

    // ---------- Convert to degrees for your UI/trackbars ----------
    q1_deg = rad2deg(q1);
    q2_deg = rad2deg(q2);
    q4_deg = rad2deg(q4);

    // ---------- Reachability check (optional but handy) ----------
    // If target (x,y) is outside the annulus |L1-L2| ? ?r2 ? (L1+L2), report failure.
    const double r = std::sqrt(r2);
    const double r_min = std::fabs(L1 - L2);
    const double r_max = (L1 + L2);
    const bool reachable_xy = (r >= r_min - 1e-6) && (r <= r_max + 1e-6);
    const bool reachable_z = (z_m >= pedestalZ_ + D3_MIN_ - 1e-6) &&
        (z_m <= pedestalZ_ + D3_MAX_ + 1e-6);
    return reachable_xy && reachable_z;
}*/
// --- Inverse kinematics: SCARA with Rz(q1)-Tx(L1)-Rz(q2)-Tx(L2)-Tz(d3)-Rz(q4)
// Conventions preserved:
//   z_e = pedestalZ_ + d3  =>  d3 = z_m - pedestalZ_
//   theta ? q1 + q2 + q4
// Returns true if target is inside planar annulus and z limits.
static inline double rad2deg(double r) { return r * 180.0 / CV_PI; }
static inline double deg2rad(double d) { return d * CV_PI / 180.0; }

bool CRobot::ikine(double x_m, double y_m, double z_m, double theta_deg,
    double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg,
    bool elbow_up) const
{
    const double EPS = 1e-9;
    const double L1 = L1_;
    const double L2 = L2_;

    // --- Planar geometry for q1, q2 ---
    const double r2 = x_m * x_m + y_m * y_m;
    const double r = std::sqrt(std::max(0.0, r2));

    // Law of cosines for q2
    double c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    c2 = std::max(-1.0, std::min(1.0, c2));            // numeric safety
    double s2 = std::sqrt(std::max(0.0, 1.0 - c2 * c2));
    if (!elbow_up) s2 = -s2;                           // branch selection
    double q2 = std::atan2(s2, c2);

    // q1 via two-atan form (stable near singularities)
    const double k1 = L1 + L2 * c2;
    const double k2 = L2 * s2;
    double q1 = std::atan2(y_m, x_m) - std::atan2(k2, k1);

    // --- Prismatic d3 from world Z (keep your convention) ---
    d3_m = z_m - pedestalZ_;        //might be wrong
    d3_m = std::max(D3_MIN_, std::min(D3_MAX_, d3_m));

    // --- Wrist: residual yaw ---
    const double theta_rad = deg2rad(theta_deg);
    double q4 = theta_rad - (q1 + q2);

    // --- Normalize angles to [-pi, pi) to avoid flips in UI/renderer ---
    auto norm_pi = [](double a) {
        a = std::fmod(a + CV_PI, 2.0 * CV_PI);
        if (a < 0) a += 2.0 * CV_PI;
        return a - CV_PI;
        };
    q1 = norm_pi(q1);
    q2 = norm_pi(q2);
    q4 = norm_pi(q4);

    // --- Convert to degrees for UI/trackbars ---
    q1_deg = rad2deg(q1);
    q2_deg = rad2deg(q2);
    q4_deg = rad2deg(q4);

    // --- Reachability checks (with small tolerance) ---
    const double r_min = std::fabs(L1 - L2) - 1e-6;
    const double r_max = (L1 + L2) + 1e-6;
    const bool reachable_xy = (r >= r_min) && (r <= r_max);
    const bool reachable_z = (z_m >= pedestalZ_ + D3_MIN_ - 1e-6) &&
        (z_m <= pedestalZ_ + D3_MAX_ + 1e-6);

    // If target is practically at the origin, keep q1 stable
    if (r < EPS) {
        // keep previous q1 by not forcing a particular solution here;
        // caller can ignore or smooth as needed.
    }

    return reachable_xy && reachable_z;
}



void CRobot::start_linear_anim()
{
    // Stop the other animation if running to avoid conflicts
    anim_running_ = false;

    lin_anim_running_ = true;
    lin_phase_ = 0;
    lin_theta_accum_deg_ = 0.0;

    // Cache the EE pose at the start so we can return to it later
    start_ex_mm_ = static_cast<double>(ee_x_mm_);
    start_ey_mm_ = static_cast<double>(ee_y_mm_);
    start_ez_mm_ = static_cast<double>(ee_z_mm_);     // travel: 150..0
    start_eth_deg_ = static_cast<double>(ee_theta_deg_);

    lin_ret_t_ = 0.0;        // reset for the eventual return
}


void CRobot::stop_linear_anim()
{
    lin_anim_running_ = false;
    lin_phase_ = 0;
    lin_theta_accum_deg_ = 0.0;
}


// Returns true while running, false when finished
bool CRobot::step_linear_anim(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (!lin_anim_running_) return false;

    // Current EE readouts are kept up-to-date by your draw paths
    double ex_mm = static_cast<double>(ee_x_mm_);
    double ey_mm = static_cast<double>(ee_y_mm_);
    double ez_mm = static_cast<double>(ee_z_mm_);       // travel: 150(top) -> 0(bottom)
    double eth = static_cast<double>(ee_theta_deg_);

    auto clamp = [](double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); };

    // Helper to "try set" a new IK target in one shot (mm/deg -> joints)
    auto try_target = [&](double tx_mm, double ty_mm, double tz_travel_mm, double t_theta_deg) -> bool
        {
            // Convert to IK inputs
            const double xt = tx_mm / 1000.0;
            const double yt = ty_mm / 1000.0;

            // Slider semantics: z_travel_mm = 150 - d3*1000
            double d3_from_travel = (150.0 - tz_travel_mm) / 1000.0;     // meters
            d3_from_travel = clamp(d3_from_travel, D3_MIN_, D3_MAX_);
            const double zt = pedestalZ_ + d3_from_travel;

            double q1d, q2d, d3d, q4d;
            if (ikine(xt, yt, zt, t_theta_deg, q1d, q2d, d3d, q4d, elbow_up_toggle_))
            {
                // Write back to joint sliders (q3_deg holds wrist)
                q1_deg = q1d;
                q2_deg = q2d;
                d3_m = d3d;
                q3_deg = q4d;
                return true;
            }
            return false;
        };

    // Workspace max radius in mm (small epsilon to avoid edge singularities)
    const double r_max_mm = 1000.0 * (L1_ + L2_) - 0.5;
    const double r_now_mm = std::hypot(ex_mm, ey_mm);

    switch (lin_phase_)
    {
        // 0) Move along +x direction from 300 -> 100 mm (your spec says 'from 300-100')
    case 0:
    {
        double target_x = ex_mm - lin_step_mm_;
        target_x = std::max(100.0, target_x);
        if (try_target(target_x, 0.0, ez_mm, eth))
        {
            ex_mm = target_x;
            ey_mm = 0.0;
            if (std::abs(ex_mm - 100.0) <= 1e-6) lin_phase_ = 1;
        }
        else
        {
            // If IK fails at this edge, just advance the phase
            lin_phase_ = 1;
        }
        break;
    }

    // 1) Move along +y from 0 -> y_max at x = 100 mm
    case 1:
    {
        const double x_fixed = 100.0;
        const double y_max = std::sqrt(std::max(0.0, r_max_mm * r_max_mm - x_fixed * x_fixed));
        double target_y = std::min(y_max, ey_mm + lin_step_mm_);

        if (try_target(x_fixed, target_y, ez_mm, eth))
        {
            ex_mm = x_fixed;
            ey_mm = target_y;
            if (ey_mm >= y_max - 1e-6) lin_phase_ = 2;
        }
        else
        {
            // If we can?t step further, go to next phase
            lin_phase_ = 2;
        }
        break;
    }

    // 2) Move "down" in z (i.e., reduce travel to 0 mm)
    // 2) Move "down" in z (i.e., reduce travel to 0 mm)
    case 2:
    {
        double target_z_travel = std::max(0.0, ez_mm - lin_step_mm_);
        if (try_target(ex_mm, ey_mm, target_z_travel, eth))
        {
            ez_mm = target_z_travel;

            // if we reached the bottom, prepare return movement
            if (ez_mm <= 1e-6)
            {
                // snapshot where we are now (the "from" pose for return)
                ret_from_ex_mm_ = ex_mm;
                ret_from_ey_mm_ = ey_mm;
                ret_from_ez_mm_ = ez_mm;           // should be ~0
                ret_from_eth_deg_ = eth;

                lin_ret_t_ = 0.0;                    // reset interpolation
                lin_phase_ = 4;                      // jump to RETURN phase
            }
        }
        else
        {
            // could not step further; still attempt a return
            ret_from_ex_mm_ = ex_mm;
            ret_from_ey_mm_ = ey_mm;
            ret_from_ez_mm_ = ez_mm;
            ret_from_eth_deg_ = eth;

            lin_ret_t_ = 0.0;
            lin_phase_ = 4;
        }
        break;
    }

    // 3) Rotate wrist +360 deg at the final position
    case 3:
    {
        double target_theta = eth + lin_step_deg_;
        if (target_theta > 180.0) target_theta -= 360.0; // keep UI nice; IK only needs relative yaw
        if (try_target(ex_mm, ey_mm, ez_mm, target_theta))
        {
            lin_theta_accum_deg_ += lin_step_deg_;
            // done?
            if (lin_theta_accum_deg_ >= 360.0 - 1e-6)
            {
                stop_linear_anim();
                return false;
            }
        }
        else
        {
            // If the wrist step fails (shouldn?t), end gracefully
            stop_linear_anim();
            return false;
        }
        break;
    }
    // 4) Return to the original EE pose across x, y, z, and theta
    case 4:
    {
        // progress 0 -> 1
        lin_ret_t_ = std::min(1.0, lin_ret_t_ + lin_ret_t_step_);

        // linear blend in task space: target = (1 - t)*from + t*start
        const double tx_mm = (1.0 - lin_ret_t_) * ret_from_ex_mm_ + lin_ret_t_ * start_ex_mm_;
        const double ty_mm = (1.0 - lin_ret_t_) * ret_from_ey_mm_ + lin_ret_t_ * start_ey_mm_;
        const double tz_tr = (1.0 - lin_ret_t_) * ret_from_ez_mm_ + lin_ret_t_ * start_ez_mm_;
        // optional: also blend wrist angle
        const double tth_deg = (1.0 - lin_ret_t_) * ret_from_eth_deg_ + lin_ret_t_ * start_eth_deg_;

        if (try_target(tx_mm, ty_mm, tz_tr, tth_deg))
        {
            if (lin_ret_t_ >= 1.0 - 1e-9)
            {
                // snap to exact start (one last IK solve to eliminate tiny drift)
                (void)try_target(start_ex_mm_, start_ey_mm_, start_ez_mm_, start_eth_deg_);

                stop_linear_anim();
                return false;
            }
        }
        else
        {
            // IK failed mid-return: end gracefully
            stop_linear_anim();
            return false;
        }
        break;
    }
    default:
        stop_linear_anim();
        return false;
    }

    return true;
}

// --- NEW: drive joints toward ArUco marker pose when track_cube_ is ON ---
// Robot.h should have:
/// bool track_cube_ = false;   // toggle set by the "Track Cube" button
/// double tool_z_offset_ = 0.00;   // meters, if your tool tip sits above EE frame
// Robot.h should have:
/// bool track_cube_ = false;   // toggle set by the "Track Cube" button
/// double tool_z_offset_ = 0.00;   // meters, if your tool tip sits above EE frame

// NOTE: requires the declaration already in Robot.h:
// bool maybe_track_cube(CCameraReal& cam,
//     double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m,
//     int marker_id = 50);

static inline double clamp01(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}




bool CRobot::maybe_track_cube(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m,
    int marker_id)
{
    if (!track_cube_) return false;

    // 1) Marker pose in BOARD/WORLD frame (B->M)
    cv::Vec3d rvec_BM, tvec_BM;
    if (!cam.get_marker_pose_in_board(marker_id, rvec_BM, tvec_BM)) {
        return false;
    }

    // 2) Desired yaw = marker yaw about board Z
    cv::Mat Rbm; cv::Rodrigues(rvec_BM, Rbm);
    const double theta_deg =
        std::atan2(Rbm.at<double>(1, 0), Rbm.at<double>(0, 0)) * 180.0 / CV_PI;

    // 3) Desired position: EXACTLY the marker origin in board frame
    //    (on-board => z=0; lifted => z>0). If your tool tip is offset above
    //    the EE frame, add tool_z_offset_.
    const double xt = tvec_BM[0];
    const double yt = tvec_BM[1];
    // BEFORE:
    // const double zt = pedestalZ_ + (tvec_BM[2] + tool_z_offset_);

    // AFTER: feed IK the z that yields the correct d3 for AR/world FK
    // flip the sign so raising the marker raises the EE target
    const double zt = pedestalZ_ + 0.11 + (tvec_BM[2] + tool_z_offset_);



    // 4) IK -> joints (UI convention: q3 slider is wrist, i.e., fkine's q4)
    double q1d, q2d, d3d, q4d;
    if (!ikine(xt, yt, zt, theta_deg, q1d, q2d, d3d, q4d, elbow_up_toggle_)) {
        return false;
    }

    q1_deg = q1d;
    q2_deg = q2d;
    q3_deg = q4d; // UI's q3 holds wrist (fkine q4)
    d3_m = clamp01(d3d, D3_MIN_, D3_MAX_);

    // 5) Mirror EE readouts (for UI display)
    cv::Mat T = fkine(q1_deg, q2_deg, d3_m, q3_deg);
    update_ee_readout_from_T(T, d3_m);
    show_applied_pose_ = true;

    return true;
}

// Call once per frame to advance any running animations.
// Skips when tracking a cube so your robot does not fight the tracker.
void CRobot::tick_animations(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (track_cube_) return; // do not animate while tracking a marker

    bool advanced = false;

    if (lin_anim_running_)
        advanced |= step_linear_anim(q1_deg, q2_deg, q3_deg, d3_m);

    if (anim_running_)
        advanced |= step_anim_partB(q1_deg, q2_deg, q3_deg, d3_m);

    if (traj_running_)
        advanced |= step_joint_traj(q1_deg, q2_deg, q3_deg, d3_m);

    if (advanced)
    {
        cv::Mat T = fkine(q1_deg, q2_deg, d3_m, q3_deg);
        update_ee_readout_from_T(T, d3_m);
    }
}



/////////////////lab 7 trajectory functions/////////////////////
Poly5Coeffs CRobot::compute_poly5(double q0, double q1,
    double qd0, double qd1) const
{
    Poly5Coeffs c;

    // We are using normalized time T = 1
    c.F = q0;
    c.E = qd0;
    c.D = 0.0;

    c.A = 3.0 * (-qd0 - qd1 - 2.0 * q0 + 2.0 * q1);
    c.B = 8.0 * qd0 + 7.0 * qd1 + 15.0 * q0 - 15.0 * q1;
    c.C = 2.0 * (-3.0 * qd0 - 2.0 * qd1 - 5.0 * q0 + 5.0 * q1);

    return c;
}
std::vector<double> CRobot::jtraj_scalar(double q0, double q1,
    double qd0, double qd1,
    int nSteps) const
{
    std::vector<double> q;
    q.reserve(nSteps);

    if (nSteps < 2)
    {
        // Degenerate case: just return the start
        q.push_back(q0);
        return q;
    }

    Poly5Coeffs c = compute_poly5(q0, q1, qd0, qd1);

    for (int k = 0; k < nSteps; ++k)
    {
        double t = 0.0;
        if (nSteps > 1)
        {
            t = static_cast<double>(k) /
                static_cast<double>(nSteps - 1);
        }

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        double s = c.A * t5 + c.B * t4 + c.C * t3
            + c.D * t2 + c.E * t + c.F;

        q.push_back(s);
    }

    return q;
}
void CRobot::start_joint_traj(const double q0[4], const double q1[4],
    int nSteps)
{
    // Stop other animations / tracking so they do not fight jtraj
    anim_running_ = false;
    lin_anim_running_ = false;
    track_cube_ = false;

    traj_q1_ = jtraj_scalar(q0[0], q1[0], 0.0, 0.0, nSteps);
    traj_q2_ = jtraj_scalar(q0[1], q1[1], 0.0, 0.0, nSteps);
    traj_q3_ = jtraj_scalar(q0[2], q1[2], 0.0, 0.0, nSteps);
    traj_d3_ = jtraj_scalar(q0[3], q1[3], 0.0, 0.0, nSteps);

    traj_nsteps_ = nSteps;
    traj_step_ = 0;
    traj_running_ = true;
}

bool CRobot::step_joint_traj(double& q1_deg, double& q2_deg,
    double& q3_deg, double& d3_m)
{
    if (!traj_running_ || traj_step_ >= traj_nsteps_) {
        traj_running_ = false;
        return false;
    }

    q1_deg = traj_q1_[traj_step_];
    q2_deg = traj_q2_[traj_step_];
    q3_deg = traj_q3_[traj_step_];
    d3_m = traj_d3_[traj_step_];

    ++traj_step_;
    if (traj_step_ >= traj_nsteps_) {
        traj_running_ = false;
    }
    return true;
}

void CRobot::start_lab7_home_target_traj(double& q1_deg, double& q2_deg,
    double& q3_deg, double& d3_m)
{
    // Lab 7 spec:
    // Home:  (0,0,0,0)
    // PoseB: (-180, 90, 90, 0.15m  == 15cm)

    const double home[4] = { 0.0,   0.0,  0.0, 0.0 };
    const double poseB[4] = { -180.0, 90.0, 90.0, 0.15 };

    // Current joints at call time
    double cur[4] = { q1_deg, q2_deg, q3_deg, d3_m };

    auto dist2 = [](const double a[4], const double b[4]) {
        double s = 0.0;
        for (int i = 0; i < 4; ++i) {
            double d = a[i] - b[i];
            s += d * d;
        }
        return s;
        };

    const double d_home = dist2(cur, home);
    const double d_poseB = dist2(cur, poseB);

    double q_from[4];
    double q_to[4];

    // Always start from current joints
    for (int i = 0; i < 4; ++i) q_from[i] = cur[i];

    // If we are closer to home, go to Pose B; otherwise go back home.
    const double* dest = (d_home <= d_poseB) ? poseB : home;
    for (int i = 0; i < 4; ++i) q_to[i] = dest[i];

    // 200 samples gives a nice smooth motion
    start_joint_traj(q_from, q_to, 200);
}


void CRobot::start_traj_to_marker(CCameraReal& cam, int marker_id,
    double& q1_deg, double& q2_deg,
    double& q3_deg, double& d3_m)
{
    cv::Vec3d rvec_BM, tvec_BM;
    if (!cam.get_marker_pose_in_board(marker_id, rvec_BM, tvec_BM)) {
        return; // no marker this frame
    }

    // Marker yaw about board Z
    cv::Mat Rbm;
    cv::Rodrigues(rvec_BM, Rbm);
    double theta_deg =
        std::atan2(Rbm.at<double>(1, 0), Rbm.at<double>(0, 0)) * 180.0 / CV_PI;

    // *** IMPORTANT: use the SAME zt as maybe_track_cube ***
    const double xt = tvec_BM[0];
    const double yt = tvec_BM[1];
    const double zt = pedestalZ_ + 0.11 + (tvec_BM[2] + tool_z_offset_);

    double q1_t, q2_t, d3_t, q4_t;
    if (!ikine(xt, yt, zt, theta_deg,
        q1_t, q2_t, d3_t, q4_t, elbow_up_toggle_))
    {
        return; // unreachable, bail out
    }

    // From current joints to target joints (UI convention: q3 is wrist)
    double q_from[4] = { q1_deg, q2_deg, q3_deg, d3_m };
    double q_to[4] = { q1_t,   q2_t,   q4_t,   d3_t };

    start_joint_traj(q_from, q_to, 200);
}

