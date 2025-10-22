#include "stdafx.h"
#include "Robot.h"

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
    float axis_length = 0.05f; // meters

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

    cv::Mat flippedX = coord3d.at(1).clone();
    flippedX.at<float>(0, 0) *= -1;  // flip X direction
    _virtualcam.transform_to_image(flippedX, X);

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
    _canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR;

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

void CRobot::drawBoxReal(cv::Mat& im,
    const std::vector<cv::Mat>& box3d,
    CCameraReal& cam,
    const cv::Scalar& color)
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

cv::Scalar CRobot::chooseColors(int idx)
{
    switch (idx)
    {
    case 0: return cv::Scalar(0, 0, 255);     // Red (BGR)
    case 1: return cv::Scalar(0, 255, 0);     // Green
    case 2: return cv::Scalar(0, 255, 255);   // Yellow
    case 3: return cv::Scalar(255, 0, 255);   // Magenta
    case 4: return cv::Scalar(42, 42, 165);   // Brown-ish
    case 5: return cv::Scalar(255, 100, 0);   // Orange-ish
    default: return cv::Scalar(200, 200, 200);
    }
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

    // Small tool box (you won’t see it yet)
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
void CRobot::draw_scara(double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg)
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
    const float prism_height = 0.15f;   // fixed 15 cm prismatic link

    // --- Helper to make a 2 cm local axis ---
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

    // ===== Forward-kinematic transform chain =====
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    cv::Mat T23 = Tx(arm_len2);
    cv::Mat T3 = T2 * T23;

    // ===== Block placements =====

    // 1️⃣ Base pedestal (red)
    auto block1 = createBox(thickness, thickness, base_height);
    transformPoints(block1, Tz(base_height / 2.0));
    blocks.push_back(block1);

    // 2️⃣ First arm (green) — sits on top of pedestal
    auto block2 = createBox(arm_len1, thickness, thickness);
    transformPoints(block2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block2);

    // 3️⃣ Second arm (blue) — attached at end of first arm
    auto block3 = createBox(arm_len2, thickness, thickness);
    transformPoints(block3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block3);

    // 4️⃣ Prismatic link (yellow) — bottom starts level with bottom of block 3
    auto block4 = createBox(thickness, thickness, prism_height);
    transformPoints(block4, T3 * Tz(-prism_height / 2.0 - d3_m + base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block4);

    // ===== Draw all links =====
    for (size_t i = 0; i < blocks.size(); ++i)
        drawBox(_canvas, blocks[i], chooseColors((int)i));

    // ===== Draw local joint axes =====
    makeSmallAxis(createHT(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0))); // world
    makeSmallAxis(T1);  // joint 1
    makeSmallAxis(T2);  // joint 2
    makeSmallAxis(T3);  // prismatic

    // ===== GUI panels =====
    _virtualcam.update_settings(_canvas);
    update_joint_controls(q1_deg, q2_deg, d3_m, q4_deg);

    // ===== Display =====
    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}
*/
void CRobot::draw_scara(double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg)
{
    // === Reset canvas and update virtual camera ===
    _canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR;
    _virtualcam.update_settings(_canvas);

    std::vector<std::vector<cv::Mat>> blocks;

    // --- Dimensions (meters) ---
    const float base_height = 0.15f;   // pedestal height
    const float arm_len1 = 0.15f;   // first arm length
    const float arm_len2 = 0.15f;   // second arm length
    const float thickness = 0.03f;   // all arms thickness
    const float prism_height = 0.15f;   // prismatic section height

    // --- Helper to make a 2 cm local axis ---
    auto makeSmallAxis = [&](const cv::Mat& T_world)
        {
            auto A = createCoord();
            for (auto& pt : A)
            {
                pt.at<float>(0, 0) *= 0.4f;  // 0.05 → 0.02 m
                pt.at<float>(1, 0) *= 0.4f;
                pt.at<float>(2, 0) *= 0.4f;
            }
            transformPoints(A, T_world);
            drawCoord(_canvas, A);
        };

    // ===== Forward-kinematic chain =====
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    cv::Mat T23 = Tx(arm_len2);
    cv::Mat T3 = T2 * T23;

    // ===== Block placements =====
    // 1️⃣ Base pedestal (red)
    auto block1 = createBox(thickness, thickness, base_height);
    transformPoints(block1, Tz(base_height / 2.0));
    blocks.push_back(block1);

    // 2️⃣ First arm (green) — rests on top of pedestal
    auto block2 = createBox(arm_len1, thickness, thickness);
    transformPoints(block2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block2);

    // 3️⃣ Second arm (blue) — attached to end of arm 1
    auto block3 = createBox(arm_len2, thickness, thickness);
    transformPoints(block3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block3);

    // 4️⃣ Prismatic link (yellow) — lifted by +0.135 m and moves downward with d3
    auto block4 = createBox(thickness, thickness, prism_height);
    transformPoints(block4, T3 * Tz(-prism_height / 2.0 - d3_m + 0.135f + base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block4);

    // ===== Draw all blocks =====
    for (size_t i = 0; i < blocks.size(); ++i)
        drawBox(_canvas, blocks[i], chooseColors((int)i));

    // ===== Draw axes at each block's position =====
    // Axis at top of pedestal (block 1)
    makeSmallAxis(T1 * Tz(base_height / 2.0));
    // Axis at joint 2 (top of arm 1)
    makeSmallAxis(T2 * Tz(base_height / 2.0 + thickness / 2.0));
    // Axis at joint 3 (top of arm 2)
    makeSmallAxis(T3 * Tz(base_height / 2.0 + thickness / 2.0));
    // Axis following prismatic (bottom of yellow block)
    makeSmallAxis(T3 * Tz(-prism_height - d3_m + base_height / 2.0 + thickness / 2.0));

    // ===== GUI panels =====
    _virtualcam.update_settings(_canvas);
    update_joint_controls(q1_deg, q2_deg, d3_m, q4_deg);

    // ===== Display =====
    cvui::update();
    cv::imshow(CANVAS_NAME, _canvas);
}



cv::Mat& CRobot::canvas() { return _canvas; }

void CRobot::update_joint_controls(double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg)
{
    int panel_x = _canvas.cols - 280;
    int panel_y = 100;

    // Background panel
    cv::rectangle(_canvas,
        cv::Point(panel_x - 10, panel_y - 30),
        cv::Point(panel_x + 270, panel_y + 340),
        cv::Scalar(50, 50, 50), cv::FILLED);

    cvui::window(_canvas, panel_x, panel_y, 260, 310, "SCARA Joint Controls");
    panel_x += 10;
    panel_y += 30;

    cvui::text(_canvas, panel_x, panel_y - 8, "q1 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q1_deg, -180.0, 180.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "q2 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q2_deg, -180.0, 180.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "d3 (m)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &d3_m, 0.0, 0.15);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "q4 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q4_deg, -180.0, 180.0);
    panel_y += 50;

    if (cvui::button(_canvas, panel_x + 70, panel_y, 100, 30, "Reset"))
    {
        q1_deg = q2_deg = q4_deg = 0.0;
        d3_m = 0.0;
    }
}

