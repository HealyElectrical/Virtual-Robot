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

    _lab3 = true;
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
    // 1️⃣ Base pedestal (red)
    auto block1 = createBox(thickness, thickness, base_height);
    transformPoints(block1, T1 * Tz(-base_height / 2.0));
    blocks.push_back(block1);

    // 2️⃣ First arm (green)
    auto block2 = createBox(arm_len1, thickness, thickness);
    transformPoints(block2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block2);

    // 3️⃣ Second arm (blue)
    auto block3 = createBox(arm_len2, thickness, thickness);
    transformPoints(block3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0));
    blocks.push_back(block3);

    // 4️⃣ Prismatic link (yellow)
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

    // === Reset canvas and update virtual camera ===
    // Do NOT overwrite the canvas here — keep whatever background (e.g. live camera) is already in place.
    if (_canvas.empty())
        _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    _lab3 = false;

    // ===== GUI =====
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);
    _virtualcam.update_settings(_canvas);

    // --- Dimensions (meters) ---
    const float thickness = 0.03f;      // link thickness
    const float LINK_LENGTH = 0.15f;    // 15 cm link length
    const float base_height = LINK_LENGTH - thickness / 2;   // pedestal height
    const float arm_len1 = LINK_LENGTH;       // first arm
    const float arm_len2 = LINK_LENGTH;       // second arm
    const float prism_height = LINK_LENGTH;   // prismatic section
    const float offset_z = 0.135f;            // visual lift for prismatic section

    // ===== Forward kinematics =====
    cv::Mat T0 = cv::Mat::eye(4, 4, CV_32F);

    // 1️ Base rotation about origin (pedestal)
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    // 2️ Shoulder rotation (arm)
    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    // 3️ Elbow rotation (arm)
    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;

    // 4️ Prismatic joint (E)
    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // ===== Block placements using Prisms =====
    // Base pedestal (upright prism, z is up)
    auto prism1 = createPrism(thickness, base_height, thickness);
    blocks.push_back(prism1);

    // Prism 2: Lowered by thickness/2
    auto prism2 = createPrism(arm_len1, thickness, thickness);
    transformPoints(prism2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0 - thickness / 2.0));
    blocks.push_back(prism2);

    // Prism 3: Lowered by thickness/2
    auto prism3 = createPrism(arm_len2, thickness, thickness);
    transformPoints(prism3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0 + thickness / 2.0 - thickness / 2.0));
    blocks.push_back(prism3);

    // Prism 4: Lowered by thickness/2 + thickness = 1.5*thickness
    auto prism4 = createPrism(thickness, prism_height, thickness);
    transformPoints(prism4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0 - 2.0f * thickness));
    blocks.push_back(prism4);

    // ===== Draw all blocks as prisms =====
    for (size_t i = 0; i < blocks.size(); ++i)
        drawPrism(_canvas, blocks[i], chooseColors((int)i, RED, YELLOW, GREEN, MAGENTA));


    // ===== Axes at each joint =====
    // at bottom of pedestal
    std::vector<cv::Mat> W = createCoord();
    drawCoord(_canvas, W);
    // at shoulder joint
    cv::Mat fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 0, 0));
    std::vector<cv::Mat> j0 = createCoord();
    transformPoints(j0, fixAxes);
    transformPoints(j0, T1);
    drawCoord(_canvas, j0);
    // at elbow joint
    std::vector<cv::Mat> j1 = createCoord();
    transformPoints(j1, fixAxes);
    transformPoints(j1, T2);
    drawCoord(_canvas, j1);
    // centre of prismatic joint
    fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 90, 0));
    std::vector<cv::Mat> j2 = createCoord();
    transformPoints(j2, fixAxes);
    transformPoints(j2, T3);
    drawCoord(_canvas, j2);
    // end-effector
    std::vector<cv::Mat> E = createCoord();
    transformPoints(E, fixAxes);
    transformPoints(E, T4);
    drawCoord(_canvas, E);


    // ===== Display =====
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
*/

void CRobot::update_joint_controls(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    // --- UI panel position ---
    int panel_x = _canvas.cols - 280;
    int panel_y = 100;

    // Background panel (taller to fit EE pose sliders)
    cv::rectangle(_canvas,
        cv::Point(panel_x - 10, panel_y - 30),
        cv::Point(panel_x + 270, panel_y + 520),
        cv::Scalar(70, 70, 70), cv::FILLED);

    // Window frame
    cvui::window(_canvas, panel_x, panel_y, 260, 490, "SCARA Joint Controls");
    panel_x += 10;
    panel_y += 30;

    // --- Joint sliders ---
    cvui::text(_canvas, panel_x, panel_y - 8, "q1 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q1_deg, -180.0, 180.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "q2 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q2_deg, -180.0, 180.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "q3 (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &q3_deg, -180.0, 180.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "d3 (m)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &d3_m, 0.0, 0.15);
    panel_y += 60;

    // --- Desired EE pose (mm / deg) ---
    cvui::text(_canvas, panel_x, panel_y, "Desired EE Pose");
    panel_y += 20;

    cvui::text(_canvas, panel_x, panel_y - 8, "x_e (mm)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &ee_x_mm_, -300.0, 300.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "y_e (mm)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &ee_y_mm_, -300.0, 300.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "z_e (mm)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &ee_z_mm_, 0.0, 150.0);
    panel_y += 50;

    cvui::text(_canvas, panel_x, panel_y - 8, "theta_e (deg)");
    cvui::trackbar(_canvas, panel_x, panel_y, 240, &ee_theta_deg_, -180.0, 180.0);
    panel_y += 50;

    // Reset (joints + pose)
    if (cvui::button(_canvas, panel_x + 70, panel_y, 100, 30, "Reset"))
    {
        q1_deg = q2_deg = q3_deg = 0.0;
        d3_m = 0.0;
        ee_x_mm_ = ee_y_mm_ = ee_z_mm_ = 0.0;
        ee_theta_deg_ = 0.0;
    }
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

// === REPLACE draw_scara_world with this version ===
void CRobot::draw_scara_world(CCameraReal& cam,
    double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m)
{
    if (!cam.have_pose) {
        // Still show the joint UI even if pose not found
        if (_canvas.empty()) _canvas = cv::Mat::zeros(_image_size, CV_8UC3);
        update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);
        cvui::update();
        cv::imshow(CANVAS_NAME, _canvas);
        return;
    }

    // Ensure we have a canvas (caller typically copied the live frame in)
    if (_canvas.empty()) _canvas = cv::Mat::zeros(_image_size, CV_8UC3);

    // ---------------- Geometry (meters) — identical to virtual draw_scara() ----------------
    const float thickness = 0.03f;
    const float LINK_LENGTH = 0.15f;
    const float base_height = LINK_LENGTH - thickness / 2.0f;
    const float arm_len1 = LINK_LENGTH;
    const float arm_len2 = LINK_LENGTH;
    const float prism_height = LINK_LENGTH;
    const float offset_z = 0.135f;

    // World (board) base
    cv::Mat T0 = T_WB_;
    cv::Mat T01 = Rz_deg(q1_deg) * Tz(base_height / 2.0);
    cv::Mat T1 = T0 * T01;

    cv::Mat T12 = Tx(arm_len1) * Rz_deg(q2_deg);
    cv::Mat T2 = T1 * T12;

    cv::Mat T23 = Tx(arm_len2) * Rz_deg(q3_deg);
    cv::Mat T3 = T2 * T23;

    cv::Mat T34 = Tz(-d3_m);
    cv::Mat T4 = T3 * T34;

    // ---------------- Update EE pose readouts BEFORE drawing UI ----------------
    // x,y,theta from T4; z is prismatic travel: 150 mm at top (d3=0) down to 0 mm as d3 increases.
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

    // ---------------- UI (trackbars, buttons) ----------------
    update_joint_controls(q1_deg, q2_deg, q3_deg, d3_m);

    // ---------------- Prisms with SAME placements as virtual draw_scara() ----------------
    std::vector<std::vector<cv::Mat>> prisms;

    // 1) Base pedestal — centered on base joint
    auto p1 = createPrism(thickness, base_height, thickness);
    transformPoints(p1, T1 * Tz(-base_height / 2.0));
    prisms.push_back(p1);

    // 2) First arm
    auto p2 = createPrism(arm_len1, thickness, thickness);
    transformPoints(p2, T1 * Tx(arm_len1 / 2.0) * Tz(base_height / 2.0));
    prisms.push_back(p2);

    // 3) Second arm
    auto p3 = createPrism(arm_len2, thickness, thickness);
    transformPoints(p3, T2 * Tx(arm_len2 / 2.0) * Tz(base_height / 2.0));
    prisms.push_back(p3);

    // 4) Prismatic
    auto p4 = createPrism(thickness, prism_height, thickness);
    transformPoints(p4, T3 * Tz(-prism_height / 2.0 - d3_m + offset_z + base_height / 2.0 + thickness / 2.0 - 2.0f * thickness));
    prisms.push_back(p4);

    // Draw prisms using real-camera projection
    for (size_t i = 0; i < prisms.size(); ++i) {
        drawPrismReal(_canvas, prisms[i], cam,
            chooseColors((int)i, RED, YELLOW, GREEN, MAGENTA));
    }

    // ---------------- Axes in REAL camera (unchanged placement) ----------------
    // World axes at robot base
    auto W = createCoord();
    transformPoints(W, T0);
    drawCoordReal(_canvas, W, cam);

    // Helper to shrink joint axes
    auto makeSmall = [&](std::vector<cv::Mat>& C)
        {
            for (auto& P : C) {
                P.at<float>(0, 0) *= 0.4f;
                P.at<float>(1, 0) *= 0.4f;
                P.at<float>(2, 0) *= 0.4f;
            }
        };

    // Shoulder joint
    cv::Mat fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 0, 0));
    auto J0 = createCoord(); makeSmall(J0); transformPoints(J0, fixAxes); transformPoints(J0, T1);
    drawCoordReal(_canvas, J0, cam);

    // Elbow joint
    auto J1 = createCoord(); makeSmall(J1); transformPoints(J1, fixAxes); transformPoints(J1, T2);
    drawCoordReal(_canvas, J1, cam);

    // Center of prismatic joint (rotate Y by 90 like virtual)
    fixAxes = createHT(cv::Vec3d(0, 0, (base_height + thickness) / 2.0f), cv::Vec3d(0, 90, 0));
    auto J2 = createCoord(); makeSmall(J2); transformPoints(J2, fixAxes); transformPoints(J2, T3);
    drawCoordReal(_canvas, J2, cam);

    // End-effector axes
    auto E = createCoord(); makeSmall(E); transformPoints(E, fixAxes); transformPoints(E, T4);
    drawCoordReal(_canvas, E, cam);

    // ---------------- Present ----------------
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
