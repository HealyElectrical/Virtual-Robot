////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated September 26, 2022
// Glen Healy - Oct 2025 - Updated for C++14
////////////////////////////////////////////////////////////////
#include "stdafx.h"

using namespace std;
using namespace cv;

using namespace dnn;
using namespace aruco;

// Add simple GUI elements
#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "Robot.h"

void lab1()
{
    // MATLAB
}

void lab2()
{
    // MATLAB
}

void lab3(int cam_id)
{
    char exit_key = -1;
    CRobot robot;

    robot.create_simple_robot();

    while (exit_key != 'q')
    {
        robot.draw_simple_robot();
        exit_key = waitKey(10);
    }
}


void lab4(int cam_id)
{
    const char* kWin = "Lab 4 - Real Camera";
    cv::namedWindow(kWin);
    cvui::init(kWin);

    CCameraReal cam;
    cam.set_resolution(1280, 720);

    // size robot so one cube = one checker square (meters)
    CRobot robot;
    robot.create_simple_robot(0.0337f);   // your square length

    // UI state
    bool trackBoard = true;
    bool showOverlay = true;   // NEW: draw robot?
    bool rotateRobot = false;  // NEW: spin?
    double x_mm = 0, y_mm = 0, z_mm = 0;

    double angle_deg = 0.0;     // rotation state
    double speed_deg = 0.8;     // degrees per frame (adjust in UI)

    cv::Mat frame; char key = -1;
    while (key != 'q' && key != 27)
    {
        cam.get_image(frame);
        if (frame.empty()) { key = (char)cv::waitKey(1); continue; }

        cam.detectBoardPose(frame);

        // Overlay: optional, and rotation optional
        if (cam.have_pose && showOverlay) {
            if (rotateRobot) {
                robot.draw_rotating_robot_on_real(frame, cam, angle_deg);
                angle_deg = std::fmod(speed_deg + speed_deg, 360.0);
            }
            else {
                // static (just reuse rotating draw with angle 0)
                robot.draw_rotating_robot_on_real(frame, cam, 0.0);
            }
        }

        if (trackBoard && cam.have_pose) {
            x_mm = cam.tvec_CB[0] * 1000.0;
            y_mm = -cam.tvec_CB[1] * 1000.0;
            z_mm = cam.tvec_CB[2] * 1000.0;
        }

        // ---- UI overlay ----
        cvui::context(kWin);     // important when you also init cvui elsewhere

        int px = 10, py = 10;
        cvui::window(frame, px, py, 260, 250, "Track / Pose");
        px += 10; py += 30;

        cvui::checkbox(frame, px, py, "Track Board (link X/Y/Z)", &trackBoard); py += 30;

        cvui::text(frame, px, py - 8, "X (mm)");
        cvui::trackbar(frame, px, py, 240, &x_mm, -1000.0, 1000.0); py += 50;

        cvui::text(frame, px, py - 8, "Y (mm)");
        cvui::trackbar(frame, px, py, 240, &y_mm, -1000.0, 1000.0); py += 50;

        cvui::text(frame, px, py - 8, "Z (mm)");
        cvui::trackbar(frame, px, py, 240, &z_mm, 0.0, 2000.0);     py += 50;

        // Robot panel
        int rx = 290, ry = 10;
        cvui::window(frame, rx, ry, 260, 120, "Robot Overlay");
        rx += 10; ry += 30;

        cvui::checkbox(frame, rx, ry, "Show robot overlay", &showOverlay); ry += 30;
        cvui::checkbox(frame, rx, ry, "Rotate robot (Z)", &rotateRobot); ry += 30;

        cvui::text(frame, rx, ry - 8, "Speed (deg/frame)");
        cvui::trackbar(frame, rx, ry, 240, &speed_deg, 0.0, 5.0);

        cvui::update();
        cv::imshow(kWin, frame);
        key = (char)cv::waitKey(1);
    }
    cv::destroyWindow(kWin);
}

void lab4calibration(int cam_id)
{
    // Simple wrapper around your CCameraReal::calibrate_board
    // It opens its own window and handles the capture loop.
    CCameraReal cam;
    cam.set_resolution(1280, 720);     // optional
    cam.calibrate_board(cam_id);       // interactive: press 'c' to capture, ESC to finish

    // Make sure any windows created during calibration are closed before returning
    cv::destroyAllWindows();
}



/*void lab5(int cam_id)
{
    char exit_key = -1;
    CRobot robot;

    while (exit_key != 'q')
    {
        robot.draw();

        exit_key = waitKey(10);
    }
}*/
/*void lab5(int cam_id)
{
    CCameraReal cam;
    CRobot robot;

    double q1_deg = 0.0;
    double q2_deg = 0.0;
    double q3_deg = 0.0;
    double d3_m = 0.0;

    const std::string windowName = CANVAS_NAME;
    cv::namedWindow(windowName);
    cvui::init(windowName);

    char key = -1;
    while (key != 'q' && key != 27)
    {
        cv::Mat frame;
        cam.get_image(frame);
        if (frame.empty()) continue;

        // detect ChArUco pose
        cam.detectBoardPose(frame);

        // use camera feed as robot background
        frame.copyTo(robot.canvas());

        // draw SCARA robot (includes trackbars + reset button)
        robot.draw_scara(q1_deg, q2_deg, q3_deg, d3_m);

        cvui::update();
        cv::imshow(windowName, robot.canvas());
        key = (char)cv::waitKey(1);
    }

    cv::destroyAllWindows();
}*/


void lab5(int /*cam_id*/)
{
    CRobot robot;
    CCameraReal cam;

    // Anchor the robot on the board/world (AR path uses this)
    robot.set_world_anchor(cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0));

    // Joints
    double q1_deg = 0.0, q2_deg = 0.0, q3_deg = 0.0, d3_m = 0.0;

    char key = -1;
    while (key != 'q' && key != 27)
    {
        // One call that:
        //  - pulls a live frame (if AR mode),
        //  - preserves last-known background if a frame/pose is missing,
        //  - draws AR or Virtual depending on the UI toggle,
        //  - updates the EE pose mirror and runs the Part B animation if enabled.
        robot.draw_scara_dispatch(cam, q1_deg, q2_deg, q3_deg, d3_m);

        key = (char)cv::waitKey(1);
    }

    cv::destroyWindow(CANVAS_NAME);
}






// Lab 6 entry point
/*void lab6(int cam_id)
{
    CRobot robot;

    // Use your existing default ctor, then switch to the cam_id we want.
    CCameraReal cam;                     // calls start_webcam(0) + set_resolution(1280,720)
    cam.start_webcam(cam_id);            // switch to the requested camera
    cam.set_resolution(1280, 720);       // optional, in case you want to enforce it again

    // Place robot base in the world (board) frame (adjust if needed)
    robot.set_world_anchor(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));

    double q1_deg = 0, q2_deg = 0, q3_deg = 0, d3_m = 0;

    for (;;)
    {
        // draw_scara_dispatch() will:
        //  - grab a frame via cam.get_image()
        //  - call cam.detectBoardPose(...)
        //  - draw AR or Virtual depending on view_mode_
        robot.draw_scara_dispatch(cam, q1_deg, q2_deg, q3_deg, d3_m);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
    }
}*/

void lab6(int cam_id)
{
    CRobot robot;
    robot.set_view_mode(ViewMode::AR);   // AR so cube & SCARA appear on live video

    CCameraReal cam;
    cam.start_webcam(cam_id);
    cam.set_resolution(1280, 720);

    robot.set_world_anchor(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));

    double q1_deg = 0, q2_deg = 0, q3_deg = 0, d3_m = 0;

    for (;;)
    {
        robot.draw_scara_dispatch(cam, q1_deg, q2_deg, q3_deg, d3_m);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;

        // (Optional) hook keyboard to tweak joints here
    }
}




// In main.cpp (or wherever your other lab functions live)

void lab7(int cam_id)
{
    // --- 1. Create robot and camera ---
    CRobot robot;

    // Start in AR so you can see cube + SCARA on the live video.
    // You can still switch to VIRTUAL in the right-hand panel.
    robot.set_view_mode(ViewMode::AR);

    CCameraReal cam;
    cam.start_webcam(cam_id);
    cam.set_resolution(1280, 720);

    // Robot base anchored at the ChArUco board origin (same as Lab 6).
    // If you later want an offset, change these.
    robot.set_world_anchor(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));

    // Joint state (degrees, degrees, degrees, meters)
    double q1_deg = 0.0;
    double q2_deg = 0.0;
    double q3_deg = 0.0;    // your "wrist" (fkine q4)
    double d3_m = 0.0;    // prismatic

    // --- 2. Main loop ---
    for (;;)
    {
        // This call:
        //  - grabs the camera frame in AR mode
        //  - draws cube on marker 50
        //  - runs tick_animations()  (Part B, linear IK, AND your new jtraj)
        //  - draws the SCARA in AR or Virtual, and the UI panel
        robot.draw_scara_dispatch(cam, q1_deg, q2_deg, q3_deg, d3_m);

        int key = cv::waitKey(1) & 0xFF;

        // Quit
        if (key == 27 || key == 'q')
            break;

        // --- OPTIONAL: keyboard shortcuts for Lab 7 ---

        //  h  : joint-space jtraj between Home and Pose B
        if (key == 'h')
        {
            robot.start_lab7_home_target_traj(q1_deg, q2_deg, q3_deg, d3_m);
        }

        //  m  : joint-space jtraj from current pose to marker 50
        if (key == 'm')
        {
            robot.start_traj_to_marker(cam, 50, q1_deg, q2_deg, q3_deg, d3_m);
        }

        // You can also switch view mode from keyboard if you want:
        if (key == 'a') robot.set_view_mode(ViewMode::AR);
        if (key == 'v') robot.set_view_mode(ViewMode::Virtual);
    }

    cv::destroyAllWindows();
}

/*void lab7(int cam_id)
{
    CRobot robot;
    robot.set_view_mode(ViewMode::AR);

    CCameraReal cam;
    cam.start_webcam(cam_id);
    cam.set_resolution(1280, 720);

    robot.set_world_anchor(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0));

    double q1_deg = 0.0;
    double q2_deg = 0.0;
    double q3_deg = 0.0;
    double d3_m = 0.0;

    cv::Mat debug_frame;

    for (;;)
    {
        // Your lab 6/7 AR drawing & trajectories
        robot.draw_scara_dispatch(cam, q1_deg, q2_deg, q3_deg, d3_m);

        // Extra window showing marker IDs for debugging
        cam.get_image(debug_frame);
        if (!debug_frame.empty())
        {
            cam.draw_marker_ids(debug_frame);
            cv::imshow("Marker IDs", debug_frame);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q')
            break;

        if (key == 'h')
            robot.start_lab7_home_target_traj(q1_deg, q2_deg, q3_deg, d3_m);

        if (key == 'm')
            robot.start_traj_to_marker(cam, 50, q1_deg, q2_deg, q3_deg, d3_m);

        if (key == 'a') robot.set_view_mode(ViewMode::AR);
        if (key == 'v') robot.set_view_mode(ViewMode::Virtual);
    }

    cv::destroyAllWindows();
}*/



int main(int argc, char* argv[])
{
    int sel = -1;
    int cam_id = 0;

    while (sel != 0)
    {
        cout << "\n*****************************************************";
        cout << "\n(1) Lab 1 - Coordinate Transforms 2D";
        cout << "\n(2) Lab 2 - Coordinate Transforms 3D";
        cout << "\n(3) Lab 3 - Virtual Camera (Simple Robot)";
        cout << "\n(4) Lab 4 - Camera Calibration (Simple Robot AR)";
        cout << "\n(5) Lab 5 - Forward Kinematics (SCARA Robot)";
        cout << "\n(6) Lab 6 - Inverse Kinematics (SCARA Robot)";
        cout << "\n(7) Lab 7 - Trajectories";
        cout << "\n(8) Lab 4 - Calibration (capture intrinsics)";  // add this line
        cout << "\n(0) Exit";
        cout << "\n>> ";

        cin >> sel;
        switch (sel)
        {
        case 1: lab1(); break;
        case 2: lab2(); break;
        case 3: lab3(cam_id); break;
        case 4: lab4(cam_id); break;
        case 5: lab5(cam_id); break;
        case 6: lab6(cam_id); break;
        case 7: lab7(cam_id); break;
        case 8: lab4calibration(cam_id); break;
        }
    }

    return 1;
}