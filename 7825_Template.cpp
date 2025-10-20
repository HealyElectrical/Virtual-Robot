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

/*void lab4(int cam_id)
{
    // --- Window & cvui setup ---
    const char* kWin = "Lab 4 - Real Camera";
    cv::namedWindow(kWin);
    cvui::init(kWin);  // NOTE: CVUI_IMPLEMENTATION should appear in exactly one .cpp

    // --- Camera ---
    CCameraReal cam;                 // falls back to rough intrinsics if XML is missing
    cam.set_resolution(1280, 720);

    // --- Robot sized to one board square per cube edge ---
    // Must match the Charuco board values you use inside detectBoardPose
    const float boardSquareLen_m = 0.0337f;   // <-- your board's square size in meters
    CRobot robot;
    robot.create_simple_robot(boardSquareLen_m);   // origin at bottom of first box

    // --- UI state ---
    bool trackBoard = true;          // link sliders to live pose
    bool drawRobot = true;          // toggle overlay
    double x_mm = 0.0, y_mm = 0.0, z_mm = 0.0;  // display in mm like Lab 3

    cv::Mat frame;
    char key = -1;

    while (key != 'q' && key != 27) // q or ESC
    {
        cam.get_image(frame);
        if (frame.empty()) { key = (char)cv::waitKey(1); continue; }

        // Detect ChArUco + draw origin axes (inside detectBoardPose)
        cam.detectBoardPose(frame);

        // If “Track Board” is ON and we have a pose, update sliders from pose
        if (trackBoard && cam.have_pose) {
            x_mm = cam.tvec_CB[0] * 1000.0;
            y_mm = cam.tvec_CB[1] * 1000.0;
            z_mm = cam.tvec_CB[2] * 1000.0;
        }

        // --- cvui panel overlay ---
        int px = 10, py = 10;
        cvui::window(frame, px, py, 260, 230, "Track / Pose");
        px += 10; py += 30;

        cvui::checkbox(frame, px, py, "Track Board (link X/Y/Z)", &trackBoard);
        py += 30;

        cvui::text(frame, px, py - 8, "X (mm)");
        cvui::trackbar(frame, px, py, 240, &x_mm, -1000.0, 1000.0);
        py += 50;

        cvui::text(frame, px, py - 8, "Y (mm)");
        cvui::trackbar(frame, px, py, 240, &y_mm, -1000.0, 1000.0);
        py += 50;

        cvui::text(frame, px, py - 8, "Z (mm)");
        cvui::trackbar(frame, px, py, 240, &z_mm, 0.0, 2000.0);
        py += 50;

        cvui::checkbox(frame, px, py, "Draw Robot Overlay", &drawRobot);

        // Draw the Lab-3 robot onto the *real* image when we have a valid pose
        if (drawRobot && cam.have_pose) {
            robot.draw_simple_robot_on_real(frame, cam);
        }

        // cvui housekeeping + show
        cvui::update();
        cv::imshow(kWin, frame);
        key = (char)cv::waitKey(1);
    }

    cv::destroyWindow(kWin);
}
*/

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




void lab5(int cam_id)
{
   char exit_key = -1;
   CRobot robot;

   while (exit_key != 'q')
   {
      robot.draw();

      exit_key = waitKey(10);
   }
}

void lab6(int cam_id)
{
}

void lab7(int cam_id)
{
}

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
      }
   }

   return 1;
}
