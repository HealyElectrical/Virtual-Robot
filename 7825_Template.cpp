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
    // --- Window & cvui setup ---
    const char* kWin = "Lab 4 - Real Camera";
    cv::namedWindow(kWin);
    cvui::init(kWin);  // IMPORTANT: only do CVUI_IMPLEMENTATION in ONE .cpp (your template already does)

    // --- Camera ---
    CCameraReal cam;                 // uses fallback intrinsics if XML missing
    cam.set_resolution(1280, 720);

    // --- UI state ---
    bool trackBoard = true;          // checkbox state
    // Show pose in millimetres to match the lab / your Lab 3 UI style
    double x_mm = 0.0, y_mm = 0.0, z_mm = 0.0;

    cv::Mat frame;
    char key = -1;

    while (key != 'q' && key != 27) // q or ESC
    {
        cam.get_image(frame);
        if (frame.empty()) { key = (char)cv::waitKey(1); continue; }

        // Run pose; your CameraReal draws axes itself
        cam.detectBoardPose(frame);

        // If “Track Board” is ON and we have a pose, update sliders from pose
        if (trackBoard && cam.have_pose) {
            // cam.tvec_CB is in meters; convert to mm for the sliders
            x_mm = cam.tvec_CB[0] * 1000.0;
            y_mm = cam.tvec_CB[1] * 1000.0;
            z_mm = cam.tvec_CB[2] * 1000.0;
        }

        // ---- cvui panel on top of the camera image ----
        int px = 10, py = 10;
        cvui::window(frame, px, py, 260, 190, "Track / Pose");
        px += 10; py += 30;

        cvui::checkbox(frame, px, py, "Track Board (link X/Y/Z)", &trackBoard);
        py += 30;

        // trackbars: min/max in mm (tweak ranges to your scene)
        cvui::text(frame, px, py - 8, "X (mm)");
        cvui::trackbar(frame, px, py, 240, &x_mm, -1000.0, 1000.0);
        py += 50;

        cvui::text(frame, px, py - 8, "Y (mm)");
        cvui::trackbar(frame, px, py, 240, &y_mm, -1000.0, 1000.0);
        py += 50;

        cvui::text(frame, px, py - 8, "Z (mm)");
        cvui::trackbar(frame, px, py, 240, &z_mm, 0.0, 2000.0);

        // IMPORTANT: refresh cvui internal state before imshow
        cvui::update();

        // show
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
