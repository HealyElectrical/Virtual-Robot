////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated September 26, 2022
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
