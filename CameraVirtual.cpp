#include "stdafx.h"
#include "CameraVirtual.h"
#include <cmath>
#include "cvui.h"

// --- helpers ---
static inline double deg2rad(double d) { return d * CV_PI / 180.0; }

CCameraVirtual::CCameraVirtual()
{
   // Initialize with a default camera image size 
   init(Size(1000, 600));
}

CCameraVirtual::~CCameraVirtual() {}

void CCameraVirtual::init(Size image_size)
{
   ////////// UI state (cvui trackbars) //////////
   _cam_setting_f = 3;      // focal length in mm (per lab)
   _cam_setting_x = 0;      // mm
   _cam_setting_y = -500;   // mm (start back so robot is in view)
   _cam_setting_z = 0;      // mm
   _cam_setting_roll = 0;      // deg
   _cam_setting_pitch = -90;    // deg (look along +Z)
   _cam_setting_yaw = 0;      // deg

   ////////// Intrinsics //////////
   // pixel size in meters/pixel (4.6 µm)
   _pixel_size = 0.0000046f;    // 4.6e-6 m/px

   // principal point at image center
   _principal_point = Point2f(static_cast<float>(image_size.width) * 0.5f,
      static_cast<float>(image_size.height) * 0.5f);

   calculate_intrinsic();
   calculate_extrinsic();
}

// ====================== LAB 3 ======================

// Intrinsic: K = [ fx 0 cx ; 0 fy cy ; 0 0 1 ],
// fx,fy in pixels. We have f in mm, pixel_size in m/px → convert f to meters first.
void CCameraVirtual::calculate_intrinsic()
{
   const double f_mm = static_cast<double>(_cam_setting_f); // mm
   const double f_m = f_mm * 1e-3;                         // meters
   const double px_m = static_cast<double>(_pixel_size);    // meters/pixel
   const double f_px = f_m / px_m;                          // pixels

   _cam_virtual_intrinsic = Mat::eye(3, 3, CV_64F);
   _cam_virtual_intrinsic.at<double>(0, 0) = f_px;                             // fx
   _cam_virtual_intrinsic.at<double>(1, 1) = f_px;                             // fy
   _cam_virtual_intrinsic.at<double>(0, 2) = static_cast<double>(_principal_point.x); // cx
   _cam_virtual_intrinsic.at<double>(1, 2) = static_cast<double>(_principal_point.y); // cy
}

// Extrinsic: world→camera  X_cam = R * X_world + t,
// with camera center C in world coords → t = -R * C
void CCameraVirtual::calculate_extrinsic()
{
   const double rx = deg2rad(static_cast<double>(_cam_setting_roll));
   const double ry = deg2rad(static_cast<double>(_cam_setting_pitch));
   const double rz = deg2rad(static_cast<double>(_cam_setting_yaw));

   Mat Rx = (Mat_<double>(3, 3) <<
      1, 0, 0,
      0, cos(rx), -sin(rx),
      0, sin(rx), cos(rx)
      );
   Mat Ry = (Mat_<double>(3, 3) <<
      cos(ry), 0, sin(ry),
      0, 1, 0,
      -sin(ry), 0, cos(ry)
      );
   Mat Rz = (Mat_<double>(3, 3) <<
      cos(rz), -sin(rz), 0,
      sin(rz), cos(rz), 0,
      0, 0, 1
      );

   // Yaw→Pitch→Roll
   Mat R = Rz * Ry * Rx;

   // Camera center in world (mm)
   Mat C = (Mat_<double>(3, 1) <<
      static_cast<double>(_cam_setting_x),
      static_cast<double>(_cam_setting_y),
      static_cast<double>(_cam_setting_z));

   Mat t = -R * C; // 3x1

   _cam_virtual_extrinsic = Mat::zeros(3, 4, CV_64F);
   R.copyTo(_cam_virtual_extrinsic(Rect(0, 0, 3, 3)));
   t.copyTo(_cam_virtual_extrinsic.col(3));
}

// Map one 3D point (in mm) to image pixel coords
void CCameraVirtual::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
   CV_Assert((pt3d_mat.rows == 3 || pt3d_mat.rows == 4) && pt3d_mat.cols == 1);

   Mat Xw(4, 1, CV_64F);
   if (pt3d_mat.rows == 3) {
      Xw.at<double>(0, 0) = pt3d_mat.at<double>(0, 0);
      Xw.at<double>(1, 0) = pt3d_mat.at<double>(1, 0);
      Xw.at<double>(2, 0) = pt3d_mat.at<double>(2, 0);
      Xw.at<double>(3, 0) = 1.0;
   }
   else {
      Xw = pt3d_mat.clone();
      Xw.at<double>(3, 0) = 1.0;
   }

   Mat Xc = _cam_virtual_extrinsic * Xw; // 3x1

   const double X = Xc.at<double>(0, 0);
   const double Y = Xc.at<double>(1, 0);
   const double Z = Xc.at<double>(2, 0);

   if (Z <= 1e-9) { pt = Point2f(-1e6f, -1e6f); return; }

   const double fx = _cam_virtual_intrinsic.at<double>(0, 0);
   const double fy = _cam_virtual_intrinsic.at<double>(1, 1);
   const double cx = _cam_virtual_intrinsic.at<double>(0, 2);
   const double cy = _cam_virtual_intrinsic.at<double>(1, 2);

   pt.x = static_cast<float>(fx * (X / Z) + cx);
   pt.y = static_cast<float>(fy * (Y / Z) + cy);
}

void CCameraVirtual::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
   pts2d.clear();
   pts2d.reserve(pts3d_mat.size());
   for (auto& P : pts3d_mat) {
      Point2f q;
      transform_to_image(P, q);
      pts2d.push_back(q);
   }
}

void CCameraVirtual::update_settings(Mat& im)
{
   cvui::window(im, 5, 5, 230, 380, "Virtual Camera Settings");

   int x = 15, y = 35, w = 200, step = 45;

   cvui::trackbar(im, x, y, w, &_cam_setting_f, 1, 20);                 cvui::text(im, x + w + 5, y + 20, "F (mm)");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_x, -1000, 1000); cvui::text(im, x + w + 5, y + 20, "X (mm)");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_y, -1000, 1000); cvui::text(im, x + w + 5, y + 20, "Y (mm)");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_z, -1000, 1000); cvui::text(im, x + w + 5, y + 20, "Z (mm)");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_roll, -180, 180); cvui::text(im, x + w + 5, y + 20, "Roll");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_pitch, -180, 180); cvui::text(im, x + w + 5, y + 20, "Pitch");
   y += step; cvui::trackbar(im, x, y, w, &_cam_setting_yaw, -180, 180); cvui::text(im, x + w + 5, y + 20, "Yaw");
   y += step;
   if (cvui::button(im, x, y, 100, 30, "Reset")) {
      init(im.size());
   }

   calculate_intrinsic();
   calculate_extrinsic();

   putText(im, "cx,cy: (" + to_string((int)_principal_point.x) + "," + to_string((int)_principal_point.y) + ")",
      Point(10, im.rows - 50), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(220, 220, 220), 1);
}
