// CCameraReal.h
#pragma once

#include <string>
#include <vector>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

class CCameraReal {
public:
   CCameraReal();
   ~CCameraReal();

   void start_webcam(int webcam_id);
   void set_resolution(int w, int h);
   void get_image(cv::Mat& im);

   bool load_camparam(const std::string& filename, cv::Mat& cam, cv::Mat& dist);
   bool save_camparam(const std::string& filename, cv::Mat& cam, cv::Mat& dist);

   // Detect ChArUco board and estimate board pose in camera frame.
   // Also fills Lab7 marker pose maps (marker_rvec_BM_ / marker_tvec_BM_).
   bool detectBoardPose(cv::Mat& frame);

   void transform_to_image(cv::Mat pt3d_mat, cv::Point2f& pt2d);
   void transform_to_image(std::vector<cv::Mat> pts3d_mat, std::vector<cv::Point2f>& pts2d);

   // Starts an interactive Charuco capture and writes intrinsics/distortion to webcam_param.xml
   void calibrate_board(int cam_id);

   // Pose state (board in camera frame)
   bool     have_pose = false;
   cv::Vec3d rvec_CB{ 0, 0, 0 };
   cv::Vec3d tvec_CB{ 0, 0, 0 };

   ///////////////// lab 6 /////////////////

   // Draw a cube on top of a specific ArUco marker (default: ID 50).
   // markerLen and cubeH are in meters (use your real values).
   bool draw_cube_on_marker(cv::Mat& frame, int marker_id = 50,
      float markerLen = 0.02042f, float cubeH = 0.03f);

   // Returns the pose of the requested marker in the board (world) frame.
   // Lab 7 version: requires detectBoardPose() has been called successfully
   // and that marker_id was visible in that call.
   bool get_marker_pose_in_board(int marker_id,
      cv::Vec3d& rvec_BM, cv::Vec3d& tvec_BM) const;

   // Cache the last detected marker pose in CAMERA frame (used by cube smoothing)
   bool      last_marker_ok_ = false;
   int       last_marker_id_ = -1;
   cv::Vec3d last_marker_rvec_C_ = cv::Vec3d(0, 0, 0);
   cv::Vec3d last_marker_tvec_C_ = cv::Vec3d(0, 0, 0);

   ///////////////// lab 7 /////////////////

   // Debug helper: draw IDs for all visible ArUco markers
   bool draw_marker_ids(cv::Mat& frame);

private:
   int _webcam_id = 0;
   cv::VideoCapture _vid_webcam;

   bool _draw_markers = true; // fixes "_draw_markers undefined"

   cv::Mat _cam_webcam_intrinsic;   // K
   cv::Mat _cam_webcam_dist_coeff;  // D

   ///////////////// lab 6 (private) /////////////////

   // Marker-50 pose cache (if you still use it)
   bool     m50_have_cache_ = false;
   cv::Vec3d m50_rvec_cache_{ 0,0,0 };
   cv::Vec3d m50_tvec_cache_{ 0,0,0 };
   int      m50_miss_count_ = 0;
   // How many consecutive missed frames we will keep drawing the last pose
   int      m50_miss_grace_ = 5;
   // Low-pass smoothing factor [0..1], higher = snappier
   double   m50_alpha_ = 0.6;

   ///////////////// lab 7 (private) /////////////////

private:
   // Optional: keep a copy of the last board frame
   cv::Mat last_board_frame_;
   bool    have_board_frame_ = false;

   // NEW for Lab 7:
   // Marker poses in BOARD frame from the last successful detectBoardPose().
   // Key = ArUco marker ID (e.g., 50, 60, 70).
   std::map<int, cv::Vec3d> marker_rvec_BM_;
   std::map<int, cv::Vec3d> marker_tvec_BM_;
};
