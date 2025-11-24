// CameraReal.cpp
#include "stdafx.h"
#include "CameraReal.h"
#include <cstdio>


#include <string>
#include <vector>
#include <iostream>


#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// ArUco/Charuco (OpenCV 4.6+)
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>





using namespace cv;
using namespace cv::aruco;

// Board constants (meters)
static constexpr int   kSquaresX = 5;
static constexpr int   kSquaresY = 7;
static constexpr float kSquareLen = 0.0337f;
static constexpr float kMarkerLen = 0.02042f;


// Build a 4x4 from rvec/tvec
static cv::Mat Rt_to_T(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
   cv::Mat R;
   cv::Rodrigues(rvec, R); // 3x3
   cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
   R.copyTo(T(cv::Rect(0, 0, 3, 3)));
   T.at<double>(0, 3) = tvec[0];
   T.at<double>(1, 3) = tvec[1];
   T.at<double>(2, 3) = tvec[2];
   return T;
}

static void T_to_Rt(const cv::Mat& T, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
   cv::Mat R = T(cv::Rect(0, 0, 3, 3)).clone();
   cv::Rodrigues(R, rvec);
   tvec = cv::Vec3d(T.at<double>(0, 3), T.at<double>(1, 3), T.at<double>(2, 3));
}


CCameraReal::CCameraReal() {
   start_webcam(0);
   set_resolution(1280, 720);

   // Try to load; otherwise use a sane guess
   if (!load_camparam("webcam_param.xml", _cam_webcam_intrinsic, _cam_webcam_dist_coeff)) {
      std::cout << "[lab4] No webcam_param.xml - using rough intrinsics.\n";
      const double fx = 900.0, fy = 900.0, cx = 640.0, cy = 360.0;
      _cam_webcam_intrinsic = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      _cam_webcam_dist_coeff = Mat::zeros(1, 5, CV_64F); // k1 k2 p1 p2 k3
   }
}

CCameraReal::~CCameraReal() {
   _vid_webcam.release();
}

void CCameraReal::start_webcam(int webcam_id) {
   _webcam_id = webcam_id;
   _vid_webcam.release();
   _vid_webcam.open(_webcam_id, CAP_DSHOW);
}

void CCameraReal::set_resolution(int w, int h) {
   if (_vid_webcam.isOpened()) {
      _vid_webcam.set(CAP_PROP_FRAME_WIDTH, w);
      _vid_webcam.set(CAP_PROP_FRAME_HEIGHT, h);
   }
}

void CCameraReal::get_image(Mat& im) {
   if (_vid_webcam.isOpened()) _vid_webcam >> im;
}

bool CCameraReal::load_camparam(const std::string& filename, Mat& cam, Mat& dist) {
   FileStorage fs(filename, FileStorage::READ);
   if (!fs.isOpened()) return false;
   fs["camera_matrix"] >> cam;
   fs["distortion_coefficients"] >> dist;
   return !cam.empty();
}

bool CCameraReal::save_camparam(const std::string& filename, Mat& cam, Mat& dist) {
   FileStorage fs(filename, FileStorage::WRITE);
   if (!fs.isOpened()) return false;
   fs << "camera_matrix" << cam;
   fs << "distortion_coefficients" << dist;
   return true;
}
void CCameraReal::calibrate_board(int /*cam_id*/)
{
   // --- Board settings (USE YOUR REAL SIZES) ---
   const cv::Size board_size(5, 7);
   const float size_aruco_square = kSquareLen;   // 0.0337f, from your file
   const float size_aruco_mark = kMarkerLen;   // 0.02042f

   cv::aruco::DetectorParameters detectorParams;
   cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   cv::aruco::CharucoParameters charucoParams;
   charucoParams.tryRefineMarkers = true;

   cv::aruco::CharucoBoard board(
      cv::Size(board_size.width, board_size.height),
      size_aruco_square, size_aruco_mark, dictionary);

   cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

   // --- Collections ---
   std::vector<cv::Mat>                  allCharucoCorners, allCharucoIds;
   std::vector<std::vector<cv::Point2f>> allImagePoints;
   std::vector<std::vector<cv::Point3f>> allObjectPoints;
   std::vector<cv::Mat>                  allImages;
   cv::Size imageSize;

   // --- Capture loop ---
   for (;;)
   {
      cv::Mat im, draw_im;
      get_image(im);
      if (im.empty()) { cv::waitKey(10); continue; }
      im.copyTo(draw_im);

      cv::Mat currentCharucoCorners, currentCharucoIds;
      std::vector<cv::Point3f> currentObjectPoints;
      std::vector<cv::Point2f> currentImagePoints;

      // Detect ChArUco (detects markers internally)
      detector.detectBoard(im, currentCharucoCorners, currentCharucoIds);

      // Draw for feedback
      if (currentCharucoCorners.total() > 3) {
         cv::aruco::drawDetectedCornersCharuco(draw_im, currentCharucoCorners, currentCharucoIds);
      }

      cv::putText(draw_im,
         "Press 'c' to add frame. ESC to finish & calibrate",
         cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

      cv::imshow("ChArUco Capture", draw_im);
      char key = (char)cv::waitKey(10);

      if (key == 'c' && currentCharucoCorners.total() > 3)
      {
         // Map detected ChArUco to 2D/3D sets
         board.matchImagePoints(currentCharucoCorners, currentCharucoIds,
            currentObjectPoints, currentImagePoints);

         if (currentImagePoints.empty() || currentObjectPoints.empty()) {
            std::cout << "Point matching failed, try again.\n";
            continue;
         }

         std::cout << "Frame captured\n";
         allCharucoCorners.push_back(currentCharucoCorners);
         allCharucoIds.push_back(currentCharucoIds);
         allImagePoints.push_back(currentImagePoints);
         allObjectPoints.push_back(currentObjectPoints);
         allImages.push_back(im.clone());
         imageSize = im.size();
      }

      if (key == 27) break; // ESC
   }

   // --- Calibrate ---
   if (allCharucoCorners.size() < 4) {
      std::cerr << "Not enough views for calibration (need >= 4)\n";
      return;
   }

   cv::Mat cameraMatrix, distCoeffs;
   std::vector<cv::Mat> rvecs, tvecs;
   const int calibrationFlags = 0;

   double repError = cv::calibrateCamera(
      allObjectPoints, allImagePoints, imageSize,
      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags
   );

   if (save_camparam("webcam_param.xml", cameraMatrix, distCoeffs))
      std::cout << "Saved webcam_param.xml\n";
   std::cout << "Reprojection error: " << repError << std::endl;

   // Optional: review captured frames
   for (size_t i = 0; i < allImages.size(); ++i) {
      cv::Mat vis = allImages[i].clone();
      if (allCharucoCorners[i].total() > 0)
         cv::aruco::drawDetectedCornersCharuco(vis, allCharucoCorners[i], allCharucoIds[i]);
      cv::imshow("ChArUco Capture", vis);
      if ((char)cv::waitKey(0) == 27) break;
   }

   cv::destroyWindow("ChArUco Capture");
}


/*bool CCameraReal::detectBoardPose(cv::Mat& frame) {
    if (frame.empty()) { have_pose = false; return false; }

    // Detect markers
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::DetectorParameters detParams;
    cv::aruco::ArucoDetector detector(dict, detParams);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(frame, corners, ids);
    if (ids.empty()) { have_pose = false; return false; }
    if (_draw_markers) cv::aruco::drawDetectedMarkers(frame, corners, ids);

    // Interpolate ChArUco
    cv::aruco::CharucoBoard board(cv::Size(kSquaresX, kSquaresY), kSquareLen, kMarkerLen, dict);
    cv::aruco::CharucoDetector charuco(board, cv::aruco::CharucoParameters(), detParams);

    cv::Mat chCorners, chIds; // Nx1 CV_32FC2 and Nx1 CV_32S
    charuco.detectBoard(frame, chCorners, chIds, corners, ids);
    if (chCorners.empty() || chIds.empty() || chCorners.rows != chIds.rows) { have_pose = false; return false; }
    if (chCorners.total() < 6) { have_pose = false; return false; }

    // Build 3D/2D
    const auto& boardCorners = board.getChessboardCorners();
    std::vector<cv::Point3f> objPts; objPts.reserve((size_t)chCorners.rows);
    std::vector<cv::Point2f> imgPts; imgPts.reserve((size_t)chCorners.rows);
    for (int i = 0; i < chCorners.rows; ++i) {
        int cid = chIds.at<int>(i);
        if (cid >= 0 && cid < (int)boardCorners.size()) {
            imgPts.push_back(chCorners.at<cv::Point2f>(i));
            objPts.push_back(boardCorners[cid]);
        }
    }
    if (objPts.size() < 6) { have_pose = false; return false; }

    // Pose
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(objPts, imgPts, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
        have_pose = false; return false;
    }
    rvec_CB = rvec;
    tvec_CB = tvec;
    have_pose = true;

    // BIG axes (your convention: X:+Y, Y:+X, Z:-Z)
    const float L = 2.0f * kSquareLen;
    std::vector<cv::Point3f> axes3D = { {0,0,0}, {0, L,0}, {L,0,0}, {0,0,-L} };
    std::vector<cv::Point2f> axes2D;
    cv::projectPoints(axes3D, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, axes2D);
    if (axes2D.size() == 4) {
        cv::line(frame, axes2D[0], axes2D[1], { 0,  0,255 }, 3); // X red
        cv::line(frame, axes2D[0], axes2D[2], { 0,255,  0 }, 3); // Y green
        cv::line(frame, axes2D[0], axes2D[3], { 255,  0,  0 }, 3); // Z blue
        cv::putText(frame, "X", axes2D[1] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 0,  0,255 }, 2);
        cv::putText(frame, "Y", axes2D[2] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 0,255,  0 }, 2);
        cv::putText(frame, "Z", axes2D[3] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 255,  0,  0 }, 2);
    }

    // MINI axes at each marker center - very defensive
    // H = K [r1 r2 t]  (homography board->image), invert safely
    cv::Mat Rcb; cv::Rodrigues(rvec_CB, Rcb);
    cv::Mat Rt_2x3; cv::hconcat(Rcb.col(0), Rcb.col(1), Rt_2x3); cv::hconcat(Rt_2x3, cv::Mat(tvec_CB), Rt_2x3);
    cv::Mat H = _cam_webcam_intrinsic * Rt_2x3;
    double det = cv::determinant(H);
    if (std::abs(det) < 1e-12) return true; // skip minis if degenerate
    cv::Mat Hinv = H.inv();

    auto imgToBoard = [&](const cv::Point2f& uv)->cv::Point2f {
        cv::Mat q = Hinv * (cv::Mat_<double>(3, 1) << (double)uv.x, (double)uv.y, 1.0);
        double w = q.at<double>(2, 0);
        if (std::abs(w) < 1e-12) return cv::Point2f();
        return cv::Point2f((float)(q.at<double>(0, 0) / w), (float)(q.at<double>(1, 0) / w));
        };

    const float Lmini = 0.5f * kMarkerLen;
    for (const auto& c : corners) {
        if (c.size() != 4) continue;
        cv::Point2f center2D(0.25f * (c[0].x + c[1].x + c[2].x + c[3].x),
            0.25f * (c[0].y + c[1].y + c[2].y + c[3].y));
        cv::Point2f xy = imgToBoard(center2D);
        if (!cv::checkRange(cv::Mat(xy))) continue; // NaN/Inf guard
        cv::Point3f C3(xy.x, xy.y, 0.0f);

        // Match big-axes convention
        std::vector<cv::Point3f> mini3 = {
            C3,
            {C3.x,         C3.y + Lmini, C3.z},  // X red: +Y
            {C3.x + Lmini, C3.y,         C3.z},  // Y green: +X
            {C3.x,         C3.y,         C3.z - Lmini} // Z blue: -Z
        };
        std::vector<cv::Point2f> mini2;
        cv::projectPoints(mini3, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, mini2);
        if (mini2.size() != 4) continue;
        cv::line(frame, mini2[0], mini2[1], { 0,  0,255 }, 2);
        cv::line(frame, mini2[0], mini2[2], { 0,255,  0 }, 2);
        cv::line(frame, mini2[0], mini2[3], { 255,  0,  0 }, 2);
    }

    return true;
}
*/

bool CCameraReal::detectBoardPose(cv::Mat& frame) {
   if (frame.empty()) { have_pose = false; return false; }

   // Detect markers
   cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   cv::aruco::DetectorParameters detParams;
   cv::aruco::ArucoDetector detector(dict, detParams);

   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> corners;
   detector.detectMarkers(frame, corners, ids);
   if (ids.empty()) { have_pose = false; return false; }
   if (_draw_markers) cv::aruco::drawDetectedMarkers(frame, corners, ids);

   // Interpolate ChArUco
   cv::aruco::CharucoBoard board(cv::Size(kSquaresX, kSquaresY), kSquareLen, kMarkerLen, dict);
   cv::aruco::CharucoDetector charuco(board, cv::aruco::CharucoParameters(), detParams);

   cv::Mat chCorners, chIds; // Nx1 CV_32FC2 and Nx1 CV_32S
   charuco.detectBoard(frame, chCorners, chIds, corners, ids);
   if (chCorners.empty() || chIds.empty() || chCorners.rows != chIds.rows) { have_pose = false; return false; }
   if (chCorners.total() < 6) { have_pose = false; return false; }

   // Build 3D/2D
   const auto& boardCorners = board.getChessboardCorners();
   std::vector<cv::Point3f> objPts; objPts.reserve((size_t)chCorners.rows);
   std::vector<cv::Point2f> imgPts; imgPts.reserve((size_t)chCorners.rows);
   for (int i = 0; i < chCorners.rows; ++i) {
      int cid = chIds.at<int>(i);
      if (cid >= 0 && cid < (int)boardCorners.size()) {
         imgPts.push_back(chCorners.at<cv::Point2f>(i));
         objPts.push_back(boardCorners[cid]);
      }
   }
   if (objPts.size() < 6) { have_pose = false; return false; }

   // Pose
   cv::Vec3d rvec, tvec;
   if (!cv::solvePnP(objPts, imgPts, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
      have_pose = false; return false;
   }
   rvec_CB = rvec;
   tvec_CB = tvec;
   have_pose = true;

   // BIG axes (your convention: X:+Y, Y:+X, Z:-Z)
   const float L = 2.0f * kSquareLen;
   std::vector<cv::Point3f> axes3D = { {0,0,0}, {0, L,0}, {L,0,0}, {0,0,-L} };
   std::vector<cv::Point2f> axes2D;
   cv::projectPoints(axes3D, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, axes2D);
   if (axes2D.size() == 4) {
      cv::line(frame, axes2D[0], axes2D[1], cv::Scalar(0, 0, 255), 3); // X red
      cv::line(frame, axes2D[0], axes2D[2], cv::Scalar(0, 255, 0), 3); // Y green
      cv::line(frame, axes2D[0], axes2D[3], cv::Scalar(255, 0, 0), 3); // Z blue
      cv::putText(frame, "X", axes2D[1] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
      cv::putText(frame, "Y", axes2D[2] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      cv::putText(frame, "Z", axes2D[3] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
   }

   // MINI axes at each marker center - very defensive
   cv::Mat Rcb; cv::Rodrigues(rvec_CB, Rcb);
   cv::Mat Rt_2x3; cv::hconcat(Rcb.col(0), Rcb.col(1), Rt_2x3); cv::hconcat(Rt_2x3, cv::Mat(tvec_CB), Rt_2x3);
   cv::Mat H = _cam_webcam_intrinsic * Rt_2x3;
   double det = cv::determinant(H);
   if (std::abs(det) < 1e-12) return true; // skip minis if degenerate
   cv::Mat Hinv = H.inv();

   auto imgToBoard = [&](const cv::Point2f& uv)->cv::Point2f {
      cv::Mat q = Hinv * (cv::Mat_<double>(3, 1) << (double)uv.x, (double)uv.y, 1.0);
      double w = q.at<double>(2, 0);
      if (std::abs(w) < 1e-12) return cv::Point2f();
      return cv::Point2f((float)(q.at<double>(0, 0) / w), (float)(q.at<double>(1, 0) / w));
      };

   const float Lmini = 0.5f * kMarkerLen;
   for (const auto& c : corners) {
      if (c.size() != 4) continue;
      cv::Point2f center2D(0.25f * (c[0].x + c[1].x + c[2].x + c[3].x),
         0.25f * (c[0].y + c[1].y + c[2].y + c[3].y));
      cv::Point2f xy = imgToBoard(center2D);
      if (!cv::checkRange(cv::Mat(xy))) continue; // NaN/Inf guard
      cv::Point3f C3(xy.x, xy.y, 0.0f);

      std::vector<cv::Point3f> mini3 = {
          C3,
          {C3.x,         C3.y + Lmini, C3.z},        // X red: +Y
          {C3.x + Lmini, C3.y,         C3.z},        // Y green: +X
          {C3.x,         C3.y,         C3.z - Lmini} // Z blue: -Z
      };
      std::vector<cv::Point2f> mini2;
      cv::projectPoints(mini3, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, mini2);
      if (mini2.size() != 4) continue;
      cv::line(frame, mini2[0], mini2[1], cv::Scalar(0, 0, 255), 2);
      cv::line(frame, mini2[0], mini2[2], cv::Scalar(0, 255, 0), 2);
      cv::line(frame, mini2[0], mini2[3], cv::Scalar(255, 0, 0), 2);
   }

   return true;
}







void CCameraReal::transform_to_image(Mat pt3d_mat, Point2f& pt) {
   if (!have_pose) { pt = Point2f(); return; }
   Point3f P(pt3d_mat.at<float>(0), pt3d_mat.at<float>(1), pt3d_mat.at<float>(2));
   std::vector<Point3f> src{ P };
   std::vector<Point2f> dst;
   projectPoints(src, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, dst);
   pt = dst[0];
}

void CCameraReal::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d) {
   pts2d.clear();
   if (!have_pose || pts3d_mat.empty()) return;

   std::vector<Point3f> src;
   src.reserve(pts3d_mat.size());
   for (auto& H : pts3d_mat)
      src.emplace_back(H.at<float>(0), H.at<float>(1), H.at<float>(2));

   projectPoints(src, rvec_CB, tvec_CB, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, pts2d);
}

//lab 6 calling cub on charuco id-50
static bool estimate_single_marker_pose_ippe(
   const std::vector<cv::Point2f>& corners, float markerLen,
   const cv::Mat& K, const cv::Mat& D,
   cv::Vec3d& rvec, cv::Vec3d& tvec)
{
   if (corners.size() != 4 || markerLen <= 0 || K.empty()) return false;

   // OpenCV ArUco corners are TL, TR, BR, BL (clockwise).
   std::vector<cv::Point3f> obj = {
       {-markerLen / 2.f,  markerLen / 2.f, 0.f}, // TL
       { markerLen / 2.f,  markerLen / 2.f, 0.f}, // TR
       { markerLen / 2.f, -markerLen / 2.f, 0.f}, // BR
       {-markerLen / 2.f, -markerLen / 2.f, 0.f}  // BL
   };

   // IPPE_SQUARE is very stable for planar squares.
   return cv::solvePnP(
      obj, corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE
   );
}



bool CCameraReal::draw_cube_on_marker(cv::Mat& frame, int marker_id, float markerLen, float cubeH)
{
   if (frame.empty() || _cam_webcam_intrinsic.empty()) return false;

   // Persistence state (survives across calls)
   static bool have_last = false;
   static cv::Vec3d last_rvec(0, 0, 0), last_tvec(0, 0, 0);
   static int hold_frames_left = 0;
   const int HOLD_FRAMES_MAX = 12;
   const double SMOOTH_ALPHA = 0.2;

   auto draw_cube_with_pose = [&](const cv::Vec3d& rvec, const cv::Vec3d& tvec) -> bool
      {
         const float s = markerLen;
         const float h = cubeH;

         std::vector<cv::Point3f> cube3d = {
             cv::Point3f(-s / 2,  s / 2, 0), cv::Point3f(s / 2,  s / 2, 0),
             cv::Point3f(s / 2, -s / 2, 0), cv::Point3f(-s / 2, -s / 2, 0),
             cv::Point3f(-s / 2,  s / 2, h), cv::Point3f(s / 2,  s / 2, h),
             cv::Point3f(s / 2, -s / 2, h), cv::Point3f(-s / 2, -s / 2, h)
         };

         std::vector<cv::Point2f> imgPts;
         cv::projectPoints(cube3d, rvec, tvec, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, imgPts);
         if (imgPts.size() != 8) return false;

         auto L = [&](int a, int b) { cv::line(frame, imgPts[a], imgPts[b], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); };
         L(0, 1); L(1, 2); L(2, 3); L(3, 0);
         L(4, 5); L(5, 6); L(6, 7); L(7, 4);
         L(0, 4); L(1, 5); L(2, 6); L(3, 7);

         cv::drawFrameAxes(frame, _cam_webcam_intrinsic, _cam_webcam_dist_coeff, rvec, tvec, 0.5f * markerLen);
         return true;
      };

   // 1) Detect marker
   cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   cv::aruco::DetectorParameters detParams;
   cv::aruco::ArucoDetector detector(dict, detParams);

   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> corners;
   detector.detectMarkers(frame, corners, ids);

   int idx = -1;
   for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == marker_id) { idx = (int)i; break; }
   }

   // 2) If not seen this frame, draw last pose if we have one
   if (idx < 0) {
      // ADD: invalidate class cache for this frame
      last_marker_ok_ = false;
      last_marker_id_ = -1;

      if (have_last && hold_frames_left > 0) {
         hold_frames_left--;
         return draw_cube_with_pose(last_rvec, last_tvec);
      }
      return false;
   }

   // 3) Estimate fresh pose (IPPE_SQUARE, two candidates)
   const std::vector<cv::Point2f>& c = corners[idx];
   if (c.size() != 4 || markerLen <= 0) {
      // ADD: invalidate class cache for this frame
      last_marker_ok_ = false;
      last_marker_id_ = -1;

      if (have_last && hold_frames_left > 0) {
         hold_frames_left--;
         return draw_cube_with_pose(last_rvec, last_tvec);
      }
      return false;
   }

   std::vector<cv::Point3f> obj = {
       cv::Point3f(-markerLen / 2.0f,  markerLen / 2.0f, 0.0f),
       cv::Point3f(markerLen / 2.0f,  markerLen / 2.0f, 0.0f),
       cv::Point3f(markerLen / 2.0f, -markerLen / 2.0f, 0.0f),
       cv::Point3f(-markerLen / 2.0f, -markerLen / 2.0f, 0.0f)
   };

   std::vector<cv::Mat> rvecs, tvecs;
   std::vector<double> reprojErrs;
   bool ok = cv::solvePnPGeneric(
      obj, c, _cam_webcam_intrinsic, _cam_webcam_dist_coeff,
      rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE,
      cv::noArray(), cv::noArray(), reprojErrs
   );

   if (!ok || rvecs.empty() || tvecs.empty()) {
      // ADD: invalidate class cache for this frame
      last_marker_ok_ = false;
      last_marker_id_ = -1;

      if (have_last && hold_frames_left > 0) {
         hold_frames_left--;
         return draw_cube_with_pose(last_rvec, last_tvec);
      }
      return false;
   }

   auto rot_diff = [](const cv::Vec3d& r1, const cv::Vec3d& r2)->double {
      cv::Mat R1, R2; cv::Rodrigues(r1, R1); cv::Rodrigues(r2, R2);
      cv::Mat R = R2 * R1.t();
      cv::Vec3d r; cv::Rodrigues(R, r);
      return std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
      };

   int best = -1;
   double bestErr = 1e100;
   for (size_t i = 0; i < rvecs.size(); ++i) {
      cv::Vec3d tv(
         tvecs[i].at<double>(0, 0),
         tvecs[i].at<double>(1, 0),
         tvecs[i].at<double>(2, 0)
      );
      if (tv[2] <= 0.0) continue;
      double err = (i < reprojErrs.size() ? reprojErrs[i] : 0.0);
      if (err < bestErr) { bestErr = err; best = (int)i; }
   }
   if (best < 0) {
      best = 0;
      double e0 = (0 < reprojErrs.size() ? reprojErrs[0] : 1e100);
      for (size_t i = 1; i < rvecs.size(); ++i) {
         double ei = (i < reprojErrs.size() ? reprojErrs[i] : 1e100);
         if (ei < e0) { e0 = ei; best = (int)i; }
      }
   }

   cv::Vec3d rvec(
      rvecs[best].at<double>(0, 0),
      rvecs[best].at<double>(1, 0),
      rvecs[best].at<double>(2, 0)
   );
   cv::Vec3d tvec(
      tvecs[best].at<double>(0, 0),
      tvecs[best].at<double>(1, 0),
      tvecs[best].at<double>(2, 0)
   );

   if (have_last && rvecs.size() > 1) {
      int alt = (best == 0 ? 1 : 0);
      cv::Vec3d alt_r(
         rvecs[alt].at<double>(0, 0),
         rvecs[alt].at<double>(1, 0),
         rvecs[alt].at<double>(2, 0)
      );
      double d_best = rot_diff(last_rvec, rvec);
      double d_alt = rot_diff(last_rvec, alt_r);
      if (d_alt + 1e-6 < d_best) rvec = alt_r;
   }

   if (have_last) {
      tvec = SMOOTH_ALPHA * tvec + (1.0 - SMOOTH_ALPHA) * last_tvec;
   }

   bool drawn = draw_cube_with_pose(rvec, tvec);
   if (drawn) {
      have_last = true;
      last_rvec = rvec;
      last_tvec = tvec;
      hold_frames_left = HOLD_FRAMES_MAX;

      // ADD: update class cache for board-frame query
      last_marker_ok_ = true;
      last_marker_id_ = marker_id;
      last_marker_rvec_C_ = rvec;
      last_marker_tvec_C_ = tvec;
   }
   else {
      // ADD: invalidate class cache for this frame
      last_marker_ok_ = false;
      last_marker_id_ = -1;

      if (have_last && hold_frames_left > 0) {
         hold_frames_left--;
         return draw_cube_with_pose(last_rvec, last_tvec);
      }
   }
   return drawn;
}





bool CCameraReal::get_marker_pose_in_board(int marker_id, cv::Vec3d& rvec_BM, cv::Vec3d& tvec_BM) const
{
   if (!have_pose) return false;
   if (!last_marker_ok_) return false;
   if (last_marker_id_ != marker_id) return false;

   cv::Mat T_C_B = Rt_to_T(rvec_CB, tvec_CB);                // board in camera
   cv::Mat T_C_M = Rt_to_T(last_marker_rvec_C_, last_marker_tvec_C_); // marker in camera
   cv::Mat T_B_C = T_C_B.inv();
   cv::Mat T_B_M = T_B_C * T_C_M;

   T_to_Rt(T_B_M, rvec_BM, tvec_BM);
   return true;
}
///////////////////lab7-functions//////


/*bool CCameraReal::draw_marker_ids(cv::Mat& frame)
{
    if (frame.empty())
        return false;

    // Detection outputs
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;

    // Use the SAME dictionary as your ChArUco board
    cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::aruco::DetectorParameters detParams;
    cv::aruco::ArucoDetector detector(dict, detParams);

    // New API: detector.detectMarkers(...)
    detector.detectMarkers(frame, corners, ids);

    if (ids.empty())
        return false;

    // Draw marker borders and IDs on the frame
    cv::aruco::drawDetectedMarkers(frame, corners, ids);

    for (size_t i = 0; i < ids.size(); ++i)
    {
        cv::Point2f center(0.0f, 0.0f);
        for (int k = 0; k < 4; ++k)
        {
            center.x += corners[i][k].x;
            center.y += corners[i][k].y;
        }
        center.x *= 0.25f;
        center.y *= 0.25f;

        char buf[16];
        std::snprintf(buf, sizeof(buf), "%d", ids[i]);

        cv::putText(frame, buf, center,
            cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    return true;
}
*/
bool CCameraReal::draw_marker_ids(cv::Mat& frame)
{
   if (frame.empty())
      return false;

   std::vector<int> ids;
   std::vector<std::vector<cv::Point2f>> corners;

   cv::aruco::Dictionary dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

   cv::aruco::DetectorParameters detParams;
   cv::aruco::ArucoDetector detector(dict, detParams);

   detector.detectMarkers(frame, corners, ids);
   if (ids.empty())
      return false;

   // This draws borders *and* IDs in OpenCV's style
   cv::aruco::drawDetectedMarkers(frame, corners, ids);

   return true;
}