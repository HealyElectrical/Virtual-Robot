// CameraReal.cpp
#include "stdafx.h"
#include "CameraReal.h"

#include <string>
#include <vector>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// ArUco/ChArUco (OpenCV 4.6+)
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

CCameraReal::CCameraReal() {
    start_webcam(0);
    set_resolution(1280, 720);

    // Try to load; otherwise use a sane guess
    if (!load_camparam("webcam_param.xml", _cam_webcam_intrinsic, _cam_webcam_dist_coeff)) {
        std::cout << "[lab4] No webcam_param.xml – using rough intrinsics.\n";
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
        cv::line(frame, axes2D[0], axes2D[1], { 0,  0,255 }, 3); // X red
        cv::line(frame, axes2D[0], axes2D[2], { 0,255,  0 }, 3); // Y green
        cv::line(frame, axes2D[0], axes2D[3], { 255,  0,  0 }, 3); // Z blue
        cv::putText(frame, "X", axes2D[1] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 0,  0,255 }, 2);
        cv::putText(frame, "Y", axes2D[2] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 0,255,  0 }, 2);
        cv::putText(frame, "Z", axes2D[3] + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.6, { 255,  0,  0 }, 2);
    }

    // MINI axes at each marker center — very defensive
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
