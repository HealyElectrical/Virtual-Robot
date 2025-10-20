// CCameraReal.h
#pragma once
#include <string>
#include <vector>
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

    bool detectBoardPose(cv::Mat& frame);

    void transform_to_image(cv::Mat pt3d_mat, cv::Point2f& pt2d);
    void transform_to_image(std::vector<cv::Mat> pts3d_mat, std::vector<cv::Point2f>& pts2d);

    // Pose state
    bool have_pose = false;
    cv::Vec3d rvec_CB{ 0,0,0 }, tvec_CB{ 0,0,0 };

private:
    int _webcam_id = 0;
    cv::VideoCapture _vid_webcam;

    bool _draw_markers = true; // <— this fixes “_draw_markers undefined”

    cv::Mat _cam_webcam_intrinsic;   // K
    cv::Mat _cam_webcam_dist_coeff;  // D
};
