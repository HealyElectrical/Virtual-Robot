#pragma once

#include <opencv2/opencv.hpp>
#include "constants.h"

using namespace std;
using namespace cv;

class CCameraVirtual
{
public:
	CCameraVirtual();
	~CCameraVirtual();

private:

	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	Mat createHT(Vec3d t, Vec3d r);
	void calculate_intrinsic();
	void calculate_extrinsic();

	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;

public:
	void init(Size image_size);

	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	void update_settings(Mat& im);
};