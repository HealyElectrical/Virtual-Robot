#include "stdafx.h"

#include "CameraVirtual.h"
#include <cmath>

#include "cvui.h"

CCameraVirtual::CCameraVirtual()
{
	// Initialize with a default camera image size 
	init(IMAGE_SIZE);
}

CCameraVirtual::~CCameraVirtual()
{
}

void CCameraVirtual::init(Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 2;     // Focal length in mm
	_cam_setting_x = 0;     // mm
	_cam_setting_y = -70;     // mm
	_cam_setting_z = -270;  // mm
	_cam_setting_roll = -90; // degrees
	_cam_setting_pitch = 0;  // degrees
	_cam_setting_yaw = 0;    // degrees


	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2); // this is the line that centres the figure 

	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();
}

void CCameraVirtual::calculate_intrinsic() {
	float f = _cam_setting_f / 1000.0f; // focal length in meters
	float fx = f / _pixel_size;
	float fy = f / _pixel_size;
	float cx = _principal_point.x;
	float cy = _principal_point.y;

	_cam_virtual_intrinsic = (Mat1f(3, 4) << fx, 0, cx, 0,
		0, fy, cy, 0,
		0, 0, 1, 0);
}

void CCameraVirtual::calculate_extrinsic() {
	Vec3d t(_cam_setting_x / 1000.0, _cam_setting_y / 1000.0, _cam_setting_z / 1000.0); // mm to m
	Vec3d r(_cam_setting_roll, _cam_setting_pitch, _cam_setting_yaw); // degrees

	// Use your createHT function to get the matrix
	_cam_virtual_extrinsic = createHT(t, r);
}

void CCameraVirtual::transform_to_image(Mat pt3d_mat, Point2f& pt) {
	// Transform to camera coordinates
	Mat pt_cam = _cam_virtual_extrinsic * pt3d_mat;
	// Project to image plane
	Mat pt_img = _cam_virtual_intrinsic * pt_cam;
	// Normalize
	pt.x = pt_img.at<float>(0, 0) / pt_img.at<float>(2, 0);
	pt.y = pt_img.at<float>(1, 0) / pt_img.at<float>(2, 0);
}

void CCameraVirtual::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d) {
	pts2d.clear();
	for (const auto& pt3d : pts3d_mat) {
		Point2f pt2d;
		transform_to_image(pt3d, pt2d); // Reuse the overloaded single point function
		pts2d.push_back(pt2d);
	}
}

Mat CCameraVirtual::createHT(Vec3d t, Vec3d r) // TODO: Create Homogeneous Transformation Matrix
{
	// constants
	const double deg2rad = 3.14159265358979323846 / 180;
	const double alpha = r[2] * deg2rad; // yaw
	const double beta = r[1] * deg2rad;  // pitch
	const double gamma = r[0] * deg2rad; // roll
	const double sa = sin(alpha);
	const double sb = sin(beta);
	const double sg = sin(gamma);
	const double ca = cos(alpha);
	const double cb = cos(beta);
	const double cg = cos(gamma);

	// matrix value calculations
	double a = ca * cb;
	double b = (ca * sb * sg) - (sa * cg);
	double c = (ca * sb * cg) + (sa * sg);
	double d = sa * cb;
	double e = (sa * sb * sg) + (ca * cg);
	double f = (sa * sb * cg) - (ca * sg);
	double g = -(sb);
	double h = cb * sg;
	double i = cb * cg;

	return ((Mat1f(4, 4) <<
		a, b, c, t[0],
		d, e, f, t[1],
		g, h, i, t[2],
		0, 0, 0, 1));
}

void CCameraVirtual::update_settings(Mat& im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 375, "Virtual Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	// Use this line if only this settings window in use
	// cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}
