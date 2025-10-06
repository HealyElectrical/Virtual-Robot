#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	init();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	_do_animate = 0;

	// IMPORTANT: initialize the virtual camera with the canvas size
	_virtualcam.init(_image_size);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB3
////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	auto deg2rad = [](double d) { return d * CV_PI / 180.0; };

	const double rx = deg2rad(r[0]); // roll about X
	const double ry = deg2rad(r[1]); // pitch about Y
	const double rz = deg2rad(r[2]); // yaw about Z

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

	Mat R = Rz * Ry * Rx;               // yaw→pitch→roll
	Mat T = Mat::eye(4, 4, CV_64F);
	R.copyTo(T(Rect(0, 0, 3, 3)));
	T.at<double>(0, 3) = t[0];
	T.at<double>(1, 3) = t[1];
	T.at<double>(2, 3) = t[2];
	return T;
}


std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector<Mat> box;
	// 8 vertices, origin at box center (units: mm)
	box.push_back((Mat_<double>(4, 1) << -w / 2, -h / 2, -d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << w / 2, -h / 2, -d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << w / 2, h / 2, -d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << -w / 2, h / 2, -d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << -w / 2, -h / 2, d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << w / 2, -h / 2, d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << w / 2, h / 2, d / 2, 1));
	box.push_back((Mat_<double>(4, 1) << -w / 2, h / 2, d / 2, 1));
	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector<Mat> coord;
	const double axis_len = 100.0; // mm (100 mm = 10 cm)

	coord.push_back((Mat_<double>(4, 1) << 0, 0, 0, 1));           // O
	coord.push_back((Mat_<double>(4, 1) << axis_len, 0, 0, 1));    // X
	coord.push_back((Mat_<double>(4, 1) << 0, axis_len, 0, 1));    // Y
	coord.push_back((Mat_<double>(4, 1) << 0, 0, axis_len, 1));    // Z
	return coord;
}



void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (auto& p : points) p = T * p;
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	// Project 3D axis endpoints to image
	_virtualcam.transform_to_image(coord3d.at(0), O); // origin
	_virtualcam.transform_to_image(coord3d.at(1), X); // +X
	_virtualcam.transform_to_image(coord3d.at(2), Y); // +Y
	_virtualcam.transform_to_image(coord3d.at(3), Z); // +Z

	// Draw (guard for behind-camera sentinel if you want)
	line(im, O, X, CV_RGB(255, 0, 0), 2); // X = red
	line(im, O, Y, CV_RGB(0, 255, 0), 2); // Y = green
	line(im, O, Z, CV_RGB(0, 0, 255), 2); // Z = blue
}


void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;
	static const int e1[12] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	static const int e2[12] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image(box3d, box2d);

	for (int i = 0; i < 12; ++i)
	{
		Point2f a = box2d[e1[i]];
		Point2f b = box2d[e2[i]];
		// skip if projected behind camera (we used a sentinel of ~-1e6)
		if (a.x < -1e5f || b.x < -1e5f) continue;
		line(im, a, b, colour, 2, LINE_AA);
	}
}


void CRobot::create_simple_robot()
{
	_simple_robot.clear();

	const double W = 50.0;                 // cube edge in mm
	auto base = createBox(W, W, W);        // a single cube centered at origin

	// ---- Vertical column: 4 boxes tall along +Z ----
	for (int i = 0; i < 4; ++i) {
		Mat T = createHT(Vec3d(0, 0, i * W), Vec3d(0, 0, 0));  // stack along +Z
		auto box = base;
		transformPoints(box, T);
		_simple_robot.push_back(box);
	}

	// ---- Crossbar at the 3rd box level ----
	const double z_cross = 2 * W;   // z = 100 mm

	// Left arm (−X)
	{
		Mat T = createHT(Vec3d(-W, 0, z_cross), Vec3d(0, 0, 0));
		auto box = base;
		transformPoints(box, T);
		_simple_robot.push_back(box);
	}

	// Right arm (+X)
	{
		Mat T = createHT(Vec3d(+W, 0, z_cross), Vec3d(0, 0, 0));
		auto box = base;
		transformPoints(box, T);
		_simple_robot.push_back(box);
	}
}




void CRobot::draw_simple_robot()
{
	_canvas = Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// UI panels (virtual cam + any robot settings you have)
	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	// Draw world axes at origin
	auto axes = createCoord();
	drawCoord(_canvas, axes);

	// --- assign colors per box (BGR order for OpenCV) ---
	std::vector<Scalar> colors = {
		 Scalar(0,   0, 255),     // Box 1 - Red
		 Scalar(0, 255,   0),     // Box 2 - Green
		 Scalar(255, 0, 255),     // Box 3 - Purple (magenta)
		 Scalar(0, 255, 255),     // Box 4 - Yellow (cyan+green)
		 Scalar(255, 128, 0),     // Box 5 - Blue-ish Orange (actually orange-blue mix)
		 Scalar(42,  42, 165)     // Box 6 - Light Brown (tan)
	};

	// Draw all boxes
	for (size_t i = 0; i < _simple_robot.size(); ++i)
	{
		Scalar c = (i < colors.size()) ? colors[i] : Scalar(255, 255, 255); // fallback white
		drawBox(_canvas, _simple_robot[i], c);
	}

	cvui::update();
	imshow(CANVAS_NAME, _canvas);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB4
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5
////////////////////////////////////////////////////////////////////////////////////////////////////////

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	if (_do_animate != 0)
	{
		int step_size = 5;
		if (_do_animate == 1)
		{
			// state 1
			if (1) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2
			if (1) { _do_animate = 3; }
		}
		else if (_do_animate == 3) {
			if (1) { _do_animate = 0; init(); }
		}
	}

	cvui::update();
}

void CRobot::draw()
{
	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}
