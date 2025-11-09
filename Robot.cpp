#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include "cvui.h"

CRobot::CRobot()
{
	_image_size = IMAGE_SIZE;
   _image_center = IMAGE_CENTER;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

   _simple_robot.clear();

	init();
}

CRobot::~CRobot()
{}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;
}

Mat CRobot::createHT(Vec3d t, Vec3d r) // TODO: Create Homogeneous Transformation Matrix
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

	return ((Mat1f(4, 4) << a, b, c, t[0], 
		                     d, e, f, t[1], 
		                     g, h, i, t[2], 
		                     0, 0, 0, 1) );
}

std::vector<Mat> CRobot::createBox(float w, float h, float d) // he already finished this one for us
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord() // note for Mik: argument defaults go in the declaration, not the definition
{
	std::vector <Mat> coord;

   float axis_length = 0.05; 
	// homog form
	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}


void CRobot::transformPoints(std::vector<Mat>& points, Mat T) // he already finished this one for us
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour) // he started this one for us
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	// If Virtual Camera
   _virtualcam.transform_to_image(box3d, box2d); /// this is the one we need to implement/code in the camera class
	// If Real Camera
	//_realcam.transform_to_image();

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d) // he started this one for us
{
	Point2f O, X, Y, Z;
	//O = IMAGE_CENTER;
	//Point2f testPoint = Point2f(20, 20);

	// If Virtual Camera
	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	// If Real Camera
	//_realcam.transform_to_image();

	line(im, O, X, RED, 1); 
	line(im, O, Y, GREEN, 1);
	line(im, O, Z, BLUE, 1); 
}



void CRobot::create_simple_robot() 
{
	float box_size = BOX_SIZE;

	std::vector<Vec3d> positions = {
		{0,0,0} ,									// centre of box 1
		{0, 0, 1 * box_size},					// centre of box 2
		{0, 0, 2 * box_size},					// centre of box 3
		{0, 0, 3 * box_size},					// centre of box 4
		{ 1 * box_size, 0, 2 * box_size },	// centre of box 5
		{-1 * box_size, 0, 2 * box_size}		// centre of box 6
	};

	for (auto& pos : positions)
	{
		std::vector<Mat> box = createBox(box_size, box_size, box_size);
		Mat T = createHT(pos, Vec3d(0, 0, 0)); // no rotation for now... instead we will rotate the entire robot later
		transformPoints(box, T);
		_simple_robot.push_back(box); // robot< 6 cubes<8 points> >
	}
}

void CRobot::draw_simple_robot()
{
	Mat im;
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + BACKGROUND_COLOR; // redraw bkrgd

	//_realcam.get_image(im);
	//im.copyTo(_canvas);

	_virtualcam.update_settings(_canvas);
	//_realcam.update_settings(_canvas);
	update_settings(_canvas);
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Draw after update
	if (TEST)
	{
		chooseTest(TEST_4, _canvas, _image_size); // currently used instead of a for-loop of drawBox(_canvas, _simple_robot[0], cv::Scalar(255, 0, 0)); & drawCoord(_canvas, O);
	}
	else {
      // draw the robot
		for (const auto& box : _simple_robot)
			drawBox(_canvas, box, chooseColors(&box - &_simple_robot[0])); // chooseColors(index, color0, color1, color2, color3, color4, color5)
      // draw the coord at the origin of the world... but it needs to be moved manually with the robot later
		std::vector<Mat> O = createCoord();
      drawCoord(_canvas, O); 
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	cv::imshow(CANVAS_NAME, _canvas);
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

Scalar CRobot::chooseColors(int idx, Scalar c0, Scalar c1, Scalar c2, Scalar c3, Scalar c4, Scalar c5)
{
    Scalar colors[6] = {c0, c1, c2, c3, c4, c5};
    return colors[idx % 6]; // wrap around if idx >= 6
}
