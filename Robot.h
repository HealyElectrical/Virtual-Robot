#pragma once

#include <opencv2/opencv.hpp>

#include "CameraVirtual.h"
#include "CameraReal.h"

using namespace std;
using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;

	void init();

	////////////////////////////////////
  // LAB 3

	vector<vector<Mat>> _simple_robot;

	CCameraVirtual _virtualcam;
	CCameraReal _realcam;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);

	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	////////////////////////////////////
	// LAB 4

	////////////////////////////////////
	// LAB 5

	int _do_animate; // Animation state machine

	void update_settings(Mat& im);

public:
	////////////////////////////////////
	// Lab 3

	Mat createHT(Vec3d t, Vec3d r);
	void create_simple_robot();
	void draw_simple_robot();

	////////////////////////////////////
	// Lab 4

	void draw();

	////////////////////////////////////
	// Lab 5

	// void fkine(); // Input joint variables, output end effector pose

	////////////////////////////////////
	// Lab 6

	// bool ikine(); // Input end effector pose, output joint angles
};

