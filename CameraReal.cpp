#include "stdafx.h"

#include "CameraReal.h"
#include <cmath>

#include "cvui.h"

CCameraReal::CCameraReal()
{
	//////////////////////////////////////
	// Start webcam video source

	start_webcam(0);

	if (load_camparam("webcam_param.xml", _cam_webcam_intrinsic, _cam_webcam_dist_coeff) == false)
	{
		cout << "\nNo webcam camera parameters found";
	}

	//////////////////////////////////////
	// CVUI interface default variables

	_draw_on_board = false;
	_draw_markers = true;
}

CCameraReal::~CCameraReal()
{
	_vid_webcam.release();
}

bool CCameraReal::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

bool CCameraReal::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

void CCameraReal::start_webcam(int webcam_id)
{
	_vid_webcam.release();
	_vid_webcam.open(webcam_id, CAP_DSHOW);
}

void CCameraReal::get_image(Mat& im)
{
  if (_vid_webcam.isOpened() == true)
	{
	  _vid_webcam >> im;
	}
}

void CCameraReal::createChArUcoBoard()
{
	Mat im;
	float size_square = 0.04;
	float size_mark = 0.02;
	Size board_size = Size(5, 7);

	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	aruco::CharucoBoard board(Size(board_size.width, board_size.height), size_square, size_mark, dictionary);

	board.generateImage(cv::Size(600, 500), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCameraReal::calibrate_board(int cam_id)
{
	// Board settings
	Size board_size = Size(5, 7);

	float size_aruco_square = 0.0;
	float size_aruco_mark = 0.0;

	aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
	aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	aruco::CharucoParameters charucoParams;
	charucoParams.tryRefineMarkers = true;

	aruco::CharucoBoard charucoboard = aruco::CharucoBoard(Size(board_size.width, board_size.height), size_aruco_square, size_aruco_mark, dictionary);
	aruco::CharucoDetector detector(charucoboard, charucoParams, detectorParams);

	// Collect data from each frame
	vector<Mat> allCharucoCorners, allCharucoIds;

	vector<vector<Point2f>> allImagePoints;
	vector<vector<Point3f>> allObjectPoints;

	vector<Mat> allImages;
	Size imageSize;

	do
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		get_image(im);
		im.copyTo(draw_im);

		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		Mat currentCharucoCorners, currentCharucoIds;
		vector<Point3f> currentObjectPoints;
		vector<Point2f> currentImagePoints;

		// Detect ChArUco board
		detector.detectBoard(im, currentCharucoCorners, currentCharucoIds);
		//! [CalibrationWithCharucoBoard1]

		// Draw results
		im.copyTo(draw_im);
		if (!markerIds.empty()) {
			aruco::drawDetectedMarkers(draw_im, markerCorners);
		}

		if (currentCharucoCorners.total() > 3) {
			aruco::drawDetectedCornersCharuco(draw_im, currentCharucoCorners, currentCharucoIds);
		}

		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
			Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

		imshow("out", draw_im);

		// Wait for key pressed
		char key = (char)waitKey(10);

		//! [CalibrationWithCharucoBoard2]
		if (key == 'c' && currentCharucoCorners.total() > 3) 
		{
			// Match image points
			charucoboard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

			if (currentImagePoints.empty() || currentObjectPoints.empty()) {
				std::cout << "Point matching failed, try again." << endl;
				continue;
			}

			std::cout << "Frame captured" << endl;

			allCharucoCorners.push_back(currentCharucoCorners);
			allCharucoIds.push_back(currentCharucoIds);
			allImagePoints.push_back(currentImagePoints);
			allObjectPoints.push_back(currentObjectPoints);
			allImages.push_back(im);

			imageSize = im.size();
		}

		if (key == 27)
		{
			break;
		}
	}
	while (1);

	std::cout << "Calibrating..." << endl;

	////////////////////////////////////////////////////////////////////////////////
	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	//! [CalibrationWithCharucoBoard3]
	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// Calibrate camera using ChArUco
	double repError = calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs, noArray(), noArray(), noArray(), noArray(), noArray(), calibrationFlags);

	save_camparam("webcam_param.xml", cameraMatrix, distCoeffs);

	std::cout << "Rep Error: " << repError << endl;

	// Show interpolated charuco corners for debugging
	if (1) {
		for (size_t frame = 0; frame < allImages.size(); frame++) 
		{
			std::cout << "Showing Calibration Image: " << frame << endl;
			Mat imageCopy = allImages[frame].clone();

			if (allCharucoCorners[frame].total() > 0) {
				aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);
			}

			imshow("out", imageCopy);
			char key = (char)waitKey(0);
			if (key == 27) {
				break;
			}
		}
	}
}

void CCameraReal::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	// projectPoints(...);
}

void CCameraReal::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	// projectPoints(...);
}

void CCameraReal::update_settings(Mat &im)
{	
	Point _camera_setting_window;

	_camera_setting_window.x = 0;
	_camera_setting_window.y = 380;

	if (im.empty() == true) { return; }

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 210, 180, "Real Camera Settings");
	
	_camera_setting_window.x = 5;
	_camera_setting_window.y += 25;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Draw on Board", &_draw_on_board);

	_camera_setting_window.y += 25;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Draw all Markers", &_draw_markers);
}
