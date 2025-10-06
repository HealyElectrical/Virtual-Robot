//Test.h
// should be referenced in every file where its functions or objects are used
#pragma once
#include <opencv2/opencv.hpp>
#include "constants.h"

#define TEST false // true or false flag to be used in main/branches to include or exclude test code

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum test_type // to be used with chooseTest()
{
   TEST_0 = 0, // to have a default value
   TEST_1,
   TEST_2,
   TEST_3,
   TEST_4
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// most basic functions to test if the test file works
void test_function();
int chooseTest(int test_no, cv::Mat& canvas, cv::Size image_size); // runs and returns the test # decided by the developper, on the passed canvas
void testDot(cv::Mat& canvas, cv::Size image_size, int radius, cv::Scalar color, int thickness);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for the following functions, make them as if the will be used in real calls, and create dummy inputs to test them in chooseTest()
std::vector<cv::Mat> createBox(float w, float h, float d); // plagiarized from craig
void testBox(const std::vector<cv::Mat>& box3d, std::vector<cv::Point2f>& box2d); 
void testCube(const std::vector<cv::Mat>& box3d, std::vector<cv::Point2f>& box2d);
cv::Mat CreateHT(cv::Vec3d t, cv::Vec3d r);
std::vector<cv::Mat> createCoord();
void testCoord(std::vector<cv::Mat>& coord3d, std::vector<cv::Point2f>& coord2d); // convert 3D coord to 2D coord, with the same perspective as testCube()
std::vector<std::vector<cv::Mat>> createRobot(float w, float h, float d); 
void testRobot(const std::vector<std::vector<cv::Mat>>& robot3d, std::vector<std::vector<cv::Point2f>>& robot2d); 
void drawRobot(cv::Mat& canvas, const std::vector<std::vector<cv::Point2f>>& robot2d); // draws the robot on the canvas, given the 2D robot vertexes

