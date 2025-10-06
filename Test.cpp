//Test.cpp
#include "Test.h"
#include "stdafx.h"
//#include <iostream>


void test_function() // to make sure the test file works
{
    std::cout << "This is a test function." << std::endl;
}

void testDot(cv::Mat& canvas, cv::Size image_size, int radius, cv::Scalar color, int thickness) // draws a dot at the centre of the canvas, cuz currently i see nothing i draw
{
    cv::circle(canvas, cv::Size(image_size.width / 2, image_size.height / 2), radius, color, thickness);
}

std::vector<cv::Mat> createBox(float w, float h, float d)
{
      std::vector<cv::Mat> box3d;
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
      box3d.push_back(cv::Mat((cv::Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));
      return box3d;
}

void testBox(const std::vector<cv::Mat>& box3d, std::vector<cv::Point2f>& box2d)// take the 3D box and offset it to the centre of the canvas, and call 'that' the 2D box
{
   cv::Size image_size = cv::Size(1000, 600);

   // Clear box2d to ensure it's empty before filling
   box2d.clear();

   // Use box3d.size() for iteration, not box2d.size()
   for (size_t i = 0; i < box3d.size(); i++)
   {
       cv::Point2f p;
       p.x = box3d[i].at<float>(0, 0) + image_size.width / 2;  // offset x to center of canvas
       p.y = box3d[i].at<float>(1, 0) + image_size.height / 2; // offset y to center of canvas
       box2d.push_back(p);
   }
}

void testCube(const std::vector<cv::Mat>& box3d, std::vector<cv::Point2f>& box2d)// draw the cube on the canvas
{
   cv::Size image_size = cv::Size(1280, 720);
   cv::Point origin = cv::Point(image_size.width / 2, image_size.height / 2);
   box2d.clear();

   // Step 1: Define camera intrinsic parameters (from MATLAB example)
   // Focal length in meters and pixel size
   float focal_length = 0.003f;  // metres
   float pixel_size = 4.6e-6f;   // metres
   int cx = origin.x;            // principal point x (center of canvas)
   int cy = origin.y;            // principal point y (center of canvas)
   float f_pixels = focal_length / pixel_size; // ~ 652

   // Step 2: Build the intrinsic camera matrix K
   // Expected matrix outcome:
    // | 652   0   500 |
    // |   0 652   300 |
    // |   0   0     1 |
   cv::Mat K = (cv::Mat_<float>(3, 3) << 
      f_pixels, 0, cx,
      0, f_pixels, cy,
      0,        0, 1);

   // Step 3: Build the extrinsic matrix [R|t] (identity, no rotation/translation)
   // Expected matrix outcome:
    // | 1 0 0 Tx |
    // | 0 1 0 Ty |
    // | 0 0 1 Tz |
   //cv::Mat Rt = cv::Mat::eye(3, 4, CV_32F); // no longer using this
   // instead make a HT matrix that moves the camera 20cm forward in the Z-direction
   float Tx = 0.0f; // metres
   float Ty = 0.0f; // metres
   float Tz = 0.2f; // metres (in front of camera)
   cv::Mat Rt = (cv::Mat_<float>(3, 4) <<
      1, 0, 0, Tx,
      0, 1, 0, Ty,
      0, 0, 1, Tz);

   // Step 4: Build the full camera matrix P = K * [R|t]
   // Expected matrix outcome: 3x4
   cv::Mat P = K * Rt;

   // Step 5: Project each 3D point to 2D using P
   for (const auto& pt3d : box3d)
   {
      // pt3d is 4x1: [X, Y, Z, 1]^T
      cv::Mat pt2d_h = P * pt3d; // 3x1 homogeneous (4x1)
      // Perspective division
      float x = pt2d_h.at<float>(0, 0) / pt2d_h.at<float>(2, 0);
      float y = pt2d_h.at<float>(1, 0) / pt2d_h.at<float>(2, 0);
      box2d.push_back(cv::Point2f(x, y)); // Expected: x, y are pixel coordinates on canvas
   }
   // Step 6: Draw cube edges using projected 2D points... but i will do this in an external function
}

cv::Mat CreateHT(cv::Vec3d t, cv::Vec3d r) // TODO: Create Homogeneous Transformation Matrix
{
   // constants
   const double deg2rad = 3.14159265358979323846 / 180;
   const double alpha = r[2] * deg2rad; // yaw
   const double beta = r[1] * deg2rad;  // pitch
   const double gamma = r[0] * deg2rad; // roll
   const double ca = cos(alpha);
   const double sa = sin(alpha);
   const double cb = cos(beta);
   const double sb = sin(beta);
   const double cg = cos(gamma);
   const double sg = sin(gamma);

   double data[] =
   {
      ca * cb,		ca * sb * sg - sa * cg,		ca * sb * cg + sa * sg,		t[0],
      sa * cb,		sa * sb * sg + ca * cg,		sa * sb * cg - ca * sg,		t[1],
      -sb,			cb * sg,							cb * cg,							t[2],
      0,				0,									0,									1
   };
   return cv::Mat(4, 4, CV_64F, data).clone(); // clone to ensure the data is copied and not just referenced
}

std::vector<cv::Mat> createCoord() // note for Mik: argument defaults go in the declaration, not the definition
{
   std::vector <cv::Mat> coord;

   float axis_length = 0.05;

   coord.push_back((cv::Mat1f(4, 1) << 0, 0, 0, 1)); // O
   coord.push_back((cv::Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
   coord.push_back((cv::Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
   coord.push_back((cv::Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

   return coord;
}

void testCoord(std::vector<cv::Mat>& coord3d, std::vector<cv::Point2f>& coord2d) // ... had to make coord3d a variable, so i could scale it...
{
   // step 1 - intialize 
   cv::Size image_size = cv::Size(1280, 720);
   cv::Point origin = cv::Point(image_size/2);
   coord2d.clear();
   // step 2 - scale coord3d to be visible on canvas
   float edge_length = coord3d[1].at<float>(0, 0); // should be 5cm
   float scale = 0.003f / 4.6e-6f; // focal length / pixel size
   for (size_t i = 0; i < coord3d.size(); i++)
   {
      coord3d[i].at<float>(0, 0) *= scale; // X
      coord3d[i].at<float>(1, 0) *= scale; // Y
      coord3d[i].at<float>(2, 0) *= scale; // Z
   }
   // step 3 - convert 2D coord to 3D coord, manually, without matrix multiplication, with offset to centre of canvas
   float x, y;
   x = coord3d[0].at<float>(0, 0) + origin.x; // X coordinate
   y = coord3d[0].at<float>(1, 0) + origin.y; // Y coordinate
   coord2d.push_back(cv::Point2f(x, y));
   x = coord3d[1].at<float>(0, 0) + origin.x; // X coordinate
   y = coord3d[1].at<float>(1, 0) + origin.y; // Y coordinate
   coord2d.push_back(cv::Point2f(x, y));
   x = coord3d[2].at<float>(0, 0) + origin.x; // X coordinate
   y = -coord3d[2].at<float>(1, 0) + origin.y; // Y coordinate
   coord2d.push_back(cv::Point2f(x, y));
   x = coord3d[3].at<float>(0, 0) - 0.707*edge_length*scale + origin.x; // X coordinate
   y = coord3d[3].at<float>(1, 0) + 0.707*edge_length*scale + origin.y; // Y coordinate
   coord2d.push_back(cv::Point2f(x, y));
}

std::vector<std::vector<cv::Mat>> createRobot(float w, float h, float d)
{
    float edge_length = 0.025; // fits on screen
    // Cube positions in meters (relative to robot/world origin)
    std::vector<cv::Vec3d> positions = {
        {0, 0, 0},
        {0, 0, 1 * edge_length},
        {0, 0, 2 * edge_length},
        {0, 0, 3 * edge_length},
        {1 * edge_length, 0, 2 * edge_length},
        {-1 * edge_length, 0, 2 * edge_length}
    };

    std::vector<std::vector<cv::Mat>> robot3d;

    for (const auto& pos : positions)
    {
        // Create 8 vertices for a cube centered at origin
        std::vector<cv::Mat> cube = createBox(w, h, d);

        // Build homogeneous transformation matrix for translation only
        cv::Mat HT = CreateHT(pos, cv::Vec3d(0, 0, 0)); // no rotation

        // Transform each vertex in the cube
        for (auto& v : cube)
        {
            cv::Mat v_d;
            v.convertTo(v_d, CV_64F);
            cv::Mat v_trans = HT * v_d;
            v_trans.convertTo(v, CV_32F); // overwrite with transformed vertex
        }

        robot3d.push_back(cube); // add this cube's vertices to robot3d
    }

    // Rotate all cubes 90 degrees about the x-axis
    cv::Mat HT_rot = CreateHT(cv::Vec3d(0, 0, 0), cv::Vec3d(90, 0, 0)); // roll=90deg, pitch=0, yaw=0

    for (auto& cube : robot3d)
    {
        for (auto& v : cube)
        {
            cv::Mat v_d;
            v.convertTo(v_d, CV_64F);
            cv::Mat v_rot = HT_rot * v_d;
            v_rot.convertTo(v, CV_32F); // overwrite with rotated vertex
        }
    }

    return robot3d; // 6 cubes, each with 8 vertices, all rotated
}

void testRobot(const std::vector<std::vector<cv::Mat>>& robot3d, std::vector<std::vector<cv::Point2f>>& robot2d)
{
    // Clear output vector
    robot2d.clear();

    // For each cube in robot3d, project its vertices to 2D using testCube
    for (const auto& cube3d : robot3d)
    {
        std::vector<cv::Point2f> cube2d;
        testCube(cube3d, cube2d);
        robot2d.push_back(cube2d);
    }
}

// Draws the robot's cubes on the canvas using robust 2D cube drawing
void drawRobot(cv::Mat& canvas, const std::vector<std::vector<cv::Point2f>>& robot2d)
{
    int draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
    int draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };
    int cubeNo = 0; 

    for (const auto& cube2d : robot2d)
    {
        // Example: vary color using cubeNo for RGB channels
        cv::Scalar color = CV_RGB((cubeNo * 40) % 256, (cubeNo * 80) % 256, (cubeNo * 120) % 256);
        cubeNo = (cubeNo + 1) % 256; // cycle cubeNo between 0-255 
        for (int i = 0; i < 12; i++)
        {
            cv::Point pt1 = cube2d.at(draw_box1[i]);
            cv::Point pt2 = cube2d.at(draw_box2[i]);
            cv::line(canvas, pt1, pt2, color, 2);
        }
    }
}

int chooseTest(int test_no, cv::Mat& canvas, cv::Size image_size)
{
   // Initialize OpenCV:
      // already done externally...
   // Run the test decided by the developer:
   switch (test_no)
   {
   case 0:
      test_function();
      break;

   case 1:
      testDot(canvas, image_size, 50, CV_RGB(255, 255, 0), cv::FILLED);
      break;

   case 2:
   {

      // Make some sort of 3D array of points and call box3d to convert it to 2D box
      std::vector<cv::Mat> box3d;
      box3d = createBox(50, 50, 50); // 50 pixels wide
      // Now transform this 3D box to 2D box
      std::vector<cv::Point2f> box2d;
      testBox(box3d, box2d); /////// instead of _virtualcam.transform_to_image(box3d, box2d); ///////
      // Now draw the 2D cube on the canvas
      float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 }; // The 12 lines connecting all vertexes 
      float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 }; // the other end of the 12 lines connecting all vertexes
      for (int i = 0; i < 12; i++)
      {
         cv::Point pt1 = box2d.at(draw_box1[i]);
         cv::Point pt2 = box2d.at(draw_box2[i]);

         cv::line(canvas, pt1, pt2, CV_RGB(255, 255, 0), 1);
      }
      //std::cout << "test incomplete...\n";

   }  break;

   case 3:
   {
      // make 3D cube
      std::vector<cv::Mat> box3d;
      box3d = createBox(0.05f, 0.05, 0.05); // 5cm edge length
      // Now transform this 3D cube to 2D cube
      std::vector<cv::Point2f> box2d;

      testCube(box3d, box2d); // instead of _virtualcam.transform_to_image(box3d, box2d);

      // Now draw the 2D box on the canvas
      float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 }; // The 12 lines connecting all vertexes 
      float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 }; // the other end of the 12 lines connecting all vertexes
      for (int i = 0; i < 12; i++)
      {
         cv::Point pt1 = box2d.at(draw_box1[i]);
         cv::Point pt2 = box2d.at(draw_box2[i]);

         cv::line(canvas, pt1, pt2, CV_RGB(255, 255, 0), 1);
      }

   }  break;

   case 4:
   {
      // create Coordinate
      cv::Point origin2d = cv::Point(image_size / 2);
      cv::Point3d origin = cv::Point3d(origin2d.x, origin2d.y, 0); // he wants +X to be right, +Z up, and +Y out of screen... but this doesn't work in my version...
      std::vector<cv::Mat> w = createCoord();
      std::vector<cv::Point2f> coord2d;
      testCoord(w, coord2d); 
      //draw it on canvas
      cv::circle(canvas, coord2d[0], 5, CV_RGB(255, 255, 255), cv::FILLED); // white dot at origin
      cv::line(canvas, coord2d[0], coord2d[1], CV_RGB(255, 0, 0), 2);  // Draw the X axis in red
      cv::line(canvas, coord2d[0], coord2d[2], CV_RGB(0, 255, 0), 2); // Draw the Y axis in green
      cv::line(canvas, coord2d[0], coord2d[3], CV_RGB(0, 0, 255), 2);  // Draw the Z axis in blue
      // now create 6 boxes
      std::vector<std::vector<cv::Mat>> robot3d = createRobot(0.025f, 0.025f, 0.025f); // fits on screen
      std::vector<std::vector<cv::Point2f>> robot2d;
      testRobot(robot3d, robot2d);
      // now draw the 2D boxes on the canvas
      drawRobot(canvas, robot2d);
      // extra debug option
      cv::Point debug_text_pos = coord2d[3];
      cv::putText(canvas, "(" + std::to_string(debug_text_pos.x) + "," + std::to_string(debug_text_pos.y) + ")", 
         origin2d + cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1);
   }  break;

   default: 
      std::cout << "test incomplete...\n"; 
      break;
   }

   return test_no;
}