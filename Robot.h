#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "CameraVirtual.h"
#include "CameraReal.h"
#include "constants.h"
#include "Test.h"

// View mode toggle
enum class ViewMode { AR, Virtual }; // adding the virtual to lab5

class CRobot

{
public:
    CRobot();
    ~CRobot();

private:
    cv::Size  _image_size;
    cv::Point _image_center;
    cv::Mat   _canvas;

    // robot<cubes<8 vertexes/points>>
    std::vector<std::vector<cv::Mat>> _simple_robot;
    std::vector<std::vector<cv::Mat>> _scara_links;
    bool _lab3;

    
    CCameraVirtual _virtualcam;
    CCameraReal    _realcam;

    // Builders / helpers
    std::vector<cv::Mat> createBox(float w, float h, float d);
    std::vector<cv::Mat> createCoord();

    void init();
    void transformPoints(std::vector<cv::Mat>& points, cv::Mat T);

    void drawBox(cv::Mat& im, std::vector<cv::Mat> box3d, cv::Scalar colour);
    void drawCoord(cv::Mat& im, std::vector<cv::Mat> coord3d);
    void update_settings(cv::Mat& im);
    cv::Scalar chooseColors(int idx = 0, cv::Scalar c0 = RUST, cv::Scalar c1 = RUST, cv::Scalar c2 = PURPLE, cv::Scalar c3 = RUST, cv::Scalar c4 = RED, cv::Scalar c5 = BLUE);

    ////////////////////////////////////
    // LAB 4 (helpers; can be private)
public:
    void drawBoxReal(cv::Mat& im,
        const std::vector<cv::Mat>& box3d,
        CCameraReal& cam,
        const cv::Scalar& color);

public:
    // Exposed so lab4() can call it
    void draw_simple_robot_on_real(cv::Mat& frame, CCameraReal& cam);

public:
    // Draw the simple robot onto the real frame with a Z-rotation (degrees)
    void draw_rotating_robot_on_real(cv::Mat& frame, CCameraReal& cam, double angle_deg);

    ////////////////////////////////////
    // LAB 5

    int _do_animate; // Animation state machine

public:
    // NOTE: const so it can be used inside const helper functions
    cv::Mat createHT(cv::Vec3d t, cv::Vec3d r) const;

    // Lab 3-style API
    void create_simple_robot();
    void draw_simple_robot();

    // ==== In Robot.h (public) ====
    void set_world_anchor(const cv::Vec3d& p_WB, const cv::Vec3d& rpy_WB);
    void draw_scara_world(CCameraReal& cam,
        double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);

    ////////////////////////////////////
    // Lab 4
    void draw();

    // Overload: size robot based on checkerboard square length (meters)
    void create_simple_robot(float squareLenMeters);

    ////////////////////////////////////
    // Lab 5 (Forward Kinematics) - public API used from template.cpp
    cv::Mat fkine(double q1_deg, double q2_deg, double d3_m, double q4_deg) const;
    void create_scara_templates();
    void draw_scara(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);
    void update_joint_controls(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);
    void drawPrism(cv::Mat& im, std::vector<cv::Mat> prism3d, cv::Scalar colour);
    std::vector<cv::Mat> createPrism(float w, float h, float d);

    // === Lab 5 Part B ===
// Draw SCARA robot in the real camera frame (anchored to ArUco board)
    void draw_scara_on_real(cv::Mat& frame, CCameraReal& cam,
        double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);


    // Small helper so UI can draw onto our canvas without exposing the member
    cv::Mat& canvas();
    // Add this to Robot.h (public section)


    // === Desired end-effector pose (UI, in mm/deg) ===
    double ee_x_mm_ = 0.0;
    double ee_y_mm_ = 0.0;
    double ee_z_mm_ = 0.0;     // 0..150 mm
    double ee_theta_deg_ = 0.0; // -180..180

    // Draw axes using real camera (projection with Z-flip like boxes/prisms)
    void drawCoordReal(cv::Mat& im, std::vector<cv::Mat> coord3d, CCameraReal& cam);

    // Draw the target EE pose axes in world (board) frame
    void draw_target_ee_world(CCameraReal& cam);
    //for animation
    void start_anim_partB();
    void stop_anim_partB();
    bool step_anim_partB(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);



    //adding virtual to lab 5
    void draw_scara_dispatch(CCameraReal& cam,
        double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);
    void set_view_mode(ViewMode m) { view_mode_ = m; }
    ViewMode view_mode() const { return view_mode_; }

    void draw_target_ee_virtual();

private:
    // ---- Lab 5 helpers & geometry (used by fkine and drawing) ----
    // Tiny helpers to build transforms
    cv::Mat Rz_deg(double deg) const;
    cv::Mat Tx(double x) const;
    cv::Mat Tz(double z) const;

    // SCARA link lengths / offsets (meters) - per spec
    double L1_ = 0.15;      // link 1 length (15 cm)
    double L2_ = 0.15;      // link 2 length (15 cm)
    double pedestalZ_ = 0.135; // 13.5 cm pedestal so prismatic 0 is at 0.15 m world Z

    double D3_MIN_ = 0.0;   // [m]
    double D3_MAX_ = 0.15;  // [m]


    // Link box dimensions for drawing (meters) - 15 x 3 x 3 cm
    double Bx_ = 0.15, By_ = 0.03, Bz_ = 0.03;

    // Reusable templates for drawing links/end-effector (in local frames)
    std::vector<cv::Mat> linkTemplate_; // box pointing +X, joint at -X face
    std::vector<cv::Mat> effTemplate_;  // small tool box

    // Draw a link/effector given a template and world transform
    void draw_link(cv::Mat& im, const std::vector<cv::Mat>& templ, const cv::Mat& T_world, const cv::Scalar& color);

    cv::Mat T_WB_ = cv::Mat::eye(4, 4, CV_32F); // Robot-base pose in Board frame

    void drawPrismReal(cv::Mat& im,
        std::vector<cv::Mat> prism3d,
        CCameraReal& cam,
        const cv::Scalar& colour);

    //for animation
    bool  anim_running_ = false;
    int   anim_phase_ = 0;     // which joint / segment we?re animating
    double anim_step_deg_ = 2.0; // deg per frame
    double anim_step_m_ = 0.003; // meters per frame (3 mm)

    //adding virtual to lab5
    ViewMode view_mode_ = ViewMode::AR;

    void update_ee_readout_from_T(const cv::Mat& T4, double d3_m);


    // cosmetic flag for 'apply pose' highlighting
    bool show_applied_pose_ = false;

    // optional: keep last live frame so the window isn?t blank in AR mode
    cv::Mat last_live_frame_;

    ////////////////////////////////////
    // Lab 6 (Inverse Kinematics) - add impl in .cpp when ready
    // bool ikine(const cv::Mat& T_target);
public:
    // In CRobot class (public:)
    bool ikine(double x_m, double y_m, double z_m, double theta_deg,
        double& q1_deg, double& q2_deg, double& d3_m, double& q4_deg,
        bool elbow_up = true) const;



    enum class SolveMode { FK, IK };
    SolveMode solve_mode_;     // default set in constructor
    bool elbow_up_toggle_;     // IK branch choice

    


private:
    // --- Linear IK animation API (kept private if only used internally) ---
    void start_linear_anim();
    void stop_linear_anim();
    bool step_linear_anim(double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m);

    // --- Linear IK animation state ---
    bool   lin_anim_running_ = false;
    int    lin_phase_ = 0;     // 0: x, 1: y, 2: z, 3: wrist
    double lin_step_mm_ = 5.0;   // mm per frame (x/y/z)
    double lin_step_deg_ = 5.0;   // deg per frame (wrist yaw)
    double lin_theta_accum_deg_ = 0.0;   // accumulated wrist rotation

    //for the aruco box
    // --- Cube tracking state (Lab 6) ---
    bool   track_cube_ = false;        // toggled by UI
    double track_alpha_pos_ = 0.35;    // smoothing for target position
    double track_alpha_yaw_ = 0.35;    // smoothing for yaw
    cv::Vec3d track_last_pB_{ 0,0,0 };   // last smoothed target in board frame
    double    track_last_yaw_deg_ = 0.0;

    // Try to update joints so EE tracks marker-50 cube (board frame)
    bool maybe_track_cube(CCameraReal& cam,
        double& q1_deg, double& q2_deg, double& q3_deg, double& d3_m,
        int marker_id = 50);
    // Tool tip vertical offset above the EE frame, in meters.
    // Set to match your end-effector geometry if needed.
    double tool_z_offset_ = 0.00;




};