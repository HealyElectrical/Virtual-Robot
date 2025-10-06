% Lab 1: Coordinate Transforms 2D
% Author: Mikhail R
% Date: September 2, 2025

%% --- CONTROL PANEL: Uncomment to run desired lab parts ---
run_toolbox_verification = false;
run_step_2_1 = false;
run_step_2_2 = false;
run_step_2_3 = false;
run_step_3_1 = false;
run_step_3_2 = true;

%% Add more function calls as you implement each part
if run_toolbox_verification
    toolbox_verification();
end
if run_step_2_1
    step_2_1_homogeneous_plot();
end
if run_step_2_2
    step_2_2_rotate_about_C();
end
if run_step_2_3
    step_2_3_rotate_about_W();
end
if run_step_3_1
    step_3_1_body_points();
end
if run_step_3_2
    step_3_2_shoulder_rotation();
end

%% --- Local Functions ---
function toolbox_verification()
    % Check if the Robotics System Toolbox is installed
    if ~license('test', 'Robotics_System_Toolbox')
        error('Robotics System Toolbox is not installed. Please install robot.mltbx.')
    else
        disp('Robotics System Toolbox is installed and ready.')
    end
end

function step_2_1_homogeneous_plot()
    % Step 2.1: Homogeneous Transformation and Plotting (RTK style)
    % Define square corners in local {C} frame
    P1 = [-1, -1]';
    P2 = [1, -1]';
    P3 = [1, 1]';
    P4 = [-1, 1]';
    pts = [P1'; P2'; P3'; P4'; P1'];

    % Setup plot volume
    figure; plotvol([-5 6 -5 5]); hold on;

    % World frame
    W = transl2(0,0);
    trplot2(W, 'frame', 'W', 'color', 'k');

    % Homogeneous transformation: translation (4,3), rotation +45 deg
    R = transl2(4,3) * trot2(45, 'deg');
    trplot2(R, 'frame', 'C', 'color', 'b');

    % Transform square corners to world frame
    ptsW = homtrans(R, pts'); % RTK homtrans expects 3xN

    % Plot square
    plot(ptsW(1,:), ptsW(2,:), 'b-');

    % Plot and label each corner
    plot_point(ptsW(:,1), 'label', 'P1', 'ko');
    plot_point(ptsW(:,2), 'label', 'P2', 'ko');
    plot_point(ptsW(:,3), 'label', 'P3', 'ko');
    plot_point(ptsW(:,4), 'label', 'P4', 'ko');
    title('Square in World Frame with {C} at (4,3), Rotated +45^o (RTK style)');
    xlabel('X'); ylabel('Y');
    grid on;
end

function step_2_2_rotate_about_C()
    % Step 2.2: Rotate square 360 deg about origin in {C} (RTK style)
    % User can set fps and total_time
    fps = 30;         % frames per second (user adjustable)
    total_time = 4;   % total animation time in seconds (user adjustable)
    nSteps = round(fps * total_time); % number of frames
    pause_time = 1 / fps;             % pause per frame

    P1 = [-1, -1]';
    P2 = [1, -1]';
    P3 = [1, 1]';
    P4 = [-1, 1]';
    pts = [P1'; P2'; P3'; P4'; P1'];

    % Setup plot volume
    figure; plotvol([-5 6 -5 5]); hold on;
    W = transl2(0,0);
    trplot2(W, 'frame', 'W', 'color', 'k');

    % Fixed translation for {C}
    T = transl2(4,3);
    angles = linspace(0, 360, nSteps);

    h = plot(NaN, NaN, 'b-'); % handle for square
    hP = gobjects(4,1); % handles for points
    for k = 1:nSteps
        cla;
        plotvol([-5 6 -5 5]); hold on;
        trplot2(W, 'frame', 'W', 'color', 'k');
        theta = angles(k);
        R = T * trot2(theta, 'deg');
        trplot2(R, 'frame', 'C', 'color', 'b');
        ptsW = homtrans(R, pts');
        h = plot(ptsW(1,:), ptsW(2,:), 'b-');
        hP(1) = plot_point(ptsW(:,1), 'label', 'P1', 'ko');
        hP(2) = plot_point(ptsW(:,2), 'label', 'P2', 'ko');
        hP(3) = plot_point(ptsW(:,3), 'label', 'P3', 'ko');
        hP(4) = plot_point(ptsW(:,4), 'label', 'P4', 'ko');
        title(sprintf('Rotating Square about {C}: %d^o', round(theta)));
        xlabel('X'); ylabel('Y'); grid on;
        drawnow;
        pause(pause_time);
    end
end

function step_2_3_rotate_about_W()
    % Step 2.3: Rotate square 360 deg about origin in {W} (RTK style)
    % User can set fps and total_time
    fps = 30;         % frames per second (user adjustable)
    total_time = 4;   % total animation time in seconds (user adjustable)
    nSteps = round(fps * total_time); % number of frames
    pause_time = 1 / fps;             % pause per frame

    P1 = [-1, -1]';
    P2 = [1, -1]';
    P3 = [1, 1]';
    P4 = [-1, 1]';
    pts = [P1'; P2'; P3'; P4'; P1'];

    % Setup plot volume
    figure; plotvol([-5 5 -5 5]); hold on;
    W = transl2(0,0);
    trplot2(W, 'frame', 'W', 'color', 'k');

    % Fixed translation for {C}
    T = transl2(4,3);
    angles = linspace(0, 360, nSteps);

    h = plot(NaN, NaN, 'b-'); % handle for square
    hP = gobjects(4,1); % handles for points
    for k = 1:nSteps
        cla;
        plotvol([-7 7 -7 7]); hold on;
        trplot2(W, 'frame', 'W', 'color', 'k');
        theta = angles(k);
        % Rotation about {W} (world origin)
        R = trot2(theta, 'deg') * T;
        trplot2(R, 'frame', 'C', 'color', 'b');
        ptsW = homtrans(R, pts');
        h = plot(ptsW(1,:), ptsW(2,:), 'b-');
        hP(1) = plot_point(ptsW(:,1), 'label', 'P1', 'ko');
        hP(2) = plot_point(ptsW(:,2), 'label', 'P2', 'ko');
        hP(3) = plot_point(ptsW(:,3), 'label', 'P3', 'ko');
        hP(4) = plot_point(ptsW(:,4), 'label', 'P4', 'ko');
        title(sprintf('Rotating Square about {W}: %d^o', round(theta)));
        xlabel('X'); ylabel('Y'); grid on;
        drawnow;
        pause(pause_time);
    end
end

function step_3_1_body_points()
    % Step 3.1: Plot body points using cascading translations (RTK style)
    % All units in inches
    % Frame origins: W(0,0), B(0,18), C(9,12.5), D(20,12.5), E(31.5,12.5)

    % Define translations between frames
    T_WB = transl2(0,18);           % W to B
    T_BC = transl2(9, -5.5);        % B to C (from B at (0,18) to C at (9,12.5))
    T_CD = transl2(11, 0);          % C to D (from C at (9,12.5) to D at (20,12.5))
    T_DE = transl2(11.5, 0);        % D to E (from D at (20,12.5) to E at (31.5,12.5))

    % Points at origin of each frame
    WP = [0;0]; BP = [0;0]; CP = [0;0]; DP = [0;0]; EP = [0;0];

    % Cascade transformations to world frame
    BP_W = homtrans(T_WB, BP);
    CP_W = homtrans(T_WB * T_BC, CP);
    DP_W = homtrans(T_WB * T_BC * T_CD, DP);
    EP_W = homtrans(T_WB * T_BC * T_CD * T_DE, EP);

    % Setup plot
    figure; plotvol([-5 40 -5 25]); hold on;
    trplot2(eye(3), 'frame', 'W', 'color', 'k');
    trplot2(T_WB, 'frame', 'B', 'color', 'r');
    trplot2(T_WB * T_BC, 'frame', 'C', 'color', 'g');
    trplot2(T_WB * T_BC * T_CD, 'frame', 'D', 'color', 'b');
    trplot2(T_WB * T_BC * T_CD * T_DE, 'frame', 'E', 'color', 'm');

    % Plot points
    plot_point(WP, 'label', 'W', 'ko');
    plot_point(BP_W, 'label', 'B', 'ro');
    plot_point(CP_W, 'label', 'C', 'go');
    plot_point(DP_W, 'label', 'D', 'bo');
    plot_point(EP_W, 'label', 'E', 'mo');

    % Connect points with lines
    pts = [WP, BP_W, CP_W, DP_W, EP_W];
    plot(pts(1,:), pts(2,:), 'k--');

    title('Body Points in World Frame (RTK style)');
    xlabel('X (inches)'); ylabel('Y (inches)'); grid on;
end

function step_3_2_shoulder_rotation()
    % Step 3.2: Apply rotation at shoulder (frame C) and animate robot body
    % All units in inches
    fps = 30; total_time = 4;
    nSteps = round(fps * total_time);
    pause_time = 1 / fps;
    angles = linspace(0, 45, nSteps); % degrees

    % Define translations between frames
    T_WB = transl2(0,18);           % W to B
    T_BC = transl2(9, -5.5);        % B to C
    T_CD = transl2(11, 0);          % C to D
    T_DE = transl2(11.5, 0);        % D to E

    % Points at origin of each frame
    WP = [0;0]; BP = [0;0]; CP = [0;0]; DP = [0;0]; EP = [0;0];

    % Setup plot
    plotSize = [-5 40 -5 40];
    figure; plotvol(plotSize); hold on;
    for k = 1:nSteps
        cla;
        plotvol(plotSize); hold on;
        trplot2(eye(3), 'frame', 'W', 'color', 'k');
        trplot2(T_WB, 'frame', 'B', 'color', 'r');
        % Apply rotation at C
        R_C = trot2(angles(k), 'deg');
        T_C = T_BC * R_C;
        trplot2(T_WB * T_C, 'frame', 'C', 'color', 'g');
        trplot2(T_WB * T_C * T_CD, 'frame', 'D', 'color', 'b');
        trplot2(T_WB * T_C * T_CD * T_DE, 'frame', 'E', 'color', 'm');

        % Cascade points
        BP_W = homtrans(T_WB, BP);
        CP_W = homtrans(T_WB * T_C, CP);
        DP_W = homtrans(T_WB * T_C * T_CD, DP);
        EP_W = homtrans(T_WB * T_C * T_CD * T_DE, EP);

        % Invisible sternum point
        sternum = [0;12.5];

        % Plot points
        plot_point(WP, 'label', 'W', 'ko');
        plot_point(BP_W, 'label', 'B', 'ro');
        plot_point(CP_W, 'label', 'C', 'go');
        plot_point(DP_W, 'label', 'D', 'bo');
        plot_point(EP_W, 'label', 'E', 'mo');

        % Connect points with lines
        pts = [WP, BP_W, CP_W, DP_W, EP_W];
        plot(pts(1,:), pts(2,:), 'k--');

        % Dashed lines from B to sternum, sternum to C
        plot([BP_W(1), sternum(1)], [BP_W(2), sternum(2)], 'k--');
        plot([sternum(1), CP_W(1)], [sternum(2), CP_W(2)], 'k--');

        title(sprintf('Shoulder Rotation: %0.1f^o', angles(k)));
        xlabel('X (inches)'); ylabel('Y (inches)'); grid on;
        drawnow;
        pause(pause_time);
    end
end