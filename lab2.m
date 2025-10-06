% Lab 2: Coordinate Transforms 3D
% Author: Mikhail R (style matched)
% Date: September 6, 2025

%% --- CONTROL PANEL: Uncomment to run desired lab parts ---
run_step_2_1 = false;
run_step_2_2 = false;
run_step_2_3 = false;
run_step_3_1 = false;
run_step_3_2 = false;
run_step_3_3 = true;

%% Add more function calls as you implement each part
if run_step_2_1
    step_2_1_plot_cube_at_origin();
end
if run_step_2_2
    step_2_2_translate_and_rotate_C();
end
if run_step_2_3
    step_2_3_translate_and_rotate_W();
end
if run_step_3_1
    step_3_1_robot_stack_and_plot();
end
if run_step_3_2
    step_3_2_print_transform_chains();
end
if run_step_3_3
    step_3_3_robot_rotate_W();
end

%% --- Local Functions ---
function step_2_1_plot_cube_at_origin()
    % Step 2.1: Set origin W, set cube frame C, draw cube and edges (RTK style)
    figure; plotvol([0 5 0 5 0 5]); hold on;
    view(115,25);
    W = SE3(0,0,0); % World frame
    trplot(W, 'frame', 'W', 'color', 'k');
    C = SE3(0,0,0); % Cube frame at origin
    trplot(C, 'frame', 'C', 'color', 'b');
    % Define cube corners in {C} frame
    cube_C = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]'; % 3x8
    % Transform corners to world frame (initially C=W)
    cube_W = C * cube_C;
    cube_W = cube_W'; % 8x3
    plot3(cube_W(:,1), cube_W(:,2), cube_W(:,3), 'bo', 'MarkerFaceColor', 'b');
    % Draw cube edges
    cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    for i = 1:size(cube_edges,1)
        plot3(cube_W(cube_edges(i,:),1), cube_W(cube_edges(i,:),2), cube_W(cube_edges(i,:),3), 'b-');
    end
    title('Cube at Origin: Frame {C} and World Frame {W}');
    xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
end

function step_2_2_translate_and_rotate_C()
    % Step 2.2: Animate translation of {C} to (2,2,2) and rotation about Z in {C} (RTK style)
    figure; plotvol([0 5 0 5 0 5]); hold on;
    view(115,25);
    W = SE3(0,0,0); % World frame
    trplot(W, 'frame', 'W', 'color', 'k');
    % Define cube corners in {C}
    cube_pts = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]'; % 3x8
    % Animate translation from (0,0,0) to (2,2,2)
    nTrans = 20;
    trans_vec = [linspace(0,2,nTrans); linspace(0,2,nTrans); linspace(0,2,nTrans)];
    for t = 1:nTrans
        cla;
        plotvol([0 5 0 5 0 5]); hold on;
        view(115,25);
        trplot(W, 'frame', 'W', 'color', 'k');
        C = SE3(trans_vec(1,t), trans_vec(2,t), trans_vec(3,t));
        trplot(C, 'frame', 'C', 'color', 'b');
        ptsW = C * cube_pts;
        ptsW = ptsW'; % 8x3
        plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), 'bo', 'MarkerFaceColor', 'b');
        cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
        for i = 1:size(cube_edges,1)
            plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), 'b-');
        end
        title(sprintf('Translating Cube & Frame {C}: (%.2f, %.2f, %.2f)', trans_vec(1,t), trans_vec(2,t), trans_vec(3,t)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
        drawnow;
        pause(0.05);
    end
    % Animate rotation about Z in {C} at (2,2,2)
    nSteps = 60;
    angles = linspace(0, 360, nSteps);
    for k = 1:nSteps
        cla;
        plotvol([0 5 0 5 0 5]); hold on;
        view(115,25);
        trplot(W, 'frame', 'W', 'color', 'k');
        theta = angles(k);
        C = SE3(2,2,2) * SE3().rpy(0,0,theta,'deg');
        trplot(C, 'frame', 'C', 'color', 'b');
        ptsW = C * cube_pts;
        ptsW = ptsW'; % 8x3
        plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), 'bo', 'MarkerFaceColor', 'b');
        cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
        for i = 1:size(cube_edges,1)
            plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), 'b-');
        end
        title(sprintf('Cube & Frame {C} at (2,2,2), Rotating about Z: %d^o', round(theta)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
        drawnow;
        pause(0.05);
    end
end

function step_2_3_translate_and_rotate_W()
    % Step 2.3: Animate translation to (2,2,2) and rotation about Z in world frame {W} (RTK style)
    DELAY = 0.01;
    VIEW = [-5 5 -5 5 0 5];
    figure; plotvol(VIEW); hold on;
    view(115,25);
    W = SE3(0,0,0); % World frame
    trplot(W, 'frame', 'W', 'color', 'k');
    cube_pts = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]'; % 3x8

    % Animate translation from (0,0,0) to (2,2,2)
    pause(1);
    nTrans = 30;
    trans_vec = [linspace(0,2,nTrans); linspace(0,2,nTrans); linspace(0,2,nTrans)];
    for t = 1:nTrans
        cla;
        plotvol(VIEW); hold on;
        view(115,25);
        trplot(W, 'frame', 'W', 'color', 'k');
        C = SE3(trans_vec(1,t), trans_vec(2,t), trans_vec(3,t));
        trplot(C, 'frame', 'C', 'color', 'b');
        ptsW = C * cube_pts;
        ptsW = ptsW'; % 8x3
        plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), 'bo', 'MarkerFaceColor', 'b');
        cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
        for i = 1:size(cube_edges,1)
            plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), 'b-');
        end
        title(sprintf('Translating Cube & Frame {C}: (%.2f, %.2f, %.2f)', trans_vec(1,t), trans_vec(2,t), trans_vec(3,t)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
        drawnow;
        pause(DELAY);
    end
    
    % Animate rotation about Z in world frame {W} at (2,2,2)
    pause(1);
    nSteps = 60;
    angles = linspace(0, 360, nSteps);
    for k = 1:nSteps
        cla;
        plotvol(VIEW); hold on;
        view(115,25);
        trplot(W, 'frame', 'W', 'color', 'k');
        theta = angles(k);
        % Rotation about Z in world frame: R * T
        C = SE3().rpy(0,0,theta,'deg') * SE3(2,2,2);
        trplot(C, 'frame', 'C', 'color', 'b');
        ptsW = C * cube_pts;
        ptsW = ptsW'; % 8x3
        plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), 'bo', 'MarkerFaceColor', 'b');
        cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
        for i = 1:size(cube_edges,1)
            plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), 'b-');
        end
        title(sprintf('Cube & Frame {C} at (2,2,2), Rotating about Z in {W}: %d^o', round(theta)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
        drawnow;
        pause(DELAY);
    end
end

function step_3_1_robot_stack_and_plot()
    % Step 3.1: Plot 6 stacked cubes with frames T1-T6 (RTK style, no animation)
    VIEW = [-2 5 -1 5 0 5];
    figure; plotvol(VIEW); hold on;
    view(130,10);
    W = SE3(0,0,0); % World frame
    trplot(W, 'frame', 'W', 'color', 'k');
    cube_pts = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]'; % 3x8
    cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    T = cell(1,6);
    colors = {'r','g','b','m','c','y'}; % Unique colors for each cube/frame
    positions = [0 0 0; 0 0 1; 0 0 2; 0 0 3; 1 0 2; -1 0 2]; % T1-T4 stack, T5/T6 at sides of T3
    for n = 1:6
        T{n} = SE3(positions(n,1), positions(n,2), positions(n,3));
        trplot(T{n}, 'frame', sprintf('T%d',n), 'color', colors{n});
        ptsW = T{n} * cube_pts;
        ptsW = ptsW'; % 8x3
        plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), [colors{n} 'o'], 'MarkerFaceColor', colors{n});
        for i = 1:size(cube_edges,1)
            plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), [colors{n} '-']);
        end
    end
    title('Robot Stack: Frames T1-T6');
    xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
end

function step_3_2_print_transform_chains()
    % Step 3.2: Print transformation matrices from each cube frame Tn to world frame W
    positions = [0 0 0; 0 0 1; 0 0 2; 0 0 3; 1 0 2; -1 0 2];
    for n = 1:6
        Tn = SE3(positions(n,1), positions(n,2), positions(n,3));
        fprintf('--- Cube T%d to W ---\n', n);
        fprintf('Translation vector (x, y, z):\n');
        disp(Tn.t');
        fprintf('Rotation matrix (3x3):\n');
        disp(Tn.R);
        fprintf('Full transformation matrix (4x4):\n');
        disp(Tn.T);
        fprintf('\n');
    end
end

function step_3_3_robot_rotate_W()
    % Step 3.3: Animate rotation of all cubes about Z axis in world frame (RTK style)
    VIEW = [-5 5 -5 5 0 5];
    DELAY = 0.01;
    figure; plotvol(VIEW); hold on;
    view(115,25);
    W = SE3(0,0,0); % World frame
    trplot(W, 'frame', 'W', 'color', 'k');
    cube_pts = [0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1]';
    cube_edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
    colors = {'r','g','b','m','c','y'};
    positions = [0 0 0; 0 0 1; 0 0 2; 0 0 3; 1 0 2; -1 0 2];
    nSteps = 60;
    angles = linspace(0, 360, nSteps);
    pause(1);
    for k = 1:nSteps
        cla;
        plotvol(VIEW); hold on;
        view(115,25);
        trplot(W, 'frame', 'W', 'color', 'k');
        theta = angles(k);
        for n = 1:6
            Tn = SE3().rpy(0,0,theta,'deg') * SE3(positions(n,1), positions(n,2), positions(n,3));
            trplot(Tn, 'frame', sprintf('T%d',n), 'color', colors{n});
            ptsW = Tn * cube_pts;
            ptsW = ptsW';
            plot3(ptsW(:,1), ptsW(:,2), ptsW(:,3), [colors{n} 'o'], 'MarkerFaceColor', colors{n});
            for i = 1:size(cube_edges,1)
                plot3(ptsW(cube_edges(i,:),1), ptsW(cube_edges(i,:),2), ptsW(cube_edges(i,:),3), [colors{n} '-']);
            end
        end
        title(sprintf('Robot Stack Rotating about Z in {W}: %d^o', round(theta)));
        xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
        drawnow;
        pause(DELAY);
    end
end

%{

%}