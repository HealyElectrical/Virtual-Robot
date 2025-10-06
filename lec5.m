%% --- CONTROL PANEL: Uncomment to run desired lab parts ---
run_step_1 = false;
run_step_2 = true;
run_step_3 = false;

%% Add more function calls as you implement each part
if run_step_1
    step1();
end
if run_step_2
    step2();
end
if run_step_3
    step3();
end

%% --- Local Functions for lab 3 ---

function step1() % Perspective Camera Model Example
    P1 = [-1,-1,-1,1]'; % it seems to be each of the 8 points of the cube with the dummy variable Q. could be generated in one for-loop
    P2 = [1,-1,-1,1]';
    P3 = [1,1,-1,1]';
    P4 = [-1,1,-1,1]';
    P5 = [-1,-1,1,1]';
    P6 = [1,-1,1,1]';
    P7 = [1,1,1,1]';
    P8 = [-1,1,1,1]';
    pts= [ (P1)'; (P2)'; (P3)'; (P4)'; (P1)'; (P5)'; (P6)'; (P2)'; (P6)'; (P7)'; (P3)'; (P7)'; (P8)'; (P4)'; (P8)'; (P5)'; ] % Prob similar to robot.cpp

    plot3 (pts(:,1), pts(:,2), pts(:,3), 'b-');
    plot (pts(:,1), pts(:,2), 'b-');

    cam = CentralCamera('focal', 0.015) % didn't have a semi-colon

    theta = 0;
    for theta=0:0.1:pi/2
        Tcam = SE3(0,0,-2)*SE3.rpy(0,theta,0); % ??
        P = []; 
        for i=1:size(pts,1)
            P = [P; (cam.C*Tcam.T*pts(i,:)')']; % ??
        end

        x = P(:,1)./P(:,3); % ??
        y = P(:,2)./P(:,3); % ??

        plot(x,y,'ob-');
        axis([-0.1 0.1 -0.1 0.1]);
        pause(0.1);
    end

    cam.C % make sure to test "cam.C"
end

function step2() % digital version of step 1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P1 = [-1,-1,-1,1]'; % it seems to be each of the 8 points of the cube with the dummy variable Q. could be generated in one for-loop
    P2 = [1,-1,-1,1]';
    P3 = [1,1,-1,1]';
    P4 = [-1,1,-1,1]';
    P5 = [-1,-1,1,1]';
    P6 = [1,-1,1,1]';
    P7 = [1,1,1,1]';
    P8 = [-1,1,1,1]';
    pts= [ (P1)'; (P2)'; (P3)'; (P4)'; (P1)'; (P5)'; (P6)'; (P2)'; (P6)'; (P7)'; (P3)'; (P7)'; (P8)'; (P4)'; (P8)'; (P5)'; ] % Prob similar to robot.cpp

    plot3 (pts(:,1), pts(:,2), pts(:,3), 'b-');
    plot (pts(:,1), pts(:,2), 'b-');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cam = CentralCamera('focal', 0.003, 'pixel', 4.6e-6, 'resolution', [1920 1080], 'centre', [1920 1080]/2) % no break

    theta = 0;
    for theta=0:0.1:pi/2
        Tcam = SE3(0,0,-2)*SE3.rpy(0,theta,0);
        P = [];

        for i=1:size(pts,1)
            P = [P; (cam.C*Tcam.T*pts(i,:)')'];
        end

        x = P(:,1)./P(:,3); % ??
        y = P(:,2)./P(:,3); % ??
        plot(x,y,'ob-');

        % big picture
        axis([-2000 20000 -2000 2000]) % no break
        
        %image
        axis([0 1920 0 1080]) % no break
        pause(0.1) % no break
    end

    cam.C % make sure to test "cam.C"
end

%% --- Local Functions for lab 4 ---

function step3() % MatLab camera calibration
    display('yo');
end

function step4() % image distortion correction (may not work alone...)
    display('yo');
end

function step5() % Pose estimation (cube)
    display('yo');
end
