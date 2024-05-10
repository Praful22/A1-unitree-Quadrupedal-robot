%AME 556 Final Project Initialization Function
%Scott Beck, Sungmo Park, Praful Sigdel
%%
clear;
clc;
close all;

%% Select a Task by uncommenting one line
%Automatically selects correct simulink file and correct parameters

% task = 1; %walking forward
% task = 2; %walking backward
% task = 3; % walking sideways
% task = 4; %spinning in place
% task = 5; %running
 task = 6; %stairs
% task = 7; %obstacle course

%% Initial Conditions and Desired Final Joint Angles

%Initial Condition Estimated from pictures, trial and error
q_hip_roll_i = 0 ;
q_hip_pitch_i = pi/4;
q_knee_i = -pi/2;

%Initial trunk height determined from setting q_i above, seeing how much the feet intersect with
%the ground, then adjusting manually
z_trunk_i = .302840;

%Initial Trunk Position - Used in generating xD
x0_Trunk = [0;0;z_trunk_i; 0;0;0];

%MPC Parameters
NMPC = 10;
dtMPC = 0.03;

%Tunable parameters for swing controller and foot placement
kp_swing = 300;
kd_swing = 10;
kStep = 0.001;

%Setting the stair distance from the robot (used in task 6 only)
firstStairDist = 0.26;

if task == 1 % Walking Forward

    %Trotting Gait (no flight phase)
    tCycle = 0.3;
    tSwing = tCycle/2;
    offsets = [0;tSwing;tSwing;0];

    %Set time to reach desired velocity
    tRamp = 1.0;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 1000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [0.75; 0; 0; 0; 0; 0]; 
end

if task == 2 %Walking Backward
    
    %Trotting Gait (no flight phase)
    tCycle = 0.3;
    tSwing = tCycle/2;
    offsets = [0;tSwing;tSwing;0];

    %Set time to reach desired velocity    
    tRamp = 1.0;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 1000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [-0.75; 0; 0; 0; 0; 0]; 
end

if task == 3 %Walking Sideways

    %Trotting Gait (no flight phase)
    tCycle = 0.3;
    tSwing = tCycle/2;
    offsets = [0;tSwing;tSwing;0];

    %Set time to reach desired velocity    
    tRamp = 1.0;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 1000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [0; 0.75; 0; 0; 0; 0]; 
end

if task == 4 %Turning in Place

    %Trotting Gait (no flight phase)   
    tCycle = 0.3;
    tSwing = tCycle/2;
    offsets = [0;tSwing;tSwing;0];
    
    %Set time to reach desired velocity (small for this task)   
    tRamp = 0.1;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 3000 3000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [0; 0; 0; 0; 0; 0.75]; 
end

if task == 5 %Running

    %Trotting Gait (with flight phase)
    tCycle = 0.30;
    tSwing = tCycle/2 + 0.06;
    offsets = [0;tSwing-0.06;tSwing-0.06;0];
    
    %Set time to reach desired velocity (small for this task)
    tRamp = 1.5;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 3000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [2.5; 0; 0; 0; 0; 0]; 
end

if task == 6 %Stairs

    %Walking Gait (one foot moved at a time)
    tCycle = 0.36;
    tSwing = tCycle/4;
    offsets = [0;2*tSwing;3*tSwing;tSwing];

    %Initial desired value of trunk is different for this task
    x0_Trunk = [0;0;z_trunk_i-.15; 0;-0.271;0];

    %Set time to reach desired velocity (small for this task)
    tRamp = 0.25;
    
    %Given weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 3000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    Tdes = 4;
    xfinal = 1 + firstStairDist;
    zfinal = 0.5+z_trunk_i-.15;
    dq_d_COM = [xfinal/Tdes; 0; zfinal/Tdes; 0; 0; 0]; 
end

if task == 7 %Obstacle Course
    %Joint cnfiguration for Crouching

    %Front Joints
    q_hip_roll_f = 0;
    q_hip_pitch_f =  1.08;
    q_knee_f = -1.77;
    %Rear Joints
    q_hip_roll_r = 0 ;
    q_hip_pitch_r = 1.34;
    q_knee_r = -2.25;
    %Concatenate
    q_d_crouch =[q_hip_roll_f;
        q_hip_pitch_f;
        q_knee_f;
        q_hip_roll_r;
        q_hip_pitch_r;
        q_knee_r];
    q_d_stand=[q_hip_roll_i;q_hip_pitch_i;q_knee_i];

    % Desired Pitch and Z Value when crouched
    Ry = -0.206;
    z_trunk_f = .1836;

    %Joint Configruration for Pre-Impact 
    
    %Front Joints
    q_hip_roll_i_f = 0 ;
    q_hip_pitch_i_f =  0.2;
    q_knee_i_f = -pi*0.4;
    %Rear Joints
    q_hip_roll_i_r = 0 ;
    q_hip_pitch_i_r = 0;
    q_knee_i_r = -pi/3;
    %Concatenate
    q_d_impact =[q_hip_roll_i_f;
        q_hip_pitch_i_f;
        q_knee_i_f;
        q_hip_roll_i_r;
        q_hip_pitch_i_r;
        q_knee_i_r];

    %Timing for Crouch and Jump
    tcrouch = 1.0;
    fFofft = 0.09+tcrouch;
    rFofft = 0.12+tcrouch;
    rlandt = 0.8+tcrouch;
    flandt = 0.8+tcrouch;
    tImpact = 0.05;
    flight_gait_t =[tcrouch;fFofft;rFofft;rlandt;flandt; tImpact];

    %Timing for "High" Jump in Obstacle Course (only used in task = 7)
    tcrouch2 = 1.3;
    fFofft2 = 0.09+tcrouch2;
    rFofft2 = 0.12+tcrouch2;
    rlandt2 = 0.84+tcrouch2;
    flandt2 = 0.84+tcrouch2;
    tImpact = 0.05;
    flight_gait_2 =[tcrouch2;fFofft2;rFofft2;rlandt2;flandt2; tImpact];

    %Trotting Gait for Walking Motion
    tCycle = 0.3;
    tSwing = tCycle/2;
    offsets = [0;tSwing;tSwing;0];

    %Time to reach desired velocities for each phase (and a time to slow
    %down prior to each jump) - used to generate xD
    tRamp = 0.5;
    tSlow = 0.3;
    tRamp_Jump = 0.05;
    tRamp_Jump2 = 0.08;
    
    %MPC weights for states with extra 0 weight for g
    Qmpc = diag([100 100 50 1000 1000 500 4 4 4 1 1 1 0]);
    Rmpc = 0.00001*eye(12);

    %Desired Velocity for Task (used to generate xD)
    dq_d_COM = [0.75; 0; 0; 0; 0; 0];
    dq_d_COM_Jump = [10; 0; 20; 0; -30; 0]; 
    dq_d_COM_Jump_2 = [12; 0; 21.2; 0; -35.6; 0];

    %Switching map between Walking Controller and Jumping Controller
    modeMap = [ 0 0;
                1.2 1;
                3.8 0;
                4.1 1;
                6.2 0;
                12.27 2;
                15.5 0;
                100.0 0];
end

%% Simulate the Robot!
if task == 6
    out = sim("FinalProjectSimulinkModel_STAIRS.slx");
elseif task == 7
    out = sim("FinalProjectSimulinkModel_OBSTACLE.slx");
else
    out = sim("FinalProjectSimulinkModel_FLATGROUND.slx");
end

%% Extract results from out structure
tOut = out.tout;

q_hip_roll_out = out.qJOut(:,[1, 4, 7, 10]); 
q_hip_pitch_out = out.qJOut(:,[2, 5, 8, 11]); 
q_knee_out = out.qJOut(:,[3, 6, 9, 12]); 

dq_hip_roll_out = out.dqJOut(:,[1, 4, 7, 10]); 
dq_hip_pitch_out = out.dqJOut(:,[2, 5, 8, 11]); 
dq_knee_out = out.dqJOut(:,[3, 6, 9, 12]); 

tauOutMat = squeeze(out.tauOut);

t_hip_roll = tauOutMat([1, 4, 7, 10],:)';
t_hip_pitch = tauOutMat([2, 5, 8, 11],:)';
t_knee = tauOutMat([3, 6, 9, 12],:)';

p_COM_Out = squeeze(out.qOut(1:3,:,:))';
R_COM_Out = squeeze(out.qOut(4:end,:,:))';
v_COM_Out = squeeze(out.dqOut(1:3,:,:))';
wb_COM_Out = squeeze(out.dqOut(4:6,:,:))';

%% Plot COM Position
% goalIdx = find(p_COM_Out(:,1)>10.0, 1,"first");
% tGoal = tOut(goalIdx);
figure();
clf();
hold on
for ii = 1:3
    plot(tOut, p_COM_Out(:,ii), "LineWidth",1.5)
end
% yline(10.0, '--r',"LineWidth",1.0);
% xline(tGoal,'--b',"LineWidth",1.0);
hold off
xlabel("Time (s)")
ylabel("COM Position (m)")
legend("X_{COM}", "Y_{COM}", "Z_{COM}", "Goal Distance", "t_{Complete}")
title("COM Position vs Time - Stairs");
fontsize(gcf,"scale",1.5)

%% Plot COM Linear Velocity
figure();
clf();
hold on
for ii = 1:3
    plot(tOut, v_COM_Out(:,ii), "LineWidth",1.5)
end
%yline(0.5, '--r',"LineWidth",1.0);
hold off
xlabel("Time (s)")
ylabel("COM Velocity (m/s)")
legend("vX_{COM}", "vY_{COM}", "vZ_{COM}")
title("COM Velocity vs Time - Obstacle Course");
fontsize(gcf,"scale",1.5)

%% Plot COM Angular Velocity
figure();
clf();
hold on
for ii = 1:3
    plot(tOut, wb_COM_Out(:,ii), "LineWidth",1.5)
end
yline(0.5, '--r',"LineWidth",1.0);
hold off
xlabel("Time (s)")
ylabel("COM Angular Velocity (rad/s)")
legend("wRoll_{COM}", "wPitch_{COM}", "wYaw_{COM}","Required Yaw Rate")
title("COM Velocity vs Time - Turning in Place");
fontsize(gcf,"scale",1.5)

%% Plot COM Euler Angles

% Convert Rotation Matrix to Euler Angles
tic
eul_COM_out = zeros(3,length(tOut));
for ii = 1:length(tOut)
    eul_COM_out(:,ii) = flip(rotm2eul(reshape(R_COM_Out(ii,:),[3,3])));
end
toc

%Plot
figure()
clf();
hold on
for ii = 1:3
    plot(tOut, unwrap(eul_COM_out(ii,:)), "LineWidth",1.5)
end
ylabel("COM Orientation (rad)")
legend("Roll_{COM}", "Pitch_{COM}", "Yaw_{COM}")
title("COM Orientation vs Time - Turning in Place");
fontsize(gcf,"scale",1.5);