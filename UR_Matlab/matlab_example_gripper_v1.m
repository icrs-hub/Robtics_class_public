%% SCRIPT_DrawCircle
% This script demonstrates use of the URsim class capabilities with the UR
% class. The URsim object is used to visualize the robot and calculate
% inverse kinematics for the system. Waypoints are executed by getting
% joint angle from the simulation object and sending them to the
% URToolboxScript controller running on the UR control box.
%
%   M. Kutzer, 14Mar2018, USNA
clearvars

% clearvars -EXCEPT hwObj
close all
clc

%% Create hardware flag (for debugging)
% -> Set value to true if you are connected to hardware
useHardware = false;
%% Initialize simulation
% -> This code has been tested on the UR5 and UR10 systems. Minor
% adjustments may be required to use it with the UR3.

if ~exist('simObj')
    % Create object
    simObj = URsim;
    % Initialize simulation
    simObj.Initialize;
    % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
    % visualization)
    %     simObj.FrameT = Tz(160);
    
    % Hide frames
    frames = '0123456E';
    for i = 1:numel(frames)
        hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
    end
end

%% Connect to hardware
% -> The message to the user *assumes* that you have:
%       (1) Properly installed URToolboxScript on the UR controller. See
%       the link below for instructions:
%
%       https://www.usna.edu/Users/weapsys/kutzer/_Code-Development/UR_Toolbox.php
%
%       (2) Configure the network card connecting the PC to the UR
%       controller to a static IP of 10.1.1.5
%
%       (3) Set the UR controller IP to 10.1.1.2

if ~exist('hwObj') && useHardware
    instruct = sprintf([...
        '\tPython module imported.\n',...
        '\tEnter server IP address: 10.1.1.5\n',...
        '\tEnter port: 30002\n',...
        '\tEnter number of connections to be made: 1\n',...
        '\tServer created.\n',...
        '\tBegin onboard controller, then press ENTER.\n',...
        '\tConnections established.\n',...
        '\tWould you like to create a URX connection as well? y\n',...
        '\tEnter URX address: 10.1.1.2\n',...
        '\tURX connection established.\n']);
    fprintf('PLEASE USE THE FOLLOWING RESPONSES:\n\n');
    fprintf(2,'%s\n\n',instruct)
    
    hwObj = UR;
end


%% Animate simulation and move the robot to test
% 홈포지션으로 가기 [0,-1.57,0,-1.57,0,0]
% Home simulation
simObj.Home;

simObj.FrameT(1,4)=[0];  % End-effector Frame부터의 Tool의 X위치 [mm]
simObj.FrameT(2,4)=[0];  % End-effector Frame부터의 Tool의 Y위치 [mm]
simObj.FrameT(3,4)=[140];  % End-effector Frame부터의 Tool의 Z위치 [mm]
grid()
drawnow

% 그리퍼 닫기 
pause(1)
if useHardware
msg(hwObj,sprintf('(5,%f,%f,%f,%f,%f,%f)',[0,0,0,0,0,0]));
end
pause(1)


if useHardware
    msg(hwObj,sprintf('(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)',[0,-1.57,0,-1.57,0,0],zeros(1,6)));
    X_all = [];
    isMoving = true;
    while isMoving
        real_tool_xyz = cellfun(@double,cell(hwObj.TPOS));
        real_x1 = real_tool_xyz(1,1)*1000;
        real_y1 = real_tool_xyz(1,2)*1000;
        real_z1 =real_tool_xyz(1,3)*1000;
        X_all(1,end+1) = real_x1;
        X_all(2,end) = real_y1;
        X_all(3,end) = real_z1;
        X_all(1,end+1) = simObj.ToolPose(1,4);
        X_all(2,end) = simObj.ToolPose(2,4);
        X_all(3,end) = simObj.ToolPose(3,4);
        if size(X_all,2) > 1
            v = X_all(:,end) - X_all(:,end-1);
            d = norm(v);
        else
            d = inf;
        end
        if d < 10
            isMoving = false;
        end
    end
    fprintf('COMPLETE\n');
end

t1=clock; % 초기시각
t2=clock; % 현재시각
syms x % 변수 x 지정
% AA 건드리지 말 것
% || @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

% 그리퍼 열기 
pause(1)
if useHardware
msg(hwObj,sprintf('(4,%f,%f,%f,%f,%f,%f)',[0,0,0,0,0,0]));
end
pause(1)

%% 1단계
eq1_1 = 0*x ;                      %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq1_2 = -1.57 +0*x ;               %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq1_3 =  -1.57*sin(0.314*x) ;      %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq1_4 = -1.57 +0*x ;       %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq1_5 =  0*x ;                     %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq1_6 = 0*x ;                      %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
sum_t1=[];                                          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
col_count=100; % 열 갯수
eq1 = 12:col_count+1;                               %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
% Allow plot to update
if useHardware
    joint_vel1 = numel(eq1(1,:)):numel(hwObj.JVELS);%% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
end
% Move through waypoints
i =1;
while etime(t2,t1) < 5                              %% !! 부등호 오른편에 있는 숫자는 시간[초] 
    % || @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    % VV 식을 건드리지 말 것
    t2=clock; % 현재시각 갱신
    sum_t1(i) = etime(t2,t1);                       % 초기 - 현재 시각[초] 
    eq1(1,i) = subs(eq1_1,sum_t1(i)) ;             
    eq1(2,i) = subs(eq1_2,sum_t1(i)) ;              
    eq1(3,i) = subs(eq1_3,sum_t1(i)) ;              
    eq1(4,i) = subs(eq1_4,sum_t1(i));               
    eq1(5,i) = subs(eq1_5,sum_t1(i));               
    eq1(6,i) = subs(eq1_6,sum_t1(i)) ;            
    % Set simulation toolpose to waypoint pose
    simObj.Joints = [eq1(1,i) eq1(2,i) eq1(3,i) eq1(4,i) eq1(5,i) eq1(6,i)];
    plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.m');
    % Move robot to match simulation
    q = simObj.Joints;
    if useHardware
        a = cellfun(@double,cell(hwObj.JVELS));
        for j = 1:6
            joint_vel1(j,i) = a(j);    
        end
        real_tool_xyz = cellfun(@double,cell(hwObj.TPOS));
        real_x1 = real_tool_xyz(1,1)*1000;          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        real_y1 = real_tool_xyz(1,2)*1000;          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        real_z1 =real_tool_xyz(1,3)*1000;           %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        plt_Waypoints = plot3(simObj.Axes,real_x1,real_y1,real_z1,'.b');
        % Get joint position from the simulation
        q = simObj.Joints;
        % Send waypoint to the robot
        msg(hwObj,sprintf('(1,%f,%f,%f,%f,%f,%f)',q));
        fprintf('COMPLETE\n');
    end
    drawnow;
    t2=clock; % 현재시각 갱신
    i=i+1;
end
% || @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
% AA 식을 건드리지 말 것


% 그리퍼 닫기 
pause(1)
if useHardware
msg(hwObj,sprintf('(5,%f,%f,%f,%f,%f,%f)',[0,0,0,0,0,0]));
end
pause(1)

%% 2단계

eq2_1 = 1.57*sin(0.314*x)-1.57;       %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq2_2 = -1.57 +0*x ;               %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq2_3 = -1.57 +0*x ;               %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq2_4 = -1.57 +0*x ;               %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq2_5 =  0*x ;                     %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
eq2_6 = 0*x ;                      %% 수식입력[rad]  %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
sum_t2=[];                                          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
col_count=100; % 열 갯수
eq2 = 12:col_count+1;                               %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
% Allow plot to update
if useHardware
    joint_vel1 = numel(eq2(1,:)):numel(hwObj.JVELS);%% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
end
% Move through waypoints
i =1;
while etime(t2,t1) < 10                             %% !! 부등호 오른편에 있는 숫자는 시간[초] 
    % || @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    % VV 식을 건드리지 말 것
    t2=clock; % 현재시각 갱신
    sum_t2(i) = etime(t2,t1);                       % 초기 - 현재 시각[초] 
    eq2(1,i) = subs(eq2_1,sum_t2(i)) ;             
    eq2(2,i) = subs(eq2_2,sum_t2(i)) ;              
    eq2(3,i) = subs(eq2_3,sum_t2(i)) ;              
    eq2(4,i) = subs(eq2_4,sum_t2(i));               
    eq2(5,i) = subs(eq2_5,sum_t2(i));               
    eq2(6,i) = subs(eq2_6,sum_t2(i)) ;        
    % Set simulation toolpose to waypoint pose
    simObj.Joints = [eq2(1,i) eq2(2,i) eq2(3,i) eq2(4,i) eq2(5,i) eq2(6,i)];
    plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.m');
    % Move robot to match simulation
    q = simObj.Joints;
    if useHardware
        a = cellfun(@double,cell(hwObj.JVELS));
        for j = 1:6
            joint_vel2(j,i) = a(j);    
        end
        real_tool_xyz = cellfun(@double,cell(hwObj.TPOS));
        real_x2 = real_tool_xyz(1,1)*1000;          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        real_y2 = real_tool_xyz(1,2)*1000;          %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        real_z2 =real_tool_xyz(1,3)*1000;           %% !!!!! 변수명 뒤 스텝별로 숫자 바꾸기
        plt_Waypoints = plot3(simObj.Axes,real_x2,real_y2,real_z2,'.b');
        % Get joint position from the simulation
        q = simObj.Joints;
        % Send waypoint to the robot
        msg(hwObj,sprintf('(1,%f,%f,%f,%f,%f,%f)',q));
        fprintf('COMPLETE\n');
    end
    drawnow;
    t2=clock; % 현재시각 갱신
    i=i+1;
end
% || @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
% AA 식을 건드리지 말 것
simObj.ToolPose()
simObj.ToolPose(1,4)
simObj.ToolPose(2,4)
simObj.ToolPose(3,4)
fprintf('Done')
