clc;
clear;
close all;

addpath('./lib');
%% DEFINE
R2D = 180/pi; % radians to degrees
D2R = pi/180;

num_drones = 3;
num_targets = 3;

%% INITIALIZING PARAMETERS regarding the drone

drone_params = containers.Map({'mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
    {1.25, 0.265, 0.0232, 0.0232, 0.0468}); % Similar to dictionary in python

drone_initInputs = [0, 0, 0, 0]';      % u1, u2, u3, u4 (T, M1, M2, M3)
% T = total thrust and M's are moments on each control axis
drone_body = [ 0.265,  0,      0, 1; ... 
               0,      -0.265, 0, 1; ...
               -0.265, 0,      0, 1; ...
               0,      0.265,  0, 1; ...
               0,      0,      0, 1; ... %Center of Drone
               0,      0,   0.15, 1]';
% PID gains            
drone_gains = containers.Map(...  
    {'P_phi',  'I_phi',   'D_phi', ...      %From attitude controler
    'P_theta', 'I_theta', 'D_theta', ...    %From attitude controler
    'P_psi',   'I_psi',   'D_psi', ...      %From attitude controler
    'P_x',     'I_x',     'D_x',...         %From position controler
    'P_y',     'I_y',     'D_y',...         %From position controler
    'P_z',     'I_z',     'D_z'},...        %From position controler
    {19,         0.0,       40, ...
     18,         0.0,       40, ...
     14,         0.0,       40, ...        
     20,         0.0,       35,...
     20,         0.0,       35,...
     20,         0.0,       35});        

simulationTime = 3000; %milliseconds

swarm(num_drones) = {NaN};
for i = 1:num_drones
    xrng = 10*rand;
    yrng = 10*rand;
    zrng = 10*rand;
    drone_initStates = [xrng, yrng, zrng, ...       % starting X,Y,Z position
                        0, 0, 0, ...            % dX, dY, dZ (velocities)
                        0, 0, 0, ...            % phi, theta, psi (euler angles)
                        0, 0, 0]';              % p, q, r (angular rates)
    swarm(i) = {Drone(drone_params, drone_initStates, drone_initInputs, drone_gains, simulationTime)};
end

targets(num_targets) = {NaN};
for i = 1:num_targets
    xrng = 1+rand*8;
    yrng = 1+rand*8;
    zrng = 1+rand*8;
    target_initPos = [xrng, yrng, zrng];
    targets(i) = {Target(target_initPos)};
end

%% Init. 3D Fig.
fig1 = figure('pos', [0 200 800 800]);
h = gca;            
view(3); 

axis equal;
grid on;

xlim([0 10]);
ylim([0 10]);
zlim([0 10]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
hold(gca, 'on');

for i = 1:num_drones
    drone_state = swarm{i}.GetState();
    wHb = [RPY2Rot(drone_state(7:9))' drone_state(1:3); 0 0 0 1];
    drone_world = wHb * drone_body;
    drone_atti = drone_world(1:3, :);
    
    fig_ARM13(i) = plot3(gca, drone_atti(1,[1 3]), drone_atti(2, [1 3]), drone_atti(3, [1 3]), '-ro', 'MarkerSize', 5);
    fig_ARM24(i) = plot3(gca, drone_atti(1,[2 4]), drone_atti(2, [2 4]), drone_atti(3, [2 4]), '-bo', 'MarkerSize', 5);
    fig_payload(i) = plot3(gca, drone_atti(1,[5 6]), drone_atti(2, [5 6]), drone_atti(3, [5 6]), '-k', 'MarkerSize', 5);
    fig_shadow(i) = plot3(gca, 0, 0, 0, 'xk', 'LineWidth', 3);
end

for i = 1:num_targets
    tar_pos = targets{i}.GetPos;
    [x, y, z] = sphere;
    fig_tar(i) = surf(0.5*x+tar_pos(1), 0.5*y+tar_pos(2), 0.5*z+tar_pos(3));
    %fig_target(i) = plot3(gca, tar_pos(1), tar_pos(2), tar_pos(3),'o', 'MarkerSize', 20, 'MarkerFaceColor', 'red');
end
hold(gca, 'off');

%% Init. Data Fig.
% fig2 = figure('pos', [800 550 800 450]);
% subplot(2,3,1);
% title('phi[deg]');
% grid on;
% hold on;
% subplot(2,3,2)
% title('theta[deg]');
% grid on;
% hold on;
% subplot(2,3,3);
% title('psi[deg]');
% grid on;
% hold on;
% subplot(2,3,4);
% title('x[m]');
% grid on;
% hold on;
% subplot(2,3,5);
% title('y[m]');
% grid on;
% hold on;
% subplot(2,3,6);
% title('zdot[m/s]');
% grid on;
% hold on;

% Desired position at 0,0,0 
commandSig = targets{1}.GetPos

myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for i = 1:simulationTime
    

    for mem = 1:length(swarm)
        drone_vel = swarm{mem}.GetState();
        drone_velocity = norm(drone_vel(4:6));
        swarm{mem}.PositionCtrl(commandSig);
        swarm{mem}.AttitudeCtrl();
        swarm{mem}.UpdateState();
        for tar = 1:length(targets)
            swarm{mem}.DistTar(targets{tar});
            targets{tar}.TestCollision(swarm{mem});
        end
        
        for tar = 1:length(targets)
            if targets{tar}.IsMapped()
                targets(tar) = [];
            end
        end
    end
    
    %% 3D plot
    if mod(simulationTime,100) == 0 % Generate frame every 100 time steps
        for mem = 1:length(swarm)
            figure(1)
            drone_state = swarm{mem}.GetState();
            wHb = [RPY2Rot(drone_state(7:9))' drone_state(1:3); 0 0 0 1];
            drone_world = wHb * drone_body;
            drone_atti = drone_world(1:3, :);
            
            set(fig_ARM13(mem), ...
                'xData', drone_atti(1,[1 3]), ...
                'yData', drone_atti(2,[1 3]), ...
                'zData', drone_atti(3,[1 3]));
            set(fig_ARM24(mem), ...
                'xData', drone_atti(1,[2 4]), ...
                'yData', drone_atti(2,[2 4]), ...
                'zData', drone_atti(3,[2 4]));
            set(fig_payload(mem), ...
                'xData', drone_atti(1,[5 6]), ...
                'yData', drone_atti(2,[5 6]), ...
                'zData', drone_atti(3,[5 6]));
            set(fig_shadow(mem), ...
                'xData', drone_state(1), ...
                'yData', drone_state(2), ...
                'zData', 0); 
        end
    end
    
    if isempty(targets)
        break;
    end
    
        pause(0.01) %Pause and grab frame
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
%         end

        %     figure(2)
        %     subplot(2,3,1);
        %         plot(i/100, drone1_state(7)*R2D, '.');
        %     subplot(2,3,2)
        %         plot(i/100, drone1_state(8)*R2D, '.');
        %     subplot(2,3,3);
        %         plot(i/100, drone1_state(9)*R2D, '.');
        %     subplot(2,3,4);
        %         plot(i/100, drone1_state(1), '.');
        %     subplot(2,3,5);
        %         plot(i/100, drone1_state(2), '.');
        %     subplot(2,3,6);
        %         plot(i/100, drone1_state(3), '.');
%         if (drone_state(3) <= 0)
%             msgbox('Crashed', 'Error', 'error');
%             break;
%         end
end
close(myVideo)




