% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: Simulated IMU data
% Models: State dynamics, error, complementary filter
% Instructions: A call would look like:
%   ComplementaryFilter(simulatedData(5,1)) where the 1 is to close plots

function [results] = ComplementaryFilter(simimu,varargin)
% simimu.Qgyro
% simimu.Qacc
% simimu.Qbias
% simimu.realeulerrad
% simimu.dynaccGlobal
% simimu.gyro
% simimu.acc
% simimu.t
% simimu.sampfreq
% simimu.gyronoisestd
% simimu.gyrobiasdriftstd
% simimu.accnoisestd

    % Setup
        % Plot imu data (via simulatedData.m)
time = simimu.t;
    % Method
        % for each time step, calculate yaw
            % if a is above a threshold, use gyro data
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        
        % vary gamma
gamma = .98;
yaw = zeros(size(time));
yawTemp = 0;

for ii = 1:size(time)
    yawTemp = yawTemp + simimu.gyro(ii, 3) * simimu.sampfreq; %*pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; % Angle around the X-axis
    
    if simimu.acc(ii, 2) > .1  % y acc threshhold
        yawAcc = atan(simimu.acc(ii, 2)) * 180 / pi;
        yaw(ii) = yawTemp * gamma + yawAcc * (1-gamma);
    else
        yaw(ii) = yawTemp;
    end
end
    % Plot
f = figure('Name','Pitch vs. Time'); %New fig
set(f, 'Position', [100, 100, 1049, 895]);

subplot(2,2,1);
plot(time, simimu.truegyro(:,3));
title('Yaw');
legend('IMU Data')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,2);
plot(time, rad2deg(yaw));
title('Complementary Filter Yaw');
legend('Complementary Filter')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,3);
plot(time, (rad2deg(simimu.truegyro(:,3)) - rad2deg(yaw)));
title('Complementary Filter Yaw Gyro Error');
legend('Error')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,4);
plot(time, (rad2deg(simimu.truegyro(:,3)) - simimu.gyro(:,3)));
title('Yaw Gyro Error');
legend('Error')
xlabel('time (seconds)'); ylabel('degrees/sec');

if(not(isempty(varargin)))
    if varargin{1}==1
        close all;
    end
end

end
