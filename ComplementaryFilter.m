% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: True virtual fish trajectory data, simulated trajectory data
% Models: State dynamics, error, complementary filter
% Instructions: A call would look like:
%   ComplementaryFilter(simulatedData(5,1))  - the 1 is to close plots

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
yaw = 0;
yawAcc = atan(simimu.acc(2)) * 180 / pi;
yaw = yaw * 0.98 + yawAcc * 0.02;
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        % infinite loop
        % vary gamma
    % Plot
f = figure('Name','Pitch vs. Time'); %New fig
set(f, 'Position', [100, 100, 1049, 895]);

plot(time, rad2deg(simimu.gyro));
title('Simulated Gyroscope with drift');
legend('Roll Sensor', 'Pitch Sensor', 'Yaw Sensor')
xlabel('time (seconds)'); ylabel('degrees/sec');


if(not(isempty(varargin)))
    if varargin{1}==1
        close all;
    end
end

    results = simimu; % Placeholder
end
