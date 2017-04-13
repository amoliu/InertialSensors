% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: Simulated IMU data
% Models: State dynamics, error, complementary filter
% Instructions: A call would look like:
%   ComplementaryFilter(simulatedData(5,1)) where the 1 is to close plots

function [simimu] = ComplementaryFilter(simimu,varargin)
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
gamma = .01;
pitch = zeros(size(time));
pitchGyro = 0;

for ii = 1:size(time)
    pitchGyro = pitchGyro + simimu.gyro(ii, 2) * simimu.sampfreq; % current angle + rate * dt
    
    if abs(simimu.acc(ii, 2)) < .1  % y acc threshhold
        angle = accToAngle(simimu.acc(ii, 1), simimu.acc(ii, 2), simimu.acc(ii, 3) );
        pitch(ii) = angle.pitch * gamma + pitchGyro * (1-gamma);
        pitchGyro = pitch(ii);
    else
        pitch(ii) = pitchGyro;
    end
end
    % Plot
f = figure('Name','Pitch vs. Time'); %New fig
set(f, 'Position', [100, 100, 1049, 895]);

subplot(2,2,1);
plot(time, rad2deg(simimu.truegyro(:,2)));
title('Pitch');
legend('IMU Data')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,2);
plot(time, (rad2deg(simimu.truegyro(:,2)) - rad2deg(simimu.gyro(:,2))));
title('Pitch Gyro Error');
legend('Error')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,3);
plot(time, rad2deg(pitch));
title('Complementary Filter Pitch');
legend('Complementary Filter')
xlabel('time (seconds)'); ylabel('degrees/sec');

subplot(2,2,4);
plot(time, (rad2deg(simimu.truegyro(:,2)) - rad2deg(pitch)));
title('Complementary Filter Pitch Gyro Error');
legend('Error')
xlabel('time (seconds)'); ylabel('degrees/sec');


if(not(isempty(varargin)))
    if varargin{1}==1
        close all;
    end
end

end

function [angle] = accToAngle(ax, ay, az)
    angle.pitch = atan2((- ax) , sqrt(ay * ay + az * az)) * 180 / pi;
    angle.roll = atan2(ay , az) * 180 / pi;
end
