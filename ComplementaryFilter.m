% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: Simulated IMU data
% Models: State dynamics, error, complementary filter
% Units: m/s^2 and radians
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
window =2001;
simimu.acc = movmean(simimu.acc * 9.81, window);

    % Method
        % for each time step, calculate yaw
            % if a is above a threshold, use gyro data
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        
        % vary gamma
gamma = .6; % .02
pitch = zeros(size(time));
pitchAccM = zeros(size(time));

for ii = 1:size(time)
    pitchGyro =  simimu.gyro(ii, 2) * simimu.sampfreq; % integrate
    if abs(simimu.acc(ii, 2)) < .981  % y acc threshhold
        pitchAcc = accToAngle(simimu.acc(ii, 1), simimu.acc(ii, 2), simimu.acc(ii, 3) ); 
        pitch(ii) = pitchAcc.pitch * gamma + pitchGyro * (1-gamma);
        pitchAccM(ii) = pitchAcc.pitch;
    else
        pitch(ii) = pitchGyro;
        pitchAccM(ii) = 0;
    end
end
    % Plot
f = figure('Name','Pitch vs. Time'); %New fig
set(f, 'Position', [100, 100, 1049, 895]);

subplot(2,2,1);
plot(time, simimu.gyro(:,2) * simimu.sampfreq, time, simimu.truegyro(:,2) * simimu.sampfreq); % integral of omega     vel = cumtrapz(acc) * dT + v0
title('Pitch');
legend('IMU Data')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,2);
plot(time, abs( (simimu.truegyro(:,2) - simimu.gyro(:,2)) * simimu.sampfreq));
title('Pitch Gyro Error (abs. val.)');
legend('Error')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,3);
plot(time, pitch, time, simimu.truegyro(:,2) * simimu.sampfreq);
title('Complementary Filter Pitch');
legend('Complementary Filter')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,4);
plot(time, abs( (simimu.truegyro(:,2) * simimu.sampfreq) - pitch ));
title('Complementary Filter Pitch Error (abs. val.)');
legend('Error')
xlabel('time (seconds)'); ylabel('radians');


if(not(isempty(varargin)))
    if varargin{1}==1
        close all;
    end
end

end

function [angle] = accToAngle(ax, ay, az)
%     angle.pitch = atan(-ax/az);     % pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    angle.pitch = atan(ax/sqrt(ay * ay + (az) * (az))); % float pitch = atan(xAxis/sqrt(pow(yAxis,2) + pow(zAxis,2)));
%     angle.pitch = asin(ax/sqrt(ax * ax + az * az));
    angle.roll = atan2(ay , az);
end
