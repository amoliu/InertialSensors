% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: Simulated IMU data
% Models: State dynamics, error, complementary filter
% Units: m/s^2 and radians
% Instructions: A call would look like:
%   ComplementaryFilter(simulatedData(5,1)) where the 1 is to close plots

function gamma = ComplementaryFilter(simimu,varargin)
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
global simimulocal gyroBias gyroOffset

    % Setup
        % Plot imu data (via simulatedData.m)
time = simimu.t;
simimu.acc = simimu.acc * 9.81;
window =100;
simimu.acc = movmean(simimu.acc, window);
simimulocal = simimu;

    % Method
        % for each time step, calculate yaw
            % if a is above a threshold, use gyro data
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        
        % vary gamma (optimization below - needs to be constrained)
gamma = .02; % .02
gyroBias = .025;
gyroOffset = .03;
pitch = zeros(size(time));

pitchPrev = 0;
for ii = 1:size(time)
    pitch(ii) = pitchPrev + simimu.gyro(ii,2) * simimu.sampfreq ;%+ .000005 * time(ii);
    if abs(simimu.acc(ii, 3)) < 9.81  % z acc threshhold, compensate for drift
        pitchAcc = accToAngle(simimu.acc(ii, 1), simimu.acc(ii, 2), simimu.acc(ii, 3) );
        pitch(ii) = pitchAcc.pitch * 10 * gamma + pitch(ii) * (1-gamma);
    end
    pitchPrev = pitch(ii);
end

% void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
% {
%     float pitchAcc, rollAcc;               
%  
%     // Integrate the gyroscope data -> int(angularSpeed) = angle
%     *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
%  
%     // Compensate for drift with accelerometer data if !bullshit
%     // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
%     int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
%     if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
%     {
% 	// Turning around the X axis results in a vector on the Y-axis
%         pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
%         *pitch = *pitch * 0.98 + pitchAcc * 0.02;
%     }
% } 

    % Plot
f = figure('Name','Pitch vs. Time'); %New fig
set(f, 'Position', [100, 100, 1049, 895]);

subplot(2,2,1);
plot(time, cumtrapz(simimu.gyro(:,2)) * simimu.sampfreq + gyroBias * time - gyroOffset , time, cumtrapz(time, simimu.truegyro(:,2)) + gyroBias * time - gyroOffset); % integral of omega     vel = cumtrapz(acc) * dT + v0
title('Pitch'); % should be magnitude .09 radians
legend('IMU Gyro Data')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,2);
plot(time, abs( cumtrapz(simimu.truegyro(:,2) - simimu.gyro(:,2)) * simimu.sampfreq));
title('Pitch Gyro Error (abs. val.)');
legend('Error')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,3);
plot(time+.17, pitch , time, cumtrapz(simimu.truegyro(:,2)) * simimu.sampfreq + gyroBias * time - gyroOffset);
title('Complementary Filter Pitch');
legend('Complementary Filter')
xlabel('time (seconds)'); ylabel('radians');

subplot(2,2,4);
plot(time, abs( cumtrapz(simimu.truegyro(:,2)) * simimu.sampfreq + gyroBias * time - gyroOffset - pitch ));
title('Complementary Filter Pitch Error (abs. val.)');
legend('Error')
xlabel('time (seconds)'); ylabel('radians');


if(not(isempty(varargin)))
    if varargin{1}==1
        close all;
    end
end

gamma = fminbnd(@error, .01, .1);
% gamma = fminunc(@error, .05);

end

function [angle] = accToAngle(ax, ay, az)
%     angle.pitch = atan(-ax/az);     % pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    angle.pitch = atan(ax/sqrt(ay * ay + (az) * (az))); % float pitch = atan(xAxis/sqrt(pow(yAxis,2) + pow(zAxis,2)));
%     angle.pitch = asin(ax/sqrt(ax * ax + az * az));
    angle.roll = atan2(ay , az);
end

function error = error(gamma)
    global simimulocal gyroBias gyroOffset
    error = 0;
    pitchlocal = zeros(size(simimulocal.t));
    pitchPrev = 0;
    for ii = 1:size(simimulocal.t)
        pitchlocal(ii) = pitchPrev + simimulocal.gyro(ii,2) * simimulocal.sampfreq;
        if abs(simimulocal.acc(ii, 3)) < 9.81  % z acc threshhold, compensate for drift
            pitchAcc = accToAngle(simimulocal.acc(ii, 1), simimulocal.acc(ii, 2), simimulocal.acc(ii, 3) );
            pitchlocal(ii) = pitchAcc.pitch * gamma + pitchlocal(ii) * (1-gamma);
        end
        pitchPrev = pitchlocal(ii);
        error = error + abs( cumtrapz(simimulocal.truegyro(ii,2)) * simimulocal.sampfreq + gyroBias * simimulocal.t(ii) - gyroOffset - pitchlocal(ii) );
    end
end
