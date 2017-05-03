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
% simimu.realeulerrad% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: Simulated IMU data
% Models: State dynamics, error, complementary filter
% Units: m/s^2 and radians
% Instructions: A call would look like:
%   ComplementaryFilter(simulatedData(5,1)) where the 1 is to close plots

function gamma = Madgwick(simimu,varargin)
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
% gamma = .02; % .02
gyroBias = .025;
gyroOffset = .03;
pitch = zeros(size(time));
gamma =  zeros(size(time));

% normA = simimu.acc(:, 3) - min(simimu.acc(:, 3));
% normA = normA ./ max(normA(:));

pitchPrev=0;

for ii = 1:size(time)
    gamma(ii) = abs(-9.8 - simimu.acc(ii, 3)) * 6; % 9.6 to 10 so abs(9.8 - simimu.acc(ii, 3)) * 5 0 to .2
%     gamma(ii) = 0;
    pitchAcc = accToAngle(simimu.acc(ii, 1), simimu.acc(ii, 2), simimu.acc(ii, 3) );
    pitch(ii) = pitchAcc.pitch *15 * (1-gamma(ii)) + (pitchPrev + simimu.gyro(ii,2) * simimu.sampfreq )* gamma(ii);
    pitchPrev = pitch(ii);
end

% plot(gamma);

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

pitchSmooth = sgolayfilt(pitch, 5, 501);

subplot(2,2,3);
plot(time+.17, pitchSmooth , time-.17, cumtrapz(simimu.truegyro(:,2)) * simimu.sampfreq + gyroBias * time - gyroOffset);
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

% gamma = fminbnd(@error, .01, .1);
% gamma = fminunc(@error, .05);

end

function [angle] = accToAngle(ax, ay, az)
%     if (9.81 > abs(az) && abs(az)> 9) %low acceleration
%         angle.pitch = acos(abs(az)/9.81);
%     else
    angle.pitch = atan(ax/sqrt(ay * ay + az * az));
%     end
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
% window =3;
% simimu.acc = movmean(simimu.acc, window);
simimulocal = simimu;

    % Method
        % for each time step, calculate yaw
            % if a is above a threshold, use gyro data
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        
        % vary gamma (optimization below - needs to be constrained)
% gamma = .02; % .02
gyroBias = .025;
gyroOffset = .03;
pitch = zeros(size(time));

normA = simimu.acc(:, 3) - min(simimu.acc(:, 3));
normA = normA ./ max(normA(:));

pitchPrev=0;

for ii = 1:size(time)
    gamma = normA(ii); %abs(9.8 - simimu.acc(ii, 3)) * 5; % 9.6 to 10 so abs(9.8 - simimu.acc(ii, 3)) * 5 0 to .2
    pitchAcc = accToAngle(simimu.acc(ii, 1), simimu.acc(ii, 2), simimu.acc(ii, 3) );
    pitch(ii) = pitchAcc.pitch * gamma + pitchPrev + simimu.gyro(ii,2) * simimu.sampfreq * (1-gamma);
    pitchPrev = pitch(ii);
end

plot(abs(sin(time*5)));
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

% gamma = fminbnd(@error, .01, .1);
% gamma = fminunc(@error, .05);

end

function [angle] = accToAngle(ax, ay, az)
%     if (9.81 > abs(az) && abs(az)> 9) %low acceleration
%         angle.pitch = acos(abs(az)/9.81);
%     else
        angle.pitch = atan(ax/sqrt(ay * ay + (az) * (az)));
%     end
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
