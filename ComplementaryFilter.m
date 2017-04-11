% Will Conlin, 4/11/17
% Madgwick Complementary Filter
% Inputs: True virtual fish trajectory data, simulated trajectory data
% Models: State dynamics, error, complementary filter

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
    
    % Method
        % angle = gamma * (angle + gyroData * dt) + (1-gamma) * accelData
        % infinite loop
        % vary gamma
        
        
    results = simimu.t; % Placeholder
end
