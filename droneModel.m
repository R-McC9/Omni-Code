% 8 rotor drone
clc
clear
close all

% parameters 
m = 1;
J = eye(3);J = diag(10+10*rand(3,1));
g = 9.81;

% thrust directions
T1(:,1) = [ 1, 0, 1]';
T1(:,2) = [-1, 0, 1]';
T1(:,3) = [-1, 0, 1]';
T1(:,4) = [ 1, 0, 1]';
T1(:,5) = [ 0, 1,-1]';
T1(:,6) = [ 0, 1,-1]';
T1(:,7) = [ 0,-1,-1]';
T1(:,8) = [ 0,-1,-1]';

% lever-arms 
L(:,1) = [ 2, 2, 1]';
L(:,2) = [-2, 2, 1]';
L(:,3) = [-2,-2, 1]';
L(:,4) = [ 2,-2, 1]';
L(:,5) = [ 2, 2,-1]';
L(:,6) = [-2, 2,-1]';
L(:,7) = [-2,-2,-1]';
L(:,8) = [ 2,-2,-1]';

% define ai for gyroscopic effects
ai = 0.05;

% input matrices 
% for gyroscopic input a is a matrix of constants?
T2 = zeros(size(T1));
for i = 1:size(T1,2)
    T2(:,i) = cross(L(:,i),T1(:,i))+ai*T1(:,i);
end

a = 1;
switch a
    case 1
        % reference orientation
        t = 0:0.05:31;
        q = [cos(pi*t/max(t));sin(pi*t/max(t)).*[1,1,1]'/sqrt(3)]';
        k = quatrotate(q,[1,0,0])';
        r1 = [cos(6*pi*t/max(t));sin(6*pi*t/max(t)).*k];

        % reference positions
        r2 = [10*cos(2*pi*t/max(t));10*sin(2*pi*t/max(t));1*cos(8*pi*t/max(t))];

        % combined reference
        r = [r1;r2];
    case 2
        %reference orientation
        t = 0:0.05:31;
        q = [1 0 0 0];
        k = quatrotate(q,[1,0,0])';
        for i = 1:length(t)
            r1(:,i) = [1 0 0 0]';
        end

        %reference positions
        r2 = [cos(2*pi*t/max(t)); sin(2*pi*t/max(t)); sin(2*pi*t/max(t))];

        %combine
        r = [r1;r2];
end

% data-structure for drone model
drone.T1 = T1;
drone.T2 = T2;
drone.J  = J;
drone.m  = m;

% save model 
save droneModel.mat drone r