%Function for generating state estimate using Kalman Filter

function [xKal, PKal] = kalmantest(x, u, z, sys, P, Q, R)

%P is covariance between current state and reference?

%Extract system dynamics
A = sys.A(:,:,1);
B = sys.B(:,:,1);
C = sys.C(:,:,1);

%Predict x(k+1) based on x(k)
xHat = A*x + B*u;

%Estimate covariance
PHat = A*P*A' + Q;

%Update
%If yTilde is 0 the estimator is doing a good job
yTilde = z - C*xHat;

%Updated covariance
S = C*PHat*C' + R;

%Optimal Kalman gains
K = PHat*C'*inv(S);

%Updated state estimate
xKal = xHat + K*yTilde;

%Updated Covariance
%Define dimensions based on A B C
PKal = (eye(13,13) - K*C)*PHat;