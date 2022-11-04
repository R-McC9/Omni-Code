% clear workspace
clc
clear
close all

warning('off')

% load drone model 
load('droneModel.mat')
nu = size(drone.T1,2);

% initial conditions
q0 = [1,0,0,0]';
w0 = [0 0 0]';
x0 = [0,0,0,0,0,0]';
x0 = [q0; w0; x0];
u0 = zeros(nu,1);
nx = length(x0);

% initial predicted state
N = 5;
xpred = x0*ones(1,N);

% simulation
dt = 0.05;
t  = 0:dt:30;
x  = x0;
u = u0;

%Create P_1|1 for kalman filter, asuming we know starting position with
%exact precision.
xKal = x0;
PKal = eye(13,13);
for k = 1:length(t)
    
    % display progress
    disp(k)
    
    % linearize dynamics
    % Give kalman filtered inputs here???
    if k > 80
        xpred = [xKal(:,k),xpred(:,2:end)];
        sys = droneLinearize(drone,xpred,dt);
    else
        xpred = [x(:,k),xpred(:,2:end)];
        sys = droneLinearize(drone,xpred,dt);
    end
    
    % disturbance "estimator"
    %d = [zeros(3,1);0;0;-9.81];

    % Make d a matrix dependent on state of system
    % Position of center of mass from center of drone
    P_0c = [0 0 0.02]';
    % rotation matrix of drone based on state
    Rq = quat2rotm(x(1:4, k)');
    % construct d
    d = [cross(Rq*P_0c,drone.m*-9.81*[0 0 1]'); drone.m*9.81*[0 0 1]'];
    
    % mpc controller
    [u(:,k),xpred] = dronempc(sys,x(:,k),r(:,k:k+N),d);

    %low pass filter to remove hiccuping?
    if k > 40
        for i = 1:height(u)
            uLP(i,1:k) = lowpass(u(i,1:k),0.2);
        end
    end
    
    % simulate drone
    [ts,xs] = ode45(@(ts,xs)droneDynamics(ts, xs, u(:,k), drone), [0,dt], x(:,k));
    x(:,k+1) = xs(end,:)';
    y(:,k+1) = [xs(end,1:4), xs(end,8:10)]';

    %Pull state and inputs from ode 45 into kalman filter, esitmate next
    %state (xKal)
    %Unsure of values for Q and R covariance matrices.
    Q = 0*eye(13,13);
    R = 0.9*eye(7,7);
    [xKal(:,k+1), PKal(:,:,k+1)] = kalmantest(x(:,k),u(:,k),y(:,k+1),sys,PKal(:,:,k),Q,R);    
end

%% attitude
figure
set(gcf,'position',[617   716   560   303])

q = x(1:4,1:end-1);
subplot(2,1,1)
plot(t,q,'-','linewidth',2)
hold on
plot(t,r(1:4,1:length(t)),'k-.','linewidth',1,'handlevisibility','off')

grid on
set(gca,'position',[0.10    0.55    0.88    0.43])
set(gca,'fontsize',14)

xlim([0,max(t)])
ylabel('Orientation','fontsize',14)

legend({'$q_0(t)$','$q_1(t)$','$q_2(t)$','$q_3(t)$'},'interpreter','latex','fontsize',14,'orientation','horizontal','location','northeast')

w = x(5:7,1:end-1);
subplot(2,1,2)
plot(t,w,'-','linewidth',2)
hold on
        xpred = [xKal(:,k),xpred(:,2:end)];
        sys = droneLinearize(drone,xpred,dt);
grid on
set(gca,'position',[0.10    0.10    0.88    0.43])
set(gca,'fontsize',14)

xlim([0,max(t)])
xlabel('Time [s]','fontsize',14)
ylabel('Angular Velocity','fontsize',14)

figure
qKal = xKal(1:4, 1:end-1);
subplot(2,1,1)
plot(t,qKal,'o')
hold on
plot(t,q,'k-.','linewidth',1,'handlevisibility','off')
title('Kalman Filtered Estimate of Attitude vs Actual Attitude')

wKal = xKal(5:7, 1:end-1);
subplot(2,1,2)
plot(t,wKal,'o')
hold on
plot(t,w,'k-.','linewidth',1,'handlevisibility','off')
title('Kalman Filtered Estimate of Angular Velocity vs Actual Angular Velocity')

figure
plot(t,uLP,'o','linewidth', 2)
hold on
plot(t,u,'-')

%% position 
figure
set(gcf,'position',[617   716   560   303])

p = x(8:10,1:end-1);
subplot(2,1,1)
plot(t,p,'-','linewidth',2)
hold on
plot(t,r(5:7,1:length(t)),'k-.','linewidth',1,'handlevisibility','off')

grid on
set(gca,'position',[0.10    0.55    0.88    0.43])
set(gca,'fontsize',14)

xlim([0,max(t)])
ylabel('Position','fontsize',14)

legend({'$x(t)$','$y(t)$','$z(t)$'},'interpreter','latex','fontsize',14,'orientation','horizontal','location','northeast')

v = x(11:13,1:end-1);
subplot(2,1,2)
plot(t,v,'-','linewidth',2)
hold on

grid on
set(gca,'position',[0.10    0.10    0.88    0.43])
set(gca,'fontsize',14)

xlim([0,max(t)])
xlabel('Time [s]','fontsize',14)
ylabel('Velocity','fontsize',14)

legend({'$\dot x(t)$','$\dot y(t)$','$\dot z(t)$'},'interpreter','latex','fontsize',14,'orientation','horizontal','location','northeast')

figure
pKal = xKal(8:10, 1:end-1);
subplot(2,1,1)
plot(t,pKal,'o')
hold on
plot(t,p,'k-.','linewidth',1,'handlevisibility','off')
title('Kalman Filtered Estimate of Position vs Actual Position')

vKal = xKal(11:13, 1:end-1);
subplot(2,1,2)
plot(t,vKal,'o')
hold on
plot(t,v,'k-.','linewidth',1,'handlevisibility','off')
title('Kalman Filtered estimate of Linear Velocity vs Actual Linear Velocity')
How do I create a low-pass filter in MATLAB?
%% trajectory

figure
plot3(x(8,:),x(9,:),x(10,:),'linewidth',2)
hold on
plot3(r(5,:),r(6,:),r(7,:),'k-.','linewidth',1,'handlevisibility','off')
xlabel('$x(t)$','interpreter','latex','fontsize',20)
ylabel('$y(t)$','interpreter','latex','fontsize',20)
zlabel('$z(t)$','interpreter','latex','fontsize',20)

%% Animation
saveAnimation = false;
results.t = t;
results.states = x;
results.ref = r;
params = drone;
params.videoName = 'trajectoryAnimation';
params.animationTimeScale = 1;
droneAnimation(results, params, saveAnimation);