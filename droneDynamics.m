% dynamic of drone
function xdot = droneDynamics(t, x, u, drone)
    
    % gravity 
    g = 9.81;

    % parameters
    T1 = drone.T1;
    T2 = drone.T2;
    J  = drone.J;
    m  = drone.m;

    % extract x q and omega from state
    q = x(  1:4,1);
    w = x(  5:7,1);
    v = x(11:13,1);

    % Rotation matrix from quaternion orientation
    R = quat2rotm(q');
    
    % Dynamics, the state xdot contains first the position x y z, velocities
    % xdot ydot zdot, then the leading coefficients for the quaternion
    % rotation, and last the angular velocities
    xdot( 1: 4,1) = 0.5 * quatmultiply(q',[0;w]')';
    xdot( 5: 7,1) = J \ (cross(-w,J*w) + T2*u);
    xdot( 8:10,1) = v;
    xdot(11:13,1) = R*T1*u/m + [0,0,-g]'/m;
    
end