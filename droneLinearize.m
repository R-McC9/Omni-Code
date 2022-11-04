function sys = droneLinearize(drone,r,dt)

% drone data-structure
T1 = drone.T1;
T2 = drone.T2;
J  = drone.J;

% 1-step MPC
nu = size(T1,2);
N  = size(r,2);

% reference trajectories
q = r(1:4,:);
w = r(5:7,:);

% identity and zeros matrices
O = zeros(3);
I = eye(3);

% linearize dynamics
for k = 1:N
    
    % skew-symmetric velocity
    W = skew(w(:,k));
    Q = skew(q(2:4,k));
    R = quat2rotm(q(:,k)');
    
    % linearization
    A(:,:,k) = [0,              -0.5*w(:,k)',  -0.5*q(2:4,k)',  O(1,:),     O(1,:);
                0.5*w(:,k),     -0.5*W,         0.5*q(1,k)*I+Q, O,          O;
                O(:,1),          O,             W - J \ W * J,  O,          O;
                O(:,1),          O,             O,              O,          I;
                O(:,1),          O,             O,              O,          O;
                ];
    B(:,:,k) = [zeros(4,nu);T2;zeros(3,nu);R*T1];
    C(:,:,k) = blkdiag(eye(4,7),eye(3,6));
    E(:,:,k) = blkdiag([zeros(4,3);eye(3)],[zeros(3);eye(3)]);
    G(:,:,k) = [T2;R*T1];
    
    % discretize
    A(:,:,k) = A(:,:,k)*dt + eye(size(A(:,:,k)));
    B(:,:,k) = B(:,:,k)*dt;
    E(:,:,k) = E(:,:,k)*dt;
    
end

% package dynamics
sys.A = A;
sys.B = B;
sys.C = C;
sys.D = zeros(size(C,1),size(B,2));
sys.E = E;
sys.G = G;