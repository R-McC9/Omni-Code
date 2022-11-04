function [ustar,xpred] = dronempc(sys,x0,r,d)

% for speed
yalmip('clear')

% extract system matrices
A = sys.A;
B = sys.B;
C = sys.C;
E = sys.E;

% problem size
[nx,nu,N] = size(B);
ny = size(C,1);

% cost matrices
Q = 1e+4*eye(ny);
R = 1e-2*eye(nu);

% terminal cost
A0 = A(2:end,2:end,1);
B0 = B(2:end,:,1);
C0 = C(:,2:end,1);
% Q0 = Q(2:end,2:end);
% [~,P] = dlqr(A0,B0,C0'*Q0*C0,R);
[~,P] = dlqr(A0,B0,C0'*Q*C0,R);
P = [zeros(12,1),eye(12)]'*P*[zeros(12,1),eye(12)];

% decision variables: predicted states and inputs
x = sdpvar(nx*ones(1,N+1),ones(1,N+1),'full');
if N > 1
    u = sdpvar(nu*ones(1,N),ones(1,N),'full');
else
    u{1} = sdpvar(nu,1,'full');
end

% reference and disturbance trajectories
if size(d,2) == 1
    d = d*ones(1,N);
end
if size(r,2) == 1
    r = r*ones(1,N);
end

% finite-time optimal control problem
cost   = 0;
constr = [x{1} == x0];
for k = 1:N
    
    % cost 
    cost = cost + (C(:,:,k)*x{k}-r(:,k))'*Q*(C(:,:,k)*x{k}-r(:,k)) + u{k}'*R*u{k};
    
    % dynamics 
    constr = constr + [x{k+1} == A(:,:,k)*x{k} + B(:,:,k)*u{k} + E(:,:,k)*d(:,k)];
    constr = constr + [-100 <= u{k}];
    constr = constr + [u{k} <=  100];
    
end
cost = cost + (x{k+1} - C(:,:,end)'*r(:,end))'*P*(x{k+1} - C(:,:,end)'*r(:,end));

% solve finite-time optimal control problem
opt = sdpsettings('verbose',0);
optimize(constr,cost,opt);

% extract predicted state and input trajectories
xpred = value(horzcat(x{2:N+1}));
ustar = value(horzcat(u{1}));