function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
% xd    = u(1:3);
% b1d   = u(4:6);

% current state

x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);



%% %  attitude controlled flight mode
%% constant
% inertia matrix
J = diag([P.Jxx P.Jyy P.Jzz]);
% Third axis of inertial frame
e3 = [0;0;1];
%% desired state
Rd = eye(3); % desired attitude
omega_d = [0;0.1;0]; % desired angular velocity
xd = [0;0;2]; % desired position

%%   error term
ex = xd - x;  % position error
ev = -v;
eR = 0.5*vee(transpose(Rd)*R - transpose(R)*Rd); % attitude error
eOmega = Omega - transpose(R)*Rd*omega_d; % angular velocity erroNr


f= 0;
M = [0;0;0];
%% controller gain
M = -P.kR*eR - P.kOmega*eOmega;


f = -dot( (-P.kx*ex - P.kv*ev - P.mass*P.gravity*e3), R*e3);


norm_eR = norm(eR);
out = [f;M;norm_eR];

end