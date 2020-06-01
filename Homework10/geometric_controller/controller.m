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
xd = u(1:3);
b1d = u(4:6);

%%   error term
ex = x-xd;  % position error
ev = [0;0;0];
ex
%% porpaler force
P.kx = 0.5;
f_ = (-P.kx*ex - P.kv*ev - P.mass*P.gravity*e3);
f_
%% compute b3c from force controller
b3c = - f_ / norm(f_);
% b2c =  cross(b3c,b1d) / norm(cross(b3c,b1d));
% b1c = -cross(b2c,b3c) / norm(cross(b2c,b3c));
b1c = -cross(b3c,cross(b3c,b1d)) / norm(cross(b3c,b1d));

b2c =  cross(b3c,b1c) / norm(cross(b3c,b1c));
Rc = [b1c, b2c, b3c]; % desired attitude
omega_d = [0;0.1;0]; % desired angular velocity

%%   error term
eR = 0.5*vee(transpose(Rc)*R - transpose(R)*Rc); % attitude error
% eOmega = Omega - transpose(R)*Rc*omega_d; % angular velocity erroNr
 eOmega = [0;0;0];

%% controller gain
P.kR = 0.1;
M = -P.kR*eR - P.kOmega*eOmega+cross(Omega,J*Omega);


f = dot(-f_ ,R*e3);



norm_eR = norm(eR);
out = [f;M;norm_eR];

end