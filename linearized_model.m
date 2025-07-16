clear
close all
clc

%matrix definition
syms x y z dot_x dot_y dot_z real
syms phi theta psi real
syms phi_dot theta_dot psi_dot real
syms u_T g mass real
syms Ib1 Ib2 Ib3 real
syms tau_b1 tau_b2 tau_b3 real

pos = [x y z]';
dot_pos = [dot_x dot_y dot_z]';
tau_b=[tau_b1 tau_b2 tau_b3]';
Ib=diag([Ib1 Ib2 Ib3]);
eta_b=[phi theta psi]';
dot_eta_b=[phi_dot theta_dot psi_dot]';

Q=[1              0                        -sin(eta_b(2));
   0        cos(eta_b(1))     cos(eta_b(2))*sin(eta_b(1));
   0       -sin(eta_b(1))    cos(eta_b(1))*cos(eta_b(2))];

Q_dot=[0             0                                                                          -cos(eta_b(2))*dot_eta_b(2);
       0 -sin(eta_b(1))*dot_eta_b(1)     -sin(eta_b(2))*sin(eta_b(1))*dot_eta_b(2)+cos(eta_b(2))*cos(eta_b(1))*dot_eta_b(1);
       0 -cos(eta_b(1))*dot_eta_b(1)     -sin(eta_b(2))*cos(eta_b(1))*dot_eta_b(2)-cos(eta_b(2))*sin(eta_b(1))*dot_eta_b(1)];


M=Q'*Ib*Q;

S = skew(Q*(dot_eta_b));

C = (Q'*S*Ib*Q)+ (Q'*Ib*Q_dot); 


Rb=[cos(eta_b(2))*cos(eta_b(3))   sin(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))-cos(eta_b(1))*sin(eta_b(3))   cos(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))+sin(eta_b(1))*sin(eta_b(3));
    cos(eta_b(2))*sin(eta_b(3))   sin(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))+cos(eta_b(1))*cos(eta_b(3))   cos(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))-sin(eta_b(1))*cos(eta_b(3));
        -sin(eta_b(2))              sin(eta_b(1))*cos(eta_b(2))                                           cos(eta_b(1))*cos(eta_b(2))];

%model
dot_dot_p_b = g*[0 0 1]'-(u_T/mass)*Rb*[0 0 1]';
dot_dot_eta_b = (inv(M))*((-C*dot_eta_b)+(Q'*tau_b));

d_x = dot_x;
dd_x = dot_dot_p_b(1);
d_y = dot_y;
dd_y = dot_dot_p_b(2);
d_z = dot_z;
dd_z = dot_dot_p_b(3);
d_ph = phi_dot;
dd_ph = dot_dot_eta_b(1);
d_th = theta_dot;
dd_th = dot_dot_eta_b(2);
d_ps = psi_dot;
dd_ps = dot_dot_eta_b(3);


X = [x; y; z; phi; theta; psi; d_x; d_y; d_z; d_ph; d_th; d_ps];

dot_X = [d_x; d_y; d_z; d_ph; d_th; d_ps; dd_x; dd_y; dd_z; dd_ph; dd_th; dd_ps];

U = [u_T; tau_b];

A_s = jacobian(dot_X, X);  % 12x12
B_s = jacobian(dot_X, U);  % 12x4


A = double(subs(A_s,[x y z phi theta psi dot_x dot_y dot_z phi_dot theta_dot psi_dot, u_T, mass, tau_b1, tau_b2, tau_b3, Ib1, Ib2, Ib3],[0 0 -1 0 0 0 0 0 0 0 0 0 11.772 1.2 0 0 0 1.2416 1.2416 2*1.2416]));
B = double(subs(B_s,[x y z phi theta psi dot_x dot_y dot_z phi_dot theta_dot psi_dot, u_T, mass, tau_b1, tau_b2, tau_b3, Ib1, Ib2, Ib3],[0 0 -1 0 0 0 0 0 0 0 0 0 11.772 1.2 0 0 0 1.2416 1.2416 2*1.2416]));


function S = skew(v)
if(numel(v)~= 1)
S= [0 -v(3) v(2); 
    v(3) 0 -v(1);
    -v(2) v(1) 0];
else
S= zeros(3);
end
end