function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, ...
          csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time] = planner_2(Ts)

r = 2;               
turns = 2;           
Tturn = 40;         
tdead = 5;           

ttot = turns * Tturn;
tot_time = ttot + tdead;

t1 = linspace(0, ttot, round(ttot/Ts));
t2 = linspace(t1(end)+Ts, tot_time, round(tdead/Ts));
t = [t1, t2];

theta0 = 0;
thetaf = 2*pi*turns;

z0 = -1;
zf = -4;

dot0 = 0; ddot0 = 0; dddot0 = 0;

theta_poly = poly7gen(theta0, thetaf, dot0, dot0, ddot0, ddot0, dddot0, dddot0, t1(1), ttot);
z_poly     = poly7gen(z0, zf, dot0, dot0, ddot0, ddot0, dddot0, dddot0, t1(1), ttot);

theta   = polyval(theta_poly, t1);
dtheta  = polyval(polyder(theta_poly), t1);
ddtheta = polyval(polyder(polyder(theta_poly)), t1);

z    = polyval(z_poly, t1);
dz   = polyval(polyder(z_poly), t1);
ddz  = polyval(polyder(polyder(z_poly)), t1);

%elica
x    = r * cos(theta);
y    = r * sin(theta);

dx   = -r * sin(theta) .* dtheta;
dy   =  r * cos(theta) .* dtheta;

ddx  = -r * (cos(theta) .* (dtheta.^2) + sin(theta) .* ddtheta);
ddy  = -r * (sin(theta) .* (dtheta.^2) - cos(theta) .* ddtheta);

%yaw
psi   = theta; 
dpsi  = dtheta;
ddpsi = ddtheta;

%hovering
N2 = length(t2);
x2 = repmat(x(end), 1, N2);
y2 = repmat(y(end), 1, N2);
z2 = repmat(z(end), 1, N2);
psi2 = repmat(psi(end), 1, N2);

dx2 = zeros(1, N2);
dy2 = zeros(1, N2);
dz2 = zeros(1, N2);
dpsi2 = zeros(1, N2);

ddx2 = zeros(1, N2);
ddy2 = zeros(1, N2);
ddz2 = zeros(1, N2);
ddpsi2 = zeros(1, N2);

p_d       = [ [x; y; z; psi], [x2; y2; z2; psi2] ];
dot_p_d   = [ [dx; dy; dz; dpsi], [dx2; dy2; dz2; dpsi2] ];
ddot_p_d  = [ [ddx; ddy; ddz; ddpsi], [ddx2; ddy2; ddz2; ddpsi2] ];

%to simulink
pos_0     = p_d(1:3,1)';
lin_vel_0 = dot_p_d(1:3,1)';
w_bb_0    = [0 0 0];

csi_d       = p_d(1:3,:);
dot_csi_d   = dot_p_d(1:3,:);
ddot_csi_d  = ddot_p_d(1:3,:);
psi_d       = p_d(4,:);
dot_psi_d   = dot_p_d(4,:);
ddot_psi_d  = ddot_p_d(4,:);

end

function a = poly7gen(p0, pf, dp0, dpf, ddp0, ddpf, dddp0, dddpf, t0, tf)
    A = [t0^7 t0^6 t0^5 t0^4 t0^3 t0^2 t0 1;
         tf^7 tf^6 tf^5 tf^4 tf^3 tf^2 tf 1;
         7*t0^6 6*t0^5 5*t0^4 4*t0^3 3*t0^2 2*t0 1 0;
         7*tf^6 6*tf^5 5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
         42*t0^5 30*t0^4 20*t0^3 12*t0^2 6*t0 2 0 0;
         42*tf^5 30*tf^4 20*tf^3 12*tf^2 6*tf 2 0 0;
         210*t0^4 120*t0^3 60*t0^2 24*t0 6 0 0 0;
         210*tf^4 120*tf^3 60*tf^2 24*tf 6 0 0 0];
    b = [p0; pf; dp0; dpf; ddp0; ddpf; dddp0; dddpf];
    a = A \ b;
end