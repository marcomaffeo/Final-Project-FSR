function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, ...
          csi_d, dot_csi_d, ddot_csi_d, psi_d,  dot_psi_d, ddot_psi_d, t, tot_time] = planner_1(Ts)

t_per_segment = 8;     
hover_time = 2;        
tdead = 2;             
n_segments = 4;
tot_time = n_segments * (t_per_segment + hover_time) + tdead;

t = linspace(0, tot_time, round(tot_time/Ts));
p_d = zeros(4, length(t));
dot_p_d = zeros(4, length(t));
ddot_p_d = zeros(4, length(t));

%goals
waypoints = [0 0 -1 0;
             3 0 -1 0;
             3 3 -1 0;
             0 3 -1 0;
             0 0 -1 0]; 

idx = 1;

for seg = 1:4

    t0 = 0;
    tf = t_per_segment;
    t1 = linspace(t0, tf, round(t_per_segment/Ts));

  
    p0 = waypoints(seg, :);
    pf = waypoints(seg+1, :);
    dot_p0 = zeros(1,4);
    dot_pf = zeros(1,4);
    ddot_p0 = zeros(1,4);
    ddot_pf = zeros(1,4);
    dddot_p0 = zeros(1,4);
    dddot_pf = zeros(1,4);

   
    a = zeros(8,4);
    A = [t0^7 t0^6 t0^5 t0^4 t0^3 t0^2 t0 1;
         tf^7 tf^6 tf^5 tf^4 tf^3 tf^2 tf 1;
         7*t0^6 6*t0^5 5*t0^4 4*t0^3 3*t0^2 2*t0 1 0;
         7*tf^6 6*tf^5 5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
         42*t0^5 30*t0^4 20*t0^3 12*t0^2 6*t0 2 0 0;
         42*tf^5 30*tf^4 20*tf^3 12*tf^2 6*tf 2 0 0;
         210*t0^4 120*t0^3 60*t0^2 24*t0 6 0 0 0;
         210*tf^4 120*tf^3 60*tf^2 24*tf 6 0 0 0];

    for j = 1:4
        b = [p0(j); pf(j); dot_p0(j); dot_pf(j); ddot_p0(j); ddot_pf(j); dddot_p0(j); dddot_pf(j)];
        a(:,j) = A \ b;
    end

    for j = 1:4
        p_j  = polyval(a(:,j)', t1);
        dp_j = polyval(polyder(a(:,j)'), t1);
        ddp_j = polyval(polyder(polyder(a(:,j)')), t1);

        p_d(j,idx:idx+length(t1)-1) = p_j;
        dot_p_d(j,idx:idx+length(t1)-1) = dp_j;
        ddot_p_d(j,idx:idx+length(t1)-1) = ddp_j;
    end

    idx = idx + length(t1);

    %hovering ogni step
    h_steps = round(hover_time/Ts);
    for j = 1:4
        p_d(j,idx:idx+h_steps-1) = pf(j);
        dot_p_d(j,idx:idx+h_steps-1) = 0;
        ddot_p_d(j,idx:idx+h_steps-1) = 0;
    end

    idx = idx + h_steps;
end

%hovering finale
for j = 1:4
    p_d(j,idx:end) = waypoints(end,j);
    dot_p_d(j,idx:end) = 0;
    ddot_p_d(j,idx:end) = 0;
end

%to Simulink
pos_0 = waypoints(1,1:3);
lin_vel_0 = [0 0 0];
w_bb_0 = [0 0 0];

csi_d       = p_d(1:3,:);
dot_csi_d   = dot_p_d(1:3,:);
ddot_csi_d  = ddot_p_d(1:3,:);

psi_d       = p_d(4,:);
dot_psi_d   = dot_p_d(4,:); 
ddot_psi_d  = ddot_p_d(4,:); 
end