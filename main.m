clear
close all 
clc

%data
g = 9.81;
mass = 1.2;
e_3 = [0 0 1]';
u_T0 = 11.772;      % thrust at the steady-state 
Ib = diag([1.2416 1.2416 2*1.2416]); 
Ts=0.001; %sampling time

% choose the planner
% 1-square; 2-helic;
planner = 1; 

% choose the controller 
% 0-hierarchical; 1-passivity; 2-mpc; 3-nonlinear mpc
controller = 0; 


if controller == 0
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]=hierarchical_control(planner,Ts);

elseif controller == 1
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]=passivity_based_control(planner,Ts);

elseif controller == 2
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]=mpc_control(planner,Ts);

elseif controller == 3
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]=nlmpc_control(planner,Ts);

else
    error('Undefined Controller');
end









