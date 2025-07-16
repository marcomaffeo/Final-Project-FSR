function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= hierarchical_control(planner,Ts)

%gains
A = [zeros(3), eye(3);
     zeros(3), zeros(3)];

B = [zeros(3);
     eye(3)];

%angular gain
Q_e = eye(6);
R_e = eye(3);

%position gain
Q_p = eye(6);
R_p = eye(3);


Kp = 50*lqr(A, B, Q_p, R_p);   %linear
Ke = 500*lqr(A, B, Q_e, R_e);  %angular

if planner == 1
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_1(Ts);

elseif planner == 2
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_2(Ts);

else
    error('Undefined Planner');
end

    %to simulink
    simIn = Simulink.SimulationInput('hierarchical_control_project');

    simIn = simIn.setVariable('t', t);
    simIn = simIn.setVariable('tot_time', tot_time);
    simIn = simIn.setVariable('pos_0', pos_0);
    simIn = simIn.setVariable('lin_vel_0', lin_vel_0);
    simIn = simIn.setVariable('w_bb_0', w_bb_0);
    simIn = simIn.setVariable('p_d', p_d);
    simIn = simIn.setVariable('dot_p_d', dot_p_d);
    simIn = simIn.setVariable('ddot_p_d', ddot_p_d);
    simIn = simIn.setVariable('csi_d', csi_d);
    simIn = simIn.setVariable('dot_csi_d', dot_csi_d);
    simIn = simIn.setVariable('ddot_csi_d', ddot_csi_d);
    simIn = simIn.setVariable('psi_d', psi_d);
    simIn = simIn.setVariable('dot_psi_d', dot_psi_d);
    simIn = simIn.setVariable('ddot_psi_d', ddot_psi_d);

    simIn = simIn.setVariable('Kp', Kp);
    simIn = simIn.setVariable('Ke', Ke);

    simIn = simIn.setModelParameter('StopTime', num2str(tot_time));

    out = sim(simIn);

%from simulink
err_p = out.err_pos;
dot_err_p = out.dot_err_pos;
err_R = out.err_ang;
err_W = out.dot_err_ang;
uT = out.uT;
tau_b = out.tau_b;
tout = out.tout;

norm_pos =out.norm_pos;
norm_ang =out.norm_ang;
norm_lin_vel =out.norm_lin_vel;
norm_ang_vel =out.norm_ang_vel;

%make_plot(out);
%save('hierarchical_data2.mat', 'norm_pos', 'norm_ang', 'norm_lin_vel', 'norm_ang_vel');

end