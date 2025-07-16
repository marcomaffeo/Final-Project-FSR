function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= passivity_based_control(planner,Ts)

%data
sigma=10;
ni=5;
Do=50*eye(3);
Ko=sigma*Do;

%gains
Kp=diag([100 20 100]);
Kd=diag([100 10 100]);


if planner == 1
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_1(Ts);

elseif planner == 2
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_2(Ts);

else
    error('Undefined Planner');
end

    %to simulink
    simIn = Simulink.SimulationInput('passivity_based_control_project');

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
    simIn = simIn.setVariable('Kd', Kd);
    simIn = simIn.setVariable('ni', ni);
    simIn = simIn.setVariable('Do', Do);
    simIn = simIn.setVariable('Ko', Ko);
    simIn = simIn.setVariable('sigma', sigma);


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
%save('passivity_data2.mat', 'norm_pos', 'norm_ang', 'norm_lin_vel', 'norm_ang_vel');

end

