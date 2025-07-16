function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time] = nlmpc_control(planner, Ts)

if planner == 1
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_1(Ts);

elseif planner == 2
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_2(Ts);

else
    error('Undefined Planner');
end

%NLMPC
nx = 12; %states
ny = 12; %outputs
nu = 4;  %input
nlmpcobj = nlmpc(nx, ny, nu);

nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = 9;
nlmpcobj.ControlHorizon = 3;

%generated from getQuadrotorDynamicsAndjacobian
nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;

%Weights
% Ts=0.001 for planner 1
% nlmpcobj.Weights.OutputVariables = 5000*[1 1 1 1 1 1 0 0 0 0 0 0]; 
% nlmpcobj.Weights.ManipulatedVariables = [10 0.15 0.15 0.15];
% nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

nlmpcobj.Weights.OutputVariables         = [1 1 0.01 1 1 1 0 0 0 0 0 0];
nlmpcobj.Weights.ManipulatedVariables    = [10 0.16 0.16 0.01];
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

%Constraints
nlmpcobj.MV(1).Min = 0;      
nlmpcobj.MV(1).Max = 12;     
nlmpcobj.MV(2).Min = -2;
nlmpcobj.MV(2).Max = 2;
nlmpcobj.MV(3).Min = -2;
nlmpcobj.MV(3).Max = 2;
nlmpcobj.MV(4).Min = -2;
nlmpcobj.MV(4).Max = 2;


%to workspace
assignin('base', 'nlmpcobj', nlmpcobj);

%to simulink
simIn = Simulink.SimulationInput('nlmpc_control_project');

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
%save('nlmpc_data2.mat', 'norm_pos', 'norm_ang', 'norm_lin_vel', 'norm_ang_vel');

end

