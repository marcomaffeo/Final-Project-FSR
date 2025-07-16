function [pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]=mpc_control(planner,Ts)

if planner == 1
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_1(Ts);

elseif planner == 2
[pos_0, lin_vel_0, w_bb_0, p_d, dot_p_d, ddot_p_d, csi_d, dot_csi_d, ddot_csi_d, psi_d, dot_psi_d, ddot_psi_d, t, tot_time]= planner_2(Ts);

else
    error('Undefined Planner');
end

%Dati
g = 9.81;
m = 1.2;
Ib1 = 1.2416;
Ib2 = 1.2416;
Ib3 = 2*1.2416;

%matrici del sys linearizzate intorno all'hovering
% X = [x y z phi theta psi x_dot y_dot z_dot  phi_dot theta_dot psi_dot]

A=[      0         0         0         0         0         0         1         0         0         0         0         0
         0         0         0         0         0         0         0         1         0         0         0         0
         0         0         0         0         0         0         0         0         1         0         0         0
         0         0         0         0         0         0         0         0         0         1         0         0
         0         0         0         0         0         0         0         0         0         0         1         0
         0         0         0         0         0         0         0         0         0         0         0         1
         0         0         0         0        -g         0         0         0         0         0         0         0
         0         0         0         g         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0];

B=[      0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
       -1/m        0         0         0
         0       1/Ib1       0         0
         0         0       1/Ib2       0
         0         0         0       1/Ib3];

C = eye(12);

D = 0;

%model
plant = ss(A,B,C,D);
plant = c2d(plant, Ts,'zoh');

%mpc
mpcobj = mpc(plant, Ts);
mpcobj.PredictionHorizon = 9;
mpcobj.ControlHorizon = 3;   

%Initial Controller State
x0 = mpcstate(mpcobj);
x0.Plant = [0 0 -1 0 0 0 0 0 0 0 0 0]'; 

%Weights
mpcobj.Weights.OutputVariables = 5000*[1 1 0.005 1 1 1 0 0 0 0 0 0]; 
mpcobj.Weights.ManipulatedVariables = [10 0.1 0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

%Constraints
mpcobj.MV(1).Min = -1;      
mpcobj.MV(1).Max = 1;     
mpcobj.MV(2).Min = -2;
mpcobj.MV(2).Max = 2;
mpcobj.MV(3).Min = -2;
mpcobj.MV(3).Max = 2;
mpcobj.MV(4).Min = -2;
mpcobj.MV(4).Max = 2;

%to workspace
assignin('base', 'mpcobj', mpcobj);
assignin('base', 'x0', x0);

%to simulink
simIn = Simulink.SimulationInput('mpc_control_project');

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
%save('mpc_data1.mat', 'norm_pos', 'norm_ang', 'norm_lin_vel', 'norm_ang_vel');

end
