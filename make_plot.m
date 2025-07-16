function [] = make_plot(out)

close all
clc

set(0, 'defaultTextInterpreter', 'latex');
set(0, 'defaultAxesTickLabelInterpreter', 'latex');
set(0, 'defaultLegendInterpreter', 'latex');


f1=figure('Renderer', 'painters', 'Position', [10 10 1000 400]);
subplot(2,1,1)
plot(out.err_pos.time, out.err_pos.signals.values,'Linewidth',2);
title('$Linear\;error$')
xlabel('$time\;[s]$');
ylabel('$Position\;error\;[m]$');
xlim([0 out.err_pos.time(end)]);
legend('$e_x$','$e_y$','$e_z$', 'Location', 'best','Orientation', 'horizontal');
set(gca, 'FontSize', 14);
grid;
subplot(2,1,2)
plot(out.dot_err_pos.time, out.dot_err_pos.signals.values,'Linewidth',2);
xlabel('$time\;[s]$');
ylabel('$Velocity\;error\;[m/s]$');
xlim([0 out.dot_err_pos.time(end)]);
legend('$\dot{e}_x$','$\dot{e}_y$','$\dot{e}_z$', 'Location', 'best','Orientation', 'horizontal');
set(gca, 'FontSize', 14);
grid;


f2=figure('Renderer', 'painters', 'Position', [10 10 1000 400]);
subplot(2,1,1)
plot(out.err_ang.time, out.err_ang.signals.values,'Linewidth',2);
title('$Angular\;error$')
xlabel('$time\;[s]$');
ylabel('$Orientation\;error\;[rad]$');
xlim([0 out.err_ang.time(end)]);
legend('$e_{\phi}$','$e_{\theta}$','$e_{\psi}$', 'Location', 'best','Orientation', 'horizontal');
set(gca, 'FontSize', 14);
grid;
subplot(2,1,2)
plot(out.dot_err_ang.time, out.dot_err_ang.signals.values,'Linewidth',2);
xlabel('$time\quad[s]$');
ylabel('$Velocity\;error\;[rad/s]$');
xlim([0 out.dot_err_ang.time(end)]);
legend('$\dot{e}_{\phi}$','$\dot{e}_{\theta}$','$\dot{e}_{\psi}$', 'Location', 'best','Orientation', 'horizontal');
set(gca, 'FontSize', 14);
grid;

f3=figure('Renderer', 'painters', 'Position', [10 10 1000 400]);
subplot(2,1,1)
plot(out.uT.time, out.uT.signals.values,'Linewidth',2);
title('$Inputs$')
xlabel('$time\;[s]$');
ylabel('$Thrust\;[N]$');
xlim([0 out.uT.time(end)]);
legend('$u_T$', 'Location', 'best');
set(gca, 'FontSize', 14);
grid;
subplot(2,1,2)
plot(out.tau_b.time, out.tau_b.signals.values,'Linewidth',2);
xlabel('$time\;[s]$');
ylabel('$Torque\;[Nm]$');
xlim([0 out.tau_b.time(end)]);
legend('$\tau_x$','$\tau_y$','$\tau_z$', 'Location', 'best','Orientation', 'horizontal');
set(gca, 'FontSize', 14);
grid;


exportgraphics(f1, 'linear_errors_nlmpc_2.pdf', 'ContentType', 'vector');
exportgraphics(f2, 'angular_errors_nlmpc_2.pdf', 'ContentType', 'vector');
exportgraphics(f3, 'inputs_nlmpc_2.pdf', 'ContentType', 'vector');

end