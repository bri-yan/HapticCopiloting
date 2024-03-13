clear; close all;
figure(1); hold on; grid on;

load('ta.mat');t = out.torque.Time;p = out.torque.Data;
plot(t,p,'k-','LineWidth',2)

load('tb.mat');t = out.torque.Time;p = out.torque.Data;
plot(t,p,'b--','LineWidth',2)

% load('tb.mat');t = out.torque_net.Time;p = out.torque_net.Data;
% plot(t,p,'r','LineWidth',2)

% load('te.mat');t = out.torque_net.Time;p = out.torque_net.Data;
% plot(t,p,'LineWidth',2)

legend('no disturbance, k=0.05', 'disturbance, k=0.05', 'Location','SouthEast')
xlabel('Time (s)')
ylabel('Torque (Nm)')