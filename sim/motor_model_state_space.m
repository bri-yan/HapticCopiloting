%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%motor parameters
%we dont know these yet
J = 1e-6; %intertia [kg * m^2]
B = 1e-6; %kinetic friction [N * m * s]

%from motor datasheet
K = 0.011; %motor constant (back emf constant = torque constant)
R = 1.17; %motor electric resistance in ohms
L = 0.58E-3; %motor inductance in ohms

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%state space model for electric motor
%our state vector is [angular position ; angular velocity ; current]
A = [0 1 0
    0 -B/J K/J
    0 -K/L -R/L];

B = [0 ; 0 ; 1/L];

%this gives us output vector = [ang position; ang velocity; torque]
C = [1 0 0; 0 1 0; 0 0 K];
D = [0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%convert state space to transfer functions
%based on controls textbook we
s = tf('s');
M = ((C/(s*eye(size(A,1))-A))*B) + D;

Gp = M(1) %angular position / voltage
Gv = M(2) %angular vecloity / voltage
Gt = M(3) %torque / voltage

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot motor response to a 6v step voltage input
V = 12;
t = 0:0.001:5;

figure;hold on; grid on;
plot(t,step(V*Gp,t))
plot(t,step(V*Gv,t))
%plot(t,step(V*Gt,t))
xlabel('time (s)')
title('motor step response')
legend('ang. position','ang. velocity')





