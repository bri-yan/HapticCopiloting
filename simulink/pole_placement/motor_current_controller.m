%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DC Motor Current Controller Design
% Yousif El-Wishahy
% ======================================
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

%motor params
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K = 0.011; %Nm/A - motor torque constant
R = 1.17; %ohm - winding resistance
L = 0.58e-3;%H - winding inductance

B = 4.32e-5;%n-m/(rad/s) - friction/damping constant
Jm = 9.18e-7; % Kg*m^2 rotor intertia
J = 2* Jm; % Kg*m^2 estimate of rotor + knob intertia


%current / voltage transfer function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = tf('s');
G = (s/L + B/(L*J))/(s^2 + s*(L*B+J*R)/(L*J) + (K^2 + B*R)/(L*J))


%poole placement using diophantine polynomail equation technique
% goal is to obtain params for a PID controller to control G
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%eq = (s^2 + s*(L*B+J*R)/(L*J) + (K^2 + B*R)/(L*J)) * s * (s + l0) + (s/L + B/(L*J)) * (p2*s^2 + p1*s + p0) == (s^2 + s +  B/(L*J)) * (s+ B/(L*J))^2;




