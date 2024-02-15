%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%motor parameters
%we dont know these yet
J = 1.62e-6; %intertia [kg * m^2]
B = 1.36e-6; % viscous friction coeffcient
K = 1.10e-4; %Kd value
M_inv = 1/J;

%from motor datasheet
Kt = 0.011; %motor constant (back emf constant = torque constant)
Ke = 0.011;
R = 1.17; %motor electric resistance in ohms
L = 0.58E-3; %motor inductance in ohms
