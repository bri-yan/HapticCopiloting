
K = 0.011; %Nm/A - motor torque constant
R = 1.17; %ohm - winding resistance
L = 0.58e-3;%H - winding inductance

B = 4.32e-5;%n-m/(rad/s) - friction/damping constant
Jm = 9.18e-7; % Kg*m^2 rotor intertia
J = 2* Jm; % Kg*m^2 estimate of rotor + knob intertia
D = 1
Kt = K
M_inv = 1/J
Lw = L
Rw = R
Km = K
D = 0.6
Bm = 0.8

mdl = "Admittance";
out = sim(mdl);

desiredpos = out.simout

outputpos = out.simout1

tiledlayout(2,1)
nexttile
plot(out.simout, 'LineWidth', 2, 'Color', [1, 0.5, 0])
title("Admittance Controller: Desired Position vs Time")
xlabel("Time (seconds)")
ylabel("Desired Position (radians)")
grid on

nexttile
plot(out.simout1, 'LineWidth', 2, 'Color', [1, 0.5, 0])
title("Admittance Controller: Resulting Position vs Time")
xlabel("Time (seconds)")
ylabel("Resulting Position (radians)")
grid on