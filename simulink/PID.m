% Code is for step response from a PID controller

s = tf('s') % s as variable for transfer function of plant
P = 1/(s^2 + 10*s + 20) % transfer function of plant


Kp = 350; %PID params
Ki = 300;
Kd = 50;
C = pid(Kp,Ki,Kd) %matlab PID object
T = feedback(C*P,1); %PID controller with feedback gain of 1
C*P

t = 0:0.01:2;
step(T,t) % step response for the transfer function

pidTuner(P,C) % for tuning PID controller in real-time
         