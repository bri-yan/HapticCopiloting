% this example shows how to solve a diophantine equation using the given function
%{
        Suppose we have the system

                                   z^-1 (0.004837 + 0.004679 z^-1)
                        Gz = -----------------------------------------------
                                    1 - 1.905 z^-1 + 0.9048 z^-2

suppose we want to place poles at z1 = 0.8187 and z2 = 0.6703, and the
observer pole to be at z = 0.2
Am = [1 -(z1+z2) z1*z2];
A0 = [1 -0.2]
now we can write 
A = [1, -1.905,  0.9048] --> na = 2
B = [0.004837, 0.004679] --> nb = 1
d = 1
nalpha = 2+1+1-1= 3
now we can get 
nr = 1+1-1 = 1; --> R = 1 + r1 z^-1
ns = 2-1 = 1; --> S = s0 + s1 z^-1

 
%}
clc; clear all; close all;

 z1 = 0.8187;
 z2 = 0.6703;
Am = [1 -(z1+z2) z1*z2];
A0 = [1 -0.2];
A = [1, -1.905,  0.9048];
B = [0.004837, 0.004679];
d = 1;

alpha = conv(Am, A0);
[S,R,] = Diophantine(A,B, d, alpha)

