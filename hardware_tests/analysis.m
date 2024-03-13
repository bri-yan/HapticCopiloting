pos = table2array(addconst1)

figure(1);grid on;hold on;
plot(pos);
xlabel('# Samples (1000 samples/s)');
ylabel('Position (deg)')
title('Step Response Test')