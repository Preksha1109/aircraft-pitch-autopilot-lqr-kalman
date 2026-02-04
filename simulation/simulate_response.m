t = 0:0.01:10;
x0 = [0 0 0 0.1];   % Initial pitch angle disturbance

[y,t,x] = initial(sys_cl, x0, t);

figure;
plot(t, x(:,4));
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
title('Closed-Loop Pitch Response');
grid on;
