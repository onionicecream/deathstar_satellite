clc;
clear;
close all;

%% sample time
Ts = 0.02;

%% motor model

num = [0 43.99];
den = [1 -0.7224 0.05095];

G_rpm = tf(num, den, Ts);

%% convert RPM -> rad/s

rpm_to_rads = 2*pi/60;

G = G_rpm * rpm_to_rads;

%% time vector

t = 0:Ts:5;

%% open-loop throttle input
% Ramp from 0% to 20%

u = 80 * (t / max(t));

%% simulate

omega = lsim(G, u, t);

%% linear fit

p_u = polyfit(t, u, 1); %is already linear     
p_w = polyfit(t, omega, 1);

du_dt = p_u(1)
dw_dt = p_w(1)

K_est = dw_dt / du_dt % dc gain

Kp_est = 1 / K_est %proportional control

%% plot

figure(1);

u_fit = polyval(p_u, t);
w_fit = polyval(p_w, t);

plot(t, u, 'b', t, u_fit, '--b', 'LineWidth', 2);
hold on;
plot(t, omega, 'r', t, w_fit, '--r', 'LineWidth', 2);

grid on;

legend('u', 'u fit', '\omega', '\omega fit');
xlabel('Time (s)');    
title('Open-loop');


%% open-loop + gain
Kp_par = Kp_est;
omega = lsim(Kp_par*G, u, t);

%% plot

figure(2);

plot(t, u, 'b', 'LineWidth', 2);
hold on;
plot(t, omega, 'r', 'LineWidth', 2);

grid on;

legend('throttle','\omega');
xlabel('Time (s)');    
title('Open-loop with gain');

%% CLOSED_LOOP

%% gains

Kp = 0.0714; %*2.7;  
C = Kp;

%% closed-loop

sys_cl = feedback(C*G, 1);

%% input

w_ref = 20 * (t/5); % ramp from 0 to 20 rad/s

% w_ref = t>1; % step input


%% response

w_out = lsim(sys_cl, w_ref, t);

%% plot

figure(3);
plot(t, w_ref, 'LineWidth', 2);
hold on;
plot(t, w_out, 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

legend('Reference', 'Motor Response');

title(['Motor Speed Loop with Kp = ', num2str(Kp)]);

%% closed-loop with feedward

t = 0:Ts:10;

% w_ref = 20* (t>1);

w_ref = 20*(t/5);

% Kp = 0.01;
% Ki = 0.05;

Kp = 0.0;
Ki = 0.1;

Kff = 1/14;

u = zeros(size(t));
w_out = zeros(size(t));

integrator = 0;

for k = 2:length(t)

    e = w_ref(k-1) - w_out(k-1);

    integrator = integrator + Ki*e*Ts;

    % offset = 50;
    offset = 0;

    u(k) = offset + Kff*w_ref(k) + Kp*e + integrator;

    u(k) = min(max(u(k),0),100);

    w_temp = lsim(G,u(1:k),t(1:k));

    w_out(k) = w_temp(end);

end

figure(4);
plot(t,w_ref,'LineWidth',2);
hold on;
plot(t,w_out,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

legend('Reference','Motor Response');
title('Feedforward + PI');