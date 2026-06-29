%%Torque-to-Throttle-to-Torque
clear; clc; close all;

Ts = 0.02;              
Jw = 2.4932e-5; %wheel inertia (kg*m^2)
t_end = 50.0;               
time = 0:Ts:t_end;
N = length(time);
throttle_offset = 50;

% sys_tf = (43.99 * z^-1) / (1 - 0.7224 * z^-1 + 0.05095 * z^-2)
b1 = 43.99;
a1 = -0.7224;
a2 = 0.05095;

rpm_to_rads = (2 * pi) / 60;


tau_cmd = zeros(1, N);     
omega_cmd = zeros(1, N);    
throttle = zeros(1, N);     


omega_rpm = zeros(1, N);   
omega_rads = zeros(1, N);   
alpha_rads = zeros(1, N);  
tau_actual = zeros(1, N);  


% tau_cmd(time >= 0.5 & time < 2.0) =  0.001; 
% tau_cmd(time >= 2.0 & time < 3.5) = -0.001; 

tau_cmd = 0.001*sin(1*time);


for k = 3:N
   
    % integrate commanded torque to find desired wheel speed
    omega_cmd(k) = omega_cmd(k-1) + (tau_cmd(k-1) / Jw) * Ts;
   
    K_tf = b1 / (1 + a1 + a2); 
    omega_cmd_rpm = omega_cmd(k) / rpm_to_rads;
    ideal_throttle = omega_cmd_rpm / K_tf;
    
    throttle_cmd = throttle_offset + ideal_throttle; 
   
    throttle(k) = max(0, min(100, throttle_cmd));

    % omega[k] = b1*u[k-1] - a1*omega[k-1] - a2*omega[k-2]
    omega_rpm(k) = b1 * throttle(k-1) - a1 * omega_rpm(k-1) - a2 * omega_rpm(k-2);
    
    omega_rads(k) = omega_rpm(k) * rpm_to_rads;

    alpha_rads(k) = (omega_rads(k) - omega_rads(k-1)) / Ts;
    
    tau_actual(k) = alpha_rads(k) * Jw;
end

figure('Position', [100, 100, 800, 600]);

start = 0.5;

subplot(3,1,1);
plot(time, tau_cmd * 1e3, 'r--', 'LineWidth', 1.5); hold on;
plot(time, tau_actual * 1e3, 'b', 'LineWidth', 1.5); xlim([start t_end]);
grid on; legend('commanded (\tau_{cmd})', 'actual Produced (\tau_{actual})');
ylabel('Torque (mN*m)');

subplot(3,1,2);
plot(time, throttle, 'k', 'LineWidth', 1.5); xlim([start t_end]);
grid on; ylabel('Throttle (%)');

subplot(3,1,3);
plot(time, omega_cmd, 'r--', 'LineWidth', 1.5); hold on; 
plot(time, omega_rads, 'b', 'LineWidth', 1.5);
grid on; legend('Commanded (\omega_{cmd})', 'Actual (\omega_{actual})');
ylabel('Speed (rad/s)'); xlabel('Time (seconds)');
xlim([start t_end]);