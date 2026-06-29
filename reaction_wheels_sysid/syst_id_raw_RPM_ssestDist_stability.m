%% sys ID: throttle -> RPM - raw pulses RPM
clear; close all; clc;
%% parameters
skip_per_period_plot = true;
MAGNETS_PER_REV = 7;
np = 2;
nz = 1;
nx = 2:7;
fs = 50;
Ts_nominal = 1/50;
f0 = 0.05;
N = 1000;
P = 21;
fmin = 0.5;
fmax = 25;
base = "logs/";
logs_offset40 = [
"log_001_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset40.0_20s_50Hz_1000samp.csv",
"log_003_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset40.0_20s_50Hz_1000samp.csv",
"log_005_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset40.0_20s_50Hz_1000samp.csv",
"log_007_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset40.0_20s_50Hz_1000samp.csv",
"log_008_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset55.0_20s_50Hz_1000samp.csv"
];
periods_offset40 = [21,21,11,11,11];
logs_offset55 = [
"log_002_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset55.0_20s_50Hz_1000samp.csv",
"log_004_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset55.0_20s_50Hz_1000samp.csv",
"log_006_ms_0.5to25.0Hz_489comp_rph_seed13_amp40.0_offset55.0_20s_50Hz_1000samp.csv",
];
periods_offset55 = [21, 11, 11];
% combine the log file paths
allLogs = [strcat(base, logs_offset40); strcat(base, logs_offset55)];
allPeriods = [periods_offset40, periods_offset55];

% train/val by index
idx_train = [1 2 3 6 7];
idx_val = [4 5 6];
logs_train = allLogs(idx_train);
periods_train = allPeriods(idx_train);
logs_val = allLogs(idx_val);
periods_val = allPeriods(idx_val);

%% estimation options
opt_tf = tfestOptions('EnforceStability', true, 'InitialCondition', 'zero');
opt_ss_no_dist = ssestOptions('Focus', 'simulation', 'EnforceStability', true, 'InitialState', 'zero');
opt_ss_dist    = ssestOptions('Focus', 'simulation', 'EnforceStability', true, 'InitialState', 'zero');

%% load training logs
N_train = length(logs_train);
d_trains = cell(N_train, 1);
names_train = cell(N_train, 1);
fprintf('--- Training logs ---\n');
for k = 1:N_train
    [d_trains{k}, names_train{k}] = load_log(fs, periods_train(k), N, logs_train(k), Ts_nominal, MAGNETS_PER_REV);
    fprintf('  [train %d] %s  (%d samples, Ts=%.4fs)\n', ...
        k, names_train{k}, size(d_trains{k}.OutputData,1), d_trains{k}.Ts);
end
d_merged = merge(d_trains{:});
% delay estimation
nk = delayest(d_merged);

%% estimate models on merged training data
delay = 0; % n sample delay
Ts = 1/fs;
sys_tf      = tfest(d_merged, np, nz,'Ts',Ts, opt_tf, 'InputDelay', delay);
sys_ss_none = ssest(d_merged, nx, 'Ts',Ts, opt_ss_no_dist, 'InputDelay', delay, 'DisturbanceModel', 'none'); 
sys_ss_dist = ssest(d_merged, nx, 'Ts',Ts, opt_ss_dist, 'InputDelay', delay); 

fprintf('\nTraining fit (merged):\n');
[~, fit_tf_train] = compare(d_merged, sys_tf);
[~, fit_ss_none_train] = compare(d_merged, sys_ss_none);
[~, fit_ss_dist_train] = compare(d_merged, sys_ss_dist);

fprintf('  tfest: %.1f%%   ssest (no dist): %.1f%%   ssest (dist): %.1f%%\n', ...
    mean(cell2mat(fit_tf_train)), mean(cell2mat(fit_ss_none_train)), mean(cell2mat(fit_ss_dist_train)));

%% load validation logs
N_val= length(logs_val);
d_vals = cell(N_val, 1);
names_val = cell(N_val, 1);
fprintf('\n--- Validation logs ---\n');
for k = 1:N_val
    [d_vals{k}, names_val{k}] = load_log(fs, periods_val(k), N, logs_val(k), Ts_nominal, MAGNETS_PER_REV);
    [~, fit_tf] = compare(d_vals{k}, sys_tf);
    [~, fit_ss_none] = compare(d_vals{k}, sys_ss_none);
    [~, fit_ss_dist] = compare(d_vals{k}, sys_ss_dist);
    fprintf('  [val %d] %-55s  tf: %.1f%%  ss(none): %.1f%%  ss(dist): %.1f%%\n', ...
        k, names_val{k}, fit_tf, fit_ss_none, fit_ss_dist);
end

%% figure 1: training input/output
figure(1); clf;
sgtitle('Training — Input and Output (offset removed)');
for k = 1:N_train
    d = d_trains{k};
    t = d.SamplingInstants;
    subplot(N_train, 2, 2*k-1);
    plot(t, d.InputData); ylabel('Throttle (%)');
    title(names_train{k}, 'Interpreter','none'); grid on;
    subplot(N_train, 2, 2*k);
    plot(t, d.OutputData); ylabel('RPM'); grid on;
end

%% figure 2: validation input/output
figure(2); clf;
sgtitle('Validation — Input and Output (offset removed)');
for k = 1:N_val
    d = d_vals{k};
    t = d.SamplingInstants;
    subplot(N_val, 2, 2*k-1);
    plot(t, d.InputData); ylabel('Throttle (%)');
    title(names_val{k}, 'Interpreter','none'); grid on;
    subplot(N_val, 2, 2*k);
    plot(t, d.OutputData); ylabel('RPM'); grid on;
end

%% figure 3: compare on validation
figure(3); clf;
for k = 1:N_val
    subplot(N_val,1,k);
    compare(d_vals{k}, sys_tf, sys_ss_none, sys_ss_dist);
    title(['VAL: ' names_val{k}], 'Interpreter','none');
    legend('Measured','tfest','ssest (none)','ssest (dist)','Location','best'); grid on;
end

%% nonparametric TF: FFT per period, averaged over periods and signals
G_per_signal = cell(length(allLogs), 1);
all_freqs_exc = [];
for ll = 1:length(allLogs)
    logfile = string(allLogs(ll));
    P_ll = allPeriods(ll);
    freqvec = (0:N-1).' * f0;
    exc_idx = find(freqvec > fmin & freqvec < fmax);
    freqs_exc = freqvec(exc_idx);
    if isempty(all_freqs_exc), all_freqs_exc = freqs_exc; end
    
    T_log = readtable(allLogs(ll), 'TextType','string');
    t_pulse = double(T_log.arduino_us) / 1e6;
    t_pulse = t_pulse - t_pulse(1);
    u_raw = double(T_log.throttle_pct);
    dt_p = diff(t_pulse);
    rpm_inst = 60 ./ (dt_p * MAGNETS_PER_REV);
    t_rpm = t_pulse(1:end-1) + dt_p/2;
    rpm_filt = rpm_inst; 
    
    t_uni = (0:N*P_ll-1).' / fs;
    u_uni = interp1(t_pulse,u_raw,t_uni,'linear','extrap');
    y_uni = interp1(t_rpm, rpm_filt, t_uni,'linear', 'extrap');
    
    u_uni = dtrend(u_uni,1);
    y_uni = dtrend(y_uni,1);
    
    minusperiods = 1; 
    u_mat = reshape(u_uni(N+1:end), N, P_ll-1);
    y_mat = reshape(y_uni(N+1:end), N, P_ll-1);
    
    U_fft = fft(u_mat, [], 1);
    Y_fft = fft(y_mat, [], 1);
    G_periods = Y_fft(exc_idx,:) ./ U_fft(exc_idx,:);
    G_per_signal{ll} = mean(G_periods, 2);
    
    if skip_per_period_plot, continue; end
    figure(20 + ll); clf;
    tiledlayout(2, 1, 'TileSpacing','tight','Padding','tight');
    [fname, ~] = fileparts(char(allLogs(ll)));
    sgtitle(sprintf('FFT per period — %s', fname), 'Interpreter','none');
    nexttile;
    for pp = 1:P_ll-minusperiods
        semilogx(freqs_exc, mag2db(abs(Y_fft(exc_idx,pp)))); hold on;
    end
    semilogx(freqs_exc, mag2db(abs(mean(Y_fft(exc_idx,:),2))), 'k-', 'LineWidth',2);
    ylabel('|Y| (dB)'); xlabel('Freq (Hz)'); title('Output'); grid on;
    nexttile;
    for pp = 1:P_ll-minusperiods
        semilogx(freqs_exc, mag2db(abs(U_fft(exc_idx,pp)))); hold on;
    end
    semilogx(freqs_exc, mag2db(abs(mean(U_fft(exc_idx,:),2))), 'k-', 'LineWidth',2);
    ylabel('|U| (dB)'); xlabel('Freq (Hz)'); title('Input'); grid on;
end

G_avg = mean(cat(2, G_per_signal{:}), 2);
mag_db = mag2db(abs(G_avg));
phase_deg = unwrap(angle(G_avg)) * 180/pi;

%% figure 5: bode + overlay
omega = logspace(log10(2*pi*0.1), log10(2*pi*20), 500);
freq_hz = omega / (2*pi);
h_tf= squeeze(freqresp(sys_tf, omega));
h_ss_none = squeeze(freqresp(sys_ss_none, omega));
h_ss_dist = squeeze(freqresp(sys_ss_dist, omega));

figure(5); clf;
subplot(2,1,1);
semilogx(freq_hz, mag2db(abs(h_tf)),'LineWidth',2); hold on;
semilogx(freq_hz,mag2db(abs(h_ss_none)), 'LineWidth',2);
semilogx(freq_hz, mag2db(abs(h_ss_dist)), 'LineWidth',2);
semilogx(all_freqs_exc, mag_db, 'k.', 'MarkerSize', 10);
ylabel('Magnitude (dB)'); grid on;
legend('tfest','ssest (none)','ssest (dist)','TF avg', 'Location','best');
title('Bode - Magnitude');

subplot(2,1,2);
semilogx(freq_hz, unwrap(angle(h_tf))*180/pi,'LineWidth',2); hold on;
semilogx(freq_hz, unwrap(angle(h_ss_none))*180/pi,'LineWidth',2);
semilogx(freq_hz, unwrap(angle(h_ss_dist))*180/pi,'LineWidth',2);
semilogx(all_freqs_exc, phase_deg, 'k.', 'MarkerSize', 10);
ylabel('Phase (deg)'); xlabel('Frequency (Hz)'); grid on;
title('Bode - Phase');

%% residual analysis
opt_resid = residOptions('InitialCondition', 'z', 'MaxLag', 25);
figure(6); clf;

subplot(2,3,1);
resid(d_merged, sys_tf, 'corr', opt_resid);
title('tfest — Training');
subplot(2,3,2);
resid(d_merged, sys_ss_none, 'corr', opt_resid);
title('ssest (none) — Training');
subplot(2,3,3);
resid(d_merged, sys_ss_dist, 'corr', opt_resid);
title('ssest (dist) — Training');

d_val_merged = merge(d_vals{:});
subplot(2,3,4);
resid(d_val_merged, sys_tf, 'corr', opt_resid);
title('tfest — Validation');
subplot(2,3,5);
resid(d_val_merged, sys_ss_none, 'corr', opt_resid);
title('ssest (none) — Validation');
subplot(2,3,6);
resid(d_val_merged, sys_ss_dist, 'corr', opt_resid);
title('ssest (dist) — Validation');


%% stability check open-loop
P_tf = sys_tf;
P_ss = sys_ss_none;
P_ss_d = sys_ss_dist;

fprintf('--------------\n\n');

fprintf('TF model:\n');
fprintf('poles:\n');
disp(pole(P_tf))
fprintf('zeros:\n');
disp(zero(P_tf))

fprintf('--------------\n\n');
fprintf('SS model (no disturbance):\n');
fprintf('poles:\n');
disp(pole(P_ss))
fprintf('zeros:\n');
disp(zero(P_ss))
fprintf('--------------\n\n');
fprintf('SS model (with disturbance):\n');
fprintf('poles:\n');
disp(pole(P_ss_d))
fprintf('zeros:\n');
disp(zero(P_ss_d))

isstable(P_tf)   % returns 1 if all poles are inside unit circle
isstable(P_ss)
isstable(P_ss_d)

% visual comparison
figure(8);clf;
pzmap(P_tf, P_ss, P_ss_d);
zgrid;
title('Open-Loop Pole-Zero Map (Discrete Time)');
legend('tfest','ssest','ssest + disturbance');

%% functions
function [d, name] = load_log(fs, P, N, logfile, Ts_nominal, mag_per_rev)
    file = string(logfile);
    [~, fname, ~] = fileparts(char(file));
    parts = strsplit(fname, '_');
    name = strjoin(parts(3:end), '_');
    T = readtable(file, 'TextType','string');
    t = double(T.arduino_us) / 1e6;
    t = t - t(1);
    u_raw = double(T.throttle_pct);
    dt = diff(t);
    rpm_inst = 60 ./ (dt * mag_per_rev);
    t_rpm  = t(1:end-1) + dt/2;
    rpm_filt = rpm_inst;
    % uniform grid
    n_total = N * P;
    t_uni = (0:n_total-1).' / fs;
    u_uni = interp1(t, u_raw, t_uni, 'linear', 'extrap');
    y_uni = interp1(t_rpm, rpm_filt, t_uni, 'linear', 'extrap');
    % discard first period
    u_uni = u_uni(N+1:end);
    y_uni = y_uni(N+1:end);
    d = iddata(y_uni, u_uni, 1/fs);
    d.InputName = 'Throttle (%)';
    d.OutputName = 'RPM';
    Ts = d.Ts;
    if abs(Ts - Ts_nominal) > 1e-6
        d = resample(d, Ts_nominal);
    else
        d.Ts = Ts_nominal;
    end
    d.OutputData = d.OutputData - mean(d.OutputData);
    d.InputData = d.InputData - mean(d.InputData);
end