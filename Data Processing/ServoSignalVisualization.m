%% Parameters from Arduino code

clear; 
clc;

T = 2.0; % cycle_period (s)
phi_s = 100; % slow phase extent (deg)
phi_0 = 50; % slow phase center (deg)
duty_cycle = 0.9; % slow phase fraction

spine_center = 160; % spine_center
spine_magnitude = 106; % spine_magnitude

%% Time vectors
t_leg = linspace(0, 6, 6000); % 6 seconds for Beuhler leg command (split into ms)
t_spine = linspace(0, 6, 6000); % same 6-second window for spine
tau_leg = mod(t_leg, T);
tau_spine = mod(t_spine, T);

%% Beuhler clock setup
degree_slow_start = phi_0 - phi_s/2;
degree_slow_end   = phi_0 + phi_s/2;

T_slow = T * duty_cycle;
T_fast = T - T_slow;
w_f = (360.0 - phi_s) / T_fast; % fast phase angular speed
time_slow_start = degree_slow_start / w_f;
time_slow_end   = time_slow_start + T_slow;
w_s = phi_s / T_slow; % slow phase angular speed

%% Leg command
leg = zeros(size(t_leg));
for k = 1:numel(t_leg)
    tau = tau_leg(k);
    if tau <= time_slow_start
        leg(k) = w_f * tau;
    elseif tau <= time_slow_end
        leg(k) = degree_slow_start + (tau - time_slow_start) * w_s;
    else
        leg(k) = degree_slow_end + (tau - time_slow_end) * w_f;
    end
end

%% Spine triangle wave (single waveform, repeated over 6 s)
tri = 4.0 * abs(mod((tau_spine / T) + 0.25, 1.0) - 0.5) - 1.0;
spine = spine_center + spine_magnitude * tri;

%% Plot
figure;

subplot(2,1,1)
plot(t_leg, leg, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Beuhler Leg Signal');
xlim([0 6]);
ylim([0 360]);

subplot(2,1,2)
plot(t_spine, spine, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Spine Triangle Signal');
xlim([0 6]);
ylim([spine_center-spine_magnitude-10, spine_center+spine_magnitude+10]);
