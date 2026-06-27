%% Parse Data from CSVs (Apr17_Data — all phases, 2-body, sand terrain)
clear
clc

Phase_String = ["Con" "Des" "No"]; % 3 phase labels for this dataset
Trial_String = ["1" "2" "3"];       % trial numbering uses _1, _2, _3 (not _T1 etc.)
nP = length(Phase_String);
nT = length(Trial_String);

data_folder = "Apr17_Data";

% Preallocate vectors (phase x trial x datapoints)
t       = zeros(nP, nT, 25000);
rear_x  = zeros(nP, nT, 25000);
rear_y  = zeros(nP, nT, 25000);
rear_z  = zeros(nP, nT, 25000);
front_x = zeros(nP, nT, 25000);
front_y = zeros(nP, nT, 25000);
front_z = zeros(nP, nT, 25000);
Length  = zeros(nP, nT);
file_loaded = false(nP, nT);

% Column offsets (1-indexed, readmatrix auto-skips headers)
t_col        = 2;  % Time is in column B
front_offset = 7;  % legacyFront position starts at column G (x=G, y=H, z=I)
rear_offset  = 23; % legacyRear position starts at column W (x=W, y=X, z=Y)

for i = 1:nP
    for j = 1:nT
        % Build filename
        string   = "Apr17_Sand_" + Phase_String(i) + "_" + Trial_String(j);
        filename = fullfile(data_folder, string + ".csv");

        % Skip if file doesn't exist
        if ~isfile(filename)
            disp("Skipping missing file: " + filename)
            continue
        end

        % Read CSV — readmatrix() automatically skips header rows
        M = readmatrix(filename);
        [L, ~] = size(M);
        Length(i,j) = L;
        file_loaded(i,j) = true;

        % Extract time
        t(i, j, 1:L) = M(:, t_col);

        % Extract front marker positions
        front_x(i, j, 1:L) = M(:, 0+front_offset);
        front_y(i, j, 1:L) = M(:, 1+front_offset);
        front_z(i, j, 1:L) = M(:, 2+front_offset);

        % Extract rear marker positions
        rear_x(i, j, 1:L) = M(:, 0+rear_offset);
        rear_y(i, j, 1:L) = M(:, 1+rear_offset);
        rear_z(i, j, 1:L) = M(:, 2+rear_offset);
    end
end

%% Compute Velocities
rear_Vx  = zeros(nP, nT, 25000);
front_Vx = zeros(nP, nT, 25000);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        for m = 1:Length(i,j)-1
            rear_Vx(i,j,m)  = rear_x(i,j,m+1)  - rear_x(i,j,m);
            front_Vx(i,j,m) = front_x(i,j,m+1) - front_x(i,j,m);
        end
    end
end
rear_Vx  = rear_Vx  * 120; % 120Hz frame rate
front_Vx = front_Vx * 120;

%% Start-Detection X (manual start input... data too messy)
%{
window    = 10;
threshold = [0.01,0.2,0.01];

start = ones(nP, nT);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        for m = 10:Length(i,j)-window
            window_mean = mean(rear_Vx(i,j,m:m+window));
            if abs(window_mean) > threshold(i)
                start(i,j) = m;
                break
            end
        end
    end
end
%}
start=[[1300, 2000, 2000]; [1200, 2000, 1700];[1550, 2800, 1900]];

%disp(start)

%% End-Detection: find where sustained motion drops back to near-zero
%{
end_window    = 50;  % number of frames to average over (same scale as start window)
end_threshold = 0.01; % mean velocity threshold in m/s — below this counts as "stopped"

end_frame = Length; % default to full length if no clear stop is found

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        for m = start(i,j)+end_window : Length(i,j)-end_window
            window_mean = mean(rear_Vx(i,j,m:m+end_window));
            if abs(window_mean) < end_threshold
                end_frame(i,j) = m;
                break
            end
        end
    end
end
%}
end_frame = [[3100,3950,4000]; [3100,3900,3900]; [3200, 4500, 3800]];

%disp(end_frame)

%% Plot Start and End (run for checking)
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        if ~file_loaded(i,j)
            title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
            continue
        end
        plot(reshape(rear_Vx(i,j,1:Length(i,j)), [1 Length(i,j)]))
        hold on
        xline(start(i,j), 'r', 'LineWidth', 2)
        xline(end_frame(i,j), 'r', 'LineWidth', 2)
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('Vx (m/s)')
        xlabel('frame')
        xlim([10, Length(i,j)])
        ylim([-0.5 0.5])
        hold off
    end
end
sgtitle("Pierre Sand Start and End")

%% Clip data
valid_lengths = (end_frame - start);
valid_lengths(~file_loaded) = NaN; % ignore missing trials when finding the minimum
clip_length = min(valid_lengths(:)) + 1; % shortest valid trial sets common length

rear_x_clipped  = zeros(nP, nT, clip_length);
rear_y_clipped  = zeros(nP, nT, clip_length);
rear_z_clipped  = zeros(nP, nT, clip_length);
front_x_clipped = zeros(nP, nT, clip_length);
front_y_clipped = zeros(nP, nT, clip_length);
front_z_clipped = zeros(nP, nT, clip_length);
t_clipped       = zeros(nP, nT, clip_length);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        s = start(i,j);

        rear_x_clipped(i,j,:)  = rear_x(i,j,  s:s+clip_length-1);
        rear_y_clipped(i,j,:)  = rear_y(i,j,  s:s+clip_length-1);
        rear_z_clipped(i,j,:)  = rear_z(i,j,  s:s+clip_length-1);
        front_x_clipped(i,j,:) = front_x(i,j, s:s+clip_length-1);
        front_y_clipped(i,j,:) = front_y(i,j, s:s+clip_length-1);
        front_z_clipped(i,j,:) = front_z(i,j, s:s+clip_length-1);
        t_clipped(i,j,:)        = t(i,j, s:s+clip_length-1);

        rear_x_clipped(i,j,:)  = rear_x_clipped(i,j,:)  - rear_x_clipped(i,j,1);
        rear_y_clipped(i,j,:)  = rear_y_clipped(i,j,:)  - rear_y_clipped(i,j,1);
        rear_z_clipped(i,j,:)  = rear_z_clipped(i,j,:)  - rear_z_clipped(i,j,1);
        front_x_clipped(i,j,:) = front_x_clipped(i,j,:) - front_x_clipped(i,j,1);
        front_y_clipped(i,j,:) = front_y_clipped(i,j,:) - front_y_clipped(i,j,1);
        front_z_clipped(i,j,:) = front_z_clipped(i,j,:) - front_z_clipped(i,j,1);
        t_clipped(i,j,:)        = t_clipped(i,j,:) - t_clipped(i,j,1);
    end
end
%% Calculate total 3D distance
rear_dist  = zeros(nP, nT, clip_length);
front_dist = zeros(nP, nT, clip_length);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        rear_dist(i,j,:)  = sqrt(rear_x_clipped(i,j,:).^2  + rear_y_clipped(i,j,:).^2  + rear_z_clipped(i,j,:).^2);
        front_dist(i,j,:) = sqrt(front_x_clipped(i,j,:).^2 + front_y_clipped(i,j,:).^2 + front_z_clipped(i,j,:).^2);
    end
end

%% Plot rear displacement (x, y, z) and total distance
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        if ~file_loaded(i,j)
            title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
            continue
        end
        hold on
        tt = squeeze(t_clipped(i,j,:));
        plot(tt, squeeze(rear_x_clipped(i,j,:)), 'r')
        plot(tt, squeeze(rear_y_clipped(i,j,:)), 'g')
        plot(tt, squeeze(rear_z_clipped(i,j,:)), 'b')
        plot(tt, squeeze(rear_dist(i,j,:)), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5)
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('distance / m')
        xlabel('time / s')
        hold off
    end
end
sgtitle("Pierre Sand Rear Displacement vs. Time")
lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
lgd.Units = 'normalized';
lgd.Position = [0.4, 0.02, 0.2, 0.02];

%% Plot front displacement (x, y, z) and total distance
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        if ~file_loaded(i,j)
            title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
            continue
        end
        hold on
        tt = squeeze(t_clipped(i,j,:));
        plot(tt, squeeze(front_x_clipped(i,j,:)), 'r')
        plot(tt, squeeze(front_y_clipped(i,j,:)), 'g')
        plot(tt, squeeze(front_z_clipped(i,j,:)), 'b')
        plot(tt, squeeze(front_dist(i,j,:)), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5)
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('distance / m')
        xlabel('time / s')
        hold off
    end
end
sgtitle("Pierre Sand Front Displacement vs. Time")
lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
lgd.Units = 'normalized';
lgd.Position = [0.4, 0.02, 0.2, 0.02];

%% Step length calculation
smooth_window = 30;
all_steps_rear  = cell(nP, 1); % collect all individual steps across trials
all_steps_front = cell(nP, 1);
all_steps       = cell(nP, 1);
avg_step        = NaN(nP, nT); % per-trial average for display

for i = 1:nP
    all_steps_rear{i}  = [];
    all_steps_front{i} = [];
    all_steps{i}        = [];
    for j = 1:nT
        if ~file_loaded(i,j), continue, end

        rear_dist_smooth  = movmean(squeeze(rear_dist(i,j,:)),  smooth_window);
        front_dist_smooth = movmean(squeeze(front_dist(i,j,:)), smooth_window);
        rear_deriv  = diff(rear_dist_smooth)  * 120;
        front_deriv = diff(front_dist_smooth) * 120;

        % Find peaks in derivative
        [~, rear_locs]  = findpeaks(rear_deriv,  'MinPeakDistance', 80, 'MinPeakProminence', 0.008);
        [~, front_locs] = findpeaks(front_deriv, 'MinPeakDistance', 80, 'MinPeakProminence', 0.008);

        % Individual step lengths for this trial
        rear_steps  = diff(rear_dist_smooth(rear_locs));
        front_steps = diff(front_dist_smooth(front_locs));

        % Append raw steps to pool for this phase
        all_steps_rear{i}  = [all_steps_rear{i},  rear_steps'];
        all_steps_front{i} = [all_steps_front{i}, front_steps'];
        all_steps{i}        = [all_steps{i}, rear_steps', front_steps'];

        % Store per-trial average for display
        avg_step(i,j) = (mean(rear_steps) + mean(front_steps)) / 2;
    end
end

% Display results per trial
disp('Average step length per trial:')
for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j)
            disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
            continue
        end
        disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(avg_step(i,j)*100, 2) + " cm")
    end
end

%% Plot of average step length per phase
avg_step_per_phase = NaN(nP, 1);
std_step_per_phase = NaN(nP, 1);

for i = 1:nP
    if isempty(all_steps{i}), continue, end
    % Mean and std computed from full pool of individual steps
    avg_step_per_phase(i) = mean(all_steps{i});
    std_step_per_phase(i) = std(all_steps{i});
end

figure
hold on
errorbar(1:nP, avg_step_per_phase * 100, std_step_per_phase * 100, 'b.', 'LineWidth', 1.5)
plot(1:nP, avg_step_per_phase * 100, 'o','MarkerEdgeColor', 'b', 'LineWidth', 1.5, 'MarkerFaceColor', 'w')
xlim([0.5, nP+0.5]) % force full x range
xticks(1:nP)
xticklabels(Phase_String)
xlabel('Coordination Type')
ylabel('Step Length (cm)')
title("Pierre Sand Average Step Length by Phase")
ylim([-1 11])

%% Vertical displacement (Y) per step — rear and front averaged
smooth_window_y = 30;
all_y_range = cell(nP, 1);
avg_y_range  = NaN(nP, nT);

for i = 1:nP
    all_y_range{i} = [];
    for j = 1:nT
        if ~file_loaded(i,j), continue, end

        dist_smooth = movmean(squeeze(rear_dist(i,j,:)), smooth_window_y);
        deriv = diff(dist_smooth) * 120;
        [~, locs] = findpeaks(deriv, 'MinPeakDistance', 80, 'MinPeakProminence', 0.008);

        if length(locs) < 2, continue, end

        y_rear_smooth  = movmean(squeeze(rear_y_clipped(i,j,:)),  smooth_window_y);
        y_front_smooth = movmean(squeeze(front_y_clipped(i,j,:)), smooth_window_y);

        y_rear_ranges  = zeros(length(locs)-1, 1);
        y_front_ranges = zeros(length(locs)-1, 1);
        for b = 1:length(locs)-1
            seg_rear  = y_rear_smooth(locs(b):locs(b+1));
            seg_front = y_front_smooth(locs(b):locs(b+1));
            y_rear_ranges(b)  = max(seg_rear)  - min(seg_rear);
            y_front_ranges(b) = max(seg_front) - min(seg_front);
        end

        all_y_range{i} = [all_y_range{i}; y_rear_ranges; y_front_ranges];
        avg_y_range(i,j) = (mean(y_rear_ranges) + mean(y_front_ranges)) / 2;
    end
end

disp('Average vertical displacement per step:')
for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j)
            disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
            continue
        end
        disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(avg_y_range(i,j)*100, 2) + " cm")
    end
end

%% Penetration depth + modeled step length per phase (only valid where data exists)
h = 1.75; % cm
D = 7;  % cm
R = D/2;

avg_y_overall = NaN(nP, 1);
std_y_overall = NaN(nP, 1);
d_phase       = NaN(nP, 1);
s_phase       = NaN(nP, 1);
std_s_phase   = NaN(nP, 1);

delta = 1e-6;
for i = 1:nP
    if isempty(all_y_range{i}), continue, end

    avg_y_overall(i) = mean(all_y_range{i});
    std_y_overall(i) = std(all_y_range{i});

    d_phase(i) = D - h - avg_y_overall(i)*100;
    s_phase(i) = 2*sqrt((R)^2 - (d_phase(i)+h-R)^2);

    % error propagation
    y_plus  = avg_y_overall(i) + delta;
    y_minus = avg_y_overall(i) - delta;
    d_plus  = D - h - y_plus*100;
    d_minus = D - h - y_minus*100;
    s_plus  = 2*sqrt((R)^2 - (d_plus+h-R)^2);
    s_minus = 2*sqrt((R)^2 - (d_minus+h-R)^2);
    ds_dy = (s_plus - s_minus) / (2*delta);
    std_s_phase(i) = abs(ds_dy) * std_y_overall(i);
end

disp('Penetration depth, aerial contribution, and modeled step length per phase:')
for i = 1:nP
    if isnan(d_phase(i))
        disp(Phase_String(i) + ": no data")
        continue
    end
    disp(Phase_String(i) + ": d = " + round(d_phase(i),2) + " cm, modeled step length = " + round(s_phase(i),2) + " ± " + round(std_s_phase(i),2) + " cm, actual step length = "+ound(avg_step_per_phase * 100,2) + " ± " + round(std_step_per_phase*100,2) + " cm")
end