%% Parse Data from CSVs
clear
clc

% Experimental conditions
Phase_String = ["Con" "Des" "0" "2" "No"]; % 5 phases
Trial_String = ["T1" "T2" "T3"];            % 3 trials per phase
nP = length(Phase_String); % Number of phases
nT = length(Trial_String); % Number of trials

% Preallocate vectors (phase x trial x datapoints)
% (20000 is the max expected number of data rows per CSV)
t       = zeros(nP, nT, 20000);
rear_x  = zeros(nP, nT, 20000);
rear_y  = zeros(nP, nT, 20000);
rear_z  = zeros(nP, nT, 20000);
front_x = zeros(nP, nT, 20000);
front_y = zeros(nP, nT, 20000);
front_z = zeros(nP, nT, 20000);
Length  = zeros(nP, nT); % Stores actual row count for each CSV

% Column offsets (1-indexed, readmatrix auto-skips headers)
t_col        = 2;  % Time is in column B
rear_offset  = 7;  % Rear marker starts at column G  (x=G, y=H, z=I)
front_offset = 23; % Front marker starts at column W (x=W, y=X, z=Y)

for i = 1:nP
    for j = 1:nT
        % Build filename from phase and trial strings
        string   = "Jun12_Rigid_" + Phase_String(i) + "_" + Trial_String(j);
        data_folder = "June12_Data";
        filename = fullfile(data_folder, string + ".csv");

        % Read CSV — readmatrix() automatically skips header rows!
        M = readmatrix(filename);

        % Get number of valid data rows and store it in Length vector
        [L, ~] = size(M);
        Length(i,j) = L;

        % Extract time
        t(i, j, 1:L) = M(:, t_col);

        % Extract rear marker positions
        rear_x(i, j, 1:L) = M(:, 0+rear_offset);  % Column G
        rear_y(i, j, 1:L) = M(:, 1+rear_offset);  % Column H
        rear_z(i, j, 1:L) = M(:, 2+rear_offset);  % Column I

        % Extract front marker positions
        front_x(i, j, 1:L) = M(:, 0+front_offset); % Column W
        front_y(i, j, 1:L) = M(:, 1+front_offset); % Column X
        front_z(i, j, 1:L) = M(:, 2+front_offset); % Column Y
    end
end
%% Compute Velocities

% Compute x-velocity by differencing consecutive position frames
rear_Vx  = zeros(nP, nT, 20000);
front_Vx = zeros(nP, nT, 20000);

for i = 1:nP
    for j = 1:nT
        for m = 1:Length(i,j)-1
            rear_Vx(i,j,m)  = rear_x(i,j,m+1)  - rear_x(i,j,m);
            front_Vx(i,j,m) = front_x(i,j,m+1) - front_x(i,j,m);
        end
    end
end

% Multiply by frame rate to convert from position-change-per-frame to m/s
rear_Vx  = rear_Vx  * 120; % 120Hz frame rate
front_Vx = front_Vx * 120;

%% Start-Detection using sliding window
window = 100; % number of frames to average over (tune this)
threshold = 0.05;  % mean velocity threshold in m/s (tune this)

start = ones(nP, nT); % default to frame 1

for i = 1:nP
    for j = 1:nT
        for m = 10:Length(i,j)-window % start 10 frames in to avoid No T3 spike
            window_mean = mean(rear_Vx(i,j,m:m+window));
            if abs(window_mean) > threshold
                start(i,j) = m;
                break
            end
        end
    end
end
%% Plot Start-Detection results (run for checking)
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        plot(reshape(rear_Vx(i,j,1:Length(i,j)), [1 Length(i,j)]))
        xline(start(i,j), 'r', 'LineWidth', 2) % red line at detected start
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('Vx (m/s)')
        xlabel('frame')

        % Zoom to window around detected start so you can actually see it
        xlim([10, Length(i,j)])
        ylim([-0.5 0.5]) % y bounds
    end
end

sgtitle("Start-Detection")

%% Check how many frames are available after start in each trial (for data trimming)
% (Run only to check)
%{
for i = 1:nP
    for j = 1:nT
        frames_after_start = Length(i,j) - start(i,j);
        disp(Phase_String(i) + " " + Trial_String(j) + ": " + frames_after_start + " frames after start")
    end
end
% Shortest trial is 2 T3... 2298 frames...
%}
%% Clip data

clip_length = 2000; % frames after start to keep

% Preallocate clipped arrays
rear_x_clipped  = zeros(nP, nT, clip_length);
rear_y_clipped  = zeros(nP, nT, clip_length);
rear_z_clipped  = zeros(nP, nT, clip_length);
front_x_clipped = zeros(nP, nT, clip_length);
front_y_clipped = zeros(nP, nT, clip_length);
front_z_clipped = zeros(nP, nT, clip_length);

t_clipped = zeros(nP, nT, clip_length);

rear_Vx_clipped  = zeros(nP, nT, clip_length);
front_Vx_clipped = zeros(nP, nT, clip_length);

for i = 1:nP
    for j = 1:nT
        s = start(i,j);

        % Clip to fixed window starting at detected start frame
        rear_x_clipped(i,j,:)  = rear_x(i,j,  s:s+clip_length-1);
        rear_y_clipped(i,j,:)  = rear_y(i,j,  s:s+clip_length-1);
        rear_z_clipped(i,j,:)  = rear_z(i,j,  s:s+clip_length-1);
        front_x_clipped(i,j,:) = front_x(i,j, s:s+clip_length-1);
        front_y_clipped(i,j,:) = front_y(i,j, s:s+clip_length-1);
        front_z_clipped(i,j,:) = front_z(i,j, s:s+clip_length-1);

        t_clipped(i,j,:) = t(i,j,s:s+clip_length-1);

        rear_Vx_clipped(i,j,:)  = rear_Vx(i,j,  s:s+clip_length-1);
        front_Vx_clipped(i,j,:) = front_Vx(i,j, s:s+clip_length-1);

        % Zero-reference to the start position
        rear_x_clipped(i,j,:)  = rear_x_clipped(i,j,:)  - rear_x_clipped(i,j,1);
        rear_y_clipped(i,j,:)  = rear_y_clipped(i,j,:)  - rear_y_clipped(i,j,1);
        rear_z_clipped(i,j,:)  = rear_z_clipped(i,j,:)  - rear_z_clipped(i,j,1);
        front_x_clipped(i,j,:) = front_x_clipped(i,j,:) - front_x_clipped(i,j,1);
        front_y_clipped(i,j,:) = front_y_clipped(i,j,:) - front_y_clipped(i,j,1);
        front_z_clipped(i,j,:) = front_z_clipped(i,j,:) - front_z_clipped(i,j,1);

        t_clipped(i,j,:) = t_clipped(i,j,:) - t_clipped(i,j,1);

    end
end

%% Calculate total distance from clipped, zero-referenced data
rear_dist  = zeros(nP, nT, clip_length);
front_dist = zeros(nP, nT, clip_length);

for i = 1:nP
    for j = 1:nT
        rear_dist(i,j,:)  = sqrt(rear_x_clipped(i,j,:).^2  + rear_y_clipped(i,j,:).^2  + rear_z_clipped(i,j,:).^2);
        front_dist(i,j,:) = sqrt(front_x_clipped(i,j,:).^2 + front_y_clipped(i,j,:).^2 + front_z_clipped(i,j,:).^2);
    end
end

%% Plot rear displacement (x, y, z) and total distance
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        hold on
        tt = squeeze(t_clipped(i,j,:));
        plot(tt, squeeze(rear_x_clipped(i,j,:)), 'r', 'DisplayName', 'X')
        plot(tt, squeeze(rear_y_clipped(i,j,:)), 'g', 'DisplayName', 'Y')
        plot(tt, squeeze(rear_z_clipped(i,j,:)), 'b', 'DisplayName', 'Z')
        plot(tt, squeeze(rear_dist(i,j,:)), 'k', 'DisplayName', 'total dist')
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('distance / m')
        xlabel('time / s')
        hold off
    end
end

sgtitle("Rear Displacement vs. Time")

lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
lgd.Units = 'normalized';
lgd.Position = [0.4, 0.04, 0.2, 0.02]; % centered at bottom [left, bottom, width, height]

%% Plot front displacement (x, y, z) and total distance
figure
for i = 1:nP
    for j = 1:nT
        subplot(nP, nT, (i-1)*nT + j)
        hold on
        tt = squeeze(t_clipped(i,j,:));
        plot(tt, squeeze(front_x_clipped(i,j,:)), 'r', 'DisplayName', 'X')
        plot(tt, squeeze(front_y_clipped(i,j,:)), 'g', 'DisplayName', 'Y')
        plot(tt, squeeze(front_z_clipped(i,j,:)), 'b', 'DisplayName', 'Z')
        plot(tt, squeeze(front_dist(i,j,:)), 'k', 'DisplayName', 'total dist')
        title(Phase_String(i) + " " + Trial_String(j))
        ylabel('distance / m')
        xlabel('time / s')
        hold off
    end
end

sgtitle("Front Displacement vs. Time")

lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
lgd.Units = 'normalized';
lgd.Position = [0.4, 0.04, 0.2, 0.02]; % centered at bottom [left, bottom, width, height]
%% Step length calculation

smooth_window = 30;
avg_step_rear = zeros(nP, nT);
avg_step_front = zeros(nP, nT);
avg_step = zeros(nP, nT); % average of rear and front

for i = 1:nP
    for j = 1:nT
        tt = squeeze(t_clipped(i,j,:));

        % Smooth and differentiate distance signals
        rear_dist_smooth = movmean(squeeze(rear_dist(i,j,:)),  smooth_window);
        front_dist_smooth = movmean(squeeze(front_dist(i,j,:)), smooth_window);

        rear_deriv  = diff(rear_dist_smooth) * 120; % m/frame * 120 frames/s ... diff always returns a vector of one element less
        front_deriv = diff(front_dist_smooth) * 120;

        % Find peaks in derivative
        [~, rear_locs] = findpeaks(rear_deriv,  'MinPeakDistance', 80, 'MinPeakProminence', 0.008);
        [~, front_locs] = findpeaks(front_deriv, 'MinPeakDistance', 80, 'MinPeakProminence', 0.008);

        % Step length = distance between consecutive peaks
        rear_steps = diff(rear_dist_smooth(rear_locs));
        front_steps = diff(front_dist_smooth(front_locs));

        avg_step_rear(i,j) = mean(rear_steps);
        avg_step_front(i,j) = mean(front_steps);
        avg_step(i,j) = (avg_step_rear(i,j) + avg_step_front(i,j)) / 2;
    end
end

% Display results per trial on command window
disp('Average step length per trial:')
for i = 1:nP
    for j = 1:nT
        disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(avg_step(i,j)*100, 2) + " cm")
    end
end

%% Average step length across trials and plot bar graph
avg_step_per_phase = mean(avg_step, 2); % mean across trials (dim 2)
std_step_per_phase = std(avg_step, 0, 2); % std across trials

figure
bar(avg_step_per_phase * 100) % convert to cm
hold on
errorbar(1:nP, avg_step_per_phase * 100, std_step_per_phase * 100, 'k.') % error bars in black

xticklabels(Phase_String)
xlabel('Phase')
ylabel('Step Length (cm)')
title('Average Step Length by Phase')
