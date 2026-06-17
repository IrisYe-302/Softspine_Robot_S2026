%% Parse Data from CSVs
clear
clc

% Experimental conditions
Phase_String = ["Con" "Des" "0" "2" "No"]; % 5 phases
Trial_String = ["T1" "T2" "T3"];            % 3 trials per phase
Terrain_String = ["Rigid" "Granular"];         % 2 terrains
nP = length(Phase_String);
nT = length(Trial_String);
nTerrain = length(Terrain_String);

% Data folders and filename prefixes per terrain
data_folders  = ["June12_Data" "June16_Data"];
file_prefixes = ["Jun12" "Jun16"];

% Preallocate vectors (terrain x phase x trial x datapoints)
t = zeros(nTerrain, nP, nT, 20000);
rear_x = zeros(nTerrain, nP, nT, 20000);
rear_y = zeros(nTerrain, nP, nT, 20000);
rear_z = zeros(nTerrain, nP, nT, 20000);
front_x = zeros(nTerrain, nP, nT, 20000);
front_y = zeros(nTerrain, nP, nT, 20000);
front_z = zeros(nTerrain, nP, nT, 20000);
Length = zeros(nTerrain, nP, nT);

% Track which files were successfully loaded
file_loaded = false(nTerrain, nP, nT);

% Column offsets (1-indexed, readmatrix auto-skips headers)
t_col = 2;  % Time is in column B
rear_offset = 7;  % Rear marker starts at column G  (x=G, y=H, z=I)
front_offset = 23; % Front marker starts at column W (x=W, y=X, z=Y)

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            % Build filename
            string   = file_prefixes(k) + "_" + Terrain_String(k) + "_" + Phase_String(i) + "_" + Trial_String(j);
            filename = fullfile(data_folders(k), string + ".csv");

            % Skip if file doesn't exist
            if ~isfile(filename)
                disp("Skipping missing file: " + filename)
                continue
            end

            % Read CSV — readmatrix() automatically skips header rows
            M = readmatrix(filename);
            [L, ~] = size(M);
            Length(k,i,j) = L;
            file_loaded(k,i,j) = true;

            % Extract time
            t(k, i, j, 1:L) = M(:, t_col);

            % Extract rear marker positions
            rear_x(k, i, j, 1:L) = M(:, 0+rear_offset);  % Column G
            rear_y(k, i, j, 1:L) = M(:, 1+rear_offset);  % Column H
            rear_z(k, i, j, 1:L) = M(:, 2+rear_offset);  % Column I

            % Extract front marker positions
            front_x(k, i, j, 1:L) = M(:, 0+front_offset); % Column W
            front_y(k, i, j, 1:L) = M(:, 1+front_offset); % Column X
            front_z(k, i, j, 1:L) = M(:, 2+front_offset); % Column Y
        end
    end
end

%% Compute Velocities
rear_Vx = zeros(nTerrain, nP, nT, 20000);
front_Vx = zeros(nTerrain, nP, nT, 20000);

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end
            for m = 1:Length(k,i,j)-1
                rear_Vx(k,i,j,m)  = rear_x(k,i,j,m+1)  - rear_x(k,i,j,m);
                front_Vx(k,i,j,m) = front_x(k,i,j,m+1) - front_x(k,i,j,m);
            end
        end
    end
end

% Multiply by frame rate to convert from position-change-per-frame to m/s
rear_Vx  = rear_Vx  * 120; % 120Hz frame rate
front_Vx = front_Vx * 120;

%% Start-Detection using sliding window
window  = 100;  % number of frames to average over (tune this)
threshold = [0.05 0.03]; % mean velocity threshold in m/s (less noise for sand)

start = ones(nTerrain, nP, nT);

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end
            for m = 10:Length(k,i,j)-window % start 10 frames in to avoid spike
                window_mean = mean(rear_Vx(k,i,j,m:m+window));
                if abs(window_mean) > threshold(k)
                    start(k,i,j) = m;
                    break
                end
            end
        end
    end
end

start(2,4,3) = 750; % Manual override
disp(start(2,4,3))

%% Plot Start-Detection results (run for checking)
for k = 1:nTerrain
    figure
    for i = 1:nP
        for j = 1:nT
            subplot(nP, nT, (i-1)*nT + j)
            if ~file_loaded(k,i,j)
                title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
                continue
            end
            plot(reshape(rear_Vx(k,i,j,1:Length(k,i,j)), [1 Length(k,i,j)]))
            xline(start(k,i,j), 'r', 'LineWidth', 2) % red line at detected start
            title(Phase_String(i) + " " + Trial_String(j))
            ylabel('Vx (m/s)')
            xlabel('frame')

            % Zoom to window around detected start so you can actually see it
            xlim([10, Length(k,i,j)])
            ylim([-0.5 0.5]) % y bounds
        end
    end
    sgtitle(Terrain_String(k) + " Start-Detection")
end

%% Check how many frames are available after start in each trial (for data trimming)
% (Run only to check)

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end
            frames_after_start = Length(k,i,j) - start(k,i,j);
            disp(Terrain_String(k) + " " + Phase_String(i) + " " + Trial_String(j) + ": " + frames_after_start + " frames after start")
        end
    end
end
% Shortest trial is granular 2 T1... 1638 frames...
%}

%% Clip data
clip_length = 1600; % frames after start to keep

% Preallocate clipped arrays
rear_x_clipped = zeros(nTerrain, nP, nT, clip_length);
rear_y_clipped = zeros(nTerrain, nP, nT, clip_length);
rear_z_clipped = zeros(nTerrain, nP, nT, clip_length);
front_x_clipped = zeros(nTerrain, nP, nT, clip_length);
front_y_clipped = zeros(nTerrain, nP, nT, clip_length);
front_z_clipped = zeros(nTerrain, nP, nT, clip_length);
t_clipped = zeros(nTerrain, nP, nT, clip_length);
rear_Vx_clipped = zeros(nTerrain, nP, nT, clip_length);
front_Vx_clipped = zeros(nTerrain, nP, nT, clip_length);

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end
            s = start(k,i,j);

            % Clip to fixed window starting at detected start frame
            rear_x_clipped(k,i,j,:) = rear_x(k,i,j, s:s+clip_length-1);
            rear_y_clipped(k,i,j,:) = rear_y(k,i,j, s:s+clip_length-1);
            rear_z_clipped(k,i,j,:) = rear_z(k,i,j, s:s+clip_length-1);
            front_x_clipped(k,i,j,:) = front_x(k,i,j, s:s+clip_length-1);
            front_y_clipped(k,i,j,:) = front_y(k,i,j, s:s+clip_length-1);
            front_z_clipped(k,i,j,:) = front_z(k,i,j, s:s+clip_length-1);
            t_clipped(k,i,j,:) = t(k,i,j, s:s+clip_length-1);
            rear_Vx_clipped(k,i,j,:) = rear_Vx(k,i,j, s:s+clip_length-1);
            front_Vx_clipped(k,i,j,:) = front_Vx(k,i,j, s:s+clip_length-1);

            % Zero-reference to the start position
            rear_x_clipped(k,i,j,:) = rear_x_clipped(k,i,j,:) - rear_x_clipped(k,i,j,1);
            rear_y_clipped(k,i,j,:) = rear_y_clipped(k,i,j,:) - rear_y_clipped(k,i,j,1);
            rear_z_clipped(k,i,j,:) = rear_z_clipped(k,i,j,:) - rear_z_clipped(k,i,j,1);
            front_x_clipped(k,i,j,:) = front_x_clipped(k,i,j,:) - front_x_clipped(k,i,j,1);
            front_y_clipped(k,i,j,:) = front_y_clipped(k,i,j,:) - front_y_clipped(k,i,j,1);
            front_z_clipped(k,i,j,:) = front_z_clipped(k,i,j,:) - front_z_clipped(k,i,j,1);
            t_clipped(k,i,j,:) = t_clipped(k,i,j,:) - t_clipped(k,i,j,1);
        end
    end
end

%% Calculate total 3D distance
rear_dist = zeros(nTerrain, nP, nT, clip_length);
front_dist = zeros(nTerrain, nP, nT, clip_length);

for k = 1:nTerrain
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end
            rear_dist(k,i,j,:) = sqrt(rear_x_clipped(k,i,j,:).^2 + rear_y_clipped(k,i,j,:).^2 + rear_z_clipped(k,i,j,:).^2);
            front_dist(k,i,j,:) = sqrt(front_x_clipped(k,i,j,:).^2 + front_y_clipped(k,i,j,:).^2 + front_z_clipped(k,i,j,:).^2);
        end
    end
end

%% Plot rear displacement (x, y, z) and total distance
for k = 1:nTerrain
    figure
    for i = 1:nP
        for j = 1:nT
            subplot(nP, nT, (i-1)*nT + j)
            if ~file_loaded(k,i,j)
                title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
                continue
            end
            hold on
            tt = squeeze(t_clipped(k,i,j,:));
            plot(tt, squeeze(rear_x_clipped(k,i,j,:)), 'r')
            plot(tt, squeeze(rear_y_clipped(k,i,j,:)), 'g')
            plot(tt, squeeze(rear_z_clipped(k,i,j,:)), 'b')
            plot(tt, squeeze(rear_dist(k,i,j,:)), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5)
            title(Phase_String(i) + " " + Trial_String(j))
            ylabel('distance / m')
            xlabel('time / s')
            hold off
        end
    end
    sgtitle(Terrain_String(k) + " Rear Displacement vs. Time")
    lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
    lgd.Units = 'normalized';
    lgd.Position = [0.4, 0.04, 0.2, 0.02];
end

%% Plot front displacement (x, y, z) and total distance
for k = 1:nTerrain
    figure
    for i = 1:nP
        for j = 1:nT
            subplot(nP, nT, (i-1)*nT + j)
            if ~file_loaded(k,i,j)
                title(Phase_String(i) + " " + Trial_String(j) + " (missing)")
                continue
            end
            hold on
            tt = squeeze(t_clipped(k,i,j,:));
            plot(tt, squeeze(front_x_clipped(k,i,j,:)), 'r')
            plot(tt, squeeze(front_y_clipped(k,i,j,:)), 'g')
            plot(tt, squeeze(front_z_clipped(k,i,j,:)), 'b')
            plot(tt, squeeze(front_dist(k,i,j,:)), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5)
            title(Phase_String(i) + " " + Trial_String(j))
            ylabel('distance / m')
            xlabel('time / s')
            hold off
        end
    end
    sgtitle(Terrain_String(k) + " Front Displacement vs. Time")
    lgd = legend('X', 'Y', 'Z', 'total dist', 'Orientation', 'horizontal');
    lgd.Units = 'normalized';
    lgd.Position = [0.4, 0.04, 0.2, 0.02];
end

%% Step length calculation
smooth_window = 30;
all_steps_rear = cell(nTerrain, nP); % collect all individual steps across trials
all_steps_front = cell(nTerrain, nP);
all_steps  = cell(nTerrain, nP);
avg_step = zeros(nTerrain, nP, nT); % per-trial average for display

for k = 1:nTerrain
    for i = 1:nP
        all_steps_rear{k,i}  = [];
        all_steps_front{k,i} = [];
        all_steps{k,i} = [];
        for j = 1:nT
            if ~file_loaded(k,i,j), continue, end

            rear_dist_smooth = movmean(squeeze(rear_dist(k,i,j,:)), smooth_window);
            front_dist_smooth = movmean(squeeze(front_dist(k,i,j,:)), smooth_window);
            rear_deriv  = diff(rear_dist_smooth)  * 120;
            front_deriv = diff(front_dist_smooth) * 120;

            % Find peaks in derivative
            [~, rear_locs] = findpeaks(rear_deriv, 'MinPeakDistance', 80, 'MinPeakProminence', 0.008);
            [~, front_locs] = findpeaks(front_deriv, 'MinPeakDistance', 80, 'MinPeakProminence', 0.008);

            % Individual step lengths for this trial
            rear_steps  = diff(rear_dist_smooth(rear_locs));
            front_steps = diff(front_dist_smooth(front_locs));

            % Append raw steps to pool for this phase
            all_steps_rear{k,i} = [all_steps_rear{k,i},  rear_steps'];
            all_steps_front{k,i} = [all_steps_front{k,i}, front_steps'];
            all_steps{k,i} = [all_steps{k,i}, rear_steps', front_steps'];

            % Store per-trial average for display
            avg_step(k,i,j) = (mean(rear_steps) + mean(front_steps)) / 2;
        end
    end
end

% Display results per trial
disp('Average step length per trial:')
for k = 1:nTerrain
    disp("--- " + Terrain_String(k) + " ---")
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(k,i,j)
                disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
                continue
            end
            disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(avg_step(k,i,j)*100, 2) + " cm")
        end
    end
end

%% Bar graph of average step length per phase (one figure per terrain)
for k = 1:nTerrain
    avg_step_per_phase = zeros(nP, 1);
    std_step_per_phase = zeros(nP, 1);

    for i = 1:nP
        % Mean and std computed from full pool of individual steps
        avg_step_per_phase(i) = mean(all_steps{k,i});
        std_step_per_phase(i) = std(all_steps{k,i});
    end

    figure
    bar(avg_step_per_phase * 100)
    hold on
    errorbar(1:nP, avg_step_per_phase * 100, std_step_per_phase * 100, 'k.', 'LineWidth', 1.5) %errorbar( x_coord, center, upper-lower-extension,...)
    xticklabels(Phase_String)
    xlabel('Phase')
    ylabel('Step Length (cm)')
    title(Terrain_String(k) + " Average Step Length by Phase")
end