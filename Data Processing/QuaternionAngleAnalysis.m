%% Parse Quaternion Data from CSVs (June12 — Con and Des only)
clear
clc

Phase_String = ["Con" "Des"];   % only the two phases needed
Trial_String = ["T1" "T2" "T3"];
nP = length(Phase_String);
nT = length(Trial_String);

data_folder = "June12_Data";

% Column indices in the CSV
col.time      = 2;      % Time is in column B
col.rearQuat  = 3:6;    % rear quaternion starts at column C (X,Y,Z,W)
col.rearX     = 9;      % rear x position, column I
col.frontQuat = 19:22;  % front quaternion starts at column S (X,Y,Z,W)

% Preallocate vectors (phase x trial x datapoints)
maxPts = 20000;

t      = zeros(nP, nT, maxPts);
Length = zeros(nP, nT);
file_loaded = false(nP, nT);

rear_quat  = zeros(nP, nT, maxPts, 4);
front_quat = zeros(nP, nT, maxPts, 4);
rear_x     = zeros(nP, nT, maxPts);

%% Read CSV files once
for i = 1:nP
    for j = 1:nT
        filename = fullfile(data_folder, "Jun12_Rigid_" + Phase_String(i) + "_" + Trial_String(j) + ".csv");

        if ~isfile(filename)
            disp("Skipping missing file: " + filename)
            continue
        end

        M = readmatrix(filename);
        [L, ~] = size(M);

        Length(i,j) = L;
        file_loaded(i,j) = true;

        t(i,j,1:L) = M(:, col.time);
        rear_quat(i,j,1:L,:)  = M(:, col.rearQuat);
        front_quat(i,j,1:L,:) = M(:, col.frontQuat);
        rear_x(i,j,1:L) = M(:, col.rearX);
    end
end

%% Compute heading angle from quaternion (X-axis projected onto XZ plane)
heading_rear  = zeros(nP, nT, maxPts);
heading_front = zeros(nP, nT, maxPts);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end

        qx = squeeze(rear_quat(i,j,:,1));
        qy = squeeze(rear_quat(i,j,:,2));
        qz = squeeze(rear_quat(i,j,:,3));
        qw = squeeze(rear_quat(i,j,:,4));
        Rx_x = 1 - 2*(qy.^2 + qz.^2);
        Rx_z = 2*(qx.*qz - qy.*qw);
        heading_rear(i,j,:) = atan2(Rx_z, Rx_x);

        qx = squeeze(front_quat(i,j,:,1));
        qy = squeeze(front_quat(i,j,:,2));
        qz = squeeze(front_quat(i,j,:,3));
        qw = squeeze(front_quat(i,j,:,4));
        Rx_x = 1 - 2*(qy.^2 + qz.^2);
        Rx_z = 2*(qx.*qz - qy.*qw);
        heading_front(i,j,:) = atan2(Rx_z, Rx_x);
    end
end

%% Angle between the two rigid bodies
angle_between = heading_front - heading_rear;
angle_between = wrapToPi(angle_between);
angle_between_deg = rad2deg(angle_between);

%% Plot raw angle over time
figure
tiledlayout(nP, nT, 'TileSpacing','compact', 'Padding','compact')

for i = 1:nP
    for j = 1:nT
        nexttile((i-1)*nT + j)

        if ~file_loaded(i,j)
            title(Trial_String(j) + " (missing)")
            axis off
            continue
        end

        L = Length(i,j);
        plot(squeeze(t(i,j,1:L)), squeeze(angle_between_deg(i,j,1:L)), 'k')
        title(Phase_String(i) + " " + Trial_String(j))
        xlabel('time / s')
        ylabel('Theta (deg)')
        ylim([-50, 50])
        grid on
    end
end

sgtitle('Theta (deg)')

%% Smooth angle data
smooth_window_angle = 30;
angle_smooth = zeros(nP, nT, maxPts);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        L = Length(i,j);
        angle_smooth(i,j,1:L) = movmean(squeeze(angle_between_deg(i,j,1:L)), smooth_window_angle);
    end
end

%% Start detection using rear velocity
rear_Vx = zeros(nP, nT, maxPts);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end

        L = Length(i,j);
        for m = 1:L-1
            rear_Vx(i,j,m) = rear_x(i,j,m+1) - rear_x(i,j,m);
        end
    end
end

rear_Vx = rear_Vx * 120; % 120 Hz frame rate

window    = 100;
threshold = [0.05, 0.04];   % one threshold per phase: Con, Des
start = ones(nP, nT);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end

        L = Length(i,j);
        for m = 10:max(10, L-window)
            if m + window > L
                break
            end

            window_mean = mean(rear_Vx(i,j,m:m+window));
            if abs(window_mean) > threshold(i)
                start(i,j) = m;
                break
            end
        end
    end
end
start(2,2) = 3*120;
start(2,3) = 13*120;

disp(start)

%% Plot start detection using angular displacement
figure
tiledlayout(nP, nT, 'TileSpacing','compact', 'Padding','compact')

for i = 1:nP
    for j = 1:nT
        nexttile((i-1)*nT + j)

        if ~file_loaded(i,j)
            title(Trial_String(j) + " (missing)")
            axis off
            continue
        end

        L = Length(i,j);
        tt = squeeze(t(i,j,1:L));
        ang = squeeze(angle_smooth(i,j,1:L));

        plot(tt, ang, 'k')
        hold on
        xline(tt(start(i,j)), 'r', 'LineWidth', 1.5)
        hold off

        title(Phase_String(i) + " " + Trial_String(j))
        xlabel('time / s')
        ylabel('Theta (deg)')
        grid on
    end
end

sgtitle('Start Detection Using Angular Displacement')

%% Zero-reference angle to start frame
angle_zeroed = zeros(nP, nT, maxPts);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        s = start(i,j);
        L = Length(i,j);
        angle_zeroed(i,j,1:L) = squeeze(angle_smooth(i,j,1:L)) - angle_smooth(i,j,s);
    end
end

%% Maximum angular displacement from neutral (after start)
max_angle_disp = NaN(nP, nT);

for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j), continue, end
        s = start(i,j);
        L = Length(i,j);
        segment = squeeze(angle_zeroed(i,j,s:L));
        max_angle_disp(i,j) = max(abs(segment));
    end
end

%% Plot smoothed + trimmed data
figure
tiledlayout(nP, nT, 'TileSpacing','compact', 'Padding','compact')

for i = 1:nP
    for j = 1:nT
        nexttile((i-1)*nT + j)

        if ~file_loaded(i,j)
            title(Trial_String(j) + " (missing)")
            axis off
            continue
        end

        L = Length(i,j);
        s = start(i,j);

        tt_trim = squeeze(t(i,j,s:L));
        ang_trim = squeeze(angle_zeroed(i,j,s:L));

        tt_trim = tt_trim - tt_trim(1);   % start time at 0

        plot(tt_trim, ang_trim, 'k')
        title(Phase_String(i) + " " + Trial_String(j))
        xlabel('time from start / s')
        ylabel('Theta (deg)')
        grid on
    end
end

sgtitle('Smoothed and Trimmed Data')

%% Display results
disp('Maximum angular displacement from neutral per trial (degrees):')
for i = 1:nP
    for j = 1:nT
        if ~file_loaded(i,j)
            disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
            continue
        end
        disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(max_angle_disp(i,j), 2) + " deg")
    end
end

%% Average theta
disp(' ')
disp("Average Theta")

for i = 1:nP
    values = max_angle_disp(i, file_loaded(i,:));

    mean_theta = mean(values);
    se_theta = std(values) / sqrt(length(values));

    disp(Phase_String(i) + ": " + round(mean_theta,2) + " ± " + round(se_theta,2) + " deg")
end