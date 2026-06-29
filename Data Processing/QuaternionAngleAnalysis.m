%% Quaternion Angle Analysis — hardcoded start/end only, no raw figures
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Datasets (will be loaded from .mat files) and manual overrides
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
datasets = struct([]);

datasets(1).name          = "June12";
datasets(1).data_folder   = "June12_Data";
datasets(1).Phase_String  = ["Con" "Des" "0" "2" "No"];
datasets(1).Trial_String  = ["T1" "T2" "T3"];
datasets(1).filename_fun  = @(phase, trial) "Jun12_Rigid_" + phase + "_" + trial;
datasets(1).rear_offset   = 7;   % rear position column (quaternion sits 4 columns before this)
datasets(1).front_offset  = 23;  % front position column (quaternion sits 4 columns before this)

% Hardcoded quaternion-start overrides
datasets(1).start_quat_override = ones(length(datasets(1).Phase_String), length(datasets(1).Trial_String));
datasets(1).start_quat_override = round([[14.8, 7.5, 0.6];[0.8, 3.8, 13.7];[5.8, 4.4, 3.8];[10, 7.6, 9.5];[2.1, 1, 2.5]] * 120);

% Hardcoded quaternion-end overrides (frame numbers)
datasets(1).end_quat_override = [];
datasets(1).end_quat_override(1,1) = 35*120; % Con T1
datasets(1).end_quat_override(1,2) = 32*120; % Con T2
datasets(1).end_quat_override(2,1) = 27*120; % Des T1
datasets(1).end_quat_override(2,2) = 32*120; % Des T2
datasets(1).end_quat_override(2,3) = 86*120; % Des T3

datasets(2).name          = "Pierre";
datasets(2).data_folder   = "Apr17_Data";
datasets(2).Phase_String  = ["Con" "Des" "No"];
datasets(2).Trial_String  = ["1" "2" "3"];
datasets(2).filename_fun  = @(phase, trial) "Apr17_Sand_" + phase + "_" + trial;
datasets(2).rear_offset   = 23;  % legacyRear position column
datasets(2).front_offset  = 7;   % legacyFront position column

% No start overrides needed here; defaults stay at frame 1
datasets(2).start_quat_override = ones(length(datasets(2).Phase_String), length(datasets(2).Trial_String));

datasets(2).start_quat_override = round([[12, 17.6, 19];[10.6, 17.8, 16.2];[11.9, 22, 16]] * 120);

datasets(2).end_quat_override = [];
datasets(2).end_quat_override = round([[26, 32, 33]; [26, 32, 31]; [26,38,31]] *120);  % Con 1

datasets(3).name          = "Sahil";
datasets(3).data_folder   = "July25_Data";
datasets(3).Phase_String  = ["Con" "Des" "wo"];
datasets(3).Trial_String  = ["1" "2" "3"];
datasets(3).filename_fun  = @(phase, trial) "July25_" + phase + "_T" + trial;
datasets(3).rear_offset   = 23;  % legacyRear position column
datasets(3).front_offset  = 7;   % legacyFront position column

datasets(3).start_quat_override = ones(length(datasets(3).Phase_String), length(datasets(3).Trial_String));

datasets(3).start_quat_override = round([[1, 1.9, 1.5];[3.5, 5.5, 1.5];[4, 3, 3]] * 120);

datasets(3).end_quat_override = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for d = 1:length(datasets)
    disp("=== Running quaternion angle analysis: " + datasets(d).name + " ===")
    run_quaternion_analysis(datasets(d));
end

%% Local function — does the full analysis (parsing through save) for one dataset
function run_quaternion_analysis(cfg)

    Phase_String = cfg.Phase_String;
    Trial_String = cfg.Trial_String;
    data_folder  = cfg.data_folder;
    filename_fun = cfg.filename_fun;
    results_mat   = "QuaternionAngleAnalysis_" + cfg.name + "_results.mat";

    nP = length(Phase_String);
    nT = length(Trial_String);

    % Column indices in the CSV — quaternion (X,Y,Z,W) sits 4 columns
    % immediately before the position block for each rigid body
    col.time      = 2; % Time is in column B
    col.rearQuat  = (cfg.rear_offset-4)  : (cfg.rear_offset-1);
    col.rearX     = cfg.rear_offset + 2;
    col.frontQuat = (cfg.front_offset-4) : (cfg.front_offset-1);

    % Preallocate vectors (phase x trial x datapoints)
    maxPts = 25000;

    t      = zeros(nP, nT, maxPts);
    Length = zeros(nP, nT);
    file_loaded = false(nP, nT);

    rear_quat  = zeros(nP, nT, maxPts, 4);
    front_quat = zeros(nP, nT, maxPts, 4);
    rear_x     = zeros(nP, nT, maxPts);

    %% Read CSV files once
    for i = 1:nP
        for j = 1:nT
            filename = fullfile(data_folder, filename_fun(Phase_String(i), Trial_String(j)) + ".csv");

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

    %% Compute heading angle (yaw) from quaternion — X-axis projected onto XZ plane
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

    %% Compute roll angle from quaternion — Y-axis projected onto YZ plane
    roll_rear  = zeros(nP, nT, maxPts);
    roll_front = zeros(nP, nT, maxPts);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end

            qx = squeeze(rear_quat(i,j,:,1));
            qy = squeeze(rear_quat(i,j,:,2));
            qz = squeeze(rear_quat(i,j,:,3));
            qw = squeeze(rear_quat(i,j,:,4));
            Ry_y = 1 - 2*(qx.^2 + qz.^2);
            Ry_z = 2*(qy.*qz + qx.*qw);
            roll_rear(i,j,:) = atan2(Ry_z, Ry_y);

            qx = squeeze(front_quat(i,j,:,1));
            qy = squeeze(front_quat(i,j,:,2));
            qz = squeeze(front_quat(i,j,:,3));
            qw = squeeze(front_quat(i,j,:,4));
            Ry_y = 1 - 2*(qx.^2 + qz.^2);
            Ry_z = 2*(qy.*qz + qx.*qw);
            roll_front(i,j,:) = atan2(Ry_z, Ry_y);
        end
    end

    %% Angle between the two rigid bodies — yaw and roll
    angle_between_yaw = wrapToPi(heading_front - heading_rear);
    angle_between_yaw_deg = rad2deg(angle_between_yaw);

    angle_between_roll = wrapToPi(roll_front - roll_rear);
    angle_between_roll_deg = rad2deg(angle_between_roll);

    %% Smooth angle data (used for trimming)
    smooth_window_angle = 30;
    yaw_smooth  = zeros(nP, nT, maxPts);
    roll_smooth = zeros(nP, nT, maxPts);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end
            L = Length(i,j);
            yaw_smooth(i,j,1:L)  = movmean(squeeze(angle_between_yaw_deg(i,j,1:L)),  smooth_window_angle, 'omitnan');
            roll_smooth(i,j,1:L) = movmean(squeeze(angle_between_roll_deg(i,j,1:L)), smooth_window_angle, 'omitnan');
        end
    end

    %% Hardcoded start/end only
    start_quat = ones(nP, nT);
    end_quat   = Length;

    if isfield(cfg, 'start_quat_override') && ~isempty(cfg.start_quat_override)
        [r, c] = find(cfg.start_quat_override > 1);
        for k = 1:length(r)
            start_quat(r(k), c(k)) = cfg.start_quat_override(r(k), c(k));
        end
    end

    if isfield(cfg, 'end_quat_override') && ~isempty(cfg.end_quat_override)
        [r, c] = find(cfg.end_quat_override > 0);
        for k = 1:length(r)
            end_quat(r(k), c(k)) = cfg.end_quat_override(r(k), c(k));
        end
    end

    disp("Quaternion-specific start:")
    disp(start_quat)
    disp("Quaternion-specific end:")
    disp(end_quat)

    %% Plot start and end cutoffs — yaw
    %{
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
            s = max(1, min(start_quat(i,j), L));
            e = max(1, min(end_quat(i,j), L));

            plot(tt, squeeze(yaw_smooth(i,j,1:L)), 'k')
            hold on
            xline(tt(s), 'b', 'LineWidth', 1.5)
            xline(tt(e), 'r', 'LineWidth', 1.5)
            hold off

            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time / s')
            ylabel('Theta yaw (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Yaw: Start (blue) and End (red)")
    %}

    %% Plot start and end cutoffs — roll
    %{
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
            s = max(1, min(start_quat(i,j), L));
            e = max(1, min(end_quat(i,j), L));

            plot(tt, squeeze(roll_smooth(i,j,1:L)), 'k')
            hold on
            xline(tt(s), 'b', 'LineWidth', 1.5)
            xline(tt(e), 'r', 'LineWidth', 1.5)
            hold off

            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time / s')
            ylabel('Theta roll (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Roll: Start (blue) and End (red)")
    %}
    %% Zero-reference yaw and roll to the hardcoded start frame
    yaw_zeroed  = zeros(nP, nT, maxPts);
    roll_zeroed = zeros(nP, nT, maxPts);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end
            s = max(1, min(start_quat(i,j), Length(i,j)));
            L = Length(i,j);
            yaw_zeroed(i,j,1:L)  = squeeze(yaw_smooth(i,j,1:L))  - yaw_smooth(i,j,s);
            roll_zeroed(i,j,1:L) = squeeze(roll_smooth(i,j,1:L)) - roll_smooth(i,j,s);
        end
    end

    %% Maximum angular displacement from neutral — yaw and roll
    max_yaw_disp  = NaN(nP, nT);
    max_roll_disp = NaN(nP, nT);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end

            s = max(1, min(start_quat(i,j), Length(i,j)));
            e = max(1, min(end_quat(i,j), Length(i,j)));

            if s >= e
                disp("Warning: start (" + s + ") >= end (" + e + ") for " + ...
                    Phase_String(i) + " " + Trial_String(j) + " — skipping")
                continue
            end

            yaw_segment  = squeeze(yaw_zeroed(i,j,s:e));
            roll_segment = squeeze(roll_zeroed(i,j,s:e));

            % Ignore major outliers using robust percentiles.
            yaw_segment  = yaw_segment(isfinite(yaw_segment));
            roll_segment = roll_segment(isfinite(roll_segment));

            if isempty(yaw_segment) || isempty(roll_segment)
                continue
            end

            % Amplitude is half the peak-to-trough distance.
            yaw_low   = prctile(yaw_segment, 5);
            yaw_high  = prctile(yaw_segment, 95);
            roll_low  = prctile(roll_segment, 5);
            roll_high = prctile(roll_segment, 95);

            max_yaw_disp(i,j)  = (yaw_high  - yaw_low)  / 2;
            max_roll_disp(i,j) = (roll_high - roll_low) / 2;
        end
    end

    %% Plot trimmed yaw data
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

            s = max(1, min(start_quat(i,j), Length(i,j)));
            e = max(1, min(end_quat(i,j), Length(i,j)));

            tt_trim  = squeeze(t(i,j,s:e));
            ang_trim = squeeze(yaw_zeroed(i,j,s:e));
            tt_trim  = tt_trim - tt_trim(1);

            plot(tt_trim, ang_trim, 'k')
            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time from start / s')
            ylabel('Theta yaw (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Trimmed Yaw Data")

    %% Plot trimmed roll data
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

            s = max(1, min(start_quat(i,j), Length(i,j)));
            e = max(1, min(end_quat(i,j), Length(i,j)));

            tt_trim  = squeeze(t(i,j,s:e));
            ang_trim = squeeze(roll_zeroed(i,j,s:e));
            tt_trim  = tt_trim - tt_trim(1);

            plot(tt_trim, ang_trim, 'k')
            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time from start / s')
            ylabel('Theta roll (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Trimmed Roll Data")

    %% Display yaw
    disp(" ")
    disp('Maximum yaw angular displacement from neutral per trial (degrees):')
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j)
                disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
                continue
            end
            disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(max_yaw_disp(i,j), 2) + " deg")
        end
    end

    %% Display roll
    disp(" ")
    disp('Maximum roll angular displacement from neutral per trial (degrees):')
    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j)
                disp(Phase_String(i) + " " + Trial_String(j) + ": missing")
                continue
            end
            disp(Phase_String(i) + " " + Trial_String(j) + ": " + round(max_roll_disp(i,j), 2) + " deg")
        end
    end

    %% Average theta — yaw and roll, per phase
    disp(" ")
    disp("Average Theta Yaw")

    avg_yaw_theta = NaN(nP, 1);
    se_yaw_theta  = NaN(nP, 1);

    for i = 1:nP
        values = max_yaw_disp(i, file_loaded(i,:));
        if isempty(values), continue, end

        avg_yaw_theta(i) = mean(values);
        se_yaw_theta(i)  = std(values) / sqrt(length(values));

        disp(Phase_String(i) + ": " + round(avg_yaw_theta(i),2) + " ± " + round(se_yaw_theta(i),2) + " deg")
    end

    disp(" ")
    disp("Average Theta Roll")

    avg_roll_theta = NaN(nP, 1);
    se_roll_theta  = NaN(nP, 1);

    for i = 1:nP
        values = max_roll_disp(i, file_loaded(i,:));
        if isempty(values), continue, end

        avg_roll_theta(i) = mean(values);
        se_roll_theta(i)  = std(values) / sqrt(length(values));

        disp(Phase_String(i) + ": " + round(avg_roll_theta(i),2) + " ± " + round(se_roll_theta(i),2) + " deg")
    end

    %% Save results
    save(results_mat, "Phase_String", "avg_yaw_theta", "se_yaw_theta", "avg_roll_theta", "se_roll_theta", "start_quat", "end_quat")
    disp(" ")
    disp("Saved quaternion angle results to " + results_mat)
    disp(" ")
end
