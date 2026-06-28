%% Quaternion Angle Analysis — runs all datasets in one shot (no config editing needed)
clear
clc

% DATASET DEFINITIONS — add/edit entries here, the loop below runs all of them
% ------------------------------------------------------------------------------- %
datasets = struct([]);

datasets(1).name          = "June12";
datasets(1).data_folder   = "June12_Data";
datasets(1).Phase_String  = ["Con" "Des" "0" "2" "No"];
datasets(1).Trial_String  = ["T1" "T2" "T3"];
datasets(1).filename_fun  = @(phase, trial) "Jun12_Rigid_" + phase + "_" + trial;
datasets(1).rear_offset   = 7;  % rear position column (quaternion sits 4 columns before this)
datasets(1).front_offset  = 23; % front position column (quaternion sits 4 columns before this)
datasets(1).threshold     = repmat(0.05, 1, length(datasets(1).Phase_String)); % one per phase

datasets(2).name          = "Pierre";
datasets(2).data_folder   = "Apr17_Data";
datasets(2).Phase_String  = ["Con" "Des" "No"];
datasets(2).Trial_String  = ["1" "2" "3"];
datasets(2).filename_fun  = @(phase, trial) "Apr17_Sand_" + phase + "_" + trial;
datasets(2).rear_offset   = 23; % legacyRear position column
datasets(2).front_offset  = 7;  % legacyFront position column
datasets(2).threshold     = repmat(0.05, 1, length(datasets(2).Phase_String));

datasets(3).name          = "Sahil";
datasets(3).data_folder   = "July25_Data";
datasets(3).Phase_String  = ["Con" "Des" "wo"];
datasets(3).Trial_String  = ["1" "2" "3"];
datasets(3).filename_fun  = @(phase, trial) "July25_" + phase + "_T" + trial;
datasets(3).rear_offset   = 23; % legacyRear position column
datasets(3).front_offset  = 7;  % legacyFront position column
datasets(3).threshold     = repmat(0.05, 1, length(datasets(3).Phase_String));
% ------------------------------------------------------------------------------- %

for d = 1:length(datasets)
    disp("=== Running quaternion angle analysis: " + datasets(d).name + " ===")
    run_quaternion_analysis(datasets(d));
end

%% Local function — Do full analysis (from parsing to save) for one dataset
function run_quaternion_analysis(cfg)

    Phase_String = cfg.Phase_String;
    Trial_String = cfg.Trial_String;
    threshold    = cfg.threshold;
    data_folder  = cfg.data_folder;
    filename_fun = cfg.filename_fun;
    results_mat  = "QuaternionAngleAnalysis_" + cfg.name + "_results.mat";

    nP = length(Phase_String);
    nT = length(Trial_String);

    % Column indices in the CSV — quaternion (X,Y,Z,W) sits 4 columns
    % immediately before the position block for each rigid body
    col.time      = 2; % Time is in column B
    col.rearQuat  = (cfg.rear_offset-4)  : (cfg.rear_offset-1);
    col.rearX     = cfg.rear_offset + 2; % rear x position (offset+2, matching the +2 pattern used elsewhere)
    col.frontQuat = (cfg.front_offset-4) : (cfg.front_offset-1);

    % Preallocate vectors (phase x trial x datapoints)
    maxPts = 25000;

    t      = zeros(nP, nT, maxPts);
    Length = zeros(nP, nT);
    file_loaded = false(nP, nT);

    rear_quat  = zeros(nP, nT, maxPts, 4);
    front_quat = zeros(nP, nT, maxPts, 4);
    rear_x     = zeros(nP, nT, maxPts);

    %% Read CSV files
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
    % Rotation of local x-axis [1;0;0] by quaternion q = [qx qy qz qw]:
    %   Rx_x = 1 - 2*(qy^2 + qz^2)
    %   Rx_z = 2*(qx*qz - qy*qw)
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
    % Rotation of local y-axis [0;1;0] by quaternion q = [qx qy qz qw]:
    %   Ry_y = 1 - 2*(qx^2 + qz^2)
    %   Ry_z = 2*(qy*qz + qx*qw)
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

    %% Angle between the two rigid bodies — yaw (XZ plane)
    angle_between_yaw = heading_front - heading_rear;
    angle_between_yaw = wrapToPi(angle_between_yaw);
    angle_between_yaw_deg = rad2deg(angle_between_yaw);

    %% Angle between the two rigid bodies — roll (YZ plane)
    angle_between_roll = roll_front - roll_rear;
    angle_between_roll = wrapToPi(angle_between_roll);
    angle_between_roll_deg = rad2deg(angle_between_roll);

    %% Plot raw yaw angle over time
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
            plot(squeeze(t(i,j,1:L)), squeeze(angle_between_yaw_deg(i,j,1:L)), 'k')
            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time / s')
            ylabel('Theta yaw (deg)')
            ylim([-50, 50])
            grid on
        end
    end

    sgtitle(cfg.name + " — Theta Yaw (deg)")

    %% Plot raw roll angle over time
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
            plot(squeeze(t(i,j,1:L)), squeeze(angle_between_roll_deg(i,j,1:L)), 'k')
            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time / s')
            ylabel('Theta roll (deg)')
            ylim([-50, 50])
            grid on
        end
    end

    sgtitle(cfg.name + " — Theta Roll (deg)")

    %% Smooth angle data (yaw and roll)
    smooth_window_angle = 30;
    yaw_smooth  = zeros(nP, nT, maxPts);
    roll_smooth = zeros(nP, nT, maxPts);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end
            L = Length(i,j);
            yaw_smooth(i,j,1:L)  = movmean(squeeze(angle_between_yaw_deg(i,j,1:L)),  smooth_window_angle);
            roll_smooth(i,j,1:L) = movmean(squeeze(angle_between_roll_deg(i,j,1:L)), smooth_window_angle);
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

    window = 100;
    % threshold is set in the DATASET DEFINITIONS block above — one value per phase
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

    % Manual overrides go here if start-detection misfires on a specific trial
    % e.g. start(2,2) = 3*120;

    disp(start)

    %% Plot start detection using yaw angular displacement (run for checking)
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
            ang = squeeze(yaw_smooth(i,j,1:L));

            plot(tt, ang, 'k')
            hold on
            xline(tt(start(i,j)), 'r', 'LineWidth', 1.5)
            hold off

            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time / s')
            ylabel('Theta yaw (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Start Detection Using Yaw Angular Displacement")

    %% Zero-reference yaw and roll to start frame
    yaw_zeroed  = zeros(nP, nT, maxPts);
    roll_zeroed = zeros(nP, nT, maxPts);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end
            s = start(i,j);
            L = Length(i,j);
            yaw_zeroed(i,j,1:L)  = squeeze(yaw_smooth(i,j,1:L))  - yaw_smooth(i,j,s);
            roll_zeroed(i,j,1:L) = squeeze(roll_smooth(i,j,1:L)) - roll_smooth(i,j,s);
        end
    end

    %% Maximum angular displacement from neutral (after start) — yaw and roll
    max_yaw_disp  = NaN(nP, nT);
    max_roll_disp = NaN(nP, nT);

    for i = 1:nP
        for j = 1:nT
            if ~file_loaded(i,j), continue, end
            s = start(i,j);
            L = Length(i,j);
            yaw_segment  = squeeze(yaw_zeroed(i,j,s:L));
            roll_segment = squeeze(roll_zeroed(i,j,s:L));
            max_yaw_disp(i,j)  = max(abs(yaw_segment));
            max_roll_disp(i,j) = max(abs(roll_segment));
        end
    end

    %% Plot smoothed + trimmed yaw data
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

            tt_trim  = squeeze(t(i,j,s:L));
            ang_trim = squeeze(yaw_zeroed(i,j,s:L));

            tt_trim = tt_trim - tt_trim(1);   % start time at 0

            plot(tt_trim, ang_trim, 'k')
            title(Phase_String(i) + " " + Trial_String(j))
            xlabel('time from start / s')
            ylabel('Theta yaw (deg)')
            grid on
        end
    end

    sgtitle(cfg.name + " — Smoothed and Trimmed Yaw Data")

    %% Display results — yaw
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

    %% Display results — roll
    disp(' ')
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

    %% Average theta — yaw and roll, per phase (pooled across trials)
    disp(' ')
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

    disp(' ')
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

    %% Save results for use in the penetration depth / step length combiner script
    save(results_mat, "Phase_String", "avg_yaw_theta", "se_yaw_theta", "avg_roll_theta", "se_roll_theta")
    disp(' ')
    disp("Saved quaternion angle results to " + results_mat)
    disp(' ')
end