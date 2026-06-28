%% Penetration Depth + Rotational Correction Combiner — runs all experiments in one shot
clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EXPERIMENT DEFINITIONS — add/edit entries here, the loop below runs all of them
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
experiments = struct([]);

experiments(1).name           = "Main (June12, Rigid/Granular)";
experiments(1).data_mat       = "DataProcessing_results.mat";
experiments(1).quat_mat       = "QuaternionAngleAnalysis_June12_results.mat";
experiments(1).has_terrain    = true; % DataProcessing.m's results are indexed by terrain too

experiments(2).name           = "Pierre (Apr17, Sand)";
experiments(2).data_mat       = "PierreDataProcessing_results.mat";
experiments(2).quat_mat       = "QuaternionAngleAnalysis_Pierre_results.mat";
experiments(2).has_terrain    = false;

experiments(3).name           = "Sahil (July25, Sand)";
experiments(3).data_mat       = "SahilDataProcessing_results.mat";
experiments(3).quat_mat       = "QuaternionAngleAnalysis_Sahil_results.mat";
experiments(3).has_terrain    = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constant — spine-to-leg length (fixed physical constant, not measured per trial)
s_l = 6.2; % cm

for e = 1:length(experiments)
    disp("=== " + experiments(e).name + " ===")
    run_combiner(experiments(e), s_l);
    disp(" ")
end

%% Local function — loads, computes a = 2*s_l*sin(theta/2), and displays actual vs. modeled (s+a)
function run_combiner(cfg, s_l)

    % Load step-length / penetration-depth results for this experiment
    data = load(cfg.data_mat);

    % Load yaw theta results from the matching quaternion analysis run
    quat = load(cfg.quat_mat);

    Phase_String = data.Phase_String; % the data-processing Phase_String is the authoritative one
    nP = length(Phase_String);

    %% Compute rotational correction "a" per phase, using yaw theta from quaternion analysis
    % a = 2*s_l*sin(theta/2) — lateral path-length contribution from the body
    % yawing while stepping. theta comes from QuaternionAngleAnalysis (max yaw
    % angular displacement from neutral, averaged across trials, per phase).
    a_phase = NaN(nP, 1);

    for i = 1:nP
        % Match this phase to the corresponding phase in the quaternion results
        quat_idx = find(quat.Phase_String == Phase_String(i));
        if isempty(quat_idx) || isnan(quat.avg_yaw_theta(quat_idx))
            continue % no yaw data available for this phase — leave a_phase as NaN
        end
        theta = quat.avg_yaw_theta(quat_idx); % degrees
        a_phase(i) = 2*s_l*sind(theta/2);
    end

    %% Display actual vs. projected (modeled, ground-contact + rotational correction) step length
    if cfg.has_terrain
        % s_phase / avg_step_per_phase_all are (nTerrain x nP) — loop over terrain too
        Terrain_String = data.Terrain_String;
        s_plus_a_phase = data.s_phase + repmat(a_phase', size(data.s_phase,1), 1); % broadcast a_phase across terrains

        disp('Actual vs. Modeled (s+a) Step Length:')
        for k = 1:length(Terrain_String)
            disp("--- " + Terrain_String(k) + " ---")
            for i = 1:nP
                actual_str = "no data";
                if ~isnan(data.avg_step_per_phase_all(k,i))
                    actual_str = round(data.avg_step_per_phase_all(k,i)*100, 2) + " ± " + round(data.std_step_per_phase_all(k,i)*100, 2) + " cm";
                end

                modeled_str = "no data";
                if ~isnan(s_plus_a_phase(k,i))
                    modeled_str = round(s_plus_a_phase(k,i), 2) + " cm (s=" + round(data.s_phase(k,i),2) + ", a=" + round(a_phase(i),2) + ")";
                end

                disp(Phase_String(i) + ":  actual = " + actual_str + "   |   modeled (s+a) = " + modeled_str)
            end
        end
    else
        % s_phase / avg_step_per_phase are flat (nP x 1) — single terrain dataset
        s_plus_a_phase = data.s_phase + a_phase;

        disp('Actual vs. Modeled (s+a) Step Length:')
        for i = 1:nP
            actual_str = "no data";
            if ~isnan(data.avg_step_per_phase(i))
                actual_str = round(data.avg_step_per_phase(i)*100, 2) + " ± " + round(data.std_step_per_phase(i)*100, 2) + " cm";
            end

            modeled_str = "no data";
            if ~isnan(s_plus_a_phase(i))
                modeled_str = round(s_plus_a_phase(i), 2) + " cm (s=" + round(data.s_phase(i),2) + ", a=" + round(a_phase(i),2) + ")";
            end

            disp(Phase_String(i) + ":  actual = " + actual_str + "   |   modeled (s+a) = " + modeled_str)
        end
    end
end