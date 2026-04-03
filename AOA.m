clear, close all
clc

% Plot style
set(0, 'DefaultTextFontSize', 14)
set(0, 'DefaultAxesFontSize', 14)
set(0, 'DefaultLineLineWidth', 1.5)
set(0, 'DefaultTextInterpreter', 'latex')

% Load data
load('LNSM_Project_Data.mat')

%% Load UE trajectory

parameters.numberOfAP = size(AP, 2);

DATASET = 'FIRST';

switch DATASET
    case 'FIRST'
        GT = ground_truth{1, 1}; % Track with obstacle (Case 1)
    case 'SECOND'
        GT = ground_truth{2, 1}; % Free track
    case 'THIRD'
        GT = ground_truth{3, 1}; % Track with obstacle (Case 2)
end

parameters.simulationTime = size(GT, 2);
parameters.samplingTime = 0.1; % [s]

% Instantaneous (average) velocity 
for t = 2:parameters.simulationTime
    GT_V(:, t) = (GT(:, t) - GT(:, t-1)) / parameters.samplingTime; % [m/s]
end

%% Load AOA measurements

TYPE = 'AOA';
parameters.sigmaAOA = deg2rad(3);

R = BuildCovarianceMatrix(parameters, TYPE);

switch DATASET
    case 'FIRST' 
        rho = AoA{1, 1};
    case 'SECOND' 
        rho = AoA{2, 1};
    case 'THIRD'
        rho = AoA{3, 1};
end


%% AoA local-to-global correction (first trajectory) + plot

% Extract local AoA and AP yaw
az_local = rho(1:10, :); % First 10 rows = local azimuths
ap_yaw_vec = APyaw(1, :); % Yaw of APs [radians]

% Construct time axis (10 Hz -> dt = 0.1 s)
Nsamples = size(az_local, 2);
t = (0:Nsamples-1) * parameters.samplingTime; % [s]

% Before correction ([deg])
figure;
plot(t, rad2deg(az_local'));
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title("Local AoA – before correction");
legend(arrayfun(@(i) sprintf('AP%d', i), 1:10, 'UniformOutput', false), ...
       'Location', 'bestoutside');
grid on;

% Correction: add AP yaw to each AP
ap_yaw_mat = repmat(ap_yaw_vec', 1, size(az_local,2));
az_global = ap_yaw_mat + az_local;

% After correction ([deg])
figure;
plot(t, rad2deg(az_global'));
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title("Local AoA – after correction");
legend(arrayfun(@(i) sprintf('AP%d', i), 1:10, 'UniformOutput', false), ...
       'Location', 'bestoutside');
grid on;

%% 2D AoA localization - EKF - NCV model

% Access Points XY
AP2 = AP(1:2, :);
numAP = size(AP2, 2);

% Time vector
Nsamples = size(az_global, 2);
tvec = (0:Nsamples-1) * parameters.samplingTime;

% EKF parameters
% Maximum number of NLS iterations
max_iter_vect = [1 5 100]; 

% EKF tracking results for different NLS iterations
x_hat_simul = {};

% Set tolerance for NLS iterations
tol = 1e-4;

% Repeat simulation for each maximum NLS iteration
for irun = 1:length(max_iter_vect)

    max_iter = max_iter_vect(irun);
    
    % State: NCV [x, y, v_x, v_y] - Nearly Constant Velocity model
    x_hat = NaN(Nsamples, 4);
    P_hat = NaN(4,4,Nsamples);
    
    x0 = [0, 0, 0, 0]'; % Initial position (from GT) & Zero velocity
    P0 = diag([10, 10, 1, 1].^2);
    
    x_pred = x0;
    P_pred = P0;
    
    dt = parameters.samplingTime;
    F = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
    
    sigma_driving = 2;
    
    L = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
    
    Q = sigma_driving * (L*L');
    
    % EKF loop
    for k = 1:Nsamples
    
        % Predict
        if k > 1
            x_pred = F * x_hat(k-1, :)';
            P_pred = F * P_hat(:, :, k-1) * F' + Q;
        end
    
        % Select only APs that are not NaN
        z_all = az_global(:, k);
        valid_idx = ~isnan(z_all);
    
        if any(valid_idx)
            AP_valid = AP2(:, valid_idx);
            z = z_all(valid_idx);
    
            % Update temporary parameters
            parameters.numberOfAP = sum(valid_idx);
    
            % Covariance matrix for valid APs
            R_sub = BuildCovarianceMatrix(parameters, TYPE);
    
            % NLS iterations
            for iter = 1:max_iter

                % Jacobian
                H_pos = BuildJacobianMatrixH(parameters, x_pred(1:2), AP_valid,[], TYPE);
                H = [H_pos, zeros(parameters.numberOfAP, 2)];
        
                % Measurement prediction
                hpos = MeasurementModel(parameters, x_pred(1:2), AP_valid, [],TYPE)';
        
                % AoA residual (wrapping)
                y_resid = z - hpos';
                y_resid = atan2( sin(y_resid), cos(y_resid) );
        
                % Update
                S = H * P_pred * H' + R_sub;
                Kk = (P_pred * H') / S;
                x_upd = x_pred + Kk * y_resid;
                P_upd = (eye(4) - Kk * H) * P_pred;

                dx = Kk * y_resid;

                % Stopping criterion
                if norm(dx(1:2)) < tol 
                    break
                end

                x_pred = x_upd;
                P_pred = P_upd;
            end

        else
            x_upd = x_pred;
            P_upd = P_pred;
        end
            
        % Save results
        x_hat(k,:) = x_upd;
        P_hat(:, :, k) = P_upd;
    end

    % Save simulation results
    x_hat_simul{1, irun} = x_hat.';

end

%% PLOT RESULTS FOR DIFFERENT NLS ITERATIONS

PlotResultsXY(GT, GT_V, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, DATASET, TYPE)

%% PLOT ERROR VS TIME FOR DIFFERENT NLS ITERATIONS

PlotErrorVsIteration(GT, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, sigma_driving, ...
    DATASET, TYPE);

%% PLOT ESTIMATED TRAJECTORY FOR DIFFERENT NLS ITERATIONS

PlotEstimatedTrajVsIter(GT(1:2,:), AP(1:2,:), x_hat_simul, max_iter_vect, DATASET, TYPE);
