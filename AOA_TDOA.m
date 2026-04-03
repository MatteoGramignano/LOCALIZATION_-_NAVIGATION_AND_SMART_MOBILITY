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

%% Load TDOA and AOA measurements

TYPE1 = 'TDOA';
parameters.sigmaTDOA = 3; % [m]
TYPE2 = 'AOA';
parameters.sigmaAOA = deg2rad(3); % [rad]

R_TDOA = BuildCovarianceMatrix(parameters, TYPE1);
R_AOA = BuildCovarianceMatrix(parameters, TYPE2);
R = [R_TDOA, zeros(size(R_TDOA, 1), size(R_AOA, 1)); zeros(size(R_AOA, 1), size(R_TDOA, 1)), R_AOA];

switch DATASET
    case 'FIRST' 
        rho_TDOA = TDoA{1, 1};
        rho_AOA_raw = AoA{1, 1};
    case 'SECOND' 
        rho_TDOA = TDoA{2, 1};
        rho_AOA_raw = AoA{2, 1};
    case 'THIRD'
        rho_TDOA = TDoA{3, 1};
        rho_AOA_raw = AoA{3, 1};
end

az_local = rho_AOA_raw(1:10, :); % First 10 rows: local azimuths
% Correction: add AP yaw to each AP
ap_yaw_mat = repmat(APyaw', 1, size(az_local, 2));
% az_global = ap_yaw_mat + az_local
rho_AOA = ap_yaw_mat + az_local; 

%% EKF - Tracking position and velocity using NCV motion model

% Maximum number of NLS iterations
max_iter_vect = [1 50 1e2]; 

% EKF tracking results for different NLS iterations
x_hat_simul = {};

% Set tolerance for NLS iterations
tol = 1e-4;

for irun = 1:length(max_iter_vect)

    max_iter = max_iter_vect(irun);

    % Initialization
    UE_init = [0; 0; 0; 0]; % [u_x; u_y; v_x; v_y] (2D)
    UE_init_COV = diag([100^2, 100^2, 1^2, 1^2]);
    x_hat = NaN(4, parameters.simulationTime);
    P_hat = zeros(4, 4, parameters.simulationTime);
    
    % Motion model parameters
    sigma_driving = 2; % [m/s^2]
    L = [0.5*parameters.samplingTime^2*eye(2); parameters.samplingTime*eye(2)]; 
    Q = sigma_driving^2 * (L * L');
    F = eye(4) + parameters.samplingTime*diag(ones(2, 1), 2);

    % Time update loop
    tic
    for time = 1:parameters.simulationTime
        
        % PREDICTION
        if time == 1
            x_pred = UE_init;
            P_pred0 = UE_init_COV;
        else
            x_pred = F * x_hat(:, time-1);
            P_pred0 = F * P_hat(:, :, time-1) * F' + Q;
        end
        P_pred = P_pred0;

        % UPDATE (EKF with handling of NaN entries)
        % Select available measurements
        valid_TDOA = ~isnan(rho_TDOA(1:(end-1), time));
        % (Last element of each column of "rho" does not contain TDOA information)
        valid_AOA = ~isnan(rho_AOA(1:end, time));

        if ~any(valid_TDOA) && ~any(valid_AOA)
            % Skip update if no valid measurements
            x_hat(:, time) = x_pred;
            P_hat(:, :, time) = P_pred;
        else
            rho_TDOA_new = rho_TDOA(valid_TDOA, time);
            rho_AOA_new = rho_AOA(valid_AOA, time);

            R_TDOA_new = R_TDOA(valid_TDOA, valid_TDOA);
            R_AOA_new = R_AOA(valid_AOA, valid_AOA);

            % Reduced covariance matrix
            R_new = [R_TDOA_new, zeros(size(R_TDOA_new, 1), size(R_AOA_new, 1));
                zeros(size(R_AOA_new, 1), size(R_TDOA_new, 1)), R_AOA_new];
    
            % NLS iterations
            for iter = 1:max_iter
                h_TDOA = MeasurementModel(parameters, x_pred(1:2), AP(1:2, :), rho_TDOA(end, time), TYPE1);  
                h_AOA = MeasurementModel(parameters, x_pred(1:2), AP(1:2, :), [], TYPE2);

                H_TDOA = BuildJacobianMatrixH(parameters, x_pred(1:2), AP(1:2, :), rho_TDOA(end, time), TYPE1);
                H_AOA = BuildJacobianMatrixH(parameters, x_pred(1:2), AP(1:2, :), [], TYPE2);
                
                h_TDOA_new = h_TDOA(valid_TDOA);
                h_AOA_new = h_AOA(valid_AOA);

                H_TDOA_new = H_TDOA(valid_TDOA, :);
                H_AOA_new = H_AOA(valid_AOA, :);
                H_new = [H_TDOA_new; H_AOA_new];
                H_new = [H_new, zeros(size(H_new, 1), 2)]; % [H_position H_velocity]

                delta_rho_TDOA = rho_TDOA_new - h_TDOA_new;
                delta_rho_AOA = rho_AOA_new - h_AOA_new;
                
                % Wrap AOA differences
                delta_rho_AOA = mod(delta_rho_AOA + pi, 2*pi) - pi; % range [-pi, pi)
                delta_rho = [delta_rho_TDOA; delta_rho_AOA];
    
                % WNLS update
                G = (P_pred * H_new') * inv(H_new * P_pred * H_new' + R_new); % Kalman gain
                dx = G * delta_rho; % State increment
    
                % State update
                x_upd = x_pred + dx;
    
                % Covariance update
                P_upd = P_pred - G * H_new * P_pred;
                
                % Stopping criterion
                if norm(dx(1:2)) < tol 
                    break
                end
    
                % Next iteration
                x_pred = x_upd; 
                P_pred = P_upd;
            end
    
            % Store updated results
            x_hat(:, time) = x_upd;
            P_hat(:, :, time) = P_upd;
        end   
    end
    x_hat_simul{1, irun} = x_hat;

end     

%% PLOT RESULTS FOR DIFFERENT NLS ITERATIONS

method = TYPE1 + "+" + TYPE2;
PlotResultsXY(GT, GT_V, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, DATASET, method)

%% PLOT ERROR VS TIME FOR DIFFERENT NLS ITERATIONS

method = TYPE1 + "+" + TYPE2;
PlotErrorVsIteration(GT, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, sigma_driving, ...
    DATASET, method);

%% PLOT ESTIMATED TRAJECTORY FOR DIFFERENT NLS ITERATIONS

method = TYPE1 + "+" + TYPE2;
PlotEstimatedTrajVsIter(GT(1:2, :), AP(1:2, :), x_hat_simul, max_iter_vect, DATASET, method);
