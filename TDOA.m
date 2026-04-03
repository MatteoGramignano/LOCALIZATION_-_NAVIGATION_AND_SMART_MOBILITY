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

%% Load TDOA measurements

TYPE = 'TDOA';
parameters.sigmaTDOA = 3; % [m]

R = BuildCovarianceMatrix(parameters, TYPE);

switch DATASET
    case 'FIRST' 
        rho = TDoA{1, 1};
    case 'SECOND' 
        rho = TDoA{2, 1};
    case 'THIRD'
        rho = TDoA{3, 1};
end

%% EKF - Tracking position and velocity - NCV motion model

% Maximum number of NLS iterations
max_iter_vect = [1 50 1e2]; 

% EKF tracking results for different NLS iterations
x_hat_simul = {};

%set tollerance for NLS iterations
tol = 1e-4;

% Repeat simulation for each maximum NLS iteration value
for irun = 1:length(max_iter_vect)

    %get the current number of max iteration for NLS
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
  
    % Time update
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

        % UPDATE (EKF with NaN handling)
        % Available measurements
        valid = ~isnan(rho(1:(end-1), time)); 
        % (Last element of each column of "rho" does not contain TDOA info)           
        if ~any(valid)
            % Update skipped
            x_hat(:, time) = x_pred;
            P_hat(:, :, time) = P_pred;
        else
            rho_new = rho(valid, time);
            R_new  = R(valid, valid); % Covariance sub-matrix
            
            % NLS iterations
            for iter = 1:max_iter
                h = MeasurementModel(parameters, x_pred(1:2), AP(1:2, :), rho(end, time), TYPE);   
                H = BuildJacobianMatrixH(parameters, x_pred(1:2), AP(1:2, :), rho(end, time), TYPE);
                H = [H, zeros(size(H, 1), 2)]; % [H_position H_velocity]
    
                h_new = h(valid); 
                H_new = H(valid, :);             
    
                delta_rho = rho_new - h_new;
    
                % WNLS step
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
    
            % Updated results
            x_hat(:, time) = x_upd;
            P_hat(:, :, time) = P_upd;
        end
    
    end

    % Save simulation results
    x_hat_simul{1, irun} = x_hat;
end     

%% PLOT RESULTS FOR DIFFERENT NLS ITERATIONS

PlotResultsXY(GT, GT_V, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, DATASET, TYPE)

%% PLOT ERROR VS TIME FOR DIFFERENT NLS ITERATIONS

PlotErrorVsIteration(GT, x_hat_simul, max_iter_vect, parameters.samplingTime, parameters.simulationTime, sigma_driving, ...
    DATASET, TYPE);

%% PLOT ESTIMATED TRAJECTORY FOR DIFFERENT NLS ITERATIONS

figure()
legend_entry = {};
idx = 1;

for irun = 1:length(max_iter_vect)

    subplot(1, 3, irun)

    % Load simulation results
    x_hat = x_hat_simul{1, irun};
     
    plot(AP(1, :), AP(2, :), '^', 'MarkerSize', 10, 'MarkerEdgeColor', [147, 0, 0]./255, 'MarkerFaceColor', [147, 0, 0]./255)
    hold on
    plot(GT(1, :), GT(2, :), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', [0.30, 0.75, 0.93], 'MarkerFaceColor', [0.30, 0.75, 0.93])
    hold on
    plot(x_hat(1, :), x_hat(2, :), '-g', 'Marker', 's')
    legend('AP', 'True UE', 'EKF estimate')
    xlabel('[m]')
    ylabel('[m]')
    axis equal
    grid on
    box on
   
    if irun ~= 1
        xlim([-40 40])
        ylim([-100 60])
        title(num2str(max_iter_vect(irun)) + " iterations")
    else
        ylim([-300 100])
        title("1 iteration")
    end

end
grid on
legend(legend_entry)
sgtitle("Dataset: "+ DATASET + " - Localization method: " + TYPE)
