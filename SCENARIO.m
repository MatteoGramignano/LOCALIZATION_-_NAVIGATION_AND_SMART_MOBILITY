
clear, close all
clc

set(0,'DefaultTextFontSize', 14)
set(0,'DefaultAxesFontSize', 14)
set(0,'DefaultLineLineWidth', 1.5)
set(0,'DefaultTextInterpreter', 'latex')

load('LNSM_Project_Data.mat')

%% Dataset selection

parameters.numberOfAP = size(AP, 2);

% Choose dataset
DATASET = 'FIRST';

switch DATASET
    case 'FIRST'
        GT = ground_truth{1, 1}; % Track with obstacle (Case 1)
        dati = TDoA{1,1};
    case 'SECOND'
        GT = ground_truth{2, 1}; % Free track
    case 'THIRD'
        GT = ground_truth{3, 1}; % Track with obstacle (Case 2)
end

parameters.simulationTime = size(GT, 2);
parameters.samplingTime = 0.1; % [s]

%% VISUALIZATION OF THE DATA

figure('Name', 'Posizione AP e Ground Truth in 2D', 'NumberTitle', 'off');
hold on; grid on;

% Plot Access Point (black squared markers)
scatter(AP(1, :), AP(2, :), 100, 'ks', 'filled', 'DisplayName', 'Access Points');

% Trajectories
colors = lines(3);

% Plot ground truth
for traj = 1:3
    GT = ground_truth{traj}; % Nx3
    plot(GT(1, :), GT(2, :), 'Color', colors(traj, :), 'LineWidth', 2, ...
        'DisplayName', sprintf('GT Traj %d', traj));
end

xlabel('X [m]');
ylabel('Y [m]');
legend('show', 'Location', 'bestoutside');
axis equal;
title('Access Points e Ground Truth (view from above)');

%% GIF creation: scenario with trajectory and Access Points

figure('Color','w','Units','pixels','Position',[100 100 1200 800]); 

% Figure parameters
xlim([-60 30]) 
ylim([-100 60])                
grid on;                       
box on                        
% Axis equal                     
xlabel('X [m]', 'FontSize', 14, 'FontWeight', 'bold'); 
ylabel('Y [m]', 'FontSize', 14, 'FontWeight', 'bold');

% Name of the GIF file to create
filename = 'SCENARIO.gif';

% Loop over time with steps of 5 samples
for time = 1:5:parameters.simulationTime
    
    cla;
    hold on;

    % Access Points (AP) available at this instant
    Ap_av = find(~isnan(dati(1:end-1, time))); % Indices of available APs (not NaN)
    master = dati(end, time); % Current master AP

    % Add master to the list of available APs
    v = [Ap_av; master];
    for i = 2:length(v)
        if v(i) <= v(i-1)
            v(i) = v(i-1) + 1;
        end
    end
    
    % Update available AP list
    Ap_av = v;  

    % All APs
    plot(AP(1, :), AP(2, :), '^', 'MarkerSize', 12, ...
        'MarkerEdgeColor', [0.3 0.3 0.3], ...
        'MarkerFaceColor', [0.7 0.7 0.7]);

    % Active APs
    plot(AP(1, Ap_av), AP(2, Ap_av), '^', 'MarkerSize', 14, ...
        'MarkerEdgeColor', [0.7 0 0], ...
        'MarkerFaceColor', [1 0 0]);

    % Ground truth trajectory
    plot(GT(1, :), GT(2, :), '-', 'LineWidth', 3, ...
        'Color', [0.2 0.6 1]);

    % Current position
    plot(GT(1, time), GT(2, time), 'o', 'MarkerSize', 14, ...
        'MarkerEdgeColor', [0 0.2 0.5], ...
        'MarkerFaceColor', [1 0.5 0]);

    title(sprintf('Localization Simulation - Time %.1f s', ...
        time*parameters.samplingTime), 'FontSize', 16, 'FontWeight', 'bold');

    set(gca,'FontSize', 14, 'LineWidth', 1.5);

    legend({'Total APs', 'Active APs', 'GT Trajectory', 'Current Position'}, ...
       'Location', 'bestoutside', 'FontSize', 12);

    % Update figure
    drawnow;  

    % Save frame to GIF
    frameFilename = sprintf('frame_%04d.png', time); % Temporary frame name
    exportgraphics(gcf, frameFilename, 'BackgroundColor', 'white', 'Resolution', 300);
    img = imread(frameFilename);
    [A, map] = rgb2ind(img, 256);
    if time == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.2);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
    end
    % Remove temporary file
    delete(frameFilename);

end
