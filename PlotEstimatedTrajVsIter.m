function plotEstimatedTrajVsIter(GT, AP, x_hat_simul, max_iter_vect, DATASET, TYPE)

    figure()
    legend_entry = {};

    for irun = 1:length(max_iter_vect)
    
        subplot(1, 3, irun)
    
        % Load simulation results
        x_hat = x_hat_simul{1, irun};
        
        plot(AP(1, :), AP(2, :), '^', 'MarkerSize', 10, 'MarkerEdgeColor', [147, 0, 0]./255, 'MarkerFaceColor', [147, 0, 0]./255)
        hold on
        plot(GT(1, :), GT(2, :), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', [0.30, 0.75, 0.93], 'MarkerFaceColor', ...
            [0.30, 0.75, 0.93])
        hold on
        plot(x_hat(1, :), x_hat(2, :), '-g', 'Marker', 's')
        legend('AP', 'True UE', 'EKF estimate')
        xlabel('[m]')
        ylabel('[m]')
        grid on
        box on
        title(num2str(max_iter_vect(irun)) + " iterations")
        xlim([-40 10])
        ylim([-100 80])
    
    end
    grid on
    legend(legend_entry)
    sgtitle("Dataset: " + DATASET + " - Localization method: " + TYPE)
   
end
