function plotErrorVsIteration(GT, x_hat_simul, max_iter_vect, samplingTime, simulationTime, sigma_driving, DATASET, TYPE)

    figure()
    legend_entry = {};
    idx = 1;
    
    for irun = 1:length(max_iter_vect)
    
        x_hat = x_hat_simul{1, irun};
        
        % Position error
        DeltaPosition_EKF = GT(1:2, :) - x_hat(1:2, :);
        err_EKF = sqrt(sum(DeltaPosition_EKF.^2, 1));
     
        semilogy(samplingTime:samplingTime:(samplingTime*simulationTime-samplingTime), err_EKF(2:end))
        
        hold on
        
        legend_entry{idx} = "Max iter = " + num2str(max_iter_vect(irun));
        idx = idx + 1;
    
    end
    xlabel('Time [s]')
    ylabel('10log$_{10}$ error','Interpreter','latex')
    title("Dataset: "+ DATASET + " - Localization method: "+ TYPE)
    subtitle("Position error - $\sigma$="+num2str(sigma_driving), 'Interpreter', 'latex')
    grid on
    legend(legend_entry)

end
