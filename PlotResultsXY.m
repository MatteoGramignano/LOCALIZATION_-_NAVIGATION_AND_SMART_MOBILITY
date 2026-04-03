function plotResultsXY(GT, GT_V, x_hat_simul, max_iter_vect, samplingTime, simulationTime, DATASET, TYPE)

    for plotindex=1:length(max_iter_vect)

    % Load simulation results
    x_hat=x_hat_simul{1,plotindex};

    % Single components of the state x
    figure()
    hold on
    subplot(2, 2, 1)
    plot(samplingTime:samplingTime:samplingTime*simulationTime, GT(1, :))
    hold on
    plot(samplingTime:samplingTime:samplingTime*simulationTime, x_hat(1, :))
    xlabel('Time [s]')
    ylabel('[m]')
    title('Position error $x$')
    legend('True', 'EKF estimate')
    
    subplot(2, 2, 2)
    plot(samplingTime:samplingTime:samplingTime*simulationTime, GT(2, :))
    hold on
    plot(samplingTime:samplingTime:samplingTime*simulationTime, x_hat(2, :))
    xlabel('Time [s]')
    ylabel('[m]')
    title('Position error $y$')
    legend('True', 'EKF estimate')

    subplot(2, 2, 3)
    plot(samplingTime:samplingTime:samplingTime*simulationTime, GT_V(1, :))
    hold on
    plot(samplingTime:samplingTime:samplingTime*simulationTime, x_hat(3, :))
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('Velocity error $v_x$')
    legend('True', 'EKF estimate')
    
    subplot(2, 2, 4)
    plot(samplingTime:samplingTime:samplingTime*simulationTime, GT_V(2, :))
    hold on
    plot(samplingTime:samplingTime:samplingTime*simulationTime, x_hat(4, :))
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('Velocity error $v_y$')
    legend('True', 'EKF estimate')

    sgtitle("Dataset: " + DATASET+ " - Localization method: " + TYPE+ " - Max iter for NLS: " + num2str(max_iter_vect(plotindex)))

    end

end
