function [R] = buildCovarianceMatrix(parameters , TYPE)

% Construct a diagonal matrix
switch TYPE
    case 'TDOA'
        R = diag(repmat(parameters.sigmaTDOA.^2, 1, (parameters.numberOfAP - 1)));
    case 'AOA'
        R = diag(repmat(parameters.sigmaAOA.^2, 1, parameters.numberOfAP));
end

end
