function [H] = BuildJacobianMatrixH(parameters, UE, AP, AP_m, TYPE)

% Compute the distance between UE and APs
distanceUEAP = sqrt(sum([UE - AP].^2, 1)); 

% Evaluate direction cosine
directionCosineX = (UE(1) - AP(1, :)) ./ distanceUEAP;
directionCosineY = (UE(2) - AP(2, :)) ./ distanceUEAP;

switch TYPE
    case 'TDOA'
        % Build H
        H = zeros((parameters.numberOfAP - 1), 2);
        for a = 1:(parameters.numberOfAP-1)
            if a < AP_m
                H(a, :) = [-directionCosineX(AP_m)+directionCosineX(a), -directionCosineY(AP_m)+directionCosineY(a)];
            end
            if a >= AP_m
                H(a, :) = [-directionCosineX(AP_m)+directionCosineX(a+1), -directionCosineY(AP_m)+directionCosineY(a+1)];
            end
        end
    case 'AOA'
        H = zeros(parameters.numberOfAP, 2);
        for a = 1:parameters.numberOfAP
            H(a, :) = [-directionCosineY(a)/distanceUEAP(a), directionCosineX(a)/distanceUEAP(a)];
        end
end

end
