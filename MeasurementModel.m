function [h] = MeasurementModel(parameters, UE, AP, AP_m, TYPE) 

% Compute the distance between UE and APs
distanceUEAP = sqrt(sum([UE - AP].^2, 1));

% Evaluate direction cosine
directionCosineX = (UE(1) - AP(1, :)) ./ distanceUEAP;
directionCosineY = (UE(2) - AP(2, :)) ./ distanceUEAP;

% Build the vector/matrix of observation
switch TYPE
    case 'TDOA'
        h = zeros((parameters.numberOfAP - 1), 1);
        for a = 1:(parameters.numberOfAP-1)
            if a < AP_m
                h(a) = distanceUEAP(a) - distanceUEAP(AP_m); 
            end
            if a >= AP_m
                h(a) = distanceUEAP(a+1) - distanceUEAP(AP_m);
            end  
        end
    case 'AOA'
        h = zeros(parameters.numberOfAP, 1);
        for a = 1:parameters.numberOfAP
            h(a) = atan2(directionCosineY(a)*distanceUEAP(a), directionCosineX(a)*distanceUEAP(a));
        end
end

end
