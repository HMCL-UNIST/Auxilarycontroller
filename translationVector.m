function translation = translationVector(yaw, pitch, roll)

% Copyright 2020 The MathWorks, Inc.

SensorLocation = [0 0];
Height = 2.1798;    % mounting height in meters from the ground
rotationMatrix = (...
    rotZ(yaw)*... % last rotation
    rotX(90-pitch)*...
    rotZ(roll)... % first rotation
    );


% Adjust for the SensorLocation by adding a translation
sl = SensorLocation;

translationInWorldUnits = [sl(2), sl(1), Height];
translation = translationInWorldUnits*rotationMatrix;

end
