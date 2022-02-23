% Given the Yaw, Pitch and Roll, figure out appropriate Euler
% angles and the sequence in which they are applied in order to
% align the camera's coordinate system with the vehicle coordinate
% system. The resulting matrix is a Rotation matrix that together
% with Translation vector define the camera extrinsic parameters.

function rotation = rotationMatrix(yaw, pitch, roll)

% Copyright 2020 The MathWorks, Inc.

rotation = (...
    rotY(180)*...            % last rotation: point Z up
    rotZ(-90)*...            % X-Y swap
    rotZ(yaw)*...       % point the camera forward
    rotX(90-pitch)*...  % "un-pitch"
    rotZ(roll)...       % 1st rotation: "un-roll"
    );

end
