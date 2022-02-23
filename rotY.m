% Rotation around Y-axis

function R = rotY(a)

% Copyright 2020 The MathWorks, Inc.

a = deg2rad(a);
R = [...
    cos(a)  0 sin(a);
    0       1 0;
    -sin(a) 0 cos(a)];

end
