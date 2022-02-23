% Rotation around X-axis

function R = rotX(a)

% Copyright 2020 The MathWorks, Inc.

a = deg2rad(a);
R = [...
    1   0        0;
    0   cos(a)  -sin(a);
    0   sin(a)   cos(a)];

end
