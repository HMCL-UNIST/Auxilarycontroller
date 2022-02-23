% Rotation around Z-axis

function R = rotZ(a)

% Copyright 2020 The MathWorks, Inc.

a = deg2rad(a);
R = [...
    cos(a) -sin(a) 0;
    sin(a)  cos(a) 0;
    0       0      1];
end
