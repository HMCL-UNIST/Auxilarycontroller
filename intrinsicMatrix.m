function intrinsicMat = intrinsicMatrix(FocalLength, Skew, PrincipalPoint)

% Copyright 2020 The MathWorks, Inc.

intrinsicMat = ...
    [FocalLength(1)  , 0                     , 0; ...
    Skew             , FocalLength(2)   , 0; ...
    PrincipalPoint(1), PrincipalPoint(2), 1];

end

