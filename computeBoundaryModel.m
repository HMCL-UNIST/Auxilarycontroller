function yWorld = computeBoundaryModel(model, xWorld)

% Copyright 2020 The MathWorks, Inc.
yWorld = polyval(model, xWorld);

end