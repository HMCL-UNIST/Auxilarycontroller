function [bboxes,scores] = vehicle_detection_yolo_v2(In)

% Copyright 2020 The MathWorks, Inc.

persistent yolodetector;
if isempty(yolodetector)
    yolodetector = coder.loadDeepLearningNetwork('yolov2ResNet50VehicleExample.mat');
end

[bboxes,scores,~] = yolodetector.detect(In, 'threshold', .2);

end
