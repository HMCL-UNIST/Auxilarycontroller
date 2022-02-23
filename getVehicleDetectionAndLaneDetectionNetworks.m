function getVehicleDetectionAndLaneDetectionNetworks

if exist('trainedLaneNet.mat','file') == 0
    disp('Downloading pretrained lane detection network (143 MB)...')
    url = 'https://www.mathworks.com/supportfiles/gpucoder/cnn_models/lane_detection/trainedLaneNet.mat';
    websave('trainedLaneNet.mat',url);
end

if  ~exist('yolov2ResNet50VehicleExample.mat','file')
    disp('Downloading pretrained vehicle detection network (98 MB)...')
    pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/yolov2ResNet50VehicleExample.mat';
    websave('yolov2ResNet50VehicleExample.mat',pretrainedURL);
end

end