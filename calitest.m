Data = out.simout.Data;
for i = 1:size(Data,3)
    laneNetOut = Data(:,:,i);
    [laneFound,ltPts,rtPts] = lane_detection_coordinates(laneNetOut);
end