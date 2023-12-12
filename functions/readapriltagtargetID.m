function [posetargetID, isdetected] = readapriltagtargetID(I,intrinsics,tagSize,targetID)
    %READAPRILTAGTARGETID detects all apriltags in an image I and read out
    % the pose of the id taregetID of interest
    [id, ~, pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    if id == targetID
        posetargetID = pose;
        isdetected = true;
    else
        if ismember(targetID,id)
            posetargetID = pose(targetID==id);
            isdetected = true;
        else
            warning_message = ['no apriltag with target ID detected \n' ,...
            'make sure the apriltag is visable in the image and you using the correct taget ID'];
            warning('u:stuffed:it',warning_message)
            posetargetID = rigidtform3d(eye(4,4));
            isdetected = false;
        end
    end
end

