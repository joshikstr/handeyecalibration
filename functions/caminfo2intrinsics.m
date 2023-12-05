function intrinsics = caminfo2intrinsics(caminfo)
    %CAMINFO2INTRINSICS Summary of this function goes here
    %   Detailed explanation goes here
    focalLength = [caminfo.K(1), caminfo.K(5)];
    principalPoint = [caminfo.K(3), caminfo.K(6)];
    imageSize = [double(caminfo.Height), double(caminfo.Width)];
    intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
end

