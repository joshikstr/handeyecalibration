function checktagposes(istagdetected)
    %CHECKTAGPOSES Summary of this function goes here
    %   Detailed explanation goes here
    % check if enough valid poses detected 
    validtagposes = sum(istagdetected);
    if validtagposes > 5
        disp('perfect. got enough valid tag poses.')
    elseif validtagposes > 1
        msg = ['hand eye calibration not recommended \n'...
            'please get more valid tag poses'];
        warning('u:stuffed:it',msg)
    else
        msg = ['hand eye calibration not possible \n' ...
            'minimun requirement are two valid tag poses'];
        error('u:stuffed:it',msg)
    end
end

